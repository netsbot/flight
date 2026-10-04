use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::{Input, Output},
    spi::
    master::Spi,
};
use flight_core::imu::{AccumulatedImu, ImuFrame};
use mpu9250::{AccelDataRate, Dlpf, Imu, InterruptConfig, InterruptEnable, Mpu9250, MpuConfig};

use crate::{CycleInstant, IMU_DATA_CHANNEL};

pub struct BoardImu<SPI: embedded_hal::spi::SpiDevice> {
    pub inner: Mpu9250<SPI, Imu>,
}

impl<SPI: embedded_hal::spi::SpiDevice> BoardImu<SPI> {
    pub fn new(spi_device: SPI) -> Self {
        let mut delay = Delay::new();

        let mut config = MpuConfig::imu();
        config
            .accel_scale(mpu9250::AccelScale::_16G)
            .gyro_scale(mpu9250::GyroScale::_1000DPS)
            .accel_data_rate(AccelDataRate::DlpfConf(Dlpf::_0))
            .gyro_temp_data_rate(mpu9250::GyroTempDataRate::DlpfConf(Dlpf::_0));

        let mut mpu = Mpu9250::imu(spi_device, &mut delay, &mut config).unwrap();

        mpu.interrupt_config(InterruptConfig::INT_ANYRD_CLEAR | InterruptConfig::LATCH_INT_EN)
            .expect("Failed to configure interrupt config");

        mpu.enable_interrupts(InterruptEnable::RAW_RDY_EN)
            .expect("Failed to enable RAW_RDY_EN");

        // Dummy read to clear initial power-on interrupt latch
        let _ = mpu.all::<[f32; 3]>();

        Self { inner: mpu }
    }

    pub fn read(&mut self) -> ImuFrame {
        let data = self.inner.all::<[f32; 3]>().unwrap();

        ImuFrame {
            accel_ms2: nalgebra::Vector3::new(data.accel[0], data.accel[1], data.accel[2]),
            gyro_rad_s: nalgebra::Vector3::new(data.gyro[0], data.gyro[1], data.gyro[2]),
            magnetometer: None,
        }
    }
}

#[embassy_executor::task]
pub async fn imu_task(
    spi: Spi<'static, Blocking>,
    cs_imu: Output<'static>,
    mut mpu_int: Input<'static>,
) {
    let mut last_imu_read = CycleInstant::now();
    let mut accum = AccumulatedImu::ZERO;
    let spi_device = ExclusiveDevice::new(spi, cs_imu, Delay::new()).unwrap();
    let mut imu = BoardImu::new(spi_device);

    loop {
        // Await data-ready hardware interrupt without startup edge deadlock
        if mpu_int.is_low() {
            mpu_int.wait_for_high().await;
        }

        let data = imu.read();

        let dt = last_imu_read.elapsed_secs_and_reset();

        // 1. Accumulate pre-integration delta angles and delta velocities
        accum.delta_angle += data.gyro_rad_s * dt;
        accum.delta_velocity += data.accel_ms2 * dt;
        accum.dt += dt;
        accum.samples += 1;

        // Send snapshot every N samples or when enough time elapsed (~5ms / 200Hz)
        if accum.samples >= 5 || accum.dt >= 0.005 {
            let _ = IMU_DATA_CHANNEL.try_send(accum);
            accum = AccumulatedImu::ZERO;
        }
    }
}
