use core::cell::RefCell;
use critical_section::Mutex;
use embedded_hal_bus::spi::CriticalSectionDevice;
use esp_hal::delay::Delay;
use esp_hal::gpio::Output;
use esp_hal::spi::Mode;
use esp_hal::spi::master::Spi;
use esp_hal::time::Rate;
use esp_hal::{Blocking, spi};
use flight_core::imu::ImuFrame;
use mpu9250::{AccelDataRate, Dlpf, Imu, InterruptConfig, InterruptEnable, Mpu9250, MpuConfig};

pub struct BoardImu {
    pub inner: Mpu9250<
        mpu9250::SpiDevice<
            CriticalSectionDevice<'static, Spi<'static, Blocking>, Output<'static>, Delay>,
        >,
        Imu,
    >,
}

impl BoardImu {
    pub fn new(
        spi_bus: &'static Mutex<RefCell<Spi<'static, Blocking>>>,
        cs_pin: Output<'static>,
    ) -> Self {
        let mut delay = Delay::new();

        let mut config = MpuConfig::imu();
        config
            .accel_scale(mpu9250::AccelScale::_16G)
            .gyro_scale(mpu9250::GyroScale::_1000DPS)
            .accel_data_rate(AccelDataRate::DlpfConf(Dlpf::_0))
            .gyro_temp_data_rate(mpu9250::GyroTempDataRate::DlpfConf(Dlpf::_0));

        let spi_device = CriticalSectionDevice::new(spi_bus, cs_pin, delay).unwrap();

        let mut mpu = Mpu9250::imu_with_reinit(spi_device, &mut delay, &mut config, |dev| {
            critical_section::with(|cs| {
                let mut bus = spi_bus.borrow_ref_mut(cs);
                bus.apply_config(
                    &spi::master::Config::default()
                        .with_frequency(Rate::from_mhz(20))
                        .with_mode(Mode::_0),
                )
                .unwrap();
            });
            Some(dev)
        })
        .unwrap();

        mpu.interrupt_config(InterruptConfig::INT_ANYRD_CLEAR | InterruptConfig::LATCH_INT_EN)
            .expect("Failed to configure interrupt config");

        mpu.enable_interrupts(InterruptEnable::RAW_RDY_EN)
            .expect("Failed to enable RAW_RDY_EN");

        // Dummy read to clear initial power-on interrupt latch
        let _ = mpu.all::<[f32; 3]>();

        Self { inner: mpu }
    }

    pub fn read(&mut self) -> Result<ImuFrame, ()> {
        let m = self.inner.all::<[f32; 3]>().map_err(|_| ())?;

        Ok(ImuFrame {
            accel_g: nalgebra::Vector3::new(m.accel[0], m.accel[1], m.accel[2]),
            gyro_rad_s: nalgebra::Vector3::new(m.gyro[0], m.gyro[1], m.gyro[2]),
            magnetometer: None,
        })
    }
}
