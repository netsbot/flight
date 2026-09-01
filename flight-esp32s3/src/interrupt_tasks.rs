use core::cell::RefCell;

use defmt::println;
use embedded_hal_async::digital::Wait;
use embedded_hal_bus::spi::RefCellDevice;
use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::{Input, Output},
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
};
use flight_core::imu::AccumulatedImu;
use static_cell::StaticCell;

use crate::{CycleInstant, IMU_DATA_CHANNEL, imu::BoardImu};

#[embassy_executor::task]
pub async fn interrupt_main(
    spi: Spi<'static, Blocking>,
    cs_imu: Output<'static>,
    mpu_int: Input<'static>,
) {
    // TODO: change to exclusive spi device
    static IMU_SPI_BUS: StaticCell<RefCell<Spi<'static, Blocking>>> = StaticCell::new();
    let spi_bus = IMU_SPI_BUS.init(RefCell::new(spi));
    let delay = Delay::new();

    let imu_spi_dev = RefCellDevice::new(spi_bus, cs_imu, delay).unwrap();
    let imu = BoardImu::new(imu_spi_dev);

    // Set SPI frequency to 20MHz
    spi_bus
        .borrow_mut()
        .apply_config(
            &Config::default()
                .with_frequency(Rate::from_mhz(20))
                .with_mode(Mode::_0),
        )
        .unwrap();

    // Get local spawner to spawn tasks inside this InterruptExecutor
    let local_spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };
    local_spawner.spawn(imu_task(mpu_int, imu).unwrap());
}

#[embassy_executor::task]
async fn imu_task(
    mut mpu_int: Input<'static>,
    mut imu: BoardImu<RefCellDevice<'static, Spi<'static, Blocking>, Output<'static>, Delay>>,
) {
    let mut last_imu_read = CycleInstant::now();
    let mut accum = AccumulatedImu::ZERO;

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

        // 2. Step 1kHz inner rate controller PID
        // rate_controller.step(
        //     data.gyro_rad_s,
        //     data.accel_g,
        //     dt,
        // );

        // Send snapshot every N samples or when enough time elapsed (~5ms / 200Hz)
        if accum.samples >= 5 || accum.dt >= 0.005 {
            let _ = IMU_DATA_CHANNEL.try_send(accum);
            accum = AccumulatedImu::ZERO;
        }
    }
}
