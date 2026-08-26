use defmt::println;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker};
use esp_hal::{
    Async, gpio::Output, spi::master::Spi,
};
use flight_core::{fusion, fusion::AhrsConfig, imu::ImuFrame};

use crate::{AHRS_CHANNEL, ATTITUDE_WATCH, baro::BoardBaro};

#[embassy_executor::task]
pub async fn ahrs_task() {
    let mut ahrs = fusion::Ahrs::new(
        AhrsConfig::default()
            .with_gain(1.5)
            .with_gyro_range(1000.0)
            .with_bias_config(None),
    );

    let sender = ATTITUDE_WATCH.sender();

    loop {
        let accum_data = AHRS_CHANNEL.receive().await;

        if accum_data.dt > 0.0 && accum_data.samples > 0 {
            let avg_gyro = accum_data.delta_angle / accum_data.dt;
            let avg_accel = accum_data.delta_velocity / accum_data.dt;

            let frame = ImuFrame {
                accel_g: avg_accel,
                gyro_rad_s: avg_gyro,
                magnetometer: None,
            };

            ahrs.update(frame, accum_data.dt);
            let euler_deg = ahrs.euler_angles().map(|v| v.to_degrees());
            sender.send(euler_deg);
        }
    }
}

#[embassy_executor::task]
pub async fn telemetry_task() {
    let mut ticker = Ticker::every(Duration::from_hz(10));
    let mut attitude_rx = ATTITUDE_WATCH.receiver().unwrap();

    loop {
        ticker.next().await;
        if let Some(att) = attitude_rx.try_get() {
            let roll = att.x;
            let pitch = att.y;
            let yaw = att.z;
            println!(
                "Attitude (deg): roll={}, pitch={}, yaw={}",
                roll, pitch, yaw
            );
        }
    }
}

#[embassy_executor::task]
pub async fn baro_task(
    spi: &'static Mutex<NoopRawMutex, Spi<'static, Async>>,
    cs: Output<'static>,
) {
    let spi_device = SpiDevice::new(spi, cs);
    let mut baro = BoardBaro::new(spi_device).await;

    for _ in 0..50 {
        let m = baro.read_altitude_msl(101200.0).await;
        println!("altitude {}", m);
    }
}
