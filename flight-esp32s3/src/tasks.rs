use defmt::{info, println};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    channel::Channel,
    mutex::Mutex,
};
use embassy_time::{Duration, Ticker};
use esp_hal::{Async, gpio::Output, spi::master::Spi};
use esp_radio::esp_now::EspNow;
use flight_core::{DroneState, altitude_estimator, fusion, fusion::AhrsConfig, imu::ImuFrame};
use flight_core::comms::Receiver;
use crate::{IMU_DATA_CHANNEL, STATE_WATCH, baro::BoardBaro};
use crate::comms::BoardRx;

static BARO_CHANNEL: Channel<CriticalSectionRawMutex, f32, 4> = Channel::new();

#[embassy_executor::task]
pub async fn drone_state_task() {
    let mut ahrs = fusion::Ahrs::new(
        AhrsConfig::default()
            .with_gain(1.5)
            .with_gyro_range(1000.0)
            .with_bias_config(None),
    );

    let mut altitude_estimator = altitude_estimator::AltitudeEstimator::new(0.0, 0.15, 0.001, 0.16);

    let state_tx = STATE_WATCH.sender();
    let baro_rx = BARO_CHANNEL.receiver();

    loop {
        let accum_data = IMU_DATA_CHANNEL.receive().await;

        if accum_data.dt > 0.0 && accum_data.samples > 0 {
            let avg_gyro = accum_data.delta_angle / accum_data.dt;
            let avg_accel = accum_data.delta_velocity / accum_data.dt;

            let frame = ImuFrame {
                accel_ms2: avg_accel,
                gyro_rad_s: avg_gyro,
                magnetometer: None,
            };

            ahrs.update(frame, accum_data.dt);
            let accel_z = ahrs.linear_acceleration_z_world();

            altitude_estimator.predict(accel_z, accum_data.dt);

            let euler_deg = ahrs.euler_angles().map(|v| v.to_degrees());
            state_tx.send(DroneState {
                attitude: euler_deg,
                altitude: altitude_estimator.altitude(),
                velocity: altitude_estimator.velocity(),
                accel_bias: altitude_estimator.accel_bias(),
                accel_z,
            });
        }

        if let Ok(baro_alt) = baro_rx.try_receive() {
            altitude_estimator.update_baro(baro_alt);
        }
    }
}

#[embassy_executor::task]
pub async fn telemetry_task() {
    let mut ticker = Ticker::every(Duration::from_hz(10));
    let mut state_rx = STATE_WATCH.receiver().unwrap();

    loop {
        ticker.next().await;
        if let Some(state) = state_rx.try_get() {
            println!(
                "Roll={}, Pitch={}, Yaw={} | Alt={} m, Vel={} m/s",
                state.attitude.x,
                state.attitude.y,
                state.attitude.z,
                state.altitude,
                state.velocity
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
    let ground_pressure = {
        let mut sum = 0.0;
        for _ in 0..50 {
            sum += baro.read_pressure().await;
        }
        sum / 50.0
    };

    let mut ticker = Ticker::every(Duration::from_hz(50));
    let baro_tx = BARO_CHANNEL.sender();

    loop {
        baro_tx
            .send(baro.read_altitude(ground_pressure).await)
            .await;
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn listen_for_commands(esp_now: EspNow<'static>) {
    let (manager, _tx, rx) = esp_now.split();
    manager.set_channel(10).unwrap();
    let target_mac = [172, 39, 110, 170, 188, 84];
    let mut board_rx = BoardRx::new(rx, target_mac);
    
    loop {
        let data = board_rx.receive().await.unwrap();
        info!("{}", data);
    }
}
