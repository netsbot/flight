use flight_core::{PlaneState, altitude_estimator, fusion, fusion::AhrsConfig, imu::ImuFrame};

use crate::{BARO_CHANNEL, IMU_DATA_CHANNEL, STATE_WATCH};

#[embassy_executor::task]
pub async fn drone_state_task() {
    let mut ahrs = fusion::Ahrs::new(
        AhrsConfig::default()
            .with_gain(1.5)
            .with_gyro_range(1000.0)
            .with_bias_config(None),
    );

    let mut altitude_estimator =
        altitude_estimator::AltitudeEstimator::new(0.0, 0.15, 0.001, 0.16);

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
            state_tx.send(PlaneState {
                attitude: euler_deg,
                altitude: altitude_estimator.altitude(),
            });
        }

        if let Ok(baro_alt) = baro_rx.try_receive() {
            altitude_estimator.update_baro(baro_alt);
        }
    }
}
