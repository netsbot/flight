use flight_core::fusion::Convention;
use flight_core::imu::{AccumulatedImu, ImuFrame};
use flight_core::mix;
use flight_sitl::WebotsConnection;
use nalgebra::Vector3;
use std::fs::File;
use std::io::Write;

fn main() {
    let mut data_source = WebotsConnection::bind("127.0.0.1:5599");

    let mut rot_setpoint = Vector3::zeros();
    let mut throttle = 50.0f32;
    let mut vertical_vel_setpoint = 0.0;
    let mut alt_setpoint = 5.0;

    let mut rate_controllers = flight_core::ControllerBundle::new_rate();
    let mut attitude_controller = flight_core::ControllerBundle::new_attitude();

    let mut vertical_vel_controller =
        flight_core::pid::PidController::new(10.0, 2.0, 1.0, 15.0, 15.0);
    let mut vertical_pos_controller = flight_core::pid::PidController::new(1.0, 0.0, 0.0, 0.0, 0.0);

    let ahrs_config = flight_core::fusion::AhrsConfig::default()
        .with_convention(Convention::Nwu)
        .with_gain(1.0)
        .with_accel_rejection(10.0) // reject accel vectors >10° from gravity (filters thrust contamination)
        .with_bias_config(None);
    let mut ahrs = flight_core::fusion::Ahrs::new(ahrs_config);
    let mut altitude_estimator =
        flight_core::altitude_estimator::AltitudeEstimator::new(0.0, 0.15, 0.001, 0.16);
    let mut accumulated = AccumulatedImu::ZERO;

    let mut count: usize = 0;

    // CSV log file
    let mut log_file = File::create("sensor_log.csv").expect("Failed to create sensor_log.csv");
    writeln!(log_file, "sim_time,dt,ax,ay,az,gx,gy,gz,roll_deg,pitch_deg,yaw_deg,rate_sp_x,rate_sp_y,rate_sp_z,m1,m2,m3,m4").unwrap();

    loop {
        let (frame, dt, sim_time, alt) = data_source.read().unwrap();

        accumulated.add_sample(frame, dt);

        let motors_setpoint = rate_controllers.step(rot_setpoint, frame.gyro_rad_s, dt);
        let output = mix(
            throttle,
            motors_setpoint.x,
            motors_setpoint.y,
            motors_setpoint.z,
        );

        data_source.write(&output).unwrap();

        if count.is_multiple_of(8) {
            ahrs.update_accumulated_imu(accumulated);
            altitude_estimator.predict(ahrs.linear_acceleration_z_world(), accumulated.dt);
            altitude_estimator.update_baro(alt);

            let rot = ahrs.euler_angles();

            rot_setpoint =
                attitude_controller.step(Vector3::new(0.0, 0.0, 0.0), rot, accumulated.dt);
            rot_setpoint.z = 0.0;

            vertical_vel_setpoint = vertical_pos_controller
                .step(altitude_estimator.altitude(), alt_setpoint, accumulated.dt)
                .clamp(-2.5, 2.5);
            throttle = (55.0
                + (vertical_vel_controller.step(
                    altitude_estimator.velocity(),
                    vertical_vel_setpoint,
                    accumulated.dt,
                )))
            .clamp(10.0, 90.0);

            if count.is_multiple_of(80) {
                println!(
                    "Attitude (deg) -> Roll: {:6.2}°, Pitch: {:6.2}°, Yaw: {:6.2}° | Rate SP: [{:.2}, {:.2}, {:.2}] rad/s | Altitude: {}",
                    rot.x.to_degrees(),
                    rot.y.to_degrees(),
                    rot.z.to_degrees(),
                    rot_setpoint.x,
                    rot_setpoint.y,
                    rot_setpoint.z,
                    alt
                );
            }

            accumulated = AccumulatedImu::ZERO;
        }

        count += 1;
    }
}
