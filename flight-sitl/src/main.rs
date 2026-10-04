use flight_core::fusion::Convention;
use flight_core::imu::AccumulatedImu;
use flight_sitl::{mix_elevons, FlightGearConnection};
use gamepads::{Button, Gamepads};
use std::fs::File;
use std::io::Write;

const LISTEN_FDM_ADDR: &str = "127.0.0.1:5500";
const SEND_CTRLS_ADDR: &str = "127.0.0.1:5501";

fn main() {
    let mut gamepads = Gamepads::new();
    let mut fg = FlightGearConnection::bind(LISTEN_FDM_ADDR, SEND_CTRLS_ADDR)
        .expect("Failed to bind FlightGear UDP sockets");

    // AHRS runs for telemetry display only - it does NOT drive the surfaces.
    let ahrs_config = flight_core::fusion::AhrsConfig::default()
        .with_convention(Convention::Nwu)
        .with_gain(1.0)
        .with_accel_rejection(10.0)
        .with_bias_config(None);
    let mut ahrs = flight_core::fusion::Ahrs::new(ahrs_config);
    let mut accumulated = AccumulatedImu::ZERO;

    let mut throttle: f32 = 0.0; // Start at idle on the runway.
    let mut rudder: f32 = 0.0;
    let mut elevator: f32 = 0.0;
    let mut aileron: f32 = 0.0;
    let mut count: usize = 0;

    let mut log_file = File::create("plane_sensor_log.csv").ok();
    if let Some(ref mut f) = log_file {
        let _ = writeln!(
            f,
            "sim_time,dt,roll_deg,pitch_deg,yaw_deg,left_elevon,right_elevon,rudder,throttle,alt_m"
        );
    }


    loop {
        gamepads.poll();

        let (frame, dt, sim_time, alt, fdm) = match fg.read() {
            Ok(data) => data,
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut || e.kind() == std::io::ErrorKind::WouldBlock => {
                continue;
            }
            Err(e) => {
                eprintln!("FlightGear read error: {e}");
                continue;
            }
        };

        accumulated.add_sample(frame, dt);

        if let Some(gamepad) = gamepads.all().next() {
            let (stick_x, stick_y) = gamepad.left_stick();
            aileron = stick_x.clamp(-1.0, 1.0);
            elevator = (-stick_y).clamp(-1.0, 1.0);

            let (rud_x, _) = gamepad.right_stick();
            rudder = rud_x.clamp(-1.0, 1.0);

            if gamepad.is_currently_pressed(Button::ActionDown) || gamepad.is_currently_pressed(Button::DPadUp) {
                throttle += 0.20 * dt;
            }
            if gamepad.is_currently_pressed(Button::ActionRight) || gamepad.is_currently_pressed(Button::DPadDown) {
                throttle -= 0.20 * dt;
            }
            throttle = throttle.clamp(0.0, 1.0);
        }

        let elevon_outputs = mix_elevons(elevator, aileron, 1.0);

        if let Err(e) = fg.write_controls(
            elevon_outputs.left_elevon as f64,
            elevon_outputs.right_elevon as f64,
            rudder as f64,
            throttle as f64,
        ) {
            eprintln!("Failed to write controls to FlightGear: {e}");
        }

        // AHRS + logging only, every 4 frames (~15 Hz).
        if count.is_multiple_of(4) {

            // skip initial empty frames
            if accumulated.delta_velocity.x == 0.0 {
                continue
            }

            ahrs.update_accumulated_imu(accumulated);

            if count.is_multiple_of(30) {
                let rot = ahrs.euler_angles();
                let airspeed_kts = fdm.vcas * 0.592484;
                println!(
                    "t={:5.1}s | Roll: {:6.1}° | Pitch: {:6.1}° | Yaw: {:6.1}° | Stick: [{:+.2}, {:+.2}] | Thr: {:2.0}% | Alt: {:5.0}ft | IAS: {:4.1}kts",
                    sim_time,
                    rot.x.to_degrees(),
                    rot.y.to_degrees(),
                    rot.z.to_degrees(),
                    aileron,
                    elevator,
                    throttle * 100.0,
                    alt * 3.28084,
                    airspeed_kts,
                );
            }

            if let Some(ref mut f) = log_file {
                let rot = ahrs.euler_angles();
                let _ = writeln!(
                    f,
                    "{:.3},{:.4},{:.2},{:.2},{:.2},{:.3},{:.3},{:.3},{:.2},{:.2}",
                    sim_time,
                    dt,
                    rot.x.to_degrees(),
                    rot.y.to_degrees(),
                    rot.z.to_degrees(),
                    elevon_outputs.left_elevon,
                    elevon_outputs.right_elevon,
                    rudder,
                    throttle,
                    alt
                );
            }

            accumulated = AccumulatedImu::ZERO;
        }

        count += 1;
    }
}
