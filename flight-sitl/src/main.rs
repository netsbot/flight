use gamepads::{Button, Gamepads};
use nalgebra::Vector3;
use std::io::{Read, Write};
use std::net::TcpListener;

fn main() {
    let mut gamepads = Gamepads::new();

    let listener = TcpListener::bind("127.0.0.1:5599").expect("Failed to bind TCP port");

    let (mut stream, addr) = listener.accept().expect("Failed to accept connection");
    println!("Connected to Webots controller at {}", addr);

    let mut buffer = [0u8; 32];
    let mut computer = flight_core::RateController::new();
    let mut last_time = 0.0f32;

    let mut throttle = 60.0f32;
    let mut input = (0.0f32, 0.0f32);
    let mut frame_count: u64 = 0;

    loop {
        // Refresh gamepad events
        gamepads.poll();

        // Read 32-byte sensor frame from Webots
        if stream.read_exact(&mut buffer).is_err() {
            println!("Webots disconnected or simulation stopped.");
            break;
        }

        // Unpack timestamp
        let current_time = f32::from_le_bytes(buffer[0..4].try_into().unwrap());
        let raw_dt = current_time - last_time;
        let dt = if raw_dt <= 0.0 { 0.001 } else { raw_dt };
        last_time = current_time;

        // Unpack IMU vectors
        let _accel = Vector3::new(
            f32::from_le_bytes(buffer[4..8].try_into().unwrap()),
            f32::from_le_bytes(buffer[8..12].try_into().unwrap()),
            f32::from_le_bytes(buffer[12..16].try_into().unwrap()),
        );

        let gyro = Vector3::new(
            f32::from_le_bytes(buffer[16..20].try_into().unwrap()),
            f32::from_le_bytes(buffer[20..24].try_into().unwrap()),
            f32::from_le_bytes(buffer[24..28].try_into().unwrap()),
        );

        let _alt = f32::from_le_bytes(buffer[28..32].try_into().unwrap());

        // Process inputs from active gamepad
        if let Some(gamepad) = gamepads.all().next() {
            let (raw_x, raw_y) = gamepad.left_stick();
            input = (raw_x * 2.0, raw_y * 2.0); // Map to [-2.0, 2.0] rad/s max

            if gamepad.is_currently_pressed(Button::ActionDown) {
                throttle += 10.0 * dt;
            }
            if gamepad.is_currently_pressed(Button::ActionRight) {
                throttle -= 10.0 * dt;
            }

            throttle = throttle.clamp(0.0, 100.0);
        }

        // Update flight computer rate controller
        let outputs = computer.step(
            Vector3::new(input.0, input.1, 0.0),
            gyro,
            throttle,
            dt,
        );

        // Print telemetry every 20 frames (~100ms) to avoid stdout lag
        frame_count += 1;
        if frame_count.is_multiple_of(20) {
            println!(
                "MOTORS| FR: {:5.2} | BL: {:5.2} | FL: {:5.2} | BR: {:5.2}\n",
                outputs.front_right, outputs.back_left, outputs.front_left, outputs.back_right
            );
        }

        // Write motor outputs using Webots slot mapping [FR, BL, FL, BR]
        let mut out_buffer = [0u8; 16];
        out_buffer[0..4].copy_from_slice(&outputs.front_right.to_le_bytes()); // Slot 0: FR
        out_buffer[4..8].copy_from_slice(&outputs.back_left.to_le_bytes());   // Slot 1: BL
        out_buffer[8..12].copy_from_slice(&outputs.front_left.to_le_bytes());  // Slot 2: FL
        out_buffer[12..16].copy_from_slice(&outputs.back_right.to_le_bytes()); // Slot 3: BR

        if stream.write_all(&out_buffer).is_err() {
            println!("Failed to send output frame to Webots.");
            break;
        }
    }
}