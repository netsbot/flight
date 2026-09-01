use flight_core::MotorOutputs;
use flight_core::imu::ImuFrame;
use nalgebra::Vector3;
use std::io;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream, ToSocketAddrs};

pub struct WebotsConnection {
    tpc_stream: TcpStream,
    last_time: Option<f32>,
    warmed_up: bool,
}

impl WebotsConnection {
    pub fn bind<A: ToSocketAddrs>(addr: A) -> Self {
        let tcp_listener = TcpListener::bind(addr).unwrap();
        let (tpc_stream, _) = tcp_listener.accept().unwrap();

        Self {
            tpc_stream,
            last_time: None,
            warmed_up: false,
        }
    }

    pub fn read(&mut self) -> Result<(ImuFrame, f32, f32, f32), io::Error> {
        let mut buffer = [0u8; 32];

        self.tpc_stream.read_exact(&mut buffer)?;

        let current_time = f32::from_le_bytes(buffer[0..4].try_into().unwrap());
        let dt = match self.last_time {
            Some(last) => {
                let raw_dt = current_time - last;
                if raw_dt <= 0.0 { 0.001 } else { raw_dt }
            }
            None => 0.001,
        };
        self.last_time = Some(current_time);

        // Webots sends garbage accel/gyro on the very first timestep before
        // physics has initialised. Discard it and return a clean frame so
        // AHRS startup quaternion is not corrupted.
        if !self.warmed_up {
            self.warmed_up = true;
            return Ok((
                ImuFrame {
                    accel_ms2: Vector3::new(0.0, 0.0, 9.81),
                    gyro_rad_s: Vector3::zeros(),
                    magnetometer: None,
                },
                dt,
                current_time,
                0.0,
            ));
        }

        // Unpack IMU vectors
        let accel = Vector3::new(
            f32::from_le_bytes(buffer[4..8].try_into().unwrap()),
            f32::from_le_bytes(buffer[8..12].try_into().unwrap()),
            f32::from_le_bytes(buffer[12..16].try_into().unwrap()),
        );

        let gyro = Vector3::new(
            f32::from_le_bytes(buffer[16..20].try_into().unwrap()),
            f32::from_le_bytes(buffer[20..24].try_into().unwrap()),
            f32::from_le_bytes(buffer[24..28].try_into().unwrap()),
        );

        let alt = f32::from_le_bytes(buffer[28..32].try_into().unwrap());

        Ok((
            ImuFrame {
                accel_ms2: accel,
                gyro_rad_s: gyro,
                magnetometer: None,
            },
            dt,
            current_time,
            alt,
        ))
    }

    pub fn write(&mut self, outputs: &MotorOutputs) -> Result<(), io::Error> {
        let mut out_buffer = [0u8; 16];
        out_buffer[0..4].copy_from_slice(&outputs.front_right.to_le_bytes()); // Slot 0: FR
        out_buffer[4..8].copy_from_slice(&outputs.back_left.to_le_bytes()); // Slot 1: BL
        out_buffer[8..12].copy_from_slice(&outputs.front_left.to_le_bytes()); // Slot 2: FL
        out_buffer[12..16].copy_from_slice(&outputs.back_right.to_le_bytes()); // Slot 3: BR

        if self.tpc_stream.write_all(&out_buffer).is_err() {
            panic!("Failed to send output frame to Webots.");
        };

        Ok(())
    }
}
