//! FlightGear network bridge: receives FDM telemetry (v24) on UDP port 5500
//! and transmits actuator control inputs (v27) on UDP port 5501.

use crate::fg_net_ctrls::{FGNetCtrls, FG_NET_CTRLS_SIZE};
use crate::fg_net_fdm::{FGNetFDM, FG_NET_FDM_SIZE};
use flight_core::imu::ImuFrame;
use nalgebra::Vector3;
use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::time::Duration;

pub struct FlightGearConnection {
    rx_socket: UdpSocket,
    tx_socket: UdpSocket,
    target_addr: SocketAddr,
    last_time: Option<f32>,
    warmed_up: bool,
}

impl FlightGearConnection {
    /// Bind the FlightGear UDP bridge.
    /// - `listen_addr`: UDP address where FlightGear sends FDM packets (e.g. "127.0.0.1:5500")
    /// - `send_addr`: UDP address where FlightGear listens for Ctrls packets (e.g. "127.0.0.1:5501")
    pub fn bind(listen_addr: &str, send_addr: &str) -> io::Result<Self> {
        let rx_socket = UdpSocket::bind(listen_addr)?;
        rx_socket.set_read_timeout(Some(Duration::from_millis(500)))?;

        let tx_socket = UdpSocket::bind("0.0.0.0:0")?;
        let target_addr: SocketAddr = send_addr
            .parse()
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;

        println!("✈️ FlightGear bridge bound: listening on {listen_addr}, sending to {send_addr}");

        Ok(Self {
            rx_socket,
            tx_socket,
            target_addr,
            last_time: None,
            warmed_up: false,
        })
    }

    /// Read next telemetry frame from FlightGear.
    /// Returns (ImuFrame, dt, current_sim_time, altitude_m, raw_fdm).
    pub fn read(&mut self) -> io::Result<(ImuFrame, f32, f32, f32, FGNetFDM)> {
        let mut buf = [0u8; 1024];
        let (len, _src) = self.rx_socket.recv_from(&mut buf)?;

        let fdm = FGNetFDM::from_be_bytes(&buf[..len])
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;

        // cur_time in unix epoch / sim time
        let current_time = fdm.cur_time as f32;
        let dt = match self.last_time {
            Some(last) => {
                let raw_dt = current_time - last;
                if raw_dt <= 0.0 || raw_dt > 0.1 {
                    1.0 / 60.0 // Default 60 Hz rate if timestamps jump
                } else {
                    raw_dt
                }
            }
            None => 1.0 / 60.0,
        };
        self.last_time = Some(current_time);

        // Convert ft/s^2 to m/s^2
        const FT_TO_M: f32 = 0.3048;
        let accel = Vector3::new(
            fdm.a_x_pilot * FT_TO_M,
            fdm.a_y_pilot * FT_TO_M,
            fdm.a_z_pilot * FT_TO_M,
        );

        // Gyro rates in rad/s
        let gyro = Vector3::new(fdm.phidot, fdm.thetadot, fdm.psidot);

        let imu = ImuFrame {
            accel_ms2: accel,
            gyro_rad_s: gyro,
            magnetometer: None,
        };

        let alt_m = fdm.altitude as f32;

        Ok((imu, dt, current_time, alt_m, fdm))
    }

    /// Send control surface commands to FlightGear.
    /// - `elevator`: pitch command [-1.0, 1.0]
    /// - `aileron`: roll command [-1.0, 1.0]
    /// - `rudder`: yaw command [-1.0, 1.0]
    /// - `throttle`: normalized thrust [0.0, 1.0]
    pub fn write_controls(
        &mut self,
        elevator: f64,
        aileron: f64,
        rudder: f64,
        throttle: f64,
    ) -> io::Result<()> {
        let ctrls = FGNetCtrls::new(elevator, aileron, rudder, throttle);
        let bytes = ctrls.to_be_bytes();
        self.tx_socket.send_to(&bytes, self.target_addr)?;
        Ok(())
    }
}
