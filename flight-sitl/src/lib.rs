use flight_core::MotorOutputs;
use flight_core::imu::ImuFrame;
use nalgebra::Vector3;
use std::io;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream, ToSocketAddrs};

pub mod elevons;
pub mod fg_connection;
pub mod fg_net_ctrls;
pub mod fg_net_fdm;

pub use elevons::{mix_elevons, ElevonOutputs};
pub use fg_connection::FlightGearConnection;
pub use fg_net_ctrls::{FGNetCtrls, FG_NET_CTRLS_SIZE, FG_NET_CTRLS_VERSION};
pub use fg_net_fdm::{FGNetFDM, FG_NET_FDM_SIZE, FG_NET_FDM_VERSION};