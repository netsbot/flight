use serde::{Deserialize, Serialize};
use thiserror::Error;

#[cfg_attr(feature = "generate-bindings", derive(ts_rs::TS))]
#[cfg_attr(feature = "generate-bindings", ts(export, export_to = "../../ground-station/src/lib/types/"))]
#[derive(Serialize, Deserialize, Debug, Copy, Clone, defmt::Format)]
pub enum Command {
    Ping,
    Pong {
        version: u8,
        drone_linked: bool,
        rssi: Option<i8>,
    },
    Altitude(f32),
    Attitude([f32; 3]),
    Time(u32),
}

#[derive(Error, Debug)]
pub enum CommsError {
    #[error("client is not connected to any peer")]
    NotConnected,
    #[error("failed to serialize/deserialize payload: {0}")]
    Serialization(#[from] postcard::Error),
    #[error("transport I/O error")]
    Io,
}

pub trait Sender {
    fn send(&mut self, command: Command) -> impl Future<Output = Result<(), CommsError>>;
}

pub trait Receiver {
    fn receive(&mut self) -> impl Future<Output = Result<Command, CommsError>>;
}
