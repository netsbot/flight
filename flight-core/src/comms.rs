use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Serialize, Deserialize, Debug, Copy, Clone, defmt::Format)]
pub enum Command {
    Altitude(f32),
    Attitude(Vector3<f32>),
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
