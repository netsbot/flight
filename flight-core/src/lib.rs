#![no_std]

pub mod fusion;

use nalgebra::Vector3;

pub struct MotorOutputs {
    front_left: u8,
    front_right: u8,
    back_left: u8,
    back_right: u8,
}

pub struct Sensors {
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
    pub magnetometer: Option<Vector3<f32>>,
    pub alt: f32,
    pub dt: f32, // in seconds
}

pub struct FlightComputer {}

impl FlightComputer {
    pub fn new() -> Self {
        Self {}
    }

    pub fn update(&self, sensors: Sensors) -> MotorOutputs {
        MotorOutputs {
            front_left: 0,
            front_right: 0,
            back_left: 0,
            back_right: 0,
        }
    }
}