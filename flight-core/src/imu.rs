use core::ops::{Add, AddAssign};
use defmt::Format;
use nalgebra::Vector3;

#[derive(Format, Clone, Copy, Debug)]
pub struct ImuFrame {
    pub accel_ms2: Vector3<f32>,
    pub gyro_rad_s: Vector3<f32>,
    pub magnetometer: Option<Vector3<f32>>,
}

#[derive(Format, Clone, Copy, Debug)]
pub struct AccumulatedImu {
    pub delta_angle: Vector3<f32>,
    pub delta_velocity: Vector3<f32>,
    pub dt: f32,
    pub samples: u32,
}

impl Default for AccumulatedImu {
    fn default() -> Self {
        Self {
            delta_angle: Vector3::zeros(),
            delta_velocity: Vector3::zeros(),
            dt: 0.0,
            samples: 0,
        }
    }
}

impl AccumulatedImu {
    pub const ZERO: Self = Self {
        delta_angle: Vector3::new(0.0, 0.0, 0.0),
        delta_velocity: Vector3::new(0.0, 0.0, 0.0),
        dt: 0.0,
        samples: 0,
    };

    pub fn add_sample(&mut self, frame: ImuFrame, dt: f32) {
        self.delta_angle += frame.gyro_rad_s * dt;
        self.delta_velocity += frame.accel_ms2 * dt;
        self.dt += dt;
        self.samples += 1;
    }
}
