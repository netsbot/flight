#![cfg_attr(not(feature = "generate-bindings"), no_std)]
pub mod altitude_estimator;
pub mod comms;
pub mod fusion;
pub mod imu;
pub mod pid;

/// Standard acceleration due to gravity in m/s² (ISO 80000-3)
pub const GRAVITY_MSS: f32 = 9.80665;

use crate::pid::PidController;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct PlaneState {
    pub attitude: Vector3<f32>,
    pub altitude: f32,
}

#[derive(Debug, Copy, Clone)]
pub enum Setpoint {
    Rates(Vector3<f32>), // deg/s
    Attitude(Vector3<f32>), // deg
}

#[derive(Debug, Copy, Clone)]
pub struct ElevonOutputs {
    pub left_elevon: f32,  // Normalized command [-1.0, 1.0]
    pub right_elevon: f32, // Normalized command [-1.0, 1.0]
}

pub struct ControllerBundle {
    pid_rate: (PidController, PidController, PidController), // roll, pitch, yaw
}

impl Default for ControllerBundle {
    fn default() -> Self {
        Self {
            pid_rate: (
                PidController::new(2.5, 0.05, 0.18, 60.0, 5.0), // Roll rate PID
                PidController::new(2.5, 0.05, 0.18, 60.0, 5.0), // Pitch rate PID
                PidController::new(2.0, 0.02, 0.00, 60.0, 5.0), // Yaw rate PID
            ),
        }
    }
}

impl ControllerBundle {
    pub fn new() -> Self {
        Self::new_rate()
    }

    pub fn new_rate() -> Self {
        Self {
            pid_rate: (
                PidController::new(2.5, 0.05, 0.18, 60.0, 5.0), // Roll rate PID
                PidController::new(2.5, 0.05, 0.18, 60.0, 5.0), // Pitch rate PID
                PidController::new(2.0, 0.02, 0.00, 60.0, 5.0), // Yaw rate PID
            ),
        }
    }

    pub fn new_attitude() -> Self {
        Self {
            pid_rate: (
                PidController::new(4.5, 0.0, 0.0, 0.0, 0.0), // Roll angle P controller
                PidController::new(4.5, 0.0, 0.0, 0.0, 0.0), // Pitch angle P controller
                PidController::new(2.5, 0.0, 0.0, 0.0, 0.0), // Yaw angle P controller
            ),
        }
    }

    pub fn with_pids(roll: PidController, pitch: PidController, yaw: PidController) -> Self {
        Self {
            pid_rate: (roll, pitch, yaw),
        }
    }

    pub fn step(&mut self, setpoint: Vector3<f32>, current: Vector3<f32>, dt: f32) -> Vector3<f32> {
        let roll_cmd = self.pid_rate.0.step(current.x, setpoint.x, dt);
        let pitch_cmd = self.pid_rate.1.step(current.y, setpoint.y, dt);
        let yaw_cmd = self.pid_rate.2.step(current.z, setpoint.z, dt);

        Vector3::new(roll_cmd, pitch_cmd, yaw_cmd)
    }
}

pub fn mix_elevons(pitch: f32, roll: f32, differential_ratio: f32) -> ElevonOutputs {
    let (left_roll, right_roll) = if roll >= 0.0 {
        (roll * differential_ratio, roll)
    } else {
        (roll, roll * differential_ratio)
    };

    ElevonOutputs {
        left_elevon: (-pitch - left_roll).clamp(-1.0, 1.0),
        right_elevon: (-pitch + right_roll).clamp(-1.0, 1.0),
    }
}
