#![no_std]
pub mod altitude_estimator;
pub mod fusion;
pub mod imu;
pub mod pid;
pub mod comms;

/// Standard acceleration due to gravity in m/s² (ISO 80000-3)
pub const GRAVITY_MSS: f32 = 9.80665;

use crate::pid::PidController;
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct DroneState {
    pub attitude: Vector3<f32>,
    pub altitude: f32,
    pub velocity: f32,
    pub accel_bias: f32,
    pub accel_z: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct MotorOutputs {
    pub front_right: f32,
    pub front_left: f32,
    pub back_left: f32,
    pub back_right: f32,
}

pub struct ControllerInput {
    pub throttle: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
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

/// Mixes throttle and PID roll/pitch/yaw into 4 normalized motor outputs [min_output, max_output]
pub fn mix(throttle: f32, roll: f32, pitch: f32, yaw: f32) -> MotorOutputs {
    let min_output = 0.0f32;
    let max_output = 100.0f32;

    if throttle <= min_output {
        return MotorOutputs {
            front_right: min_output,
            front_left: min_output,
            back_left: min_output,
            back_right: min_output,
        };
    }

    // 1. Raw matrix mix (Quad-X configuration)
    let mut m1 = throttle - pitch - roll - yaw; // Front Right
    let mut m2 = throttle - pitch + roll + yaw; // Front Left
    let mut m3 = throttle + pitch + roll - yaw; // Back Left
    let mut m4 = throttle + pitch - roll + yaw; // Back Right

    // 2. Find maximum and minimum motor commands in current mix
    let max_m = m1.max(m2).max(m3).max(m4);
    let min_m = m1.min(m2).min(m3).min(m4);

    // 3. Range Reduction
    if max_m > max_output {
        let overflow = max_m - max_output;
        m1 -= overflow;
        m2 -= overflow;
        m3 -= overflow;
        m4 -= overflow;
    } else if min_m < min_output {
        let underflow = min_output - min_m;
        m1 += underflow;
        m2 += underflow;
        m3 += underflow;
        m4 += underflow;
    }

    // 4. Hard clamp as safety backstop
    MotorOutputs {
        front_right: m1.clamp(min_output, max_output),
        front_left: m2.clamp(min_output, max_output),
        back_left: m3.clamp(min_output, max_output),
        back_right: m4.clamp(min_output, max_output),
    }
}
