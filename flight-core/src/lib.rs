#![no_std]

mod altitude_estimator;
pub mod fusion;
pub mod imu;
pub mod pid;

use crate::pid::PidController;
use nalgebra::Vector3;

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

pub struct RateController {
    pid_rate: (PidController, PidController, PidController), // roll, pitch, yaw
}

impl Default for RateController {
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

impl RateController {
    pub fn new() -> Self {
        Self {
            pid_rate: (
                PidController::new(2.5, 0.05, 0.18, 60.0, 5.0), // Roll rate PID
                PidController::new(2.5, 0.05, 0.18, 60.0, 5.0), // Pitch rate PID
                PidController::new(2.0, 0.02, 0.00, 60.0, 5.0), // Yaw rate PID
            ),
        }
    }

    /// Runs at 4 kHz / 8 kHz on Core 1
    pub fn step(
        &mut self,
        target_rate: Vector3<f32>,
        gyro: Vector3<f32>,
        throttle: f32,
        dt: f32,
    ) -> MotorOutputs {
        let roll_cmd = self.pid_rate.0.update(gyro.x, target_rate.x, dt);
        let pitch_cmd = self.pid_rate.1.update(gyro.y, target_rate.y, dt);
        let yaw_cmd = self.pid_rate.2.update(gyro.z, target_rate.z, dt);

        mix(throttle, roll_cmd, pitch_cmd, yaw_cmd)
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
