#![no_std]

mod altitude_estimator;
pub mod fusion;
pub mod pid;

use crate::fusion::Convention::Enu;
use crate::fusion::{Ahrs, AhrsConfig};
use crate::pid::{PidController, PidDebug};
use nalgebra::Vector3;

#[derive(Debug, Clone, Copy)]
pub struct MotorOutputs {
    pub front_right: f32,
    pub front_left: f32,
    pub back_left: f32,
    pub back_right: f32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct FlightComputerDebug {
    pub roll: PidDebug,
    pub pitch: PidDebug,
    pub yaw: PidDebug,
}

pub struct ControllerInput {
    pub throttle: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

pub struct Sensors {
    pub accel: Vector3<f32>,
    pub gyro: Vector3<f32>,
    pub magnetometer: Option<Vector3<f32>>,
    pub alt: f32,
    pub dt: f32, // in seconds
}

pub struct FlightComputer {
    ahrs: Ahrs,
    pid_angular_vel: (PidController, PidController, PidController), // roll, pitch, yaw
    pid_altitude: PidController,
}

impl FlightComputer {
    pub fn new() -> Self {
        Self {
            ahrs: Ahrs::new(
                AhrsConfig::default()
                    .with_convention(Enu)
                    .with_gain(0.12) // Fast enough to lock gravity, slow enough to reject linear acceleration
                    .with_accel_rejection(45.0),
            ),

            pid_angular_vel: (
                // PidController::new(Kp, Ki, Kd, d_cutoff_hz, i_limit)
                PidController::new(2.5, 0.05, 0.18, 60.0, 5.0), // Roll rate PID
                PidController::new(2.5, 0.05, 0.18, 60.0, 5.0), // Pitch rate PID
                PidController::new(2.0, 0.02, 0.00, 60.0, 5.0),
            ),

            pid_altitude: PidController::new(1.2, 0.1, 0.8, 10.0, 100.0),
        }
    }

    pub fn update(
        &mut self,
        sensors: Sensors,
        input: ControllerInput,
    ) -> (MotorOutputs, FlightComputerDebug) {
        self.ahrs.update(&sensors);
        let roll_rate = sensors.gyro.y;
        let pitch_rate = -sensors.gyro.x; // Negate so pitching nose down gives negative error to pitch back up
        let yaw_rate = sensors.gyro.z;

        let (out_r, debug_r) = self
            .pid_angular_vel
            .0
            .update(roll_rate, input.roll, sensors.dt);

        let (out_p, debug_p) = self
            .pid_angular_vel
            .1
            .update(pitch_rate, input.pitch, sensors.dt);
        let (out_y, debug_y) = self
            .pid_angular_vel
            .2
            .update(yaw_rate, input.yaw, sensors.dt);

        let outputs = mix(input.throttle, 0.0, 0.0, 0.0);

        let debug = FlightComputerDebug {
            roll: debug_r,
            pitch: debug_p,
            yaw: debug_y,
        };

        (outputs, debug)
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
