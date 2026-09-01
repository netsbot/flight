use core::f32::consts::TAU;

#[derive(Debug, Clone, Copy, Default)]
pub struct PidDebug {
    pub error: f32,
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub output: f32,
}

pub struct PidController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub d_cutoff_hz: f32,
    pub i_limit: f32, // Maximum value for integral accumulator (anti-windup)

    integral: f32,
    prev_measurement: f32,
    d_filter_state: f32,
    initialized: bool,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32, d_cutoff_hz: f32, i_limit: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            d_cutoff_hz,
            i_limit,
            integral: 0.0,
            prev_measurement: 0.0,
            d_filter_state: 0.0,
            initialized: false,
        }
    }

    /// Reset internal state buffers (e.g., when enabling/disabling the controller)
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_measurement = 0.0;
        self.d_filter_state = 0.0;
        self.initialized = false;
    }

    pub fn step(&mut self, measurement: f32, setpoint: f32, dt: f32) -> f32 {
        if dt <= 0.0 {
            return 0.0;
        }

        // Initialize state on first frame to prevent derivative spike on startup
        if !self.initialized {
            self.prev_measurement = measurement;
            self.initialized = true;
        }

        // 1. Proportional term
        let error = setpoint - measurement;
        let p_out = self.kp * error;

        // 2. Integral term with explicit anti-windup clamp
        self.integral += error * dt;
        if self.i_limit > 0.0 {
            self.integral = self.integral.clamp(-self.i_limit, self.i_limit);
        }
        let i_out = self.ki * self.integral;

        // 3. Derivative term (Derivative on Measurement)
        let raw_d = -(measurement - self.prev_measurement) / dt;
        self.prev_measurement = measurement;

        // Low-pass filter (PT1) on derivative step
        // alpha = dt / (dt + RC) where RC = 1 / (2 * pi * cutoff_freq)
        let alpha = if self.d_cutoff_hz > 0.0 {
            let rc = 1.0 / (TAU * self.d_cutoff_hz);
            dt / (dt + rc)
        } else {
            1.0 // Disable filter (pass raw_d directly)
        };

        self.d_filter_state += alpha * (raw_d - self.d_filter_state);
        let d_out = self.kd * self.d_filter_state;

        let output = p_out + i_out + d_out;

        output
    }
}
