use crate::imu::{AccumulatedImu, ImuFrame};
use core::f32::consts::PI;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use num_traits::Float;

const STARTUP_GAIN: f32 = 10.0;
const STARTUP_PERIOD: f32 = 3.0;
const CUTOFF_FREQ: f32 = 0.02;

pub struct Ahrs {
    config: AhrsConfig,

    quaternion: UnitQuaternion<f32>,
    accel: Vector3<f32>,
    half_gravity: Vector3<f32>,

    startup: bool,
    ramped_gain: f32,
    ramped_gain_step: f32,

    angular_rate_recovery: bool,

    half_accel_feedback: Vector3<f32>,
    half_magnetometer_feedback: Vector3<f32>,
    accel_ignored: bool,
    accel_recovery_trigger: i32,
    accel_recovery_threshold: i32,
    magnetometer_ignored: bool,
    magnetometer_recovery_trigger: i32,
    magnetometer_recovery_threshold: i32,

    // Bias correction fields
    bias_timeout: Option<u32>,
    bias_timer: u32,
    bias_offset: Vector3<f32>,
}

impl Ahrs {
    pub fn new(config: AhrsConfig) -> Self {
        let bias_timeout = config
            .bias_config
            .as_ref()
            .map(|bias_cfg| (bias_cfg.stationary_period * bias_cfg.sample_rate) as u32);

        Self {
            quaternion: UnitQuaternion::identity(),
            accel: Vector3::zeros(),
            half_gravity: Vector3::zeros(),
            startup: true,
            ramped_gain: STARTUP_GAIN,
            ramped_gain_step: (STARTUP_GAIN - config.gain()) / STARTUP_PERIOD,
            angular_rate_recovery: false,
            half_accel_feedback: Vector3::zeros(),
            half_magnetometer_feedback: Vector3::zeros(),
            accel_ignored: false,
            accel_recovery_threshold: config.rejection_timeout(),
            accel_recovery_trigger: 0,
            magnetometer_ignored: false,
            magnetometer_recovery_threshold: config.rejection_timeout(),
            magnetometer_recovery_trigger: 0,
            bias_timeout,
            bias_timer: 0,
            bias_offset: Vector3::zeros(),
            config,
        }
    }

    pub fn restart(&mut self) {
        self.quaternion = UnitQuaternion::identity();
        self.accel = Vector3::zeros();
        self.half_gravity = Vector3::zeros();
        self.startup = true;
        self.ramped_gain = STARTUP_GAIN;
        self.angular_rate_recovery = false;
        self.half_accel_feedback = Vector3::zeros();
        self.half_magnetometer_feedback = Vector3::zeros();
        self.accel_ignored = false;
        self.accel_recovery_trigger = 0;
        self.magnetometer_ignored = false;
        self.magnetometer_recovery_trigger = 0;
        // Reset bias timer but keep offset
        self.bias_timer = 0;
    }

    pub fn update(&mut self, imu_frame: ImuFrame, dt: f32) {
        self.config.set_sample_period(dt);

        // Apply bias correction if enabled (gyro in deg/s)
        let corrected_gyro = if let Some(bias_cfg) = &self.config.bias_config {
            let gyro_corrected = imu_frame.gyro_rad_s - self.bias_offset;

            let thresh = bias_cfg.stationary_threshold.to_radians();
            if gyro_corrected.x.abs() > thresh
                || gyro_corrected.y.abs() > thresh
                || gyro_corrected.z.abs() > thresh
            {
                self.bias_timer = 0;
            } else if self.bias_timer < self.bias_timeout.unwrap_or(0) {
                self.bias_timer += 1;
            } else {
                let coeff = 2.0 * PI * CUTOFF_FREQ * dt;
                self.bias_offset = self.bias_offset + gyro_corrected.scale(coeff);
            }

            gyro_corrected
        } else {
            imu_frame.gyro_rad_s
        };

        self.accel = imu_frame.accel_ms2;

        if corrected_gyro.x.abs() > self.config.gyro_range()
            || corrected_gyro.y.abs() > self.config.gyro_range()
            || corrected_gyro.z.abs() > self.config.gyro_range()
        {
            let quaternion = self.quaternion;
            self.restart();
            self.quaternion = quaternion;
            self.angular_rate_recovery = true;
        }

        if self.startup {
            self.ramped_gain -= self.ramped_gain_step * self.config.sample_period();
            if self.ramped_gain < self.config.gain() || self.config.gain() == 0.0 {
                self.ramped_gain = self.config.gain();
                self.startup = false;
                self.angular_rate_recovery = false;
            }
        }

        self.half_gravity = self.config.convention().half_gravity(self.quaternion);

        let mut half_accel_feedback = Vector3::zeros();
        self.accel_ignored = true;

        // Reject accel when magnitude deviates >10% from 1 g (GRAVITY_MSS).
        // Thrust adds directly to az so the vector direction stays near-vertical
        // (angle rejection misses it) while the magnitude spikes well above 1g.
        const ACCEL_MAGNITUDE_TOLERANCE: f32 = 0.1; // 10%
        let accel_magnitude = imu_frame.accel_ms2.norm();
        let accel_magnitude_ok =
            (accel_magnitude - crate::GRAVITY_MSS).abs() <= crate::GRAVITY_MSS * ACCEL_MAGNITUDE_TOLERANCE;

        if accel_magnitude_ok && imu_frame.accel_ms2 != Vector3::zeros() {
            self.half_accel_feedback =
                Self::feedback(imu_frame.accel_ms2.normalize(), self.half_gravity);

            if self.startup
                || self.half_accel_feedback.norm_squared() <= self.config.accel_rejection()
            {
                self.accel_ignored = false;
                self.accel_recovery_trigger -= 9;
            } else {
                self.accel_recovery_trigger += 1;
            }

            if self.accel_recovery_trigger > self.accel_recovery_threshold {
                self.accel_recovery_threshold = 0;
                self.accel_ignored = false;
            } else {
                self.accel_recovery_threshold = self.config.rejection_timeout();
            }

            self.accel_recovery_trigger = self
                .accel_recovery_trigger
                .clamp(0, self.config.rejection_timeout());

            if !self.accel_ignored {
                half_accel_feedback = self.half_accel_feedback;
            }
        }

        let mut half_magnetometer_feedback = Vector3::zeros();
        self.magnetometer_ignored = true;

        if let Some(magnetometer) = imu_frame.magnetometer {
            let half_magnetic = self.config.convention().half_magnetic(self.quaternion);

            self.half_magnetometer_feedback = Self::feedback(
                self.half_gravity.cross(&magnetometer).normalize(),
                half_magnetic,
            );

            if self.startup
                || self.half_magnetometer_feedback.norm_squared()
                    <= self.config.magnetic_rejection()
            {
                self.magnetometer_ignored = false;
                self.magnetometer_recovery_trigger -= 9;
            } else {
                self.magnetometer_recovery_trigger += 1;
            }

            if self.magnetometer_recovery_trigger > self.magnetometer_recovery_threshold {
                self.magnetometer_recovery_threshold = 0;
                self.magnetometer_ignored = false;
            } else {
                self.magnetometer_recovery_threshold = self.config.rejection_timeout();
            }

            self.magnetometer_recovery_trigger = self
                .magnetometer_recovery_trigger
                .clamp(0, self.config.rejection_timeout());

            if self.magnetometer_ignored == false {
                half_magnetometer_feedback = self.half_magnetometer_feedback;
            }
        }

        let half_gyro = corrected_gyro.scale(0.5);
        let adjusted_half_gyro =
            half_gyro + (half_accel_feedback + half_magnetometer_feedback).scale(self.ramped_gain);

        self.quaternion = UnitQuaternion::new_normalize(
            self.quaternion.into_inner()
                + (self.quaternion.into_inner()
                    * Quaternion::from_parts(
                        0.0,
                        adjusted_half_gyro * self.config.sample_period(),
                    )),
        );

        if imu_frame.magnetometer.is_none() && self.startup {
            self.set_heading(0.0);
        }
    }

    pub fn update_accumulated_imu(&mut self, accum: AccumulatedImu) {
        let avg_gyro = accum.delta_angle / accum.dt;
        let avg_accel = accum.delta_velocity / accum.dt;

        let frame = ImuFrame {
            accel_ms2: avg_accel,
            gyro_rad_s: avg_gyro,
            magnetometer: None,
        };

        self.update(frame, accum.dt);
    }

    pub fn set_heading(&mut self, heading: f32) {
        let yaw = (self.quaternion.w * self.quaternion.k + self.quaternion.i * self.quaternion.j)
            .atan2(
                0.5 - self.quaternion.j * self.quaternion.j - self.quaternion.k * self.quaternion.k,
            );

        let half_yaw_minus_heading: f32 = 0.5 * (yaw - heading.to_radians());

        let rotation = UnitQuaternion::new_unchecked(Quaternion::new(
            half_yaw_minus_heading.cos(),
            0.0,
            0.0,
            -half_yaw_minus_heading.sin(),
        ));

        // UnitQuaternion * UnitQuaternion works out-of-the-box
        self.quaternion = rotation * self.quaternion;
    }

    pub fn quaternion(&self) -> UnitQuaternion<f32> {
        self.quaternion
    }

    pub fn euler_angles(&self) -> Vector3<f32> {
        let q = self.quaternion;
        let roll = (q.j * q.k + q.w * q.i).atan2(q.w * q.w + q.k * q.k - 0.5);
        let pitch = (2.0 * (q.w * q.j - q.i * q.k)).asin();
        let yaw = (q.i * q.j + q.w * q.k).atan2(q.w * q.w + q.i * q.i - 0.5);
        Vector3::new(roll, pitch, yaw)
    }

    pub fn is_startup(&self) -> bool {
        self.startup
    }

    pub fn is_angular_rate_recovery(&self) -> bool {
        self.angular_rate_recovery
    }

    pub fn is_accel_ignored(&self) -> bool {
        self.accel_ignored
    }

    pub fn is_magnetometer_ignored(&self) -> bool {
        self.magnetometer_ignored
    }

    /// Computes the linear upward acceleration in world frame in m/s² (with gravity removed).
    /// Assumes self.accel is in m/s². Returns ~0.0 m/s² when resting or in steady hover.
    pub fn linear_acceleration_z_world(&self) -> f32 {
        let accel_world = self.quaternion.transform_vector(&self.accel);
        accel_world.z - crate::GRAVITY_MSS
    }

    /// Computes the full 3D linear acceleration in world frame in m/s² (with 1g gravity removed from Z).
    /// Assumes self.accel is in m/s².
    pub fn linear_acceleration_world(&self) -> Vector3<f32> {
        let mut a_world = self.quaternion.transform_vector(&self.accel);
        a_world.z -= crate::GRAVITY_MSS;
        a_world
    }

    #[inline]
    fn feedback(sensor: Vector3<f32>, reference: Vector3<f32>) -> Vector3<f32> {
        let cross = sensor.cross(&reference);
        if sensor.dot(&reference) < 0.0 {
            cross.normalize()
        } else {
            cross
        }
    }
}

pub enum Convention {
    Nwu,
    Enu,
    Ned,
}

impl Default for Convention {
    fn default() -> Self {
        Self::Enu
    }
}

impl Convention {
    pub fn half_gravity(&self, q: UnitQuaternion<f32>) -> Vector3<f32> {
        match self {
            Convention::Nwu | Convention::Enu => Vector3::new(
                q.i * q.k - q.w * q.j,
                q.j * q.k + q.w * q.i,
                q.w * q.w - 0.5 + q.k * q.k,
            ),
            Convention::Ned => Vector3::new(
                q.w * q.j - q.i * q.k,
                -(q.j * q.k + q.w * q.i),
                0.5 - q.w * q.w - q.k * q.k,
            ),
        }
    }

    pub fn half_magnetic(&self, q: UnitQuaternion<f32>) -> Vector3<f32> {
        match self {
            Convention::Nwu => Vector3::new(
                q.i * q.j + q.w * q.k,
                q.w * q.w - 0.5 + q.j * q.j,
                q.j * q.k - q.w * q.i,
            ),
            Convention::Enu => Vector3::new(
                0.5 - q.w * q.w - q.i * q.i,
                q.w * q.k - q.i * q.j,
                -(q.i * q.k + q.w * q.j),
            ),
            Convention::Ned => Vector3::new(
                -(q.i * q.j + q.w * q.k),
                0.5 - q.w * q.w - q.j * q.j,
                q.w * q.i - q.j * q.k,
            ),
        }
    }
}

pub struct AhrsConfig {
    sample_period: f32,
    convention: Convention,
    gain: f32,
    gyro_range: f32,
    accel_rejection: f32,
    magnetic_rejection: f32,
    rejection_timeout_secs: f32,
    bias_config: Option<BiasConfig>,
}

impl Default for AhrsConfig {
    fn default() -> Self {
        Self {
            sample_period: 0.01,
            convention: Convention::default(),
            gain: 0.5,
            gyro_range: 0.0,
            accel_rejection: 90.0,
            magnetic_rejection: 90.0,
            rejection_timeout_secs: 2.0,
            bias_config: Some(BiasConfig::default()),
        }
    }
}

impl AhrsConfig {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_sample_rate(self, sample_rate: f32) -> Self {
        Self {
            sample_period: 1.0 / sample_rate,
            ..self
        }
    }

    pub fn set_sample_period(&mut self, sample_period: f32) {
        self.sample_period = sample_period
    }

    pub fn with_convention(self, convention: Convention) -> Self {
        Self { convention, ..self }
    }

    pub fn with_gain(self, gain: f32) -> Self {
        Self { gain, ..self }
    }

    pub fn with_gyro_range(self, gyro_range: f32) -> Self {
        Self { gyro_range, ..self }
    }

    pub fn with_accel_rejection(self, accel_rejection: f32) -> Self {
        Self {
            accel_rejection,
            ..self
        }
    }

    pub fn with_magnetic_rejection(self, magnetic_rejection: f32) -> Self {
        Self {
            magnetic_rejection,
            ..self
        }
    }

    pub fn with_rejection_timeout(self, rejection_timeout_secs: f32) -> Self {
        Self {
            rejection_timeout_secs,
            ..self
        }
    }

    pub fn with_bias_config(self, bias_config: Option<BiasConfig>) -> Self {
        Self {
            bias_config,
            ..self
        }
    }

    #[inline]
    pub fn sample_period(&self) -> f32 {
        self.sample_period
    }

    #[inline]
    pub fn gyro_range(&self) -> f32 {
        if self.gyro_range == 0.0 {
            f32::MAX
        } else {
            0.98 * self.gyro_range.to_radians()
        }
    }

    #[inline]
    pub fn accel_rejection(&self) -> f32 {
        if self.gain == 0.0 || self.rejection_timeout_secs == 0.0 || self.accel_rejection == 0.0 {
            f32::MAX
        } else {
            (0.5 * self.accel_rejection.to_radians().sin()).powi(2)
        }
    }

    #[inline]
    pub fn magnetic_rejection(&self) -> f32 {
        if self.gain == 0.0 || self.rejection_timeout_secs == 0.0 || self.magnetic_rejection == 0.0
        {
            f32::MAX
        } else {
            (0.5 * self.magnetic_rejection.to_radians().sin()).powi(2)
        }
    }

    #[inline]
    pub fn rejection_timeout(&self) -> i32 {
        if self.sample_period <= 0.0 {
            0
        } else {
            (self.rejection_timeout_secs / self.sample_period) as i32
        }
    }

    #[inline]
    pub fn gain(&self) -> f32 {
        self.gain
    }

    #[inline]
    pub fn convention(&self) -> &Convention {
        &self.convention
    }
}

pub struct BiasConfig {
    pub sample_rate: f32,
    pub stationary_threshold: f32,
    pub stationary_period: f32,
}

impl Default for BiasConfig {
    fn default() -> Self {
        Self {
            sample_rate: 100.0,
            stationary_threshold: 3.0, // deg/s
            stationary_period: 3.0,
        }
    }
}

impl BiasConfig {
    pub fn new() -> Self {
        Self::default()
    }
}
