use crate::Sensors;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

const STARTUP_GAIN: f32 = 10.0;
const STARTUP_PERIOD: f32 = 3.0;

struct Fusion {
    config: Config,

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
}

impl Fusion {
    pub fn new(config: Config) -> Self {
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
            config,
        }
    }

    fn restart(&mut self) {
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
    }

    pub fn update(&mut self, sensors: Sensors) {
        self.accel = sensors.accel;

        if sensors.gyro.x.abs() > self.config.gyro_range()
            || sensors.gyro.y.abs() > self.config.gyro_range()
            || sensors.gyro.z.abs() > self.config.gyro_range()
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

        if sensors.accel != Vector3::zeros() {
            self.half_accel_feedback = Self::feedback(sensors.accel.normalize(), self.half_gravity);

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

        if let Some(magnetometer) = sensors.magnetometer {
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

        let half_gyro = sensors.gyro.scale(0.5f32.to_radians());
        let adjusted_half_gyro =
            half_gyro + (half_accel_feedback + half_magnetometer_feedback.scale(self.ramped_gain));

        self.quaternion = UnitQuaternion::new_normalize(
            self.quaternion.into_inner()
                + (self.quaternion.into_inner()
                    * Quaternion::from_parts(
                        0.0,
                        adjusted_half_gyro * self.config.sample_period(),
                    )),
        );

        if sensors.magnetometer.is_none() && self.startup {
            self.set_heading(0.0);
        }
    }

    fn set_heading(&mut self, heading: f32) {
        let yaw = (self.quaternion.w * self.quaternion.k + self.quaternion.i * self.quaternion.j)
            .atan2(
                0.5 - self.quaternion.j * self.quaternion.j - self.quaternion.k * self.quaternion.k,
            );

        let half_yaw_minus_heading = 0.5 * (yaw - heading.to_radians());

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

pub struct Config {
    sample_rate: f32,
    convention: Convention,
    gain: f32,
    gyro_range: f32,
    accel_rejection: f32,
    magnetic_rejection: f32,
    rejection_timeout_secs: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sample_rate: 100.0,
            convention: Convention::Nwu,
            gain: 0.5,
            gyro_range: 0.0,
            accel_rejection: 90.0,
            magnetic_rejection: 90.0,
            rejection_timeout_secs: 0.0,
        }
    }
}

impl Config {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_sample_rate(mut self, rate: f32) -> Self {
        self.sample_rate = rate;
        self
    }

    pub fn with_convention(mut self, convention: Convention) -> Self {
        self.convention = convention;
        self
    }

    pub fn with_gain(mut self, gain: f32) -> Self {
        self.gain = gain;
        self
    }

    pub fn with_gyro_range(mut self, range: f32) -> Self {
        self.gyro_range = range;
        self
    }

    pub fn with_accel_rejection(mut self, rejection: f32) -> Self {
        self.accel_rejection = rejection;
        self
    }

    pub fn with_magnetic_rejection(mut self, rejection: f32) -> Self {
        self.magnetic_rejection = rejection;
        self
    }

    pub fn with_rejection_timeout(mut self, seconds: f32) -> Self {
        self.rejection_timeout_secs = seconds;
        self
    }

    #[inline]
    pub fn sample_period(&self) -> f32 {
        1.0 / self.sample_rate
    }

    #[inline]
    pub fn gyro_range(&self) -> f32 {
        if self.gyro_range == 0.0 {
            f32::MAX
        } else {
            0.98 * self.gyro_range
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
        (self.sample_rate * self.rejection_timeout_secs) as i32
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

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn initialization() {
        let config = Config::default().with_sample_rate(100.0);
        let mut fusion = Fusion::new(config);

        let sensors = Sensors {
            accel: Vector3::zeros(),
            gyro: Vector3::zeros(),
            magnetometer: None,
            alt: 0.0,
            dt: 0.0,
        };

        fusion.update(sensors);
        let euler = fusion.quaternion().euler_angles();

        assert_eq!(euler, (0.0, 0.0, 0.0));
    }
}
