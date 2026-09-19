use nalgebra::ComplexField;
use embedded_hal::pwm::SetDutyCycle;

pub struct Servo<S: SetDutyCycle> {
    channel: S,
    period_us: u16,
    config: ServoConfig,
}

impl<S: SetDutyCycle> Servo<S> {
    pub fn new(channel: S, period_us: u16, config: ServoConfig) -> Self {
        Self {
            channel,
            period_us,
            config,
        }
    }

    // set servo to normalized deflection [-1.0, 1.0]
    pub fn set_deflection(&mut self, deflection: f32) -> Result<(), S::Error> {
        let clamped_input = deflection.clamp(-1.0, 1.0);
        let directional_input = if self.config.reversed {
            -clamped_input
        } else {
            clamped_input
        };

        let half_throw = if directional_input >= 0.0 {
            self.config.max_us.saturating_sub(self.config.centre_us) as f32
        } else {
            self.config.centre_us.saturating_sub(self.config.min_us) as f32
        };

        let target_pulse = self.config.centre_us as f32 + (directional_input * half_throw);
        self.set_pulse_us(target_pulse.round() as u16)
    }

    #[inline]
    pub fn set_pulse_us(&mut self, pulse_us: u16) -> Result<(), S::Error> {
        self.channel.set_duty_cycle_fraction(
            pulse_us.clamp(self.config.min_us, self.config.max_us),
            self.period_us,
        )
    }

    #[inline]
    pub fn get_config(&self) -> &ServoConfig {
        &self.config
    }

    #[inline]
    pub fn set_config(&mut self, servo_config: ServoConfig) {
        self.config = servo_config
    }
}

pub struct ServoConfig {
    min_us: u16,
    max_us: u16,
    centre_us: u16,
    reversed: bool,
}

impl ServoConfig {
    pub fn new(min_us: u16, max_us: u16, centre_us: u16, reversed: bool) -> Self {
        Self {
            min_us,
            max_us,
            centre_us,
            reversed,
        }
    }

    pub fn with_min_us(self, min_us: u16) -> Self {
        Self { min_us, ..self }
    }
    pub fn with_max_us(self, max_us: u16) -> Self {
        Self { max_us, ..self }
    }
    pub fn with_centre_us(self, centre_us: u16) -> Self {
        Self {
            centre_us: centre_us.clamp(self.min_us, self.max_us),
            ..self
        }
    }
    pub fn with_reversed(self, reversed: bool) -> Self {
        Self { reversed, ..self }
    }
}

impl Default for ServoConfig {
    fn default() -> Self {
        Self {
            min_us: 1000,
            max_us: 2000,
            centre_us: 1500,
            reversed: false,
        }
    }
}
