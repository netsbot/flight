use nalgebra::{Matrix1x3, Matrix3, SMatrix, SVector, UnitQuaternion, Vector3};

#[derive(Debug, Clone)]
pub struct AltitudeEstimator {
    state: Vector3<f32>,
    covariance: Matrix3<f32>,
    accel_noise_var: f32,
    bias_noise_var: f32,
    baro_noise_var: f32,
}

impl AltitudeEstimator {
    pub fn new(
        initial_alt: f32,
        accel_noise_var: f32,
        bias_noise_var: f32,
        baro_noise_var: f32,
    ) -> Self {
        let state = Vector3::new(initial_alt, 0.0, 0.0);

        // Initial state covariance (1.0 on diagonal)
        let covariance = Matrix3::identity();

        Self {
            state,
            covariance,
            accel_noise_var,
            bias_noise_var,
            baro_noise_var,
        }
    }

    pub fn rotate_accel_to_world(
        attitude: &UnitQuaternion<f32>,
        accel_body_ms2: &Vector3<f32>,
    ) -> f32 {
        let accel_world = attitude.transform_vector(accel_body_ms2);
        // Subtract 1g of gravity (accel is already in m/s²)
        accel_world.z - crate::GRAVITY_MSS
    }

    pub fn predict(&mut self, accel_z_world: f32, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        let dt2 = dt * dt;

        #[rustfmt::skip]
        let f = Matrix3::new(
            1.0,  dt, -0.5 * dt2,
            0.0, 1.0,       -dt,
            0.0, 0.0,       1.0,
        );

        let b = SVector::<f32, 3>::new(0.5 * dt2, dt, 0.0);

        self.state = f * self.state + b * accel_z_world;

        // Process noise matrix Q
        let q_a = self.accel_noise_var;
        let q_b = self.bias_noise_var;

        #[rustfmt::skip]
        let q = Matrix3::new(
            0.25 * dt2 * dt2 * q_a, 0.5 * dt2 * dt * q_a, 0.0,
            0.5 * dt2 * dt * q_a,   dt2 * q_a,           0.0,
            0.0,                   0.0,                 q_b * dt,
        );

        // 2. Predict Covariance: P = F*P*Fᵀ + Q
        self.covariance = f * self.covariance * f.transpose() + q;
    }

    pub fn update_baro(&mut self, baro_alt: f32) {
        let h = Matrix1x3::new(1.0, 0.0, 0.0);
        let y = baro_alt - self.state[0];
        let s = (h * self.covariance * h.transpose())[0] + self.baro_noise_var;
        let k = (self.covariance * h.transpose()) / s;
        self.state += k * y;
        let identity = Matrix3::identity();
        self.covariance = (identity - k * h) * self.covariance;
    }

    #[inline]
    pub fn altitude(&self) -> f32 {
        self.state[0]
    }

    #[inline]
    pub fn velocity(&self) -> f32 {
        self.state[1]
    }

    #[inline]
    pub fn accel_bias(&self) -> f32 {
        self.state[2]
    }
}
