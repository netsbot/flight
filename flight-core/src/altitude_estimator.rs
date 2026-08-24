use nalgebra::{SMatrix, SVector, UnitQuaternion, Vector3};

#[derive(Debug, Clone)]
pub struct AltitudeEstimator {
    x: SVector<f32, 3>,

    p: SMatrix<f32, 3, 3>,

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
        let x = SVector::<f32, 3>::new(initial_alt, 0.0, 0.0);

        // Initial state covariance (1.0 on diagonal)
        let p = SMatrix::<f32, 3, 3>::identity();

        Self {
            x,
            p,
            accel_noise_var,
            bias_noise_var,
            baro_noise_var,
        }
    }

    pub fn rotate_accel_to_world(
        attitude: &UnitQuaternion<f32>,
        accel_body_g: &Vector3<f32>,
    ) -> f32 {
        let accel_world_g = attitude.transform_vector(accel_body_g);
        // Subtract 1.0g of gravity and convert to m/s²
        (accel_world_g.z - 1.0) * 9.80665
    }

    pub fn predict(&mut self, accel_z_world: f32, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        let dt2 = dt * dt;

        #[rustfmt::skip]
        let f = SMatrix::<f32, 3, 3>::new(
            1.0,  dt, -0.5 * dt2,
            0.0, 1.0,       -dt,
            0.0, 0.0,       1.0,
        );

        let b = SVector::<f32, 3>::new(0.5 * dt2, dt, 0.0);

        self.x = f * self.x + b * accel_z_world;

        // Process noise matrix Q
        let q_a = self.accel_noise_var;
        let q_b = self.bias_noise_var;

        #[rustfmt::skip]
        let q = SMatrix::<f32, 3, 3>::new(
            0.25 * dt2 * dt2 * q_a, 0.5 * dt2 * dt * q_a, 0.0,
            0.5 * dt2 * dt * q_a,   dt2 * q_a,           0.0,
            0.0,                   0.0,                 q_b * dt,
        );

        // 2. Predict Covariance: P = F*P*Fᵀ + Q
        self.p = f * self.p * f.transpose() + q;
    }

    pub fn update_baro(&mut self, baro_alt: f32) {
        let h = SMatrix::<f32, 1, 3>::new(1.0, 0.0, 0.0);

        let y = baro_alt - self.x[0];

        let s = (h * self.p * h.transpose())[0] + self.baro_noise_var;

        let k = (self.p * h.transpose()) / s;

        self.x += k * y;

        let identity = SMatrix::<f32, 3, 3>::identity();
        self.p = (identity - k * h) * self.p;
    }

    pub fn altitude(&self) -> f32 {
        self.x[0]
    }

    pub fn velocity(&self) -> f32 {
        self.x[1]
    }

    pub fn accel_bias(&self) -> f32 {
        self.x[2]
    }
}
