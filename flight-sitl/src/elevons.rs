pub struct ElevonOutputs {
    pub left_elevon: f32,   // Normalized command [-1.0, 1.0]
    pub right_elevon: f32,  // Normalized command [-1.0, 1.0]
    pub throttle: f32,      // Normalized command [0.0, 1.0]
}

/// Mixes pitch and roll commands into elevon deflection angles.
/// - Left elevon:  Pitch - Roll
/// - Right elevon: Pitch + Roll
pub fn mix_elevons(throttle: f32, pitch: f32, roll: f32) -> ElevonOutputs {
    let left = (-pitch - roll).clamp(-1.0, 1.0);
    let right = (-pitch + roll).clamp(-1.0, 1.0);
    let thr = throttle.clamp(0.0, 1.0);

    ElevonOutputs {
        left_elevon: left,
        right_elevon: right,
        throttle: thr,
    }
}
