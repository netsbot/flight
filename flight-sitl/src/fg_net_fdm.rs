//! FlightGear NetFDM binary network interface (Protocol Version 24).
//!
//! Maps directly to FlightGear's `src/Network/net_fdm.hxx` and JSBSim's `FGOutputFG`.
//! Used for receiving 60 Hz 6-DOF telemetry over UDP from FlightGear running with:
//! `--native-fdm=socket,out,60,127.0.0.1,5500,udp`
//!
//! Multi-byte integer and floating-point fields are in Network Byte Order (Big-Endian).

pub const FG_NET_FDM_VERSION: u32 = 24;
pub const FG_MAX_ENGINES: usize = 4;
pub const FG_MAX_WHEELS: usize = 3;
pub const FG_MAX_TANKS: usize = 4;

/// Total wire size of FGNetFDM struct in bytes: exactly 408 bytes.
pub const FG_NET_FDM_SIZE: usize = 408;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct FGNetFDM {
    pub version: u32,
    pub padding: u32,

    // Positions
    pub longitude: f64,     // Geodetic longitude (radians)
    pub latitude: f64,      // Geodetic latitude (radians)
    pub altitude: f64,      // Altitude above sea level (meters)
    pub agl: f32,           // Altitude above ground level (meters)
    pub phi: f32,           // Roll angle (radians, right wing down positive)
    pub theta: f32,         // Pitch angle (radians, nose up positive)
    pub psi: f32,           // Yaw / true heading (radians, clockwise from North)
    pub alpha: f32,         // Angle of attack (radians)
    pub beta: f32,          // Sideslip angle (radians)

    // Velocities
    pub phidot: f32,        // Roll rate (radians/sec)
    pub thetadot: f32,      // Pitch rate (radians/sec)
    pub psidot: f32,        // Yaw rate (radians/sec)
    pub vcas: f32,          // Calibrated airspeed (fps)
    pub climb_rate: f32,    // Climb rate (feet per second)
    pub v_north: f32,       // North velocity in local frame (fps)
    pub v_east: f32,        // East velocity in local frame (fps)
    pub v_down: f32,        // Down / vertical velocity in local frame (fps)
    pub v_body_u: f32,      // Body axis forward velocity (fps)
    pub v_body_v: f32,      // Body axis rightward velocity (fps)
    pub v_body_w: f32,      // Body axis downward velocity (fps)

    // Accelerations
    pub a_x_pilot: f32,     // X accel in body frame (ft/sec^2)
    pub a_y_pilot: f32,     // Y accel in body frame (ft/sec^2)
    pub a_z_pilot: f32,     // Z accel in body frame (ft/sec^2)

    // Stall & Slip
    pub stall_warning: f32, // 0.0 - 1.0 indicating amount of stall
    pub slip_deg: f32,      // Slip ball deflection (degrees)

    // Engine Status
    pub num_engines: u32,
    pub eng_state: [u32; FG_MAX_ENGINES],
    pub rpm: [f32; FG_MAX_ENGINES],
    pub fuel_flow: [f32; FG_MAX_ENGINES],
    pub fuel_px: [f32; FG_MAX_ENGINES],
    pub egt: [f32; FG_MAX_ENGINES],
    pub cht: [f32; FG_MAX_ENGINES],
    pub mp_osi: [f32; FG_MAX_ENGINES],
    pub tit: [f32; FG_MAX_ENGINES],
    pub oil_temp: [f32; FG_MAX_ENGINES],
    pub oil_px: [f32; FG_MAX_ENGINES],

    // Fuel Tanks
    pub num_tanks: u32,
    pub fuel_quantity: [f32; FG_MAX_TANKS],

    // Gear Status
    pub num_wheels: u32,
    pub wow: [u32; FG_MAX_WHEELS],
    pub gear_pos: [f32; FG_MAX_WHEELS],
    pub gear_steer: [f32; FG_MAX_WHEELS],
    pub gear_compression: [f32; FG_MAX_WHEELS],

    // Environment
    pub cur_time: u32,
    pub warp: i32,
    pub visibility: f32,

    // Control Surface Positions (normalized)
    pub elevator: f32,
    pub elevator_trim_tab: f32,
    pub left_flap: f32,
    pub right_flap: f32,
    pub left_aileron: f32,
    pub right_aileron: f32,
    pub rudder: f32,
    pub nose_wheel: f32,
    pub speedbrake: f32,
    pub spoilers: f32,
}

impl Default for FGNetFDM {
    fn default() -> Self {
        Self {
            version: FG_NET_FDM_VERSION,
            padding: 0,
            longitude: 0.0,
            latitude: 0.0,
            altitude: 0.0,
            agl: 0.0,
            phi: 0.0,
            theta: 0.0,
            psi: 0.0,
            alpha: 0.0,
            beta: 0.0,
            phidot: 0.0,
            thetadot: 0.0,
            psidot: 0.0,
            vcas: 0.0,
            climb_rate: 0.0,
            v_north: 0.0,
            v_east: 0.0,
            v_down: 0.0,
            v_body_u: 0.0,
            v_body_v: 0.0,
            v_body_w: 0.0,
            a_x_pilot: 0.0,
            a_y_pilot: 0.0,
            a_z_pilot: -32.174,
            stall_warning: 0.0,
            slip_deg: 0.0,
            num_engines: 1,
            eng_state: [2, 0, 0, 0],
            rpm: [25000.0, 0.0, 0.0, 0.0],
            fuel_flow: [0.0; FG_MAX_ENGINES],
            fuel_px: [0.0; FG_MAX_ENGINES],
            egt: [0.0; FG_MAX_ENGINES],
            cht: [0.0; FG_MAX_ENGINES],
            mp_osi: [0.0; FG_MAX_ENGINES],
            tit: [0.0; FG_MAX_ENGINES],
            oil_temp: [0.0; FG_MAX_ENGINES],
            oil_px: [0.0; FG_MAX_ENGINES],
            num_tanks: 1,
            fuel_quantity: [0.1; FG_MAX_TANKS],
            num_wheels: 1,
            wow: [0; FG_MAX_WHEELS],
            gear_pos: [1.0, 0.0, 0.0],
            gear_steer: [0.0; FG_MAX_WHEELS],
            gear_compression: [0.0; FG_MAX_WHEELS],
            cur_time: 1234567890,
            warp: 0,
            visibility: 25000.0,
            elevator: 0.0,
            elevator_trim_tab: 0.0,
            left_flap: 0.0,
            right_flap: 0.0,
            left_aileron: 0.0,
            right_aileron: 0.0,
            rudder: 0.0,
            nose_wheel: 0.0,
            speedbrake: 0.0,
            spoilers: 0.0,
        }
    }
}

impl FGNetFDM {
    /// Parse a big-endian raw network packet received from FlightGear.
    pub fn from_be_bytes(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() < FG_NET_FDM_SIZE {
            return Err(format!(
                "Packet too small: expected {} bytes, got {}",
                FG_NET_FDM_SIZE,
                bytes.len()
            ));
        }

        let mut offset = 0;
        macro_rules! read_u32 {
            () => {{
                let val = u32::from_be_bytes(bytes[offset..offset + 4].try_into().unwrap());
                offset += 4;
                val
            }};
        }
        macro_rules! read_i32 {
            () => {{
                let val = i32::from_be_bytes(bytes[offset..offset + 4].try_into().unwrap());
                offset += 4;
                val
            }};
        }
        macro_rules! read_f32 {
            () => {{
                let val = f32::from_be_bytes(bytes[offset..offset + 4].try_into().unwrap());
                offset += 4;
                val
            }};
        }
        macro_rules! read_f64 {
            () => {{
                let val = f64::from_be_bytes(bytes[offset..offset + 8].try_into().unwrap());
                offset += 8;
                val
            }};
        }

        let version = read_u32!();
        if version != FG_NET_FDM_VERSION {
            return Err(format!(
                "Unexpected FGNetFDM version: expected {}, got {}",
                FG_NET_FDM_VERSION, version
            ));
        }
        let padding = read_u32!();

        let longitude = read_f64!();
        let latitude = read_f64!();
        let altitude = read_f64!();
        let agl = read_f32!();
        let phi = read_f32!();
        let theta = read_f32!();
        let psi = read_f32!();
        let alpha = read_f32!();
        let beta = read_f32!();

        let phidot = read_f32!();
        let thetadot = read_f32!();
        let psidot = read_f32!();
        let vcas = read_f32!();
        let climb_rate = read_f32!();
        let v_north = read_f32!();
        let v_east = read_f32!();
        let v_down = read_f32!();
        let v_body_u = read_f32!();
        let v_body_v = read_f32!();
        let v_body_w = read_f32!();

        let a_x_pilot = read_f32!();
        let a_y_pilot = read_f32!();
        let a_z_pilot = read_f32!();

        let stall_warning = read_f32!();
        let slip_deg = read_f32!();

        let num_engines = read_u32!();
        let mut eng_state = [0u32; FG_MAX_ENGINES];
        for v in &mut eng_state { *v = read_u32!(); }
        let mut rpm = [0.0f32; FG_MAX_ENGINES];
        for v in &mut rpm { *v = read_f32!(); }
        let mut fuel_flow = [0.0f32; FG_MAX_ENGINES];
        for v in &mut fuel_flow { *v = read_f32!(); }
        let mut fuel_px = [0.0f32; FG_MAX_ENGINES];
        for v in &mut fuel_px { *v = read_f32!(); }
        let mut egt = [0.0f32; FG_MAX_ENGINES];
        for v in &mut egt { *v = read_f32!(); }
        let mut cht = [0.0f32; FG_MAX_ENGINES];
        for v in &mut cht { *v = read_f32!(); }
        let mut mp_osi = [0.0f32; FG_MAX_ENGINES];
        for v in &mut mp_osi { *v = read_f32!(); }
        let mut tit = [0.0f32; FG_MAX_ENGINES];
        for v in &mut tit { *v = read_f32!(); }
        let mut oil_temp = [0.0f32; FG_MAX_ENGINES];
        for v in &mut oil_temp { *v = read_f32!(); }
        let mut oil_px = [0.0f32; FG_MAX_ENGINES];
        for v in &mut oil_px { *v = read_f32!(); }

        let num_tanks = read_u32!();
        let mut fuel_quantity = [0.0f32; FG_MAX_TANKS];
        for v in &mut fuel_quantity { *v = read_f32!(); }

        let num_wheels = read_u32!();
        let mut wow = [0u32; FG_MAX_WHEELS];
        for v in &mut wow { *v = read_u32!(); }
        let mut gear_pos = [0.0f32; FG_MAX_WHEELS];
        for v in &mut gear_pos { *v = read_f32!(); }
        let mut gear_steer = [0.0f32; FG_MAX_WHEELS];
        for v in &mut gear_steer { *v = read_f32!(); }
        let mut gear_compression = [0.0f32; FG_MAX_WHEELS];
        for v in &mut gear_compression { *v = read_f32!(); }

        let cur_time = read_u32!();
        let warp = read_i32!();
        let visibility = read_f32!();

        let elevator = read_f32!();
        let elevator_trim_tab = read_f32!();
        let left_flap = read_f32!();
        let right_flap = read_f32!();
        let left_aileron = read_f32!();
        let right_aileron = read_f32!();
        let rudder = read_f32!();
        let nose_wheel = read_f32!();
        let speedbrake = read_f32!();
        let spoilers = read_f32!();

        assert_eq!(offset, FG_NET_FDM_SIZE);

        Ok(Self {
            version,
            padding,
            longitude,
            latitude,
            altitude,
            agl,
            phi,
            theta,
            psi,
            alpha,
            beta,
            phidot,
            thetadot,
            psidot,
            vcas,
            climb_rate,
            v_north,
            v_east,
            v_down,
            v_body_u,
            v_body_v,
            v_body_w,
            a_x_pilot,
            a_y_pilot,
            a_z_pilot,
            stall_warning,
            slip_deg,
            num_engines,
            eng_state,
            rpm,
            fuel_flow,
            fuel_px,
            egt,
            cht,
            mp_osi,
            tit,
            oil_temp,
            oil_px,
            num_tanks,
            fuel_quantity,
            num_wheels,
            wow,
            gear_pos,
            gear_steer,
            gear_compression,
            cur_time,
            warp,
            visibility,
            elevator,
            elevator_trim_tab,
            left_flap,
            right_flap,
            left_aileron,
            right_aileron,
            rudder,
            nose_wheel,
            speedbrake,
            spoilers,
        })
    }

    /// Serialize into big-endian byte array.
    pub fn to_be_bytes(&self) -> [u8; FG_NET_FDM_SIZE] {
        let mut buf = [0u8; FG_NET_FDM_SIZE];
        let mut offset = 0;

        macro_rules! write_field {
            ($val:expr, u32) => {
                buf[offset..offset + 4].copy_from_slice(&$val.to_be_bytes());
                offset += 4;
            };
            ($val:expr, i32) => {
                buf[offset..offset + 4].copy_from_slice(&$val.to_be_bytes());
                offset += 4;
            };
            ($val:expr, f32) => {
                buf[offset..offset + 4].copy_from_slice(&$val.to_be_bytes());
                offset += 4;
            };
            ($val:expr, f64) => {
                buf[offset..offset + 8].copy_from_slice(&$val.to_be_bytes());
                offset += 8;
            };
        }

        write_field!(self.version, u32);
        write_field!(self.padding, u32);

        write_field!(self.longitude, f64);
        write_field!(self.latitude, f64);
        write_field!(self.altitude, f64);
        write_field!(self.agl, f32);
        write_field!(self.phi, f32);
        write_field!(self.theta, f32);
        write_field!(self.psi, f32);
        write_field!(self.alpha, f32);
        write_field!(self.beta, f32);

        write_field!(self.phidot, f32);
        write_field!(self.thetadot, f32);
        write_field!(self.psidot, f32);
        write_field!(self.vcas, f32);
        write_field!(self.climb_rate, f32);
        write_field!(self.v_north, f32);
        write_field!(self.v_east, f32);
        write_field!(self.v_down, f32);
        write_field!(self.v_body_u, f32);
        write_field!(self.v_body_v, f32);
        write_field!(self.v_body_w, f32);

        write_field!(self.a_x_pilot, f32);
        write_field!(self.a_y_pilot, f32);
        write_field!(self.a_z_pilot, f32);

        write_field!(self.stall_warning, f32);
        write_field!(self.slip_deg, f32);

        write_field!(self.num_engines, u32);
        for &v in &self.eng_state { write_field!(v, u32); }
        for &v in &self.rpm { write_field!(v, f32); }
        for &v in &self.fuel_flow { write_field!(v, f32); }
        for &v in &self.fuel_px { write_field!(v, f32); }
        for &v in &self.egt { write_field!(v, f32); }
        for &v in &self.cht { write_field!(v, f32); }
        for &v in &self.mp_osi { write_field!(v, f32); }
        for &v in &self.tit { write_field!(v, f32); }
        for &v in &self.oil_temp { write_field!(v, f32); }
        for &v in &self.oil_px { write_field!(v, f32); }

        write_field!(self.num_tanks, u32);
        for &v in &self.fuel_quantity { write_field!(v, f32); }

        write_field!(self.num_wheels, u32);
        for &v in &self.wow { write_field!(v, u32); }
        for &v in &self.gear_pos { write_field!(v, f32); }
        for &v in &self.gear_steer { write_field!(v, f32); }
        for &v in &self.gear_compression { write_field!(v, f32); }

        write_field!(self.cur_time, u32);
        write_field!(self.warp, i32);
        write_field!(self.visibility, f32);

        write_field!(self.elevator, f32);
        write_field!(self.elevator_trim_tab, f32);
        write_field!(self.left_flap, f32);
        write_field!(self.right_flap, f32);
        write_field!(self.left_aileron, f32);
        write_field!(self.right_aileron, f32);
        write_field!(self.rudder, f32);
        write_field!(self.nose_wheel, f32);
        write_field!(self.speedbrake, f32);
        write_field!(self.spoilers, f32);

        assert_eq!(offset, FG_NET_FDM_SIZE, "Size mismatch in FGNetFDM serialization");
        buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fg_net_fdm_roundtrip() {
        assert_eq!(std::mem::size_of::<FGNetFDM>(), FG_NET_FDM_SIZE);
        let mut original = FGNetFDM::default();
        original.latitude = 0.6565;
        original.longitude = -2.1358;
        original.altitude = 100.5;
        original.vcas = 40.0;
        original.theta = 0.05;

        let bytes = original.to_be_bytes();
        assert_eq!(bytes.len(), FG_NET_FDM_SIZE);
        assert_eq!(&bytes[0..4], &24u32.to_be_bytes());

        let decoded = FGNetFDM::from_be_bytes(&bytes).expect("Failed to parse bytes");
        assert_eq!(decoded.version, 24);
        assert!((decoded.latitude - original.latitude).abs() < 1e-12);
        assert!((decoded.longitude - original.longitude).abs() < 1e-12);
        assert!((decoded.altitude - original.altitude).abs() < 1e-12);
        assert!((decoded.vcas - original.vcas).abs() < 1e-6);
        assert!((decoded.theta - original.theta).abs() < 1e-6);
    }
}
