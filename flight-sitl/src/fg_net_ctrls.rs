//! FlightGear NetCtrls binary network interface (Protocol Version 27).
//!
//! Maps directly to FlightGear's `src/Network/net_ctrls.hxx`.
//! Used for sending actuator commands (aileron, elevator, rudder, throttle)
//! over UDP to an external FlightGear instance running with:
//! `--native-ctrls=socket,in,60,,5501,udp`
//!
//! Multi-byte values are big-endian network byte order.
//! Struct size is exactly 528 bytes.

pub const FG_NET_CTRLS_VERSION: u32 = 27;
pub const FG_MAX_ENGINES: usize = 4;
pub const FG_MAX_TANKS: usize = 8;
pub const RESERVED_SPACE: usize = 25;

/// Total wire size of FGNetCtrls struct in bytes: exactly 744 bytes.
pub const FG_NET_CTRLS_SIZE: usize = 744;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct FGNetCtrls {
    pub version: u32,
    pub padding0: u32,

    // Aero controls (-1.0 ... 1.0)
    pub aileron: f64,
    pub elevator: f64,
    pub rudder: f64,
    pub aileron_trim: f64,
    pub elevator_trim: f64,
    pub rudder_trim: f64,
    pub flaps: f64,          // 0.0 ... 1.0
    pub spoilers: f64,
    pub speedbrake: f64,

    // Faults
    pub flaps_power: u32,
    pub flap_motor_ok: u32,

    // Engine controls
    pub num_engines: u32,
    pub master_bat: [u32; FG_MAX_ENGINES],
    pub master_alt: [u32; FG_MAX_ENGINES],
    pub magnetos: [u32; FG_MAX_ENGINES],
    pub starter_power: [u32; FG_MAX_ENGINES],
    pub padding1: u32,
    pub throttle: [f64; FG_MAX_ENGINES],    // 0.0 ... 1.0
    pub mixture: [f64; FG_MAX_ENGINES],
    pub condition: [f64; FG_MAX_ENGINES],
    pub fuel_pump_power: [u32; FG_MAX_ENGINES],
    pub prop_advance: [f64; FG_MAX_ENGINES],
    pub feed_tank_to: [u32; 4],
    pub reverse: [u32; 4],

    // Engine faults
    pub engine_ok: [u32; FG_MAX_ENGINES],
    pub mag_left_ok: [u32; FG_MAX_ENGINES],
    pub mag_right_ok: [u32; FG_MAX_ENGINES],
    pub spark_plugs_ok: [u32; FG_MAX_ENGINES],
    pub oil_press_status: [u32; FG_MAX_ENGINES],
    pub fuel_pump_ok: [u32; FG_MAX_ENGINES],

    // Fuel management
    pub num_tanks: u32,
    pub fuel_selector: [u32; FG_MAX_TANKS],
    pub xfer_pump: [u32; 5],
    pub cross_feed: u32,
    pub padding2: u32,

    // Brakes
    pub brake_left: f64,
    pub brake_right: f64,
    pub copilot_brake_left: f64,
    pub copilot_brake_right: f64,
    pub brake_parking: f64,

    // Gear & Avionics
    pub gear_handle: u32,
    pub master_avionics: u32,

    // Comms
    pub comm_1: f64,
    pub comm_2: f64,
    pub nav_1: f64,
    pub nav_2: f64,

    // Environment
    pub wind_speed_kt: f64,
    pub wind_dir_deg: f64,
    pub turbulence_norm: f64,
    pub temp_c: f64,
    pub press_inhg: f64,
    pub hground: f64,
    pub magvar: f64,
    pub icing: u32,
    pub speedup: u32,
    pub freeze: u32,

    // Reserved padding
    pub reserved: [u32; RESERVED_SPACE],
}

impl Default for FGNetCtrls {
    fn default() -> Self {
        Self {
            version: FG_NET_CTRLS_VERSION,
            padding0: 0,
            aileron: 0.0,
            elevator: 0.0,
            rudder: 0.0,
            aileron_trim: 0.0,
            elevator_trim: 0.0,
            rudder_trim: 0.0,
            flaps: 0.0,
            spoilers: 0.0,
            speedbrake: 0.0,
            flaps_power: 1,
            flap_motor_ok: 1,
            num_engines: 1,
            master_bat: [1; FG_MAX_ENGINES],
            master_alt: [1; FG_MAX_ENGINES],
            magnetos: [3; FG_MAX_ENGINES],
            starter_power: [1; FG_MAX_ENGINES],
            padding1: 0,
            throttle: [0.0; FG_MAX_ENGINES],
            mixture: [1.0; FG_MAX_ENGINES],
            condition: [1.0; FG_MAX_ENGINES],
            fuel_pump_power: [1; FG_MAX_ENGINES],
            prop_advance: [1.0; FG_MAX_ENGINES],
            feed_tank_to: [0; 4],
            reverse: [0; 4],
            engine_ok: [1; FG_MAX_ENGINES],
            mag_left_ok: [1; FG_MAX_ENGINES],
            mag_right_ok: [1; FG_MAX_ENGINES],
            spark_plugs_ok: [1; FG_MAX_ENGINES],
            oil_press_status: [0; FG_MAX_ENGINES],
            fuel_pump_ok: [1; FG_MAX_ENGINES],
            num_tanks: 1,
            fuel_selector: [1; FG_MAX_TANKS],
            xfer_pump: [0; 5],
            cross_feed: 0,
            padding2: 0,
            brake_left: 0.0,
            brake_right: 0.0,
            copilot_brake_left: 0.0,
            copilot_brake_right: 0.0,
            brake_parking: 0.0,
            gear_handle: 1,
            master_avionics: 1,
            comm_1: 0.0,
            comm_2: 0.0,
            nav_1: 0.0,
            nav_2: 0.0,
            wind_speed_kt: 0.0,
            wind_dir_deg: 0.0,
            turbulence_norm: 0.0,
            temp_c: 15.0,
            press_inhg: 29.92,
            hground: 0.0,
            magvar: 0.0,
            icing: 0,
            speedup: 1,
            freeze: 0,
            reserved: [0; RESERVED_SPACE],
        }
    }
}

impl FGNetCtrls {
    /// Populate a controls packet for elevator, aileron, and throttle.
    pub fn new(elevator: f64, aileron: f64, rudder: f64, throttle: f64) -> Self {
        let mut ctrls = Self::default();
        ctrls.elevator = elevator;
        ctrls.aileron = aileron;
        ctrls.rudder = rudder;
        ctrls.throttle[0] = throttle;
        ctrls
    }

    /// Serialize into big-endian byte array for transmission to FlightGear.
    pub fn to_be_bytes(&self) -> [u8; FG_NET_CTRLS_SIZE] {
        let mut buf = [0u8; FG_NET_CTRLS_SIZE];
        let mut offset = 0;

        macro_rules! write_field {
            ($val:expr, u32) => {
                buf[offset..offset + 4].copy_from_slice(&$val.to_be_bytes());
                offset += 4;
            };
            ($val:expr, f64) => {
                buf[offset..offset + 8].copy_from_slice(&$val.to_be_bytes());
                offset += 8;
            };
        }

        write_field!(self.version, u32);
        write_field!(self.padding0, u32);

        write_field!(self.aileron, f64);
        write_field!(self.elevator, f64);
        write_field!(self.rudder, f64);
        write_field!(self.aileron_trim, f64);
        write_field!(self.elevator_trim, f64);
        write_field!(self.rudder_trim, f64);
        write_field!(self.flaps, f64);
        write_field!(self.spoilers, f64);
        write_field!(self.speedbrake, f64);

        write_field!(self.flaps_power, u32);
        write_field!(self.flap_motor_ok, u32);

        write_field!(self.num_engines, u32);
        for &v in &self.master_bat { write_field!(v, u32); }
        for &v in &self.master_alt { write_field!(v, u32); }
        for &v in &self.magnetos { write_field!(v, u32); }
        for &v in &self.starter_power { write_field!(v, u32); }
        write_field!(self.padding1, u32);
        for &v in &self.throttle { write_field!(v, f64); }
        for &v in &self.mixture { write_field!(v, f64); }
        for &v in &self.condition { write_field!(v, f64); }
        for &v in &self.fuel_pump_power { write_field!(v, u32); }
        for &v in &self.prop_advance { write_field!(v, f64); }
        for &v in &self.feed_tank_to { write_field!(v, u32); }
        for &v in &self.reverse { write_field!(v, u32); }

        for &v in &self.engine_ok { write_field!(v, u32); }
        for &v in &self.mag_left_ok { write_field!(v, u32); }
        for &v in &self.mag_right_ok { write_field!(v, u32); }
        for &v in &self.spark_plugs_ok { write_field!(v, u32); }
        for &v in &self.oil_press_status { write_field!(v, u32); }
        for &v in &self.fuel_pump_ok { write_field!(v, u32); }

        write_field!(self.num_tanks, u32);
        for &v in &self.fuel_selector { write_field!(v, u32); }
        for &v in &self.xfer_pump { write_field!(v, u32); }
        write_field!(self.cross_feed, u32);
        write_field!(self.padding2, u32);

        write_field!(self.brake_left, f64);
        write_field!(self.brake_right, f64);
        write_field!(self.copilot_brake_left, f64);
        write_field!(self.copilot_brake_right, f64);
        write_field!(self.brake_parking, f64);

        write_field!(self.gear_handle, u32);
        write_field!(self.master_avionics, u32);

        write_field!(self.comm_1, f64);
        write_field!(self.comm_2, f64);
        write_field!(self.nav_1, f64);
        write_field!(self.nav_2, f64);

        write_field!(self.wind_speed_kt, f64);
        write_field!(self.wind_dir_deg, f64);
        write_field!(self.turbulence_norm, f64);
        write_field!(self.temp_c, f64);
        write_field!(self.press_inhg, f64);
        write_field!(self.hground, f64);
        write_field!(self.magvar, f64);

        write_field!(self.icing, u32);
        write_field!(self.speedup, u32);
        write_field!(self.freeze, u32);

        for &v in &self.reserved { write_field!(v, u32); }

        assert_eq!(offset, FG_NET_CTRLS_SIZE, "Size mismatch in FGNetCtrls serialization");
        buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fg_net_ctrls_size_and_endianness() {
        assert_eq!(std::mem::size_of::<FGNetCtrls>(), FG_NET_CTRLS_SIZE);
        let ctrls = FGNetCtrls::default();
        let bytes = ctrls.to_be_bytes();
        assert_eq!(bytes.len(), FG_NET_CTRLS_SIZE);

        // Version 27 big-endian
        assert_eq!(&bytes[0..4], &27u32.to_be_bytes());
    }
}
