#![no_std]

use core::{sync::atomic::AtomicI32, time::Duration};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch};
use flight_core::{DroneState, imu::AccumulatedImu};

pub mod baro;
pub mod comms;
pub mod dshot;
mod imu;
pub mod interrupt_tasks;
pub mod profiler;
pub mod servos;
pub mod state;

static THROTTLE: AtomicI32 = AtomicI32::new(0);
static IMU_DATA_CHANNEL: Channel<CriticalSectionRawMutex, AccumulatedImu, 8> = Channel::new();
static BARO_CHANNEL: Channel<CriticalSectionRawMutex, f32, 4> = Channel::new();
static STATE_WATCH: Watch<CriticalSectionRawMutex, DroneState, 2> = Watch::new();
pub static TARGET_MAC: [u8; 6] = [172, 39, 110, 170, 188, 84];

#[derive(Copy, Clone)]
pub struct CycleInstant(u32);

impl CycleInstant {
    const CPU_HZ_INV: f32 = 1.0 / 240_000_000.0;

    /// Returns the current hardware cycle instant.
    #[inline(always)]
    pub fn now() -> Self {
        Self(xtensa_lx::timer::get_cycle_count())
    }

    /// Returns the raw CPU cycle difference since an earlier instant.
    #[inline(always)]
    pub fn cycles_since(&self, earlier: Self) -> u32 {
        self.0.wrapping_sub(earlier.0)
    }

    /// Returns delta-t in seconds (`f32`) since an earlier instant.
    #[inline(always)]
    pub fn secs_since(&self, earlier: Self) -> f32 {
        (self.cycles_since(earlier) as f32) * Self::CPU_HZ_INV
    }

    /// Returns `Duration` since an earlier instant.
    #[inline(always)]
    pub fn duration_since(&self, earlier: Self) -> Duration {
        Duration::from_secs_f32(self.secs_since(earlier))
    }

    /// Returns the raw CPU cycles elapsed from `self` until now.
    #[inline(always)]
    pub fn elapsed_cycles(&self) -> u32 {
        Self::now().cycles_since(*self)
    }

    /// Returns elapsed time in seconds (`f32`) from `self` until now.
    #[inline(always)]
    pub fn elapsed_secs(&self) -> f32 {
        Self::now().secs_since(*self)
    }

    /// Returns elapsed `Duration` from `self` until now.
    #[inline(always)]
    pub fn elapsed(&self) -> Duration {
        Self::now().duration_since(*self)
    }

    /// Resets `self` to `CycleInstant::now()`.
    #[inline(always)]
    pub fn reset(&mut self) {
        *self = Self::now();
    }

    /// Computes the elapsed delta-t in seconds from `self` to now, and updates `self` to now.
    #[inline(always)]
    pub fn elapsed_secs_and_reset(&mut self) -> f32 {
        let now = Self::now();
        let dt = now.secs_since(*self);
        *self = now;
        dt
    }

    /// Computes the elapsed `Duration` from `self` to now, and updates `self` to now.
    #[inline(always)]
    pub fn elapsed_and_reset(&mut self) -> Duration {
        let now = Self::now();
        let dur = now.duration_since(*self);
        *self = now;
        dur
    }
}
