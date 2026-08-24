use embedded_profiling::{EPInstant, EmbeddedProfiler};

pub struct Esp32Profiler;

impl EmbeddedProfiler for Esp32Profiler {
    fn read_clock(&self) -> EPInstant {
        let count: u32;

        unsafe {
            core::arch::asm!("rsr.ccount {0}", out(reg) count);
        }

        EPInstant::from_ticks(count / 240)
    }
}
