use embedded_profiling::{EPInstant, EPSnapshot, EmbeddedProfiler};

pub struct Esp32Profiler;

impl EmbeddedProfiler for Esp32Profiler {
    fn read_clock(&self) -> EPInstant {
        let count = xtensa_lx::timer::get_cycle_count();
        EPInstant::from_ticks(count / 240)
    }

    fn log_snapshot(&self, snapshot: &EPSnapshot) {
        defmt::info!("<EPSS {}: {}>", snapshot.name, snapshot.duration)
    }
}
