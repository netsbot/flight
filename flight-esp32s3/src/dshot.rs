use core::marker::PhantomData;

use dshot_frame::NormalDshot;
use esp_hal::{
    Async,
    gpio::Level,
    rmt,
    rmt::{ConfigError, Error, PulseCode, TxChannelCreator},
};

pub struct BoardDshot<'a, S: DshotSpeed> {
    tx: rmt::Channel<'a, Async, rmt::Tx>,
    channel_hz: u32,
    _s: PhantomData<S>,
}
impl<'a, S: DshotSpeed> BoardDshot<'a, S> {
    pub fn new<C>(
        tx: rmt::Channel<'a, Async, rmt::Tx>,
        channel_hz: u32,
    ) -> Result<Self, ConfigError>
    where
        C: TxChannelCreator<'a, Async>,
    {
        Ok(Self {
            tx,
            channel_hz,
            _s: PhantomData,
        })
    }

    pub async fn send(&mut self, frame: dshot_frame::Frame<NormalDshot>) -> Result<(), Error> {
        let value = frame.inner();
        let pulses: [_; 17] = core::array::from_fn(|i| {
            if i < 16 {
                let bit = (value & (1 << (15 - i))) != 0;
                self.bit_to_pulse(bit)
            } else {
                PulseCode::default()
            }
        });

        self.tx.transmit(&pulses).await
    }

    #[inline(always)]
    fn bit_to_pulse(&self, bit: bool) -> PulseCode {
        let t_high_ns = if bit { S::T1H_NS } else { S::T0H_NS };
        let t_low_ns = S::T_TOTAL_NS - t_high_ns;

        // Convert ns to RMT ticks with rounding:
        // ticks = ns * channel_hz / 1e9, using u64 to avoid overflow
        // (5000 ns * 80 MHz = 4e11, overflows u32).
        let t_high_ticks =
            ((t_high_ns as u64 * self.channel_hz as u64 + 500_000_000) / 1_000_000_000) as u16;
        let t_low_ticks =
            ((t_low_ns as u64 * self.channel_hz as u64 + 500_000_000) / 1_000_000_000) as u16;

        PulseCode::new(Level::High, t_high_ticks, Level::Low, t_low_ticks)
    }
}

pub trait DshotSpeed {
    const T0H_NS: u16;
    const T1H_NS: u16;
    const T_TOTAL_NS: u16;
}

pub struct Dshot150;
impl DshotSpeed for Dshot150 {
    const T0H_NS: u16 = 2500;
    const T1H_NS: u16 = 5000;
    const T_TOTAL_NS: u16 = 6670;
}

pub struct Dshot300;
impl DshotSpeed for Dshot300 {
    const T0H_NS: u16 = 1250;
    const T1H_NS: u16 = 2500;
    const T_TOTAL_NS: u16 = 3330;
}

pub struct Dshot600;
impl DshotSpeed for Dshot600 {
    const T0H_NS: u16 = 625;
    const T1H_NS: u16 = 1250;
    const T_TOTAL_NS: u16 = 1670;
}
