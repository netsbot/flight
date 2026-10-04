use bme280_rs::{AsyncBme280, Configuration, Filter, Oversampling};
use embassy_time::{Delay, Duration, Ticker};
use esp_hal::Async;
use esp_hal::i2c::master::I2c;
use nalgebra::ComplexField;

use crate::BARO_CHANNEL;

pub struct BoardBaro<I2C: embedded_hal_async::i2c::I2c> {
    inner: AsyncBme280<I2C, Delay>,
}

impl<I2c: embedded_hal_async::i2c::I2c> BoardBaro<I2c> {
    pub async fn new(spi_device: I2c) -> Self {
        let config = Configuration::default()
            .with_pressure_oversampling(Oversampling::Oversample4)
            .with_filter(Filter::Filter2);
        let mut baro = AsyncBme280::new(spi_device, Delay);
        baro.init().await.unwrap();
        baro.set_sampling_configuration(config).await.unwrap();

        Self { inner: baro }
    }

    #[inline(always)]
    pub async fn read_pressure(&mut self) -> f32 {
        self.inner.read_pressure().await.unwrap().unwrap()
    }

    #[inline(always)]
    pub async fn read_altitude(&mut self, qnh: f32) -> f32 {
        const SCALE_FACTOR: f32 = 44330.77;
        const EXPONENT: f32 = 0.190263;

        SCALE_FACTOR * (1.0 - (self.read_pressure().await / qnh).powf(EXPONENT))
    }
}

#[embassy_executor::task]
pub async fn baro_task(i2c: I2c<'static, Async>) {
    let mut baro = BoardBaro::new(i2c).await;
    let ground_pressure = {
        let mut sum = 0.0;
        for _ in 0..50 {
            sum += baro.read_pressure().await;
        }
        sum / 50.0
    };

    let mut ticker = Ticker::every(Duration::from_hz(50));
    let baro_tx = BARO_CHANNEL.sender();

    loop {
        baro_tx
            .send(baro.read_altitude(ground_pressure).await)
            .await;
        ticker.next().await;
    }
}
