use bmp280::{Configuration, IIRFilter, Oversampling, spi::Bmp280};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Ticker};
use esp_hal::{Async, gpio::Output, spi::master::Spi};
use nalgebra::ComplexField;

use crate::BARO_CHANNEL;

pub struct BoardBaro<SPI: embedded_hal_async::spi::SpiDevice> {
    inner: Bmp280<SPI, Delay>,
}

impl<SPI: embedded_hal_async::spi::SpiDevice> BoardBaro<SPI> {
    pub async fn new(spi_device: SPI) -> Self {
        let config = Configuration::default()
            .with_pressure_oversampling(Oversampling::Oversampling4X)
            .with_iir_filter(IIRFilter::Coefficient2);
        let baro = Bmp280::new_with_config(spi_device, Delay, config)
            .await
            .unwrap();

        Self { inner: baro }
    }

    pub async fn read_pressure(&mut self) -> f32 {
        let data = self.inner.measure().await.unwrap();
        data.pressure
    }

    pub async fn read_altitude(&mut self, qnh: f32) -> f32 {
        const SCALE_FACTOR: f32 = 44330.77;
        const EXPONENT: f32 = 0.190263;

        SCALE_FACTOR * (1.0 - (self.read_pressure().await / qnh).powf(EXPONENT))
    }
}

#[embassy_executor::task]
pub async fn baro_task(
    spi: &'static Mutex<NoopRawMutex, Spi<'static, Async>>,
    cs: Output<'static>,
) {
    let spi_device = SpiDevice::new(spi, cs);
    let mut baro = BoardBaro::new(spi_device).await;
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



