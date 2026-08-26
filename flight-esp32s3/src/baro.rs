use bmp280::{Configuration, IIRFilter, Oversampling, spi::Bmp280};
use embassy_time::Delay;
use nalgebra::ComplexField;

pub(crate) struct BoardBaro<SPI: embedded_hal_async::spi::SpiDevice> {
    inner: Bmp280<SPI, Delay>,
}

impl<SPI: embedded_hal_async::spi::SpiDevice> BoardBaro<SPI> {
    pub async fn new(spi_device: SPI) -> Self {
        let config = Configuration::default()
            .with_temperature_oversampling(Oversampling::Oversampling2X)
            .with_pressure_oversampling(Oversampling::Oversampling16X)
            .with_iir_filter(IIRFilter::Coefficient16);
        let baro = Bmp280::new_with_config(spi_device, Delay, config)
            .await
            .unwrap();

        Self { inner: baro }
    }

    pub async fn read_pressure(&mut self) -> f32 {
        let data = self.inner.measure().await.unwrap();
        data.pressure
    }

    pub async fn read_altitude_msl(&mut self, qnh: f32) -> f32 {
        const SCALE_FACTOR: f32 = 44330.77;
        const EXPONENT: f32 = 0.190263;

        SCALE_FACTOR * (1.0 - (self.read_pressure().await / qnh).powf(EXPONENT))
    }
}
