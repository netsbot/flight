use nalgebra::ComplexField;
use bmp280::{Configuration, Oversampling::Oversampling4X, spi::Bmp280};
use esp_hal::delay::Delay;

pub(crate) struct BoardBaro<SPI: embedded_hal::spi::SpiDevice> {
    inner: Bmp280<SPI, Delay>,
}

impl<SPI: embedded_hal::spi::SpiDevice> BoardBaro<SPI> {
    pub fn new(spi_device: SPI) -> Self {
        let config = Configuration::default().with_pressure_oversampling(Oversampling4X);
        let baro = Bmp280::new_with_config(spi_device, Delay::new(), config).unwrap();

        Self { inner: baro }
    }

    pub fn read_pressure(&mut self) -> f32 {
        let data = self.inner.measure().unwrap();
        data.pressure
    }

    pub fn read_altitude_msl(&mut self) -> f32 {
        const SCALE_FACTOR: f32 = 44330.77;
        const EXPONENT: f32 = 0.190263;

        SCALE_FACTOR * (1.0 - (self.read_pressure() / 1013.25).powf(EXPONENT))
    }
}
