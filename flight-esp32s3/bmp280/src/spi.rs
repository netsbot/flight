//! BME280 driver for sensors attached via SPI.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;

use super::{BME280Common, Interface};
use super::{
    Configuration, Error, IIRFilter, Measurements, Oversampling,
    BME280_P_T_CALIB_DATA_LEN, BME280_P_T_DATA_LEN,
};

/// Representation of a BMP280
#[derive(Debug, Default)]
pub struct Bmp280<SPI, D> {
    common: BME280Common<SPIInterface<SPI>>,
    delay: D,
}

impl<SPI, SPIE, D> Bmp280<SPI, D>
where
    SPI: SpiDevice<Error = SPIE>,
    D: DelayNs,
{
    /// Create a new Bmp280 struct and initialize it with default configuration
    pub async fn new(
        spi: SPI,
        delay: D,
    ) -> Result<Self, Error<SPIError<SPIE>>> {
        Self::new_with_config(
            spi,
            delay,
            Configuration::default()
                .with_pressure_oversampling(Oversampling::Oversampling16X)
                .with_temperature_oversampling(Oversampling::Oversampling2X)
                .with_iir_filter(IIRFilter::Coefficient16),
        )
        .await
    }

    /// Create a new Bmp280 struct and initialize it with custom configuration
    pub async fn new_with_config(
        spi: SPI,
        mut delay: D,
        config: Configuration,
    ) -> Result<Self, Error<SPIError<SPIE>>> {
        let mut common = BME280Common {
            interface: SPIInterface { spi },
            calibration: None,
            config,
        };
        common.init(&mut delay, config).await?;
        Ok(Self { common, delay })
    }

    /// Re-initialize the sensor applying the given configuration.
    pub async fn reconfigure(
        &mut self,
        config: Configuration,
    ) -> Result<(), Error<SPIError<SPIE>>> {
        self.common.init(&mut self.delay, config).await
    }

    /// Captures and processes sensor data for temperature and pressure
    pub async fn measure(
        &mut self,
    ) -> Result<Measurements<SPIError<SPIE>>, Error<SPIError<SPIE>>> {
        self.common.measure(&mut self.delay).await
    }

    /// Returns the calibration data read from the sensor
    pub fn calibration(&self) -> Option<&super::CalibrationData> {
        self.common.calibration()
    }
}

/// Register access functions for SPI
#[derive(Debug, Default)]
pub(crate) struct SPIInterface<SPI> {
    /// concrete SPI device implementation
    spi: SPI,
}

impl<SPI> Interface for SPIInterface<SPI>
where
    SPI: SpiDevice,
{
    type Error = SPIError<SPI::Error>;

    async fn read_register(&mut self, register: u8) -> Result<u8, Error<Self::Error>> {
        let mut result = [0u8];
        self.read_any_register(register, &mut result).await?;
        Ok(result[0])
    }

    async fn read_data(
        &mut self,
        register: u8,
    ) -> Result<[u8; BME280_P_T_DATA_LEN], Error<Self::Error>> {
        let mut data = [0; BME280_P_T_DATA_LEN];
        self.read_any_register(register, &mut data).await?;
        Ok(data)
    }

    async fn read_pt_calib_data(
        &mut self,
        register: u8,
    ) -> Result<[u8; BME280_P_T_CALIB_DATA_LEN], Error<Self::Error>> {
        let mut data = [0; BME280_P_T_CALIB_DATA_LEN];
        self.read_any_register(register, &mut data).await?;
        Ok(data)
    }

    async fn write_register(&mut self, register: u8, payload: u8) -> Result<(), Error<Self::Error>> {
        // In SPI mode, write bit is 0 (register & 0x7f)
        let header = [register & 0x7f, payload];
        self.spi
            .write(&header)
            .await
            .map_err(|e| Error::Bus(SPIError::SPI(e)))?;
        Ok(())
    }
}

impl<SPI> SPIInterface<SPI>
where
    SPI: SpiDevice,
{
    async fn read_any_register(
        &mut self,
        register: u8,
        data: &mut [u8],
    ) -> Result<(), Error<SPIError<SPI::Error>>> {
        // In SPI mode, read bit is 1 (register | 0x80)
        let cmd = [register | 0x80];
        self.spi
            .transaction(&mut [
                embedded_hal_async::spi::Operation::Write(&cmd),
                embedded_hal_async::spi::Operation::Read(data),
            ])
            .await
            .map_err(|e| Error::Bus(SPIError::SPI(e)))?;
        Ok(())
    }
}

/// Error which occurred during an SPI transaction
#[derive(Clone, Copy, Debug)]
pub enum SPIError<SPIE> {
    /// The SPI implementation returned an error
    SPI(SPIE),
}
