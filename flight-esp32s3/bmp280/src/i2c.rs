//! BME280 driver for sensors attached via I2C.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::I2c;
use embedded_hal::i2c::ErrorType;

use super::{BME280Common, Interface};
use super::{
    Configuration, Error, IIRFilter, Measurements, Oversampling,
    BME280_P_T_CALIB_DATA_LEN, BME280_P_T_DATA_LEN,
};

const BME280_I2C_ADDR_PRIMARY: u8 = 0x76;
const BME280_I2C_ADDR_SECONDARY: u8 = 0x77;

/// Representation of a BME280
#[derive(Debug, Default)]
pub struct BME280<I2C, D> {
    common: BME280Common<I2CInterface<I2C>>,
    delay: D,
}

impl<I2C, D> BME280<I2C, D>
where
    I2C: I2c + ErrorType,
    D: DelayNs,
{
    /// Create a new BME280 struct using the primary I²C address `0x76` and initialize it
    pub async fn new_primary(
        i2c: I2C,
        delay: D,
    ) -> Result<Self, Error<I2C::Error>> {
        Self::new(i2c, BME280_I2C_ADDR_PRIMARY, delay).await
    }

    /// Create a new BME280 struct using the secondary I²C address `0x77` and initialize it
    pub async fn new_secondary(
        i2c: I2C,
        delay: D,
    ) -> Result<Self, Error<I2C::Error>> {
        Self::new(i2c, BME280_I2C_ADDR_SECONDARY, delay).await
    }

    /// Create a new BME280 struct using a custom I²C address and initialize it with default config
    pub async fn new(
        i2c: I2C,
        address: u8,
        delay: D,
    ) -> Result<Self, Error<I2C::Error>> {
        Self::new_with_config(
            i2c,
            address,
            delay,
            Configuration::default()
                .with_pressure_oversampling(Oversampling::Oversampling16X)
                .with_temperature_oversampling(Oversampling::Oversampling2X)
                .with_iir_filter(IIRFilter::Coefficient16),
        )
        .await
    }

    /// Create a new BME280 struct using a custom I²C address and initialize it with custom config
    pub async fn new_with_config(
        i2c: I2C,
        address: u8,
        mut delay: D,
        config: Configuration,
    ) -> Result<Self, Error<I2C::Error>> {
        let mut common = BME280Common {
            interface: I2CInterface { i2c, address },
            calibration: None,
        };
        common.init(&mut delay, config).await?;
        Ok(Self { common, delay })
    }

    /// Re-initialize the sensor applying the given configuration.
    pub async fn reconfigure(
        &mut self,
        config: Configuration,
    ) -> Result<(), Error<I2C::Error>> {
        self.common.init(&mut self.delay, config).await
    }

    /// Captures and processes sensor data for temperature and pressure
    pub async fn measure(
        &mut self,
    ) -> Result<Measurements<I2C::Error>, Error<I2C::Error>> {
        self.common.measure(&mut self.delay).await
    }
}

/// Register access functions for I2C
#[derive(Debug, Default)]
pub(crate) struct I2CInterface<I2C> {
    /// concrete I²C device implementation
    i2c: I2C,
    /// I²C device address
    address: u8,
}

impl<I2C> Interface for I2CInterface<I2C>
where
    I2C: I2c + ErrorType,
{
    type Error = I2C::Error;

    async fn read_register(&mut self, register: u8) -> Result<u8, Error<I2C::Error>> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .await
            .map_err(Error::Bus)?;
        Ok(data[0])
    }

    async fn read_data(
        &mut self,
        register: u8,
    ) -> Result<[u8; BME280_P_T_DATA_LEN], Error<I2C::Error>> {
        let mut data = [0; BME280_P_T_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .await
            .map_err(Error::Bus)?;
        Ok(data)
    }

    async fn read_pt_calib_data(
        &mut self,
        register: u8,
    ) -> Result<[u8; BME280_P_T_CALIB_DATA_LEN], Error<I2C::Error>> {
        let mut data = [0; BME280_P_T_CALIB_DATA_LEN];
        self.i2c
            .write_read(self.address, &[register], &mut data)
            .await
            .map_err(Error::Bus)?;
        Ok(data)
    }

    async fn write_register(&mut self, register: u8, payload: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .write(self.address, &[register, payload])
            .await
            .map_err(Error::Bus)
    }
}
