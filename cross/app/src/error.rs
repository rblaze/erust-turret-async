#![deny(unsafe_code)]

#[derive(Debug)]
pub enum Error {
    Sensor(vl53l1x::Error<stm32f1xx_hal::i2c::Error>),
}

impl From<vl53l1x::Error<stm32f1xx_hal::i2c::Error>> for Error {
    fn from(sensor_error: vl53l1x::Error<stm32f1xx_hal::i2c::Error>) -> Self {
        Error::Sensor(sensor_error)
    }
}
