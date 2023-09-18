#![deny(unsafe_code)]

use core::num::TryFromIntError;

#[derive(Debug)]
pub enum Error {
    Conversion(TryFromIntError),
    Environment(async_scheduler::executor::EnvError),
    InvalidDuration,
    InvalidScale,
    Sensor(vl53l1x::Error<stm32f1xx_hal::i2c::Error>),
    Servo(servo::Error),
    UnexpectedlyBlocks,
}

impl From<TryFromIntError> for Error {
    fn from(error: TryFromIntError) -> Self {
        Error::Conversion(error)
    }
}

impl From<async_scheduler::executor::EnvError> for Error {
    fn from(error: async_scheduler::executor::EnvError) -> Self {
        Error::Environment(error)
    }
}

impl From<vl53l1x::Error<stm32f1xx_hal::i2c::Error>> for Error {
    fn from(sensor_error: vl53l1x::Error<stm32f1xx_hal::i2c::Error>) -> Self {
        Error::Sensor(sensor_error)
    }
}

impl From<servo::Error> for Error {
    fn from(servo_error: servo::Error) -> Self {
        Error::Servo(servo_error)
    }
}

impl From<nb::Error<()>> for Error {
    fn from(_: nb::Error<()>) -> Self {
        Error::UnexpectedlyBlocks
    }
}
