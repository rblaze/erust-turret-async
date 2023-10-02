#![deny(unsafe_code)]

use core::num::TryFromIntError;

use crate::storage::StorageError;

#[derive(Debug)]
pub enum Error {
    AlreadyTaken,
    Conversion(TryFromIntError),
    Environment(async_scheduler::executor::EnvError),
    ExecutorSpawn(async_scheduler::executor::SpawnError),
    FileSystem(simplefs::Error<StorageError>),
    InvalidDuration,
    InvalidScale,
    Mailbox(async_scheduler::mailbox::Error),
    Sensor(vl53l1x::Error<stm32f1xx_hal::i2c::Error>),
    Servo(servo::Error),
    Timer(stm32f1xx_hal::timer::Error),
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

impl From<async_scheduler::executor::SpawnError> for Error {
    fn from(error: async_scheduler::executor::SpawnError) -> Self {
        Error::ExecutorSpawn(error)
    }
}

impl From<async_scheduler::mailbox::Error> for Error {
    fn from(error: async_scheduler::mailbox::Error) -> Self {
        Error::Mailbox(error)
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

impl From<simplefs::Error<StorageError>> for Error {
    fn from(fs_error: simplefs::Error<StorageError>) -> Self {
        Error::FileSystem(fs_error)
    }
}

impl From<stm32f1xx_hal::timer::Error> for Error {
    fn from(timer_error: stm32f1xx_hal::timer::Error) -> Self {
        Error::Timer(timer_error)
    }
}
