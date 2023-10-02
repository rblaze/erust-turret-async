use crate::board::{SpiBus, SpiCs};

use core::cell::RefCell;
use spi_memory::Read;

type SpiMemory = spi_memory::series25::Flash<SpiBus, SpiCs>;
pub type StorageError = spi_memory::Error<SpiBus, SpiCs>;

pub struct Storage {
    flash: RefCell<SpiMemory>,
}

impl Storage {
    const FLASH_SIZE: usize = 2 * 1024 * 1024;

    pub fn new(spi: SpiBus, cs: SpiCs) -> Result<Self, simplefs::Error<StorageError>> {
        Ok(Self {
            flash: RefCell::new(SpiMemory::init(spi, cs)?),
        })
    }
}

impl simplefs::Storage for Storage {
    type Error = StorageError;

    fn capacity(&self) -> usize {
        Self::FLASH_SIZE
    }

    fn read(&self, off: usize, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.flash.borrow_mut().read(off as u32, buf)
    }
}
