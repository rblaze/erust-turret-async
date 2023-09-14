use crate::error::Error;
use crate::system_time::Ticker;

use stm32f1xx_hal::pac;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::timer::Timer;

// Set max available clock frequency.
// Not important for CPU but audio PWM resolution is barely enough even this way.
// In hindsight, should have used chip with DAC.
const CLOCK_FREQ: u32 = 64_000_000;

pub struct Board {
    pub ticker: Ticker,
}

impl Board {
    pub fn new(cp: pac::CorePeripherals, dp: pac::Peripherals) -> Result<Self, Error> {
        // Enable debug while sleeping to keep probe-rs happy while WFI
        dp.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        dp.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());

        // Configure the clock.
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(Hertz::Hz(CLOCK_FREQ))
            .freeze(&mut flash.acr);

        let ticker = Ticker::new(Timer::syst(cp.SYST, &clocks));

        Ok(Board { ticker })
    }
}
