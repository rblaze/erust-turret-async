#![deny(unsafe_code)]

use crate::error::Error;
use crate::system_time::Ticker;

use stm32f1xx_hal::gpio::{Alternate, OpenDrain};
use stm32f1xx_hal::gpio::{PB6, PB7};
use stm32f1xx_hal::i2c::{BlockingI2c, I2c, Mode};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::timer::Timer;
use vl53l1x::{BootState, VL53L1X};

// Set max available clock frequency.
// Not important for CPU but audio PWM resolution is barely enough even this way.
// In hindsight, should have used chip with DAC.
const CLOCK_FREQ: u32 = 64_000_000;

pub type I2cScl = PB6<Alternate<OpenDrain>>;
pub type I2cSda = PB7<Alternate<OpenDrain>>;
pub type I2cBus = BlockingI2c<pac::I2C1, (I2cScl, I2cSda)>;

pub type Sensor = VL53L1X<I2cBus>;

pub struct Board {
    pub ticker: Ticker,
    pub sensor: Sensor,
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

        let mut afio = dp.AFIO.constrain();
        let mut gpiob = dp.GPIOB.split();

        // Configure I2C.
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let i2c = I2c::i2c1(
            dp.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::standard(100.kHz()),
            clocks,
        )
        .blocking_default(clocks);

        // Configure ranging sensor.
        let mut sensor = VL53L1X::new(i2c, vl53l1x::ADDR);
        while sensor.boot_state()? != BootState::Booted {
            // Wait 10 ms until next timer tick.
            ticker.wait_for_tick();
        }
        sensor.sensor_init()?;

        Ok(Board { ticker, sensor })
    }
}
