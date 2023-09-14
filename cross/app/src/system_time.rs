#![deny(unsafe_code)]

use core::cell::Cell;
use cortex_m_rt::exception;
use critical_section::Mutex;
use fugit::RateExtU32;
use stm32f1xx_hal::pac::SYST;
use stm32f1xx_hal::timer::{SysEvent, Timer};

const HERTZ: u32 = 100;

#[allow(unused)]
pub type Instant = fugit::TimerInstantU32<HERTZ>;
pub type Duration = fugit::TimerDurationU32<HERTZ>;

static TICKS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

#[derive(Clone, Copy, Debug)]
pub struct Ticker {}

impl Ticker {
    // Setup SysTick to tick at 100Hz
    pub fn new(syst: Timer<SYST>) -> Self {
        let mut counter = syst.counter_hz();

        counter.start(HERTZ.Hz()).unwrap();
        counter.listen(SysEvent::Update);

        Ticker {}
    }

    // Get current tick count.
    pub fn get_ticks(&self) -> u32 {
        critical_section::with(|cs| TICKS.borrow(cs).get())
    }

    // Get current timestamp.
    #[allow(unused)]
    pub fn now(&self) -> Instant {
        let ticks = self.get_ticks();
        Instant::from_ticks(ticks)
    }

    // Wait for the next tick.
    // Makes sure the ticker is enabled.
    pub fn wait_for_tick(&self) {
        cortex_m::asm::wfi();
    }
}

#[exception]
fn SysTick() {
    critical_section::with(|cs| {
        let ticks = TICKS.borrow(cs).get();
        TICKS.borrow(cs).set(ticks + 1);
    });
}
