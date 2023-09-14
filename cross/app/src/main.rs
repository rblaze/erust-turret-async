#![no_std]
#![no_main]

use cortex_m::asm::wfi;
use cortex_m_rt::entry;
use rtt_target::rtt_init_print;
use stm32f1xx_hal::pac;

use panic_probe as _;
// use panic_halt as _;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let _cp = pac::CorePeripherals::take().unwrap();
    let _dp = pac::Peripherals::take().unwrap();

    loop {
        wfi();
    }
}
