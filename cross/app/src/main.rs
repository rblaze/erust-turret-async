#![no_std]
#![no_main]

mod audio;
mod board;
mod env;
mod error;
mod ranging;
mod storage;
mod system_time;
mod util;

use crate::error::Error;

use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use cortex_m_rt::entry;
use futures::task::LocalFutureObj;
use rtt_target::rtt_init_print;
use stm32f1xx_hal::pac;

use panic_probe as _;
// use panic_halt as _;

async fn panic_if_exited<F: core::future::Future<Output = Result<(), Error>>>(f: F) {
    f.await.expect("error in task");
    unreachable!()
}

#[entry]
fn main() -> ! {
    move || -> Result<_, Error> {
        rtt_init_print!();

        let cp = pac::CorePeripherals::take().ok_or(Error::AlreadyTaken)?;
        let dp = pac::Peripherals::take().ok_or(Error::AlreadyTaken)?;
        let board = board::Board::new(cp, dp)?;

        let audio = audio::Audio::new();
        let mut ranging =
            ranging::Ranging::new(&audio, board.adc_ratio, board.sensor, board.sensor_servo)?;

        env::init_env(board.ticker)?;

        let audio_task = pin!(panic_if_exited(audio.task(
            board.storage,
            board.audio_enable,
            board.audio_pwm,
            board.audio_clock,
            board.audio_dma,
            board.random,
        )));

        let ranging_task = pin!(panic_if_exited(async {
            ranging.calibrate().await?;
            ranging.scan().await?;
            unreachable!()
        }));

        let mut executor: LocalExecutor<2> = LocalExecutor::new();
        executor.spawn(LocalFutureObj::new(audio_task))?;
        executor.spawn(LocalFutureObj::new(ranging_task))?;

        executor.run();
        unreachable!()
    }()
    .expect("error in main")
}
