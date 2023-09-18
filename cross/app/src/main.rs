#![no_std]
#![no_main]

mod board;
mod env;
mod error;
mod ranging;
mod system_time;
mod util;

use crate::error::Error;

use core::pin::pin;

use async_scheduler::executor::LocalExecutor;
use cortex_m_rt::entry;
use futures::task::LocalFutureObj;
use num::rational::Ratio;
use rtt_target::rtt_init_print;
use stm32f1xx_hal::pac;

use panic_probe as _;
// use panic_halt as _;

async fn main_wrapper(
    servo_scale: Ratio<u16>,
    sensor: board::Sensor,
    sensor_servo: board::SensorServo,
) {
    async_main(servo_scale, sensor, sensor_servo)
        .await
        .expect("error in async_main");
}

async fn async_main(
    servo_scale: Ratio<u16>,
    sensor: board::Sensor,
    sensor_servo: board::SensorServo,
) -> Result<(), Error> {
    let mut ranging = ranging::Ranging::new(servo_scale, sensor, sensor_servo)?;
    ranging.calibrate().await?;
    ranging.scan().await?;

    Ok(())
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let board = board::Board::new(cp, dp).unwrap();

    env::init_env(board.ticker).unwrap();

    let mut pinned_main = pin!(main_wrapper(
        board.adc_ratio,
        board.sensor,
        board.sensor_servo
    ));
    let task = LocalFutureObj::new(&mut pinned_main);

    let mut executor: LocalExecutor = LocalExecutor::new();
    executor.spawn(task).unwrap();

    executor.run();

    panic!("unexpected exit from executor.run()")
}
