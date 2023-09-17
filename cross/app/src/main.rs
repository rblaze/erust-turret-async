#![no_std]
#![no_main]

mod board;
mod error;
mod ranging;
mod system_time;
mod util;

use error::Error;
use num::rational::Ratio;
use system_time::{Duration, Ticker};
use util::sleep;

use core::cell::{Cell, OnceCell};
use core::pin::pin;
use core::sync::atomic::Ordering;

use async_scheduler::executor::{set_environment, Environment, Executor, LocalExecutor};
use cortex_m_rt::entry;
use futures::task::FutureObj;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::pac;

use panic_probe as _;
// use panic_halt as _;

struct Env {
    ticker: Ticker,
    current_executor: Cell<Option<&'static dyn Executor>>,
}

impl Env {
    const fn new(ticker: Ticker) -> Self {
        Self {
            ticker,
            current_executor: Cell::new(None),
        }
    }
}

impl Environment for Env {
    fn sleep_if_zero(&self, mask: &core::sync::atomic::AtomicU32) {
        critical_section::with(|_| {
            if mask.load(Ordering::Acquire) == 0 {
                // Critical section prevents interrupt handler from updating 'mask' here.
                // Pending interrupt will wake up CPU and exit critical section.
                self.ticker.wait_for_tick();
            }
        });
    }

    fn ticks(&self) -> u32 {
        self.ticker.get_ticks()
    }

    fn enter_executor(&self, executor: &dyn Executor) {
        if self.current_executor.get().is_some() {
            panic!("double-entering executor");
        }

        let r = unsafe { core::mem::transmute::<&dyn Executor, &'static dyn Executor>(executor) };
        self.current_executor.set(Some(r));
    }

    fn leave_executor(&self) {
        self.current_executor
            .replace(None)
            .expect("leaving executor without entering");
    }

    fn current_executor(&self) -> Option<&dyn Executor> {
        self.current_executor.get()
    }
}

struct EnvHolder {
    env: OnceCell<Env>,
}

// App is running single thread and interrupt handlers never use environment.
unsafe impl Sync for Env {}
unsafe impl Sync for EnvHolder {}

static ENV: EnvHolder = EnvHolder {
    env: OnceCell::new(),
};

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
    let _ranging_baseline = ranging::calibrate(servo_scale, sensor, sensor_servo).await?;

    loop {
        rprintln!("iteration");

        sleep(Duration::secs(1)).await;
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let board = board::Board::new(cp, dp).unwrap();

    set_environment(ENV.env.get_or_init(|| Env::new(board.ticker))).unwrap();

    let mut pinned_main = pin!(main_wrapper(
        board.adc_ratio,
        board.sensor,
        board.sensor_servo
    ));
    let task = FutureObj::new(&mut pinned_main);

    let mut executor: LocalExecutor = LocalExecutor::new();
    executor.spawn(task).unwrap();

    executor.run();

    panic!("unexpected exit from executor.run()")
}
