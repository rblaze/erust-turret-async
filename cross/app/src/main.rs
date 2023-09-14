#![no_std]
#![no_main]

mod board;
mod error;
mod system_time;

use system_time::{Duration, Ticker};

use core::cell::Cell;
use core::pin::pin;
use core::sync::atomic::Ordering;

use async_scheduler::executor::{set_environment, sleep, Environment, Executor, LocalExecutor};
use cortex_m_rt::entry;
use futures::task::FutureObj;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::pac;

use panic_probe as _;
// use panic_halt as _;

struct Env {
    ticker: Cell<Option<Ticker>>,
    current_executor: Cell<Option<&'static dyn Executor>>,
}

// App is running single thread and interrupt handlers never use environment.
unsafe impl Sync for Env {}

impl Env {
    const fn new() -> Self {
        Self {
            ticker: Cell::new(None),
            current_executor: Cell::new(None),
        }
    }

    fn set_ticker(&self, ticker: Ticker) {
        self.ticker.set(Some(ticker));
    }

    fn ticker(&self) -> Ticker {
        self.ticker.get().expect("ticker not set")
    }
}

impl Environment for Env {
    fn sleep_if_zero(&self, mask: &core::sync::atomic::AtomicU32) {
        critical_section::with(|_| {
            if mask.load(Ordering::Acquire) == 0 {
                // Critical section prevents interrupt handler from updating 'mask' here.
                // Pending interrupt will wake up CPU and exit critical section.
                self.ticker().wait_for_tick();
            }
        });
    }

    fn ticks(&self) -> u32 {
        self.ticker().get_ticks()
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

async fn print_loop() {
    rprintln!("iteration");

    sleep(Duration::secs(1).ticks()).await;
}

static ENV: Env = Env::new();

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let board = board::Board::new(cp, dp).unwrap();
    ENV.set_ticker(board.ticker);

    set_environment(&ENV).unwrap();

    let mut pinned_loop = pin!(print_loop());
    let task = FutureObj::new(&mut pinned_loop);

    let mut executor: LocalExecutor = LocalExecutor::new();
    executor.spawn(task).unwrap();

    executor.run();

    panic!("unexpected exit from executor.run()")
}
