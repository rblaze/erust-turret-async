use core::cell::Cell;
use core::sync::atomic::Ordering;

use async_scheduler::executor::{set_environment, Environment, Executor};
use cortex_m::peripheral::{scb::VectActive, SCB};
use once_cell::sync::OnceCell;

use crate::{error::Error, system_time::Ticker};

fn in_thread_mode() -> bool {
    match SCB::vect_active() {
        VectActive::ThreadMode => true,
        VectActive::Exception(_) => false,
        VectActive::Interrupt { .. } => false,
    }
}

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
        debug_assert!(in_thread_mode(), "calling environment in interrupt handler");

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
        debug_assert!(in_thread_mode(), "calling environment in interrupt handler");

        if self.current_executor.get().is_some() {
            panic!("double-entering executor");
        }

        let r = unsafe { core::mem::transmute::<&dyn Executor, &'static dyn Executor>(executor) };
        self.current_executor.set(Some(r));
    }

    fn leave_executor(&self) {
        debug_assert!(in_thread_mode(), "calling environment in interrupt handler");

        self.current_executor
            .replace(None)
            .expect("leaving executor without entering");
    }

    fn current_executor(&self) -> Option<&dyn Executor> {
        debug_assert!(in_thread_mode(), "calling environment in interrupt handler");

        self.current_executor.get()
    }
}

// App is running single thread and interrupt handlers never use environment.
unsafe impl Sync for Env {}
unsafe impl Send for Env {}

static ENV: OnceCell<Env> = OnceCell::new();

pub fn init_env(ticker: Ticker) -> Result<(), Error> {
    set_environment(ENV.get_or_init(|| Env::new(ticker)))?;

    Ok(())
}
