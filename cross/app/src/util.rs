#![deny(unsafe_code)]

use crate::system_time::Duration;

pub async fn sleep(duration: Duration) {
    async_scheduler::executor::sleep(duration.ticks()).await;
}
