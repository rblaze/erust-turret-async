#![deny(unsafe_code)]

use num::rational::Ratio;
use num::{One, Zero};
use rtt_target::debug_rprintln;
use vl53l1x::{DistanceMode, TimingBudget};

use crate::board;
use crate::error::Error;
use crate::system_time::Duration;
use crate::util::sleep;

const MAX_STEPS: usize = 100;
const NUM_CALIBRATION_SAMPLES: u16 = 5;

const SENSOR_TIMING_BUDGET: Duration = Duration::millis(100);
const SENSOR_INTERMEASURMENT_TIME: Duration = Duration::millis(120);
const SENSOR_RETRY_TIME: Duration = Duration::millis(10);
const SERVO_RESET_TIME: Duration = Duration::millis(500);
const SERVO_STEP_TIME: Duration = Duration::millis(100);

pub async fn calibrate(
    scale: Ratio<u16>,
    mut sensor: board::Sensor,
    mut servo: board::SensorServo,
) -> Result<u32, Error> {
    sensor.set_timing_budget(TimingBudget::Ms100)?;
    sensor.set_distance_mode(DistanceMode::Long)?;
    sensor.set_inter_measurement(SENSOR_INTERMEASURMENT_TIME.convert())?;

    servo.set(Ratio::zero())?;
    sleep(SERVO_RESET_TIME).await;

    let num_steps = get_num_steps_from_angle_scale(scale)?;

    for step in 0..num_steps {
        sensor.start_ranging()?;
        for _ in 0..NUM_CALIBRATION_SAMPLES {
            sleep(SENSOR_TIMING_BUDGET).await;
            while !(sensor.check_for_data_ready()?) {
                debug_rprintln!("sensor not ready");
                // Try again shortly
                sleep(SENSOR_RETRY_TIME).await;
            }

            let distance = sensor.get_distance()?;
            sensor.clear_interrupt()?;

            debug_rprintln!("calibration: {}", distance);
            // TODO process calibration data
        }
        // TODO finalize step data

        sensor.stop_ranging()?;
        servo.set(Ratio::new((step + 1) as u16, num_steps as u16))?;
        sleep(SERVO_STEP_TIME).await;
    }

    // TODO return baseline data
    Ok(42)
}

pub fn get_num_steps_from_angle_scale(scale: Ratio<u16>) -> Result<usize, Error> {
    if scale > Ratio::one() {
        return Err(Error::InvalidScale);
    }

    let long_scale = Ratio::new((*scale.numer()).into(), (*scale.denom()).into());
    let total_steps = (Ratio::from_integer(MAX_STEPS) * long_scale).to_integer();
    debug_rprintln!("using {} steps", total_steps);

    Ok(total_steps)
}
