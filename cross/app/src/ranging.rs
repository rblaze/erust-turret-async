#![deny(unsafe_code)]

use calibration::Calibration;
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

pub struct Ranging {
    total_steps: usize,
    baseline: [u16; MAX_STEPS],
}

impl Ranging {
    pub async fn calibrate(
        scale: Ratio<u16>,
        mut sensor: board::Sensor,
        mut servo: board::SensorServo,
    ) -> Result<Self, Error> {
        sensor.set_timing_budget(TimingBudget::Ms100)?;
        sensor.set_distance_mode(DistanceMode::Long)?;
        sensor.set_inter_measurement(SENSOR_INTERMEASURMENT_TIME.convert())?;

        servo.set(Ratio::zero())?;
        sleep(SERVO_RESET_TIME).await;

        let mut ret = Self {
            total_steps: get_num_steps_from_angle_scale(scale)?,
            baseline: [0; MAX_STEPS],
        };

        for step in 0..ret.total_steps {
            let mut calibration_data = Calibration::new();

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
                calibration_data.add_sample(distance);
            }
            sensor.stop_ranging()?;
            servo.set(Ratio::new((step + 1) as u16, ret.total_steps as u16))?;

            let point = calibration_data.get_point();

            let buffer = 3 * point.stddev;
            let threshold = if point.mean > buffer {
                point.mean - buffer
            } else {
                0
            };
            debug_rprintln!("point {:?} threshold {}", point, threshold);

            ret.baseline[step] = threshold;

            sleep(SERVO_STEP_TIME).await;
        }

        Ok(ret)
    }
}

fn get_num_steps_from_angle_scale(scale: Ratio<u16>) -> Result<usize, Error> {
    if scale > Ratio::one() {
        return Err(Error::InvalidScale);
    }

    let long_scale = Ratio::new((*scale.numer()).into(), (*scale.denom()).into());
    let total_steps = (Ratio::from_integer(MAX_STEPS) * long_scale).to_integer();
    debug_rprintln!("using {} steps", total_steps);

    Ok(total_steps)
}
