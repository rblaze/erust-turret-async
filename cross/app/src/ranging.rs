#![deny(unsafe_code)]

use calibration::Calibration;
use num::rational::Ratio;
use num::{One, Zero};
use rtt_target::debug_rprintln;
use vl53l1x::{DistanceMode, TimingBudget};

use crate::audio::{Audio, Sound};
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

pub struct Ranging<'a> {
    audio: &'a Audio,
    sensor: board::Sensor,
    servo: board::SensorServo,
    total_steps: usize,
    baseline: [u16; MAX_STEPS],
}

impl<'a> Ranging<'a> {
    pub fn new(
        audio: &'a Audio,
        scale: Ratio<u16>,
        sensor: board::Sensor,
        servo: board::SensorServo,
    ) -> Result<Self, Error> {
        Ok(Self {
            audio,
            sensor,
            servo,
            total_steps: get_num_steps_from_angle_scale(scale)?,
            baseline: [0; MAX_STEPS],
        })
    }

    pub async fn calibrate(&mut self) -> Result<(), Error> {
        self.audio.play(Sound::Startup);

        self.sensor.set_timing_budget(TimingBudget::Ms100)?;
        self.sensor.set_distance_mode(DistanceMode::Long)?;
        self.sensor
            .set_inter_measurement(SENSOR_INTERMEASURMENT_TIME.convert())?;

        self.servo.set(Ratio::zero())?;
        sleep(SERVO_RESET_TIME).await;

        for step in 0..self.total_steps {
            self.servo
                .set(Ratio::new(step as u16, self.total_steps as u16))?;
            sleep(SERVO_STEP_TIME).await;

            let mut calibration_data = Calibration::new();

            self.sensor.start_ranging()?;
            for _ in 0..NUM_CALIBRATION_SAMPLES {
                sleep(SENSOR_INTERMEASURMENT_TIME).await;
                while !(self.sensor.check_for_data_ready()?) {
                    debug_rprintln!("sensor not ready");
                    // Try again shortly
                    sleep(SENSOR_RETRY_TIME).await;
                }

                let distance = self.sensor.get_distance()?;
                self.sensor.clear_interrupt()?;

                debug_rprintln!("calibration: {}", distance);
                calibration_data.add_sample(distance);
            }
            self.sensor.stop_ranging()?;

            let point = calibration_data.get_point();

            let buffer = 3 * point.stddev;
            let threshold = if point.mean > buffer {
                point.mean - buffer
            } else {
                0
            };
            debug_rprintln!("point {:?} threshold {}", point, threshold);

            self.baseline[step] = threshold;
        }

        Ok(())
    }

    /// Scans around with a sensor and sends detection events to targeting.
    pub async fn scan(&mut self) -> Result<(), Error> {
        // Servo already should be in this position but let's make sure.
        self.servo.set(Ratio::one())?;
        sleep(SERVO_RESET_TIME).await;

        loop {
            // Scan backward
            for step in (0..self.total_steps).rev() {
                self.servo
                    .set(Ratio::new(step as u16, self.total_steps as u16))?;
                sleep(SERVO_STEP_TIME).await;

                let distance = self.measure().await?;
                debug_rprintln!("step {} distance {}", step, distance);

                if distance < self.baseline[step] {
                    debug_rprintln!("detected");
                }
            }

            // Scan forward
            for step in 0..self.total_steps {
                self.servo
                    .set(Ratio::new(step as u16, self.total_steps as u16))?;
                sleep(SERVO_STEP_TIME).await;

                let distance = self.measure().await?;

                debug_rprintln!("step {} distance {}", step, distance);

                if distance < self.baseline[step] {
                    debug_rprintln!("detected");
                }
            }
        }
    }

    async fn measure(&mut self) -> Result<u16, Error> {
        self.sensor.start_ranging()?;
        sleep(SENSOR_TIMING_BUDGET).await;

        while !(self.sensor.check_for_data_ready()?) {
            debug_rprintln!("sensor not ready");
            // Try again shortly
            sleep(SENSOR_RETRY_TIME).await;
        }

        let distance = self.sensor.get_distance()?;
        self.sensor.clear_interrupt()?;
        self.sensor.stop_ranging()?;

        Ok(distance)
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
