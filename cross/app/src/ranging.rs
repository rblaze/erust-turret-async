#![deny(unsafe_code)]

use calibration::Calibration;
use num::rational::Ratio;
use num::{One, Zero};
use rtt_target::debug_rprintln;
use vl53l1x::{DistanceMode, TimingBudget};

use crate::audio::{Audio, Sound};
use crate::board;
use crate::error::Error;
use crate::system_time::{Duration, Instant, Ticker};
use crate::util::sleep;

const MAX_STEPS: usize = 100;
const NUM_CALIBRATION_SAMPLES: u16 = 5;

const SENSOR_TIMING_BUDGET: Duration = Duration::millis(100);
const SENSOR_INTERMEASURMENT_TIME: Duration = Duration::millis(120);
const SENSOR_RETRY_TIME: Duration = Duration::millis(10);
const SERVO_RESET_TIME: Duration = Duration::millis(500);
const SERVO_STEP_TIME: Duration = Duration::millis(100);

const MIN_TARGET_LOCK_STEPS: i32 = 8;
const MAX_LOCK_BREAK_STEPS: i32 = 4;

const LASER_OFF_DELAY: Duration = Duration::secs(5);
const TARGET_LOST_DELAY: Duration = Duration::secs(60);
const TARGET_ACQUIRED_INTERVAL: Duration = Duration::secs(30);

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum TargetState {
    NoContact,
    EarlyContact {
        start_position: i32,
    },
    Lock {
        start_position: i32,
        end_position: i32,
    },
}

impl TargetState {
    fn report_contact(self, step: u16) -> Self {
        let position = step as i32;
        match self {
            TargetState::NoContact => Self::EarlyContact {
                start_position: position,
            },
            TargetState::EarlyContact { start_position } => {
                let contact_length = num::abs(position - start_position);
                if contact_length >= MIN_TARGET_LOCK_STEPS {
                    Self::Lock {
                        start_position,
                        end_position: position,
                    }
                } else {
                    self
                }
            }
            TargetState::Lock { start_position, .. } => Self::Lock {
                start_position,
                end_position: position,
            },
        }
    }

    fn report_no_contact(self, step: u16) -> Self {
        let position = step as i32;

        match self {
            TargetState::NoContact => self,
            TargetState::EarlyContact { .. } => Self::NoContact,
            TargetState::Lock { end_position, .. } => {
                let contact_gap = num::abs(position - end_position);
                if contact_gap > MAX_LOCK_BREAK_STEPS {
                    Self::NoContact
                } else {
                    self
                }
            }
        }
    }

    fn has_lock(&self) -> bool {
        match self {
            TargetState::NoContact => false,
            TargetState::EarlyContact { .. } => false,
            TargetState::Lock { .. } => true,
        }
    }

    fn midrange(&self) -> Option<u16> {
        match self {
            TargetState::NoContact => None,
            TargetState::EarlyContact { .. } => None,
            TargetState::Lock {
                start_position,
                end_position,
            } => Some((start_position + (end_position - start_position) / 2) as u16),
        }
    }

    fn flip(self, position: u16) -> Self {
        match self {
            TargetState::NoContact => self,
            TargetState::EarlyContact { .. } => TargetState::NoContact,
            TargetState::Lock { .. } => TargetState::Lock {
                start_position: position as i32,
                end_position: position as i32,
            },
        }
    }
}

pub struct Ranging<'a> {
    audio: &'a Audio,
    ticker: Ticker,
    sensor: board::Sensor,
    sensor_servo: board::SensorServo,
    laser: board::Laser,
    laser_servo: board::LaserServo,
    target_lock_led: board::Led,
    laser_off_time: Option<Instant>,
    target_lost_time: Option<Instant>,
    last_lock_time: Instant,
    total_steps: u16,
    baseline: [u16; MAX_STEPS],
}

impl<'a> Ranging<'a> {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        audio: &'a Audio,
        ticker: Ticker,
        scale: Ratio<u16>,
        sensor: board::Sensor,
        sensor_servo: board::SensorServo,
        laser: board::Laser,
        laser_servo: board::LaserServo,
        target_lock_led: board::Led,
    ) -> Result<Self, Error> {
        Ok(Self {
            audio,
            ticker,
            sensor,
            sensor_servo,
            laser,
            laser_servo,
            target_lock_led,
            laser_off_time: None,
            target_lost_time: None,
            last_lock_time: Instant::from_ticks(0),
            total_steps: Self::get_num_steps_from_angle_scale(scale)?,
            baseline: [0; MAX_STEPS],
        })
    }

    pub async fn calibrate(&mut self) -> Result<(), Error> {
        self.audio.play(Sound::Startup);

        self.laser.set_low();
        self.target_lock_led.set_low();

        self.sensor.set_timing_budget(TimingBudget::Ms100)?;
        self.sensor.set_distance_mode(DistanceMode::Long)?;
        self.sensor
            .set_inter_measurement(SENSOR_INTERMEASURMENT_TIME.convert())?;

        self.sensor_servo.set(Ratio::zero())?;
        sleep(SERVO_RESET_TIME).await;

        for step in 0..self.total_steps {
            self.sensor_servo.set(Ratio::new(step, self.total_steps))?;
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
            let threshold = point.mean.saturating_sub(buffer);
            debug_rprintln!("point {:?} threshold {}", point, threshold);

            self.baseline[step as usize] = threshold;
        }

        Ok(())
    }

    /// Scans around with a sensor and sends detection events to targeting.
    pub async fn scan(&mut self) -> Result<(), Error> {
        self.audio.play(Sound::BeginScan);

        // Servo already should be in this position but let's make sure.
        self.sensor_servo.set(Ratio::one())?;
        sleep(SERVO_RESET_TIME).await;

        let mut state = TargetState::NoContact;

        loop {
            // Scan backward
            state = state.flip(self.total_steps - 1);
            for step in (0..self.total_steps).rev() {
                state = self.scan_step(state, step).await?;
            }

            // Scan forward
            state = state.flip(0);
            for step in 0..self.total_steps {
                state = self.scan_step(state, step).await?;
            }
        }
    }

    async fn scan_step(&mut self, state: TargetState, step: u16) -> Result<TargetState, Error> {
        let had_lock = state.has_lock();

        self.sensor_servo.set(Ratio::new(step, self.total_steps))?;
        sleep(SERVO_STEP_TIME).await;

        let distance = self.measure().await?;

        let detected = distance < self.baseline[step as usize];
        debug_rprintln!("step {} distance {} {}", step, distance, detected);

        let now = self.ticker.now();
        let new_state;

        if detected {
            self.target_lock_led.set_high();

            new_state = state.report_contact(step);

            if let Some(laser_position) = new_state.midrange() {
                self.laser_servo
                    .set(Ratio::new(laser_position, self.total_steps))?;
                self.laser.set_high();
                self.laser_off_time = Some(now + LASER_OFF_DELAY);
                self.target_lost_time = None;

                if !had_lock {
                    // Switching from no-lock to lock
                    let sound = if now - self.last_lock_time >= TARGET_ACQUIRED_INTERVAL {
                        Sound::TargetAcquired
                    } else {
                        Sound::ContactRestored
                    };

                    self.audio.play(sound);
                }
            }
        } else {
            self.target_lock_led.set_low();
            new_state = state.report_no_contact(step);
            if had_lock && !new_state.has_lock() {
                // Switching from lock to no-lock
                self.last_lock_time = now;
            }
        }

        self.check_times(now);
        Ok(new_state)
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

    fn check_times(&mut self, now: Instant) {
        if self.laser_off_time.is_some_and(|t| t < now) {
            self.laser_off_time = None;
            self.target_lost_time = Some(now + TARGET_LOST_DELAY);
            self.laser.set_low();
            self.audio.play(Sound::ContactLost);
        }

        if self.target_lost_time.is_some_and(|t| t < now) {
            self.target_lost_time = None;
            self.audio.play(Sound::TargetLost);
        }
    }

    fn get_num_steps_from_angle_scale(scale: Ratio<u16>) -> Result<u16, Error> {
        if scale > Ratio::one() {
            return Err(Error::InvalidScale);
        }

        let long_scale = Ratio::new((*scale.numer()).into(), (*scale.denom()).into());
        let total_steps = (Ratio::from_integer(MAX_STEPS) * long_scale).to_integer();
        debug_rprintln!("using {} steps", total_steps);

        Ok(total_steps as u16)
    }
}
