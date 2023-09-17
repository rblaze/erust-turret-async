#![deny(unsafe_code)]

use crate::error::Error;
use crate::system_time::Ticker;

use num::rational::Ratio;
use rtt_target::debug_rprintln;
use servo::{Bounds, Servo};
use stm32f1xx_hal::adc::Adc;
use stm32f1xx_hal::gpio::{Alternate, OpenDrain, PushPull};
use stm32f1xx_hal::gpio::{PA8, PA9, PB6, PB7};
use stm32f1xx_hal::i2c::{BlockingI2c, I2c, Mode};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::time::{Hertz, MilliSeconds};
use stm32f1xx_hal::timer::{PwmChannel, Timer};
use vl53l1x::{BootState, VL53L1X};

// Set max available clock frequency.
// Not important for CPU but audio PWM resolution is barely enough even this way.
// In hindsight, should have used chip with DAC.
const CLOCK_FREQ: u32 = 64_000_000;
const SERVO_FREQ: Hertz = Hertz::Hz(50);

pub type I2cScl = PB6<Alternate<OpenDrain>>;
pub type I2cSda = PB7<Alternate<OpenDrain>>;
pub type I2cBus = BlockingI2c<pac::I2C1, (I2cScl, I2cSda)>;
pub type SensorServoPin = PA8<Alternate<PushPull>>;
pub type LaserServoPin = PA9<Alternate<PushPull>>;

pub type Sensor = VL53L1X<I2cBus>;
pub type SensorServo = Servo<PwmChannel<pac::TIM1, 0>>;
pub type LaserServo = Servo<PwmChannel<pac::TIM1, 1>>;

pub struct Board {
    pub ticker: Ticker,
    pub laser_servo: LaserServo,
    pub sensor: Sensor,
    pub sensor_servo: SensorServo,
    pub adc_ratio: Ratio<u16>,
}

impl Board {
    pub fn new(cp: pac::CorePeripherals, dp: pac::Peripherals) -> Result<Self, Error> {
        // Enable debug while sleeping to keep probe-rs happy while WFI
        dp.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        dp.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());

        // Configure the clock.
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(Hertz::Hz(CLOCK_FREQ))
            .freeze(&mut flash.acr);

        let ticker = Ticker::new(Timer::syst(cp.SYST, &clocks));

        let mut afio = dp.AFIO.constrain();
        let mut gpioa = dp.GPIOA.split();
        let mut gpiob = dp.GPIOB.split();

        // Get servos range limit.
        let mut adc = Adc::adc1(dp.ADC1, clocks);
        let mut servo_range_ch = gpioa.pa1.into_analog(&mut gpioa.crl);
        let adc_reading: u16 = adc.read(&mut servo_range_ch)?;
        let adc_max = adc.max_sample();
        adc.release(); // No longer needed

        debug_rprintln!("range {} of {}", adc_reading, adc_max);
        // Avoid too small range
        let adc_value = adc_reading.max(adc_max / 10);
        let adc_ratio = Ratio::new(adc_value, adc_max);

        // Configure servos.
        let sensor_servo_pin: SensorServoPin = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
        let laser_servo_pin: LaserServoPin = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);

        let (sensor_servo_pwm, laser_servo_pwm) = dp
            .TIM1
            .pwm_hz(
                (sensor_servo_pin, laser_servo_pin),
                &mut afio.mapr,
                SERVO_FREQ,
                &clocks,
            )
            .split();

        debug_rprintln!("pwm max duty {}", sensor_servo_pwm.get_max_duty());

        let period: MilliSeconds = SERVO_FREQ
            .try_into_duration()
            .ok_or(Error::InvalidDuration)?;
        let period_ms = period.to_millis().try_into()?;

        let bounds = Bounds::scale_from_period_ms(&sensor_servo_pwm, period_ms, adc_ratio)?;
        debug_rprintln!("sensor {}", bounds);
        let mut sensor_servo = Servo::new(sensor_servo_pwm, bounds);
        sensor_servo.enable();

        let bounds = Bounds::scale_from_period_ms(&laser_servo_pwm, period_ms, adc_ratio)?;
        let mut laser_servo = Servo::new(laser_servo_pwm, bounds);
        laser_servo.enable();

        // Configure I2C.
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
        let i2c = I2c::i2c1(
            dp.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::standard(100.kHz()),
            clocks,
        )
        .blocking_default(clocks);

        // Configure ranging sensor.
        let mut sensor = VL53L1X::new(i2c, vl53l1x::ADDR);
        while sensor.boot_state()? != BootState::Booted {
            // Wait 10 ms until next timer tick.
            ticker.wait_for_tick();
        }
        sensor.sensor_init()?;

        Ok(Board {
            ticker,
            laser_servo,
            sensor,
            sensor_servo,
            adc_ratio,
        })
    }
}
