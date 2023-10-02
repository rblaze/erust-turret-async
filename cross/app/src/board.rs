use crate::error::Error;
use crate::storage::Storage;
use crate::system_time::Ticker;

use fastrand::Rng;
use fugit::TimerDurationU32;
use num::rational::Ratio;
use rtt_target::debug_rprintln;
use servo::{Bounds, Servo};
use stm32f1xx_hal::adc::Adc;
use stm32f1xx_hal::device::{SPI2, TIM3};
use stm32f1xx_hal::dma::dma1;
use stm32f1xx_hal::gpio::{Alternate, Floating, Input, OpenDrain, Output, PushPull};
use stm32f1xx_hal::gpio::{PA4, PA8, PA9, PB0, PB12, PB13, PB14, PB15, PB6, PB7};
use stm32f1xx_hal::i2c::{BlockingI2c, I2c, Mode};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::spi::{Spi, Spi2NoRemap};
use stm32f1xx_hal::time::{Hertz, MilliSeconds};
use stm32f1xx_hal::timer::{Ch, CounterHz, Pwm, PwmChannel, Tim3NoRemap, Timer};
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

pub type SpiCs = PB12<Output<PushPull>>;
pub type SpiClk = PB13<Alternate<PushPull>>;
pub type SpiMiso = PB14<Input<Floating>>;
pub type SpiMosi = PB15<Alternate<PushPull>>;
pub type SpiBus = Spi<SPI2, Spi2NoRemap, (SpiClk, SpiMiso, SpiMosi), u8>;

pub type AudioEnable = PA4<Output<PushPull>>;
pub type AudioPwmPin = PB0<Alternate<PushPull>>;
pub type AudioDma = dma1::C2;
pub type AudioPwm = Pwm<TIM3, Tim3NoRemap, Ch<2>, AudioPwmPin, CLOCK_FREQ>;
pub type AudioClock = CounterHz<pac::TIM2>;

pub struct Board {
    pub ticker: Ticker,
    pub laser_servo: LaserServo,
    pub sensor: Sensor,
    pub sensor_servo: SensorServo,
    pub adc_ratio: Ratio<u16>,
    pub audio_enable: AudioEnable,
    pub audio_dma: AudioDma,
    pub audio_pwm: AudioPwm,
    pub audio_clock: AudioClock,
    pub storage: Storage,
    pub random: Rng,
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

        // Audio hardware setup
        // Mute pin
        let audio_enable = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

        // Setup TIM3 as PWM for audio output
        let audio_pin: AudioPwmPin = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
        let audio_pwm = dp.TIM3.pwm(
            audio_pin,
            &mut afio.mapr,
            TimerDurationU32::from_ticks(256),
            &clocks,
        );

        // Setup TIM2 as DMA driver
        let audio_clock = dp.TIM2.counter_hz(&clocks);

        unsafe {
            // Set bit 8 (UDE, Update DMA Request Enable) in DIER (DMA/Interrupt Enable Register).
            // Every time timer expires and generates Update event, it also triggers DMA request.
            (*pac::TIM2::ptr()).dier.write(|w| w.ude().set_bit());
        }

        // Setup audio DMA
        let dma1 = dp.DMA1.split();
        let mut audio_dma = dma1.2;

        // Send data to TIM3 channel 3 CCR
        audio_dma
            .set_peripheral_address(unsafe { (*TIM3::ptr()).ccr3() as *const _ as u32 }, false);

        #[rustfmt::skip]
        audio_dma.ch().cr.modify(|_, w| { w
            .mem2mem().clear_bit()
            .pl().high()
            .msize().bits8()
            .psize().bits16()
            .circ().clear_bit()
            .dir().set_bit()
        });

        // Trigger interrupt when entire block is sent to PWM input.
        audio_dma.listen(stm32f1xx_hal::dma::Event::TransferComplete);

        unsafe {
            // Enable DMA interrupt.
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA1_CHANNEL2);
        }

        // Scoop some randomish data for PRNG
        let random = Rng::with_seed(adc_reading as u64 | cp.DWT.cyccnt.read() as u64);

        // Setup SPI storage
        let spi_cs = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        let spi_clk = gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);
        let spi_miso = gpiob.pb14.into_floating_input(&mut gpiob.crh);
        let spi_mosi = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);

        let spi = Spi::spi2(
            dp.SPI2,
            (spi_clk, spi_miso, spi_mosi),
            embedded_hal::spi::MODE_0,
            10.MHz(),
            clocks,
        );

        let storage = Storage::new(spi, spi_cs)?;

        Ok(Board {
            ticker,
            laser_servo,
            sensor,
            sensor_servo,
            adc_ratio,
            audio_enable,
            audio_dma,
            audio_pwm,
            audio_clock,
            storage,
            random,
        })
    }
}
