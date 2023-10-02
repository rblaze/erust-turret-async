use core::sync::atomic::{compiler_fence, Ordering};

use fastrand::Rng;
use fugit::HertzU32;
use rtt_target::debug_rprintln;
use simplefs::FileSystem;
use stm32f1xx_hal::device::DMA1;
use stm32f1xx_hal::pac::interrupt;
use stm32f1xx_hal::timer::Channel;

use crate::board::{AudioClock, AudioDma, AudioEnable, AudioPwm};
use crate::error::Error;
use crate::storage::Storage;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sound {
    Startup,
    BeginScan,
    TargetAcquired,
    ContactLost,
    ContactRestored,
    TargetLost,
    #[allow(dead_code)]
    PickedUp, // Sensor not on board
}

pub struct Audio {
    mailbox: async_scheduler::mailbox::Mailbox<Sound>,
}

impl Audio {
    pub fn new() -> Self {
        Self {
            mailbox: async_scheduler::mailbox::Mailbox::new(),
        }
    }

    pub fn play(&self, sound: Sound) {
        self.mailbox.post(sound);
    }

    pub async fn task(
        &self,
        storage: Storage,
        audio_enable: AudioEnable,
        audio_pwm: AudioPwm,
        audio_clock: AudioClock,
        audio_dma: AudioDma,
        random: Rng,
    ) -> Result<(), Error> {
        let mut audio_impl = AudioImpl {
            mailbox: &self.mailbox,
            audio_enable,
            audio_pwm,
            audio_clock,
            audio_dma,
            random,
        };

        audio_impl.task(storage).await
    }
}

struct AudioImpl<'a> {
    mailbox: &'a async_scheduler::mailbox::Mailbox<Sound>,
    audio_enable: AudioEnable,
    audio_pwm: AudioPwm,
    audio_clock: AudioClock,
    audio_dma: AudioDma,
    random: Rng,
}

impl AudioImpl<'_> {
    pub async fn task(&mut self, storage: Storage) -> Result<(), Error> {
        let fs = FileSystem::mount(storage)?;

        loop {
            let sound = self.mailbox.read().await?;
            let clips = match sound {
                Sound::Startup => STARTUP_CLIPS,
                Sound::BeginScan => BEGIN_SCAN_CLIPS,
                Sound::TargetAcquired => TARGET_ACQUIRED_CLIPS,
                Sound::ContactLost => CONTACT_LOST_CLIPS,
                Sound::ContactRestored => CONTACT_RESTORED_CLIPS,
                Sound::TargetLost => TARGET_LOST_CLIPS,
                Sound::PickedUp => PICKED_UP_CLIPS,
            };
            let clip = self.pick_clip(clips);
            self.play_clip(&fs, clip).await?;
        }
    }

    fn pick_clip(&mut self, clips: &[Clip]) -> Clip {
        // TODO use random shuffle for each clip set.
        // This will provide more diverse clips for short runs.
        let index = self.random.usize(0..clips.len());
        clips[index]
    }

    async fn play_clip(&mut self, fs: &FileSystem<Storage>, clip: Clip) -> Result<(), Error> {
        debug_rprintln!("Playing sound: {:?}", clip);

        let mut buffers = [[0; BUF_SIZE]; 2];
        let mut file = fs.open(clip.file_index())?;
        let mut next_play_buffer = 0;
        let mut next_buffer_len = file.read(&mut buffers[next_play_buffer])?;

        self.start_playback()?;

        let playback_result: Result<(), Error> = {
            while next_buffer_len > 0 {
                // Start playing buffer
                self.play_buffer(&buffers[next_play_buffer][0..next_buffer_len])?;

                // Load next buffer
                next_play_buffer = (next_play_buffer + 1) % 2;
                next_buffer_len = file.read(&mut buffers[next_play_buffer])?;

                // Wait until buffer completed
                BUFFER_DONE.read().await?;
            }
            Ok(())
        };

        self.end_playback().and(playback_result)
    }

    fn start_playback(&mut self) -> Result<(), Error> {
        self.audio_enable.set_high();
        self.audio_pwm.enable(Channel::C3);
        self.audio_clock.start(SOUND_FREQ)?;

        Ok(())
    }

    fn play_buffer(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.audio_dma.stop();

        self.audio_dma
            .set_memory_address(buffer.as_ptr() as u32, true);
        self.audio_dma.set_transfer_length(buffer.len());

        compiler_fence(Ordering::Release);

        self.audio_dma.start();

        Ok(())
    }

    fn end_playback(&mut self) -> Result<(), Error> {
        self.audio_enable.set_low();
        self.audio_pwm.disable(Channel::C3);
        self.audio_pwm.set_duty(Channel::C3, 0);
        self.audio_clock.cancel()?;

        Ok(())
    }
}

#[allow(dead_code)]
// Clips are unsigned 8 bit, 16 KHz.
pub const SOUND_FREQ: HertzU32 = HertzU32::Hz(16000);

// Sound buffer size.
const BUF_SIZE: usize = 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Clip {
    SfxDeploy,
    SfxActive,
    Searching,
    Activated,
    SentryModeActivated,
    CouldYouComeOverHere,
    Deploying,
    HelloFriend,
    WhoIsThere,
    TargetAcquired,
    Gotcha,
    ISeeYou,
    ThereYouAre,
    SfxRetract,
    SfxPing,
    Hi,
    SfxAlert,
    IsAnyoneThere,
    Hellooooo,
    AreYouStillThere,
    TargetLost,
    Malfunctioning,
    PutMeDown,
    WhoAreYou,
    PleasePutMeDown,
}

impl Clip {
    const fn file_index(self) -> usize {
        self as usize
    }
}

const STARTUP_CLIPS: &[Clip] = &[Clip::SfxDeploy, Clip::SfxActive];
const BEGIN_SCAN_CLIPS: &[Clip] = &[
    Clip::Searching,
    Clip::Activated,
    Clip::SentryModeActivated,
    Clip::CouldYouComeOverHere,
    Clip::Deploying,
];
const TARGET_ACQUIRED_CLIPS: &[Clip] = &[
    Clip::HelloFriend,
    Clip::WhoIsThere,
    Clip::TargetAcquired,
    Clip::Gotcha,
    Clip::ISeeYou,
    Clip::ThereYouAre,
];
const CONTACT_LOST_CLIPS: &[Clip] = &[Clip::SfxRetract];
const CONTACT_RESTORED_CLIPS: &[Clip] = &[Clip::SfxPing, Clip::Hi, Clip::SfxAlert];
const TARGET_LOST_CLIPS: &[Clip] = &[
    Clip::IsAnyoneThere,
    Clip::Hellooooo,
    Clip::AreYouStillThere,
    Clip::TargetLost,
];
const PICKED_UP_CLIPS: &[Clip] = &[
    Clip::Malfunctioning,
    Clip::PutMeDown,
    Clip::WhoAreYou,
    Clip::PleasePutMeDown,
];

static BUFFER_DONE: async_scheduler::sync::mailbox::Mailbox<()> =
    async_scheduler::sync::mailbox::Mailbox::new();

#[interrupt]
unsafe fn DMA1_CHANNEL2() {
    BUFFER_DONE.post(());
    // Clear interrupt flags
    (*DMA1::ptr()).ifcr.write(|w| w.cgif2().clear());
}
