//! Sound effects.
//!
//! - The **engine** is a physically-informed model from the MIT `enginesound`
//!   crate, streamed through a custom rodio source. RPM follows a rate-limited
//!   smoothed value, so speed changes glide instead of stepping.
//! - The **boost** is a real thruster recording (`assets/audio/boost.wav`, cut
//!   from the project owner's `boost2.wav`; see the README there), looped
//!   seamlessly, because a recorded roar is much harder to fake than a thud.
//! - The **car/ball impact**, **ball bounce** and **demolition** are one-shot
//!   PCM clips synthesised at startup and scaled by impact speed and distance.
//!
//! Impact sounds come from RocketSim's physics events (see
//! [`SimEvent`](crate::sim::SimEvent)), so a bounce is a real contact normal
//! speed, not a velocity-delta guess.

use std::f32::consts::TAU;
use std::num::NonZero;
use std::path::Path;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;

use bevy::asset::Asset;
use bevy::audio::{
    AddAudioSource, AudioPlayer, AudioSink, AudioSinkPlayback, AudioSource, ChannelCount,
    Decodable, PlaybackSettings, SampleRate, Source, Volume,
};
use bevy::prelude::*;
use bevy::reflect::TypePath;
use enginesound::{fix_engine, Engine, Generator, LowPassFilter};

use super::{Boosting, Mode, Playback, Sim, Vis};
use crate::sim::SimEvent;

/// Sample rate of every generated clip. 48 kHz is the usual device rate, so
/// playback does not resample.
const SAMPLE_RATE: u32 = 48_000;

/// Engine preset and stream parameters.
const ENGINE_PRESET: &str = include_str!("../../assets/audio/engine.esc");
const ENGINE_SAMPLE_RATE: u32 = 48_000;
/// How many samples the synth produces per audio-thread refill.
const ENGINE_BLOCK: usize = 128;
/// `Generator::volume`; its output is roughly ±0.6 at idle at this level.
const ENGINE_GENERATOR_VOLUME: f32 = 0.35;
/// Idle and redline, in engine RPM. These are the model's input, not the car's
/// wheel speed.
const ENGINE_IDLE_RPM: f32 = 850.0;
const ENGINE_MAX_RPM: f32 = 7000.0;

/// How loud the engine is at idle and at full speed. The two cars share a
/// mixer, so these stay low enough that boost and impacts sit on top.
const ENGINE_IDLE_VOLUME: f32 = 0.18;
const ENGINE_FULL_VOLUME: f32 = 0.38;
const BOOST_VOLUME: f32 = 0.40;
const CAR_HIT_VOLUME: f32 = 0.85;
const BOUNCE_VOLUME: f32 = 0.75;
const DEMOLITION_VOLUME: f32 = 0.95;

/// Distance at which a one-shot is half as loud. The arena is 81 m long, so a
/// 20 m reference keeps a far bounce audible without washing out near ones.
const SOUND_REFERENCE_METRES: f32 = 20.0;

/// A streaming engine-synthesis asset, one per car.
///
/// The preset is embedded, and the target RPM is shared between the game thread
/// (which writes it) and the audio thread (which reads it at each block).
#[derive(Asset, TypePath)]
struct EngineSource {
    preset: Arc<str>,
    /// Target RPM as `f32` bits.
    rpm: Arc<AtomicU32>,
}

struct EngineDecoder {
    generator: Generator,
    rpm: Arc<AtomicU32>,
    block: Vec<f32>,
    index: usize,
}

impl Decodable for EngineSource {
    type Decoder = EngineDecoder;

    fn decoder(&self) -> Self::Decoder {
        let mut engine: Engine =
            ron::from_str(&self.preset).expect("embedded enginesound preset is invalid");
        fix_engine(&mut engine, ENGINE_SAMPLE_RATE);
        engine.rpm =
            f32::from_bits(self.rpm.load(Ordering::Relaxed)).clamp(ENGINE_IDLE_RPM, ENGINE_MAX_RPM);

        let mut generator = Generator::new(
            ENGINE_SAMPLE_RATE,
            engine,
            LowPassFilter::new(0.5, ENGINE_SAMPLE_RATE),
        );
        generator.volume = ENGINE_GENERATOR_VOLUME;

        EngineDecoder {
            generator,
            rpm: self.rpm.clone(),
            block: vec![0.0; ENGINE_BLOCK],
            index: 0,
        }
    }
}

impl Iterator for EngineDecoder {
    type Item = f32;

    fn next(&mut self) -> Option<f32> {
        if self.index >= self.block.len() {
            let target = f32::from_bits(self.rpm.load(Ordering::Relaxed))
                .clamp(ENGINE_IDLE_RPM, ENGINE_MAX_RPM);
            // A little extra smoothing in the audio thread, so a frame-rate
            // step in the game-side target cannot become an RPM step.
            let current = self.generator.engine.rpm;
            self.generator.engine.rpm = current + (target - current) * 0.25;
            self.generator.generate(&mut self.block);
            self.index = 0;
        }
        let sample = self.block[self.index];
        self.index += 1;
        // The model can overshoot briefly when its waveguides resonate.
        Some(sample.clamp(-1.0, 1.0))
    }
}

impl Source for EngineDecoder {
    fn current_span_len(&self) -> Option<usize> {
        None
    }

    fn channels(&self) -> ChannelCount {
        NonZero::new(1).expect("mono")
    }

    fn sample_rate(&self) -> SampleRate {
        NonZero::new(ENGINE_SAMPLE_RATE).expect("sample rate is non-zero")
    }

    fn total_duration(&self) -> Option<std::time::Duration> {
        None
    }
}

pub struct SfxPlugin;

impl Plugin for SfxPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<SimEvent>()
            .add_audio_source::<EngineSource>()
            .add_systems(Startup, prepare_clips)
            .add_systems(PostStartup, spawn_loops)
            .add_systems(Update, (drive_engine, drive_boost, play_events));
    }
}

/// The real thruster recording, processed into a seamless loop.
///
/// The source is the project owner's `boost2.wav`; see `assets/audio/README.md`
/// for the edit (the license/provenance still needs to be recorded there).
/// Embedding it keeps the viewer independent of the working directory (and of
/// an asset-loading step).
const BOOST_WAV: &[u8] = include_bytes!("../../assets/audio/boost.wav");

/// Write every clip to `dir` as a WAV.
///
/// This backs `--dump-audio` so a sound can be auditioned or spectrogrammed
/// without launching the viewer. It runs before RocketSim (and the window)
/// start, so it also works on a machine with no arena meshes.
pub fn dump_clips(dir: &Path) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    let clips = [
        // A slow idle→redline→idle sweep lets the engine be judged (and the
        // smoothness heard) without driving the car.
        ("engine", render_engine_sweep(7.0)),
        ("ball_hit", car_hit_ball()),
        ("bounce", ball_bounce()),
        ("demolition", demolition()),
    ];
    for (name, samples) in clips {
        std::fs::write(
            dir.join(format!("{name}.wav")),
            wav_bytes(&samples, SAMPLE_RATE),
        )?;
    }
    std::fs::write(dir.join("boost.wav"), BOOST_WAV)?;
    Ok(())
}

/// Render a slow idle→redline→idle sweep for `--dump-audio`.
fn render_engine_sweep(seconds: f32) -> Vec<f32> {
    let mut engine: Engine =
        ron::from_str(ENGINE_PRESET).expect("embedded enginesound preset is invalid");
    fix_engine(&mut engine, ENGINE_SAMPLE_RATE);
    engine.rpm = ENGINE_IDLE_RPM;

    let mut generator = Generator::new(
        ENGINE_SAMPLE_RATE,
        engine,
        LowPassFilter::new(0.5, ENGINE_SAMPLE_RATE),
    );
    generator.volume = ENGINE_GENERATOR_VOLUME;

    let mut warmup = vec![0.0; ENGINE_SAMPLE_RATE as usize];
    generator.generate(&mut warmup);

    let mut out = vec![0.0; (seconds * ENGINE_SAMPLE_RATE as f32) as usize];
    let mut index = 0;
    while index < out.len() {
        let end = (index + ENGINE_BLOCK).min(out.len());
        let phase = (index as f32 / out.len() as f32).clamp(0.0, 1.0);
        let load = 0.5 - 0.5 * (TAU * phase).cos();
        generator.engine.rpm = ENGINE_IDLE_RPM + (ENGINE_MAX_RPM - ENGINE_IDLE_RPM) * load;
        generator.generate(&mut out[index..end]);
        index = end;
    }
    out
}

/// Render a steady-RPM engine sample, for tests.
#[cfg(test)]
fn render_engine(rpm: f32, seconds: f32) -> Vec<f32> {
    let mut engine: Engine =
        ron::from_str(ENGINE_PRESET).expect("embedded enginesound preset is invalid");
    fix_engine(&mut engine, ENGINE_SAMPLE_RATE);
    engine.rpm = rpm.clamp(ENGINE_IDLE_RPM, ENGINE_MAX_RPM);

    let mut generator = Generator::new(
        ENGINE_SAMPLE_RATE,
        engine,
        LowPassFilter::new(0.5, ENGINE_SAMPLE_RATE),
    );
    generator.volume = ENGINE_GENERATOR_VOLUME;

    // Let the acoustic chambers settle before recording.
    let mut warmup = vec![0.0; (1.5 * ENGINE_SAMPLE_RATE as f32) as usize];
    generator.generate(&mut warmup);

    let mut out = vec![0.0; (seconds * ENGINE_SAMPLE_RATE as f32) as usize];
    generator.generate(&mut out);
    out
}

/// The generated clips, shared by every emitter.
#[derive(Resource)]
struct SfxClips {
    /// One streaming synth per car, so each follows its own RPM.
    engine: [Handle<EngineSource>; 2],
    /// The RPM control shared with each engine synth's audio thread.
    engine_rpm: [Arc<AtomicU32>; 2],
    boost: Handle<AudioSource>,
    ball_hit: Handle<AudioSource>,
    bounce: Handle<AudioSource>,
    demolition: Handle<AudioSource>,
}

/// A streaming engine emitter attached to a car's visual parent.
#[derive(Component)]
struct EngineAudio {
    car: usize,
    /// Smoothed 0..1 level, so demolitions and pause do not click.
    volume: f32,
    /// Smoothed 0..1 RPM/load. Pitch and volume both follow this, and it is
    /// rate-limited in [`smooth_rpm`], so a collision cannot jerk the revs.
    rpm: f32,
    /// Target RPM for the audio thread.
    rpm_control: Arc<AtomicU32>,
}

/// How quickly the smoothed RPM chases its target (e-fold, per second).
const ENGINE_RPM_RESPONSE: f32 = 3.5;
/// Hard ceiling on RPM change per second. This is what makes a sudden speed
/// change glide instead of stepping: a full 0→1 rev takes at least two thirds
/// of a second even if the target jumps instantly.
const ENGINE_RPM_MAX_RATE: f32 = 1.5;

/// One-pole chase with a slew limit. `dt` is in seconds.
fn smooth_rpm(current: f32, target: f32, dt: f32) -> f32 {
    let blend = 1.0 - (-dt * ENGINE_RPM_RESPONSE).exp();
    let next = current + (target - current) * blend;
    let max_step = ENGINE_RPM_MAX_RATE * dt;
    next.clamp(current - max_step, current + max_step)
}

/// A looping thruster emitter attached to a car's visual parent.
#[derive(Component)]
struct BoostAudio {
    car: usize,
    volume: f32,
}

/// Synthesise the one-shots and create the two engine synths.
fn prepare_clips(
    mut commands: Commands,
    mut sources: ResMut<Assets<AudioSource>>,
    mut engines: ResMut<Assets<EngineSource>>,
) {
    let add = |sources: &mut Assets<AudioSource>, samples: Vec<f32>| {
        sources.add(AudioSource {
            bytes: wav_bytes(&samples, SAMPLE_RATE).into(),
        })
    };
    let make_engine = |engines: &mut Assets<EngineSource>| {
        let rpm = Arc::new(AtomicU32::new(ENGINE_IDLE_RPM.to_bits()));
        let handle = engines.add(EngineSource {
            preset: Arc::from(ENGINE_PRESET),
            rpm: rpm.clone(),
        });
        (handle, rpm)
    };
    let (engine0, rpm0) = make_engine(&mut engines);
    let (engine1, rpm1) = make_engine(&mut engines);

    commands.insert_resource(SfxClips {
        engine: [engine0, engine1],
        engine_rpm: [rpm0, rpm1],
        boost: sources.add(AudioSource {
            bytes: BOOST_WAV.to_vec().into(),
        }),
        ball_hit: add(&mut sources, car_hit_ball()),
        bounce: add(&mut sources, ball_bounce()),
        demolition: add(&mut sources, demolition()),
    });
}

/// Spawn one engine and one thruster loop per car.
///
/// They are children of the car's visual parent, so they follow it; the sound
/// is deliberately non-spatial (see the module docs), but the loop still belongs
/// with the car that owns it.
fn spawn_loops(mut commands: Commands, clips: Res<SfxClips>, cars: Query<(Entity, &Vis)>) {
    for (entity, vis) in &cars {
        let Vis::Car(car) = vis else { continue };
        let (Some(engine), Some(rpm_control)) =
            (clips.engine.get(*car), clips.engine_rpm.get(*car))
        else {
            continue;
        };
        commands.entity(entity).with_children(|parent| {
            parent.spawn((
                EngineAudio {
                    car: *car,
                    volume: 0.0,
                    rpm: 0.0,
                    rpm_control: rpm_control.clone(),
                },
                AudioPlayer(engine.clone()),
                // The decoder is an infinite generator, so it never reaches the
                // end of a `Once` playback.
                PlaybackSettings::ONCE.with_volume(Volume::Linear(0.0)),
            ));
            parent.spawn((
                BoostAudio {
                    car: *car,
                    volume: 0.0,
                },
                AudioPlayer::new(clips.boost.clone()),
                PlaybackSettings::LOOP.with_volume(Volume::Linear(0.0)),
            ));
        });
    }
}

/// Fade and pitch each engine from its car's speed and throttle.
fn drive_engine(
    time: Res<Time>,
    playback: Res<Playback>,
    mode: Res<Mode>,
    sim: Res<Sim>,
    cameras: Query<&GlobalTransform, With<Camera3d>>,
    mut engines: Query<(&mut EngineAudio, &mut AudioSink)>,
) {
    let quiet = playback.paused || matches!(*mode, Mode::Capture { .. });
    let listener = cameras
        .iter()
        .next()
        .map(|camera| camera.translation())
        .unwrap_or(Vec3::ZERO);
    // Volume gets its own slightly slower fade (~200 ms), so pause and
    // demolition do not click.
    let volume_blend = 1.0 - (-time.delta_secs() * 5.0).exp();
    let dt = time.delta_secs();

    for (mut audio, mut sink) in &mut engines {
        let car = sim.0.car_at(audio.car);
        let throttle = sim.0.last_input(audio.car).throttle.abs() as f32;

        // Total speed rather than forward speed: RPM should not collapse to
        // idle every time the car slides sideways or rotates on a wall.
        let moving = (car.speed / 26.0).clamp(0.0, 1.0);
        // Pedal at a standstill still revs the engine, hence the throttle term.
        // Boost is deliberately *not* a term: a thruster does not change engine
        // RPM directly. It raises speed, and this mapping follows that as the
        // car actually accelerates. The boost loop is a separate layer.
        let target_rpm = (0.90 * moving + 0.10 * throttle).min(1.0);
        audio.rpm = smooth_rpm(audio.rpm, target_rpm, dt);

        // Map the smoothed 0..1 load onto the synth's RPM range. The audio
        // thread reads this at each block and smooths it again, so a sudden
        // collision or throttle press becomes a rev, not a jump.
        let engine_rpm = ENGINE_IDLE_RPM + (ENGINE_MAX_RPM - ENGINE_IDLE_RPM) * audio.rpm;
        audio
            .rpm_control
            .store(engine_rpm.to_bits(), Ordering::Relaxed);

        let target_volume = if quiet || car.is_demoed {
            0.0
        } else {
            let level = ENGINE_IDLE_VOLUME + (ENGINE_FULL_VOLUME - ENGINE_IDLE_VOLUME) * audio.rpm;
            // The acoustic model loses level as RPM rises; lift it back so the
            // engine does not go quiet when it revs.
            let level = level * (1.0 + 1.2 * audio.rpm);
            // The opponent's engine is the same model, a little further back in
            // the mix and quieter with distance: the human's car is what the
            // camera is attached to.
            if audio.car == 0 {
                level
            } else {
                level * 0.8 * distance_gain(listener, car.pos)
            }
        };

        // Stop burning CPU on the synth while the simulation is paused or a
        // capture is running. A demolished car keeps running so its volume can
        // fade out smoothly.
        if quiet {
            sink.pause();
        } else {
            sink.play();
        }

        audio.volume += (target_volume - audio.volume) * volume_blend;
        sink.set_volume(Volume::Linear(audio.volume));
    }
}

/// Fade the thruster loop in while a car is actually spending boost.
fn drive_boost(
    time: Res<Time>,
    playback: Res<Playback>,
    mode: Res<Mode>,
    boosting: Res<Boosting>,
    mut audios: Query<(&mut BoostAudio, &mut AudioSink)>,
) {
    if playback.paused || matches!(*mode, Mode::Capture { .. }) {
        for (mut audio, mut sink) in &mut audios {
            audio.volume = 0.0;
            sink.set_volume(Volume::Linear(0.0));
        }
        return;
    }

    let blend = 1.0 - (-time.delta_secs() * 14.0).exp();
    for (mut audio, mut sink) in &mut audios {
        let on = boosting.0.get(audio.car).copied().unwrap_or(false);
        let target = if on { BOOST_VOLUME } else { 0.0 };
        audio.volume += (target - audio.volume) * blend;
        sink.set_volume(Volume::Linear(audio.volume));
        // A touch of pitch rise as it settles in, so repeated taps do not sound
        // like an on/off switch.
        sink.set_speed(0.92 + 0.12 * (audio.volume / BOOST_VOLUME));
    }
}

/// Turn physics events into one-shot sounds, attenuated by distance from the
/// camera. Captures are silent: a screenshot should not make the machine bark.
fn play_events(
    mut events: MessageReader<SimEvent>,
    mode: Res<Mode>,
    clips: Res<SfxClips>,
    cameras: Query<&GlobalTransform, With<Camera3d>>,
    mut commands: Commands,
) {
    if matches!(*mode, Mode::Capture { .. }) {
        events.clear();
        return;
    }

    let listener = cameras
        .iter()
        .next()
        .map(|camera| camera.translation())
        .unwrap_or(Vec3::ZERO);

    for event in events.read() {
        match *event {
            SimEvent::BallBounce { pos, speed } => {
                // A soft bounce is quieter *and* lower; a hard one is a sharp
                // crack. 40 m/s is about a full-power shot.
                let impact = (speed / 40.0).clamp(0.0, 1.0);
                let volume = BOUNCE_VOLUME * (0.22 + 0.78 * impact) * distance_gain(listener, pos);
                commands.spawn((
                    AudioPlayer::new(clips.bounce.clone()),
                    PlaybackSettings::DESPAWN
                        .with_volume(Volume::Linear(volume))
                        .with_speed(0.82 + 0.38 * impact),
                ));
            }
            SimEvent::CarHitBall { pos, strength } => {
                // A touch is a soft knock; a full-speed hit is a sharp thock.
                // 25 m/s is about a hard aerial hit.
                let impact = (strength / 25.0).clamp(0.0, 1.0);
                let volume = CAR_HIT_VOLUME * (0.25 + 0.75 * impact) * distance_gain(listener, pos);
                commands.spawn((
                    AudioPlayer::new(clips.ball_hit.clone()),
                    PlaybackSettings::DESPAWN
                        .with_volume(Volume::Linear(volume))
                        .with_speed(0.90 + 0.25 * impact),
                ));
            }
            SimEvent::Demolition { pos } => {
                let volume = DEMOLITION_VOLUME * distance_gain(listener, pos);
                commands.spawn((
                    AudioPlayer::new(clips.demolition.clone()),
                    PlaybackSettings::DESPAWN.with_volume(Volume::Linear(volume)),
                ));
            }
        }
    }
}

/// Simple inverse-distance falloff, 1.0 at the ear and falling towards
/// [`SOUND_REFERENCE_METRES`] around the arena.
fn distance_gain(listener: Vec3, source: Vec3) -> f32 {
    let distance = listener.distance(source);
    SOUND_REFERENCE_METRES / (SOUND_REFERENCE_METRES + distance)
}

// ---------------------------------------------------------------------------
// Synthesis
// ---------------------------------------------------------------------------

/// A tiny xorshift generator. Deterministic so every run sounds the same.
struct Rng(u32);

impl Rng {
    fn new(seed: u32) -> Self {
        Self(seed | 1)
    }

    fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    /// Uniform in `0.0..1.0`.
    fn unit(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 / (1u32 << 24) as f32
    }

    /// Uniform in `-1.0..1.0`.
    fn bipolar(&mut self) -> f32 {
        self.unit() * 2.0 - 1.0
    }
}

/// One-shot car-on-ball knock: the original punchy mid knock, with a low shelf
/// for weight. Keeping the knock means the hit still cuts through on small
/// speakers; the shelf makes it feel heavier.
fn car_hit_ball() -> Vec<f32> {
    const DURATION: f32 = 0.30;
    let samples = (SAMPLE_RATE as f32 * DURATION) as usize;
    let mut out = Vec::with_capacity(samples);
    let mut rng = Rng::new(0x0CA2_8A11);
    let mut body_phase = 0.0;
    let mut knock_phase = 0.0;

    for i in 0..samples {
        let t = i as f32 / SAMPLE_RATE as f32;

        let body_freq = 90.0 + 170.0 * (-t * 30.0).exp();
        body_phase = (body_phase + TAU * body_freq / SAMPLE_RATE as f32) % TAU;
        let body = body_phase.sin() * (-t * 24.0).exp();

        let knock_freq = 240.0 + 500.0 * (-t * 40.0).exp();
        knock_phase = (knock_phase + TAU * knock_freq / SAMPLE_RATE as f32) % TAU;
        let knock = knock_phase.sin() * (-t * 55.0).exp() * 0.5;

        let contact = if t < 0.008 {
            rng.bipolar() * (1.0 - t / 0.008) * 0.5
        } else {
            0.0
        };

        out.push(body * 0.7 + knock + contact);
    }

    normalize(low_shelf(out, 170.0, 8.0), 0.95)
}

/// One-shot ball-on-surface thud: the pitched-down body and knock, with a low
/// shelf for extra weight.
fn ball_bounce() -> Vec<f32> {
    const DURATION: f32 = 0.40;
    let samples = (SAMPLE_RATE as f32 * DURATION) as usize;
    let mut out = Vec::with_capacity(samples);
    let mut rng = Rng::new(0x0BA1_1B0C);
    let mut body_phase = 0.0;
    let mut knock_phase = 0.0;

    for i in 0..samples {
        let t = i as f32 / SAMPLE_RATE as f32;

        let body_freq = 65.0 + 135.0 * (-t * 12.0).exp();
        body_phase = (body_phase + TAU * body_freq / SAMPLE_RATE as f32) % TAU;
        let body = body_phase.sin() * (-t * 18.0).exp();

        let knock_freq = 150.0 + 280.0 * (-t * 26.0).exp();
        knock_phase = (knock_phase + TAU * knock_freq / SAMPLE_RATE as f32) % TAU;
        let knock = knock_phase.sin() * (-t * 36.0).exp() * 0.35;

        let contact = if t < 0.012 {
            rng.bipolar() * (1.0 - t / 0.012) * 0.28
        } else {
            0.0
        };

        out.push(body * 0.8 + knock + contact);
    }

    normalize(low_shelf(out, 170.0, 7.0), 0.9)
}

/// RBJ low-shelf biquad. A gentle shelf below `cutoff_hz` adds weight without
/// removing the mid knock that keeps an impact audible on small speakers.
fn low_shelf(samples: Vec<f32>, cutoff_hz: f32, gain_db: f32) -> Vec<f32> {
    let a = 10.0f32.powf(gain_db / 40.0);
    let w0 = TAU * cutoff_hz / SAMPLE_RATE as f32;
    let cos_w0 = w0.cos();
    let alpha = w0.sin() / (2.0 * 0.707);
    let sqrt_a = a.sqrt();

    let b0 = a * ((a + 1.0) - (a - 1.0) * cos_w0 + 2.0 * sqrt_a * alpha);
    let b1 = 2.0 * a * ((a - 1.0) - (a + 1.0) * cos_w0);
    let b2 = a * ((a + 1.0) - (a - 1.0) * cos_w0 - 2.0 * sqrt_a * alpha);
    let a0 = (a + 1.0) + (a - 1.0) * cos_w0 + 2.0 * sqrt_a * alpha;
    let a1 = -2.0 * ((a - 1.0) + (a + 1.0) * cos_w0);
    let a2 = (a + 1.0) + (a - 1.0) * cos_w0 - 2.0 * sqrt_a * alpha;

    let (b0, b1, b2, a1, a2) = (b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0);
    let mut out = Vec::with_capacity(samples.len());
    let (mut x1, mut x2, mut y1, mut y2) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    for x in samples {
        let y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        out.push(y);
        x2 = x1;
        x1 = x;
        y2 = y1;
        y1 = y;
    }
    out
}

/// One-shot demolition: a bright noise crack collapsing into a sub-bass boom.
fn demolition() -> Vec<f32> {
    const DURATION: f32 = 1.4;
    let samples = (SAMPLE_RATE as f32 * DURATION) as usize;
    let mut out = Vec::with_capacity(samples);
    let mut rng = Rng::new(0x0DED_0BEE);
    let mut phase = 0.0;
    let mut lowpass = 0.0;

    for i in 0..samples {
        let t = i as f32 / SAMPLE_RATE as f32;
        let attack = (t / 0.006).min(1.0);
        let envelope = attack * (-t * 3.6).exp();

        // The noise loses its top end as the blast passes.
        let cutoff = 160.0 + 5200.0 * (-t * 5.0).exp();
        let coefficient = 1.0 - (-TAU * cutoff / SAMPLE_RATE as f32).exp();
        lowpass += (rng.bipolar() - lowpass) * coefficient;

        // Sub-bass drop: 105 Hz down to 30 Hz.
        let boom_freq = 30.0 + 75.0 * (-t * 4.0).exp();
        phase = (phase + TAU * boom_freq / SAMPLE_RATE as f32) % TAU;
        let boom = phase.sin() * (-t * 2.2).exp() * 0.75;

        let crack = if t < 0.02 {
            rng.bipolar() * (1.0 - t / 0.02) * 0.9
        } else {
            0.0
        };

        out.push(lowpass * 2.0 * envelope + boom + crack);
    }

    normalize(out, 0.97)
}

/// Scale a clip so its loudest sample is `peak`, and clamp anything that would
/// still clip after mixing.
fn normalize(mut samples: Vec<f32>, peak: f32) -> Vec<f32> {
    let loudest = samples
        .iter()
        .fold(0.0f32, |loudest, sample| loudest.max(sample.abs()));
    if loudest > 1e-6 {
        let gain = peak / loudest;
        for sample in &mut samples {
            *sample = (*sample * gain).clamp(-1.0, 1.0);
        }
    }
    samples
}

/// Wrap mono `f32` samples in a 16-bit PCM WAV container, which is what
/// Bevy's `wav` feature knows how to decode.
fn wav_bytes(samples: &[f32], sample_rate: u32) -> Vec<u8> {
    const CHANNELS: u16 = 1;
    const BITS: u16 = 16;
    let data_len = samples.len() * 2;
    let mut out = Vec::with_capacity(44 + data_len);

    out.extend_from_slice(b"RIFF");
    out.extend_from_slice(&(36 + data_len as u32).to_le_bytes());
    out.extend_from_slice(b"WAVEfmt ");
    out.extend_from_slice(&16u32.to_le_bytes());
    out.extend_from_slice(&1u16.to_le_bytes()); // PCM
    out.extend_from_slice(&CHANNELS.to_le_bytes());
    out.extend_from_slice(&sample_rate.to_le_bytes());
    out.extend_from_slice(&(sample_rate * 2).to_le_bytes()); // byte rate
    out.extend_from_slice(&(CHANNELS * BITS / 8).to_le_bytes()); // block align
    out.extend_from_slice(&BITS.to_le_bytes());
    out.extend_from_slice(b"data");
    out.extend_from_slice(&(data_len as u32).to_le_bytes());

    for sample in samples {
        let value = (sample.clamp(-1.0, 1.0) * i16::MAX as f32) as i16;
        out.extend_from_slice(&value.to_le_bytes());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wav_header_describes_mono_pcm() {
        let bytes = wav_bytes(&[0.0, 0.5, -0.5], 48_000);
        assert_eq!(&bytes[0..4], b"RIFF");
        assert_eq!(&bytes[8..16], b"WAVEfmt ");
        assert_eq!(u16::from_le_bytes([bytes[20], bytes[21]]), 1); // PCM
        assert_eq!(u16::from_le_bytes([bytes[22], bytes[23]]), 1); // mono
        assert_eq!(
            u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]),
            48_000
        );
        assert_eq!(
            u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]),
            6
        );
        assert_eq!(bytes.len(), 44 + 6);
    }

    #[test]
    fn embedded_engine_preset_generates_audio() {
        let samples = render_engine(
            ENGINE_IDLE_RPM + 0.5 * (ENGINE_MAX_RPM - ENGINE_IDLE_RPM),
            1.0,
        );
        assert!(!samples.is_empty());
        assert!(samples.iter().all(|sample| sample.is_finite()));
        assert!(samples.iter().any(|sample| sample.abs() > 0.05));
        assert!(samples.iter().all(|sample| sample.abs() <= 1.0));
    }

    #[test]
    fn streaming_engine_decoder_produces_audio() {
        let rpm = Arc::new(AtomicU32::new(3000.0f32.to_bits()));
        let source = EngineSource {
            preset: Arc::from(ENGINE_PRESET),
            rpm: rpm.clone(),
        };
        let mut decoder = source.decoder();
        assert_eq!(decoder.sample_rate().get(), ENGINE_SAMPLE_RATE);
        assert_eq!(decoder.channels().get(), 1);

        let samples: Vec<f32> = decoder
            .by_ref()
            .take(ENGINE_SAMPLE_RATE as usize / 2)
            .collect();
        assert!(samples.iter().any(|sample| sample.abs() > 0.02));

        // The live RPM control must affect the running source.
        rpm.store(ENGINE_MAX_RPM.to_bits(), Ordering::Relaxed);
        let samples: Vec<f32> = decoder.take(ENGINE_SAMPLE_RATE as usize / 2).collect();
        assert!(samples.iter().any(|sample| sample.abs() > 0.02));
    }

    #[test]
    fn boost_asset_is_a_valid_seamless_recording() {
        let (samples, rate, channels) = decode_wav(BOOST_WAV);
        assert_eq!(rate, 44_100);
        assert_eq!(channels, 2);
        assert!(
            (samples.len() as f32 / rate as f32 - 2.5).abs() < 0.01,
            "boost loop is {} s",
            samples.len() as f32 / rate as f32
        );
        assert!(samples.iter().any(|sample| sample.abs() > 0.4));

        // The offline crossfade should make the wrap look like any other step.
        let last = samples.len() - 1;
        let wrap = (samples[0] - samples[last]).abs();
        let step = samples
            .windows(2)
            .map(|pair| (pair[1] - pair[0]).abs())
            .sum::<f32>()
            / last as f32;
        assert!(
            wrap < step * 2.0 + 0.01,
            "boost seam jumps by {wrap} (step {step})"
        );
    }

    /// Minimal 16-bit PCM WAV reader, enough for the embedded boost asset.
    fn decode_wav(bytes: &[u8]) -> (Vec<f32>, u32, u16) {
        assert_eq!(&bytes[0..4], b"RIFF");
        assert_eq!(&bytes[8..12], b"WAVE");
        assert_eq!(&bytes[36..40], b"data");
        let channels = u16::from_le_bytes([bytes[22], bytes[23]]);
        let rate = u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
        let bits = u16::from_le_bytes([bytes[34], bytes[35]]);
        assert_eq!(bits, 16);
        let data_len = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
        let data = &bytes[44..44 + data_len];
        let mut mono = Vec::with_capacity(data.len() / (2 * channels as usize));
        for frame in data.chunks_exact(2 * channels as usize) {
            let mut sum = 0.0;
            for channel in 0..channels as usize {
                let offset = channel * 2;
                sum += i16::from_le_bytes([frame[offset], frame[offset + 1]]) as f32 / 32768.0;
            }
            mono.push(sum / channels as f32);
        }
        (mono, rate, channels)
    }

    #[test]
    fn rpm_changes_are_rate_limited_and_settle() {
        let dt = 1.0 / 60.0;
        let mut rpm = 0.0;
        for _ in 0..10 {
            let next = smooth_rpm(rpm, 1.0, dt);
            assert!(next - rpm <= ENGINE_RPM_MAX_RATE * dt + 1e-6);
            rpm = next;
        }
        for _ in 0..240 {
            rpm = smooth_rpm(rpm, 1.0, dt);
        }
        assert!(rpm > 0.9 && rpm <= 1.0, "rpm settled at {rpm}");

        let before = rpm;
        let after = smooth_rpm(rpm, 0.0, dt);
        assert!(before - after <= ENGINE_RPM_MAX_RATE * dt + 1e-6);
    }

    #[test]
    fn generated_one_shots_decode_with_rodio() {
        for samples in [car_hit_ball(), ball_bounce(), demolition()] {
            let source = AudioSource {
                bytes: wav_bytes(&samples, SAMPLE_RATE).into(),
            };
            let mut decoder = source.decoder();
            assert!(decoder.by_ref().take(2_000).any(|sample| sample != 0.0));
        }
    }

    #[test]
    fn one_shots_decay_to_silence() {
        for (name, samples) in [
            ("ball_hit", car_hit_ball()),
            ("bounce", ball_bounce()),
            ("demolition", demolition()),
        ] {
            let tail = &samples[samples.len() - 64..];
            let tail_peak = tail
                .iter()
                .fold(0.0f32, |loudest, sample| loudest.max(sample.abs()));
            assert!(tail_peak < 0.02, "{name} does not decay (peak {tail_peak})");
            assert!(
                samples.iter().any(|sample| sample.abs() > 0.3),
                "{name} is too quiet"
            );
        }
    }

    #[test]
    fn distance_falloff_is_monotonic() {
        let near = distance_gain(Vec3::ZERO, Vec3::new(0.0, 0.0, 2.0));
        let far = distance_gain(Vec3::ZERO, Vec3::new(0.0, 0.0, 60.0));
        assert!(near > far);
        assert!((near - 1.0).abs() < 0.2);
        assert!(far > 0.2);
    }
}
