//! RocketSim-backed simulation.
//!
//! `rl_view` owns the scene; this module owns the physics. It wraps a
//! [`rocketsim::Arena`] and exposes just the handful of values the viewer reads
//! (car/ball pose, boost, pads, score, tick) in the viewer's coordinate frame,
//! so the render code does not need to know anything about RocketSim.
//!
//! # Coordinate frames
//!
//! RocketSim is **right-handed, Z-up**: `+X` is forward, the long axis is `Y`,
//! and Blue defends `-Y`. The viewer's scene is Bevy's **right-handed, Y-up**
//! frame with the long axis on `Z` and Blue defending `-Z`. Both are
//! right-handed, so the mapping is a pure *rotation* (determinant +1), not a
//! mirror:
//!
//! ```text
//! view = (-x, z, y)
//! ```
//!
//! which sends RocketSim's up (`+Z`) to the viewer's up (`+Y`) and Blue's end
//! (`-Y`) to `-Z`. Because it is a rotation, a pseudovector like the ball's spin
//! (`ang_vel`) needs no sign flip, and the car's basis stays orthonormal.
//!
//! # Units
//!
//! RocketSim lengths are in "uu" (1 uu = 1 cm), so divide by 100 for metres.
//! `ang_vel` is already in rad/s. (Note: the `50 uu = 1 m` in RocketSim's crate
//! docs refers to its internal Bullet scale and does not apply to the public
//! API.)
//!
//! # Assets
//!
//! RocketSim collides the ball against Rocket League's actual arena mesh, which
//! is loaded once from `./collision_meshes/` at startup. Those files come from
//! `RLArenaCollisionDumper`; without them [`Sim::new`] panics with instructions.

use bevy::math::Vec3;
use bevy::prelude::Message;
use rlbot_bridge::{
    BallSnapshot, BodyState, CarSnapshot, FieldSpec, GameSnapshot, GoalSpec,
    InputState as BridgeInput, PadSpec, PadState, Phase, PredictionSlice,
};
use rocketsim::{Arena, ArenaConfig, ArenaEvent, CarBodyConfig, CarControls, GameMode, Team};

/// RocketSim length unit (uu = 1 cm) to metres.
const UU_TO_M: f32 = 1.0 / 100.0;

/// Metres per RocketSim Bullet unit. The raw collision-mesh vertices are in
/// Bullet units, where 1 BT = 0.5 m (5120 uu = 102.4 BT = 51.2 m).
const BT_TO_M: f32 = 0.5;

// --- Cosmetic constants ---------------------------------------------------
//
// These size the viewer's meshes. They are RocketSim's values (the physics is
// RocketSim), so there is one source of truth; the RocketSim constant each one
// comes from is named in its doc comment. Units are metres.

/// Soccar tick rate: RocketSim `consts::TICK_RATE` (120 Hz).
pub const TICK_HZ: f64 = 120.0;

/// How long the bridge reports `MatchPhase::Kickoff` after a kickoff reset.
/// RocketSim has no countdown/replay state machine, so this is the smallest
/// useful approximation: bots see a kickoff, then the match goes active.
const KICKOFF_TICKS: u64 = 120;

/// Number of ball-prediction slices. RLBot's reference server sends 720 (6 s at
/// 120 Hz); the viewer uses the same length so bots that index the full array
/// behave as they do against RLBot.
const PREDICTION_TICKS: usize = 720;

/// Soccar ball radius: RocketSim `consts::ball::RADIUS_SOCCAR` = 91.25 uu.
pub const BALL_RADIUS: f32 = 0.9125;

/// Soccar goal: half the mouth width, the crossbar height and the recess depth,
/// from RocketSim `consts::goal::SOCCAR_GOAL_HALF_WIDTH` (892.755 uu),
/// `SOCCAR_GOAL_HEIGHT` (642.775 uu) and `SOCCAR_GOAL_DEPTH` (880 uu).
pub const GOAL_HALF_WIDTH: f32 = 8.92755;
pub const GOAL_HEIGHT: f32 = 6.42775;
pub const GOAL_DEPTH: f32 = 8.8;

/// Octane hitbox half-extents: RocketSim `CarBodyConfig::OCTANE.hitbox_size`
/// (120.507 x 86.6994 x 38.6591 uu). These size the *visual* car mesh; collision
/// uses RocketSim's hitbox directly.
pub const CAR_HALF_LENGTH: f32 = 0.602535;
pub const CAR_HALF_WIDTH: f32 = 0.433_497;
pub const CAR_HALF_HEIGHT: f32 = 0.193_295_5;

/// Octane wheelbase, from RocketSim's front/back wheel offsets (51.25 and
/// -33.75 uu, i.e. 85 uu = 0.85 m apart). Visual wheel placement only.
pub const CAR_WHEELBASE: f32 = 0.85;

/// Normal impact speed at which a ball contact starts sounding like a bounce
/// rather than a roll. Rolling on the floor has almost no normal velocity.
const BALL_BOUNCE_MIN_SPEED: f32 = 1.2;

/// Minimum ticks between bounce sounds. A hard contact is reported for several
/// consecutive ticks; without this a single bounce would thud four times.
const BALL_BOUNCE_COOLDOWN_TICKS: u64 = 3;

/// Ball speed change (m/s) below which a car/ball contact is a nudge, not a hit.
const CAR_HIT_MIN_SPEED: f32 = 0.8;

/// Minimum ticks between car/ball impact sounds, for the same reason as the
/// bounce cooldown: a car pushing the ball reports a contact every tick.
const CAR_HIT_COOLDOWN_TICKS: u64 = 4;

/// RocketSim position (uu) -> viewer position (metres). See the module docs.
fn pos_view(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(-x, z, y) * UU_TO_M
}

/// RocketSim direction -> viewer direction. Same rotation as [`pos_view`] with
/// no unit change (for unit vectors and rad/s angular velocities).
fn dir_view(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(-x, z, y)
}

/// Player input, in the viewer's own terms; `main.rs` fills this from keyboard
/// and pad and [`step`] folds it into RocketSim's `CarControls`.
#[derive(Clone, Copy, Debug, Default)]
pub struct Input {
    /// -1 brake/reverse .. +1 accelerate.
    pub throttle: f64,
    /// -1 right .. +1 left.
    pub steer: f64,
    /// -1 nose down .. +1 nose up.
    pub pitch: f64,
    /// -1 right .. +1 left (also the dodge lean).
    pub yaw: f64,
    /// -1 right .. +1 left (Q / E).
    pub roll: f64,
    pub boost: bool,
    pub handbrake: bool,
    pub jump: bool,
}

impl Input {
    /// Driving/meta constructor; air axes centred.
    pub fn new(throttle: f64, steer: f64, boost: bool, handbrake: bool, jump: bool) -> Self {
        Self {
            throttle: throttle.clamp(-1.0, 1.0),
            steer: steer.clamp(-1.0, 1.0),
            boost,
            handbrake,
            jump,
            ..Default::default()
        }
    }

    /// Full constructor including the air-orientation axes.
    #[allow(clippy::too_many_arguments)]
    pub fn with_air(
        throttle: f64,
        steer: f64,
        pitch: f64,
        yaw: f64,
        roll: f64,
        boost: bool,
        handbrake: bool,
        jump: bool,
    ) -> Self {
        Self {
            throttle: throttle.clamp(-1.0, 1.0),
            steer: steer.clamp(-1.0, 1.0),
            pitch,
            yaw,
            roll,
            boost,
            handbrake,
            jump,
        }
    }
}

/// Everything the viewer reads about the car, already in the viewer's frame.
#[derive(Clone, Copy)]
pub struct CarView {
    pub pos: Vec3,
    pub forward: Vec3,
    pub up: Vec3,
    /// Whether at least three wheels are in contact with a surface, so the car
    /// is driving rather than flying. True on the walls as well as the floor.
    pub is_on_ground: bool,
    /// Whether the car has been demolished. RocketSim keeps the car in the
    /// world while `demo_respawn_timer` runs; the viewer hides it until it
    /// respawns, so a demolition does not look like an idle car teleporting.
    pub is_demoed: bool,
    /// Total speed, m/s.
    pub speed: f32,
    /// Speed along the car's forward axis, m/s (negative when reversing).
    pub forward_speed: f32,
    /// Boost meter, 0..100.
    pub boost: f32,
}

/// Everything the viewer reads about the ball.
#[derive(Clone, Copy)]
pub struct BallView {
    pub pos: Vec3,
    /// Velocity, m/s, in the viewer's frame.
    pub vel: Vec3,
    /// Total speed, m/s.
    pub speed: f32,
    /// Angular velocity (spin), rad/s.
    pub spin: Vec3,
}

/// A single boost pad.
#[derive(Clone, Copy)]
pub struct PadView {
    pub pos: Vec3,
    pub is_big: bool,
    /// `true` when off cooldown (i.e. the boost is there to take).
    pub available: bool,
}

/// A physics event the viewer can hear. Positions and speeds are already in the
/// viewer's frame and units (metres), like the rest of this module's output.
///
/// `main.rs` forwards these to Bevy's message stream and `audio.rs` turns each
/// one into a one-shot sound.
#[derive(Clone, Copy, Debug, Message)]
pub enum SimEvent {
    /// The ball hit a wall, floor, ceiling or goal mesh.
    BallBounce {
        /// Contact point.
        pos: Vec3,
        /// Impact speed along the surface normal, m/s.
        speed: f32,
    },
    /// A car touched the ball.
    CarHitBall {
        /// Contact point.
        pos: Vec3,
        /// Impact strength in m/s (the larger of the added hit velocity and the
        /// ball's velocity change, so a stationary car struck by a fast ball
        /// still registers).
        strength: f32,
    },
    /// A car was demolished by another car.
    Demolition {
        /// Contact point.
        pos: Vec3,
    },
}

/// Fold the viewer's [`Input`] into RocketSim's `CarControls`.
///
/// Every axis passes straight through: the viewer's `Input` is already in
/// RocketSim's terms. Measured with `examples/axis_probe.rs`, positive `steer`,
/// `yaw` and `roll` all move/rotate the car to its **left**, and positive
/// `pitch` is nose-up - so the viewer's `yaw` is built to keep `steer`'s sign and
/// `pitch` as `-throttle`. The frame bridge is a rotation, so it preserves signs;
/// there is no handedness flip to correct, and negating an axis here would only
/// mirror it.
fn controls_from(input: &Input) -> CarControls {
    CarControls {
        throttle: input.throttle as f32,
        steer: input.steer as f32,
        pitch: input.pitch as f32,
        yaw: input.yaw as f32,
        roll: input.roll as f32,
        jump: input.jump,
        boost: input.boost,
        handbrake: input.handbrake,
    }
}

/// The simulation: a RocketSim Soccar arena with one or two cars.
pub struct Sim {
    arena: Arena,
    /// RocketSim handles, in viewer/player order. Index 0 is the human.
    cars: Vec<usize>,
    /// The viewer's last applied input per car. RLBot wants `last_input` in
    /// every packet, and RocketSim's state does not retain it.
    last_inputs: Vec<Input>,
    /// Tick at which `MatchPhase::Kickoff` gives way to `Active`.
    kickoff_until: u64,
    /// A car-less arena used to extrapolate the ball for RLBot's
    /// `BallPrediction`. It never steps the match arena.
    prediction_arena: Arena,
    /// Boost-pad indices ordered by y, then x, which is the order RLBot uses
    /// for both `FieldInfo` and `GamePacket`.
    pad_order: Vec<usize>,
    /// Goals scored, indexed `[blue, orange]`.
    score: [u32; 2],
    /// Events produced by the most recent [`Sim::step`], for the audio layer.
    events: Vec<SimEvent>,
    /// Earliest tick at which another ball-bounce event may be emitted.
    next_bounce_tick: u64,
    /// Ball velocity before the current tick, so a car/ball contact can be
    /// scored by how much the ball's velocity changed.
    prev_ball_vel: rocketsim::Vec3A,
    /// Earliest tick at which another car/ball impact sound may be emitted.
    next_car_hit_tick: u64,
}

impl Sim {
    /// Load the arena meshes and start a Soccar game with one Blue car.
    ///
    /// # Panics
    ///
    /// If the collision meshes in `./collision_meshes/` are missing or invalid.
    pub fn new() -> Self {
        Self::new_with_opponent(false)
    }

    /// Start with a Blue human car and an Orange opponent car.
    pub fn with_opponent() -> Self {
        Self::new_with_opponent(true)
    }

    fn new_with_opponent(opponent: bool) -> Self {
        // RocketSim needs the dumped arena collision meshes before it can build
        // an arena. See the module docs.
        init_rocketsim();

        // Fixed RNG seed: RocketSim draws kickoffs/respawns from it, and an
        // unseeded arena makes the whole run - and so the self-driving capture -
        // vary from launch to launch.
        let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_rng_seed(0));
        let mut cars = vec![arena.add_car(Team::Blue, CarBodyConfig::OCTANE)];
        if opponent {
            cars.push(arena.add_car(Team::Orange, CarBodyConfig::OCTANE));
        }
        arena.reset_to_random_kickoff(None);

        // `ball_only` is on for a Soccar arena with no cars, so this is the
        // cheap way to extrapolate the ball without cloning the full match.
        let prediction_arena = Arena::new(GameMode::Soccar);

        let mut pad_order: Vec<usize> = (0..arena.num_boost_pads()).collect();
        pad_order.sort_by(|&a, &b| {
            let pa = arena.get_boost_pad_config(a).pos;
            let pb = arena.get_boost_pad_config(b).pos;
            pa.y.partial_cmp(&pb.y)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| pa.x.partial_cmp(&pb.x).unwrap_or(std::cmp::Ordering::Equal))
        });

        let kickoff_until = arena.tick_count() + KICKOFF_TICKS;
        let prev_ball_vel = arena.get_ball_state().vel;
        Self {
            arena,
            last_inputs: vec![Input::default(); cars.len()],
            cars,
            kickoff_until,
            prediction_arena,
            pad_order,
            score: [0, 0],
            events: Vec::new(),
            next_bounce_tick: 0,
            prev_ball_vel,
            next_car_hit_tick: 0,
        }
    }

    /// Advance one tick (1/120 s), scoring and resetting on a goal.
    ///
    /// `inputs` must be in car order; index 0 is the human car. Missing entries
    /// are treated as neutral, which keeps a bot-less run working. Afterwards
    /// [`Sim::events`] holds the tick's audible contacts.
    pub fn step(&mut self, inputs: &[Input]) {
        self.events.clear();

        for (index, &car) in self.cars.iter().enumerate() {
            let input = inputs.get(index).copied().unwrap_or_default();
            self.last_inputs[index] = input;
            self.arena.set_car_controls(car, controls_from(&input));
        }
        self.prev_ball_vel = self.arena.get_ball_state().vel;
        self.arena.step_tick();
        self.collect_events();

        // RocketSim reports a goal but does not score it or reset itself, so we
        // do both here. Blue defends -Y, so a ball at +Y is in Orange's net.
        if self.arena.is_ball_scored() {
            if self.arena.get_ball_state().pos.y > 0.0 {
                self.score[0] += 1;
            } else {
                self.score[1] += 1;
            }
            self.reset_positions_after_goal();
        }
    }

    /// Translate the tick's RocketSim contacts into viewer-space events.
    ///
    /// Only contacts with an audible impact are kept: a ball resting or rolling
    /// on the floor reports `BallHitWorld` every tick, but its normal speed is
    /// near zero and it must stay silent.
    fn collect_events(&mut self) {
        let ball = *self.arena.get_ball_state();
        let tick = self.arena.tick_count();
        let ball_delta = (ball.vel - self.prev_ball_vel).length() * UU_TO_M;

        for event in self.arena.get_last_step_events() {
            match *event {
                ArenaEvent::BallHitWorld(hit) => {
                    // The normal points off the surface, and after the solver
                    // runs the ball is moving away from it, so this is the
                    // impact speed (whoosh along the wall is ignored).
                    let speed = ball.vel.dot(hit.contact_normal).abs() * UU_TO_M;
                    if speed >= BALL_BOUNCE_MIN_SPEED && tick >= self.next_bounce_tick {
                        self.next_bounce_tick = tick + BALL_BOUNCE_COOLDOWN_TICKS;
                        self.events.push(SimEvent::BallBounce {
                            pos: pos_view(
                                hit.contact_point.x,
                                hit.contact_point.y,
                                hit.contact_point.z,
                            ),
                            speed,
                        });
                    }
                }
                ArenaEvent::CarHitBall(hit) => {
                    // `extra_hit_vel` is the impulse the car added; a stationary
                    // car struck by a moving ball adds none, so also use half
                    // the ball's velocity change (the solver bounce is counted
                    // twice in the change: in and out).
                    let added = hit.extra_hit_vel.length() * UU_TO_M;
                    let strength = added.max(ball_delta * 0.5);
                    if strength >= CAR_HIT_MIN_SPEED && tick >= self.next_car_hit_tick {
                        self.next_car_hit_tick = tick + CAR_HIT_COOLDOWN_TICKS;
                        self.events.push(SimEvent::CarHitBall {
                            pos: pos_view(
                                hit.contact_point.x,
                                hit.contact_point.y,
                                hit.contact_point.z,
                            ),
                            strength,
                        });
                    }
                }
                ArenaEvent::CarHitCar(hit) if hit.is_demo => {
                    self.events.push(SimEvent::Demolition {
                        pos: pos_view(
                            hit.contact_point.x,
                            hit.contact_point.y,
                            hit.contact_point.z,
                        ),
                    });
                }
                _ => {}
            }
        }
    }

    /// Events from the most recent [`Sim::step`], in the viewer's frame.
    pub fn events(&self) -> &[SimEvent] {
        &self.events
    }

    /// Reset the score and put the cars and ball back on their kickoff spots.
    pub fn reset(&mut self) {
        self.score = [0, 0];
        self.reset_positions_after_goal();
    }

    fn reset_positions_after_goal(&mut self) {
        self.arena.reset_to_random_kickoff(None);
        self.last_inputs.fill(Input::default());
        self.kickoff_until = self.arena.tick_count() + KICKOFF_TICKS;
        self.events.clear();
    }

    /// Ticks simulated so far.
    pub fn tick(&self) -> u64 {
        self.arena.tick_count()
    }

    /// Goals scored, `[blue, orange]`.
    pub fn score(&self) -> [u32; 2] {
        self.score
    }

    /// Number of boost pads (34 for Soccar).
    pub fn num_pads(&self) -> usize {
        self.arena.num_boost_pads()
    }

    /// Pad `i`, in the viewer's frame.
    pub fn pad(&self, i: usize) -> PadView {
        let config = self.arena.get_boost_pad_config(i);
        let state = self.arena.get_boost_pad_state(i);
        PadView {
            pos: pos_view(config.pos.x, config.pos.y, config.pos.z),
            is_big: config.is_big,
            available: state.cooldown <= 0.0,
        }
    }

    /// The human car's state, in the viewer's frame.
    pub fn car(&self) -> CarView {
        self.car_at(0)
    }

    /// One car's state, in the viewer's frame.
    pub fn car_at(&self, index: usize) -> CarView {
        let car = self.arena.get_car_state(self.cars[index]);
        let forward = car.get_forward_dir();
        let up = car.get_up_dir();
        CarView {
            pos: pos_view(car.pos.x, car.pos.y, car.pos.z),
            forward: dir_view(forward.x, forward.y, forward.z),
            up: dir_view(up.x, up.y, up.z),
            is_on_ground: car.is_on_ground,
            is_demoed: car.is_demoed,
            speed: car.vel.length() * UU_TO_M,
            forward_speed: car.vel.dot(forward) * UU_TO_M,
            boost: car.boost,
        }
    }

    /// Number of simulated cars (1, or 2 with an RLBot opponent).
    pub fn num_cars(&self) -> usize {
        self.cars.len()
    }

    /// The last input applied to car `index`, for view-only wheel animation.
    pub fn last_input(&self, index: usize) -> Input {
        self.last_inputs.get(index).copied().unwrap_or_default()
    }

    /// The raw static field data RLBot expects, in RLBot's frame and units.
    pub fn rlbot_field(&self) -> FieldSpec {
        FieldSpec {
            pads: self
                .pad_order
                .iter()
                .map(|&index| {
                    let pad = self.arena.get_boost_pad_config(index);
                    PadSpec {
                        pos: [pad.pos.x, pad.pos.y, pad.pos.z],
                        is_big: pad.is_big,
                    }
                })
                .collect(),
            goals: soccar_goals(),
        }
    }

    /// Build one RLBot tick packet.
    ///
    /// `prediction` is optional because it costs a few hundred ball-only
    /// RocketSim steps; only ask for it when a connected bot wants it.
    pub fn rlbot_snapshot(&mut self, prediction: bool) -> GameSnapshot {
        let cars = (0..self.num_cars())
            .map(|index| self.rlbot_car(index))
            .collect();
        let pads = self
            .pad_order
            .iter()
            .map(|&index| {
                let state = self.arena.get_boost_pad_state(index);
                PadState {
                    is_active: state.cooldown <= 0.0,
                    timer: state.cooldown.max(0.0),
                }
            })
            .collect();
        let ball = self.rlbot_ball();
        let prediction = if prediction {
            self.rlbot_ball_prediction()
        } else {
            Vec::new()
        };

        let tick = self.tick();
        let seconds_elapsed = tick as f32 / TICK_HZ as f32;
        GameSnapshot {
            frame: tick as u32,
            seconds_elapsed,
            // The viewer has no match timer; report unlimited time so bots do
            // not try to end a match that will never end on its own.
            game_time_remaining: seconds_elapsed,
            is_unlimited_time: true,
            phase: if tick < self.kickoff_until {
                Phase::Kickoff
            } else {
                Phase::Active
            },
            scores: self.score,
            cars,
            pads,
            ball,
            prediction,
            is_overtime: false,
        }
    }

    fn rlbot_car(&self, index: usize) -> CarSnapshot {
        let car = self.arena.get_car_state(self.cars[index]);
        let info = self.arena.get_car_info(self.cars[index]);
        let hitbox = info.config.hitbox_size;
        let hitbox_offset = info.config.hitbox_pos_offset;
        CarSnapshot {
            physics: body_state(&car.phys),
            boost: car.boost,
            is_supersonic: car.is_supersonic,
            is_on_ground: car.is_on_ground,
            is_jumping: car.is_jumping,
            is_flipping: car.is_flipping,
            has_jumped: car.has_jumped,
            has_double_jumped: car.has_double_jumped,
            has_flipped: car.has_flipped,
            air_time_since_jump: car.air_time_since_jump,
            flip_time: car.flip_time,
            is_demoed: car.is_demoed,
            demo_respawn_timer: car.demo_respawn_timer,
            hitbox: [hitbox.x, hitbox.y, hitbox.z],
            hitbox_offset: [hitbox_offset.x, hitbox_offset.y, hitbox_offset.z],
            last_input: bridge_input(self.last_inputs[index]),
            // Same mapping as `rlbot-rocketsim`: a dodge vector in the car's
            // local frame, with (0, 0) for a stall.
            dodge_dir: [car.flip_rel_torque.y, -car.flip_rel_torque.x],
        }
    }

    fn rlbot_ball(&self) -> BallSnapshot {
        let ball = self.arena.get_ball_state();
        BallSnapshot {
            physics: body_state(&ball.phys),
            diameter: BALL_RADIUS * 200.0,
        }
    }

    /// Extrapolate the ball with no cars for RLBot's `BallPrediction`.
    ///
    /// `prediction_arena` is a Soccar arena with no cars, so its `ball_only`
    /// path skips car physics entirely. The match arena is never advanced here.
    fn rlbot_ball_prediction(&mut self) -> Vec<PredictionSlice> {
        let start = *self.arena.get_ball_state();
        self.prediction_arena.set_ball_state(start);
        let now = self.tick() as f32 / TICK_HZ as f32;
        let mut slices = Vec::with_capacity(PREDICTION_TICKS);
        for i in 0..PREDICTION_TICKS {
            if i > 0 {
                self.prediction_arena.step_tick();
            }
            let ball = self.prediction_arena.get_ball_state();
            slices.push(PredictionSlice {
                game_seconds: now + i as f32 / TICK_HZ as f32,
                physics: body_state(&ball.phys),
            });
        }
        slices
    }

    /// Test-only helper: drop the ball from `height` metres at the centre.
    /// Used to exercise [`SimEvent::BallBounce`] without needing a screenshot.
    #[cfg(test)]
    fn debug_drop_ball(&mut self, height: f32) {
        self.debug_set_ball(Vec3::new(0.0, height, 0.0), Vec3::ZERO);
    }

    /// Test-only helper: put the ball at a viewer-space position and velocity.
    #[cfg(test)]
    fn debug_set_ball(&mut self, pos: Vec3, vel: Vec3) {
        let mut ball = *self.arena.get_ball_state();
        // Inverse of `pos_view`/`dir_view`: viewer = (-x, z, y).
        ball.pos = rocketsim::Vec3A::new(-pos.x, pos.z, pos.y) * 100.0;
        ball.vel = rocketsim::Vec3A::new(-vel.x, vel.z, vel.y) * 100.0;
        ball.ang_vel = rocketsim::Vec3A::ZERO;
        self.arena.set_ball_state(ball);
    }

    /// Debug-only helper for eyeballing wall detail: park the car flat on the
    /// viewer's +X wall, facing +Z, halfway up. Used by an env-var path in
    /// `main` for free-cam screenshots; it is not part of normal play.
    pub fn debug_park_on_wall(&mut self) {
        let car = self.cars[0];
        let mut state = *self.arena.get_car_state(car);
        // viewer = (-x, z, y), so viewer (40.767, 10, 3) -> rs
        // (-40.767, 3, 10) metres, and RocketSim positions are in uu.
        state.phys.pos = rocketsim::Vec3A::new(
            -(40.96 - CAR_HALF_HEIGHT) * 100.0,
            3.0 * 100.0,
            10.0 * 100.0,
        );
        // forward +Z viewer = +Y rs; right +Y viewer = +Z rs; up -X viewer = +X rs.
        state.phys.rot_mat = rocketsim::Mat3A::from_cols(
            rocketsim::Vec3A::new(0.0, 1.0, 0.0),
            rocketsim::Vec3A::new(0.0, 0.0, 1.0),
            rocketsim::Vec3A::new(1.0, 0.0, 0.0),
        );
        state.phys.vel = rocketsim::Vec3A::ZERO;
        state.phys.ang_vel = rocketsim::Vec3A::ZERO;
        self.arena.set_car_state(car, state);
    }

    /// The ball's state, in the viewer's frame.
    pub fn ball(&self) -> BallView {
        let ball = self.arena.get_ball_state();
        BallView {
            pos: pos_view(ball.pos.x, ball.pos.y, ball.pos.z),
            vel: dir_view(ball.vel.x, ball.vel.y, ball.vel.z) * UU_TO_M,
            speed: ball.vel.length() * UU_TO_M,
            spin: dir_view(ball.ang_vel.x, ball.ang_vel.y, ball.ang_vel.z),
        }
    }
}

/// Load RocketSim's collision meshes.
///
/// The viewer normally runs from the workspace root, so `./collision_meshes`
/// works. `cargo test` runs from the `view/` crate root, so fall back to the
/// workspace directory one level up.
fn init_rocketsim() {
    if rocketsim::init_from_default(true).is_ok() {
        return;
    }
    if rocketsim::init("../collision_meshes", true).is_ok() {
        return;
    }
    panic!(
        "RocketSim could not load its arena collision meshes.\n\
         Dump them with RLArenaCollisionDumper and put the `.cmf` files in\n\
         `./collision_meshes/` (relative to where you run the viewer)."
    );
}

fn body_state(phys: &rocketsim::PhysState) -> BodyState {
    BodyState {
        pos: [phys.pos.x, phys.pos.y, phys.pos.z],
        rot: rotator_from_mat(phys.rot_mat),
        vel: [phys.vel.x, phys.vel.y, phys.vel.z],
        ang_vel: [phys.ang_vel.x, phys.ang_vel.y, phys.ang_vel.z],
    }
}

/// RocketSim's rotation matrix (columns: forward, right, up) to RLBot's
/// `Rotator` (pitch, yaw, roll in radians).
///
/// RLBot and RocketSim use the same right-handed, Z-up frame. RLBot's yaw is
/// zero at +X and increases clockwise from above; roll is positive to the
/// right. This is the same decomposition used by `rlbot-rocketsim`, kept here
/// so the only RocketSim frame knowledge remains in this module.
fn rotator_from_mat(m: rocketsim::Mat3A) -> [f32; 3] {
    let forward = m.x_axis;
    let right = m.y_axis;
    let up = m.z_axis;
    [
        forward.z.atan2(forward.x.hypot(forward.y)),
        forward.y.atan2(forward.x),
        (-right.z).atan2(up.z),
    ]
}

fn bridge_input(input: Input) -> BridgeInput {
    BridgeInput {
        throttle: input.throttle as f32,
        steer: input.steer as f32,
        pitch: input.pitch as f32,
        yaw: input.yaw as f32,
        roll: input.roll as f32,
        jump: input.jump,
        boost: input.boost,
        handbrake: input.handbrake,
    }
}

/// Soccar's two goals, in RLBot units. The back walls are at y = +/-5120 and
/// Blue defends -Y. `direction` points away from the opening, i.e. into the
/// net (RLBot convention), so bots can step 500 uu from the goal centre back
/// onto the pitch with `location - direction * 500`.
fn soccar_goals() -> Vec<GoalSpec> {
    let height = GOAL_HEIGHT * 100.0;
    let width = GOAL_HALF_WIDTH * 200.0;
    let z = height * 0.5;
    vec![
        GoalSpec {
            team: 0,
            pos: [0.0, -5120.0, z],
            direction: [0.0, -1.0, 0.0],
            width,
            height,
        },
        GoalSpec {
            team: 1,
            pos: [0.0, 5120.0, z],
            direction: [0.0, 1.0, 0.0],
            width,
            height,
        },
    ]
}

/// RocketSim's Soccar arena collision surface, in the viewer's frame (metres):
/// triangle positions and three vertex indices per triangle.
///
/// This is the geometry the ball and car actually collide with, so drawing it
/// makes the visible surface and the collision surface identical. Vertices are
/// in Bullet units in RocketSim's frame, so they take the same rotation as the
/// sim state (`pos_view`) with the BT scale instead of the uu scale.
pub fn arena_collision_mesh() -> (Vec<Vec3>, Vec<[u32; 3]>) {
    let files = rocketsim::get_arena_collision_mesh_files(GameMode::Soccar);
    let mut positions = Vec::new();
    let mut triangles = Vec::new();

    // The dumped meshes carry only the *curved* parts of the arena (the corners,
    // the wall-to-floor and wall-to-ceiling fillets, and the goal ends). RocketSim
    // builds the flat floor, ceiling and long side walls from static planes (see
    // its `setup_arena_collision_shapes`), so add those here to draw the whole
    // collision surface. The arena AABB bounds match those planes.
    let aabb = rocketsim::consts::arena::get_aabb(GameMode::Soccar);
    let (a, b) = (
        pos_view(aabb.min.x, aabb.min.y, aabb.min.z),
        pos_view(aabb.max.x, aabb.max.y, aabb.max.z),
    );
    // `pos_view` swaps axes, so re-sort to get the true min/max corners.
    let (lo, hi) = (a.min(b), a.max(b));

    fn quad(positions: &mut Vec<Vec3>, triangles: &mut Vec<[u32; 3]>, p: [Vec3; 4]) {
        let base = positions.len() as u32;
        positions.extend_from_slice(&p);
        triangles.push([base, base + 1, base + 2]);
        triangles.push([base, base + 2, base + 3]);
    }

    // Floor (+Y), ceiling (-Y) and the two long side walls (+/-X). Winding is
    // irrelevant: the viewer orients every face inward.
    quad(
        &mut positions,
        &mut triangles,
        [
            Vec3::new(lo.x, lo.y, lo.z),
            Vec3::new(hi.x, lo.y, lo.z),
            Vec3::new(hi.x, lo.y, hi.z),
            Vec3::new(lo.x, lo.y, hi.z),
        ],
    );
    quad(
        &mut positions,
        &mut triangles,
        [
            Vec3::new(lo.x, hi.y, lo.z),
            Vec3::new(hi.x, hi.y, lo.z),
            Vec3::new(hi.x, hi.y, hi.z),
            Vec3::new(lo.x, hi.y, hi.z),
        ],
    );
    quad(
        &mut positions,
        &mut triangles,
        [
            Vec3::new(hi.x, lo.y, lo.z),
            Vec3::new(hi.x, lo.y, hi.z),
            Vec3::new(hi.x, hi.y, hi.z),
            Vec3::new(hi.x, hi.y, lo.z),
        ],
    );
    quad(
        &mut positions,
        &mut triangles,
        [
            Vec3::new(lo.x, lo.y, lo.z),
            Vec3::new(lo.x, lo.y, hi.z),
            Vec3::new(lo.x, hi.y, hi.z),
            Vec3::new(lo.x, hi.y, lo.z),
        ],
    );

    for file in &files {
        let base = positions.len() as u32;
        for v in file.get_vertices() {
            positions.push(Vec3::new(-v.x, v.z, v.y) * BT_TO_M);
        }
        for tri in file.get_indices().as_chunks::<3>().0 {
            triangles.push([
                base + tri[0] as u32,
                base + tri[1] as u32,
                base + tri[2] as u32,
            ]);
        }
    }
    (positions, triangles)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn air_control_axes_pass_through_unnegated() {
        // `controls_from` must not flip any axis: the viewer's `Input` is already
        // in RocketSim's sign convention (measured by `examples/axis_probe.rs`,
        // where positive steer/yaw/roll all go to the car's left). This would
        // fail if a negation were reintroduced.
        let input = Input::with_air(0.0, 0.0, 0.3, -0.6, 0.9, false, false, false);
        let controls = controls_from(&input);
        assert_eq!(controls.pitch, 0.3);
        assert_eq!(controls.yaw, -0.6);
        assert_eq!(controls.roll, 0.9);
    }

    #[test]
    fn rotator_matches_known_car_attitudes() {
        let identity = rotator_from_mat(rocketsim::Mat3A::IDENTITY);
        assert!(identity.iter().all(|value| value.abs() < 1e-5));

        // Nose straight up: forward +Z, right +Y, up -X.
        let nose_up = rocketsim::Mat3A::from_cols(
            rocketsim::Vec3A::Z,
            rocketsim::Vec3A::Y,
            -rocketsim::Vec3A::X,
        );
        assert!((rotator_from_mat(nose_up)[0] - std::f32::consts::FRAC_PI_2).abs() < 1e-5);

        // Facing +Y with up +Z: RLBot yaw is +90 degrees (towards the right).
        let yaw_right = rocketsim::Mat3A::from_cols(
            rocketsim::Vec3A::Y,
            -rocketsim::Vec3A::X,
            rocketsim::Vec3A::Z,
        );
        assert!((rotator_from_mat(yaw_right)[1] - std::f32::consts::FRAC_PI_2).abs() < 1e-5);

        // Rolled right 90 degrees: right points down (-Z), up points right (+Y).
        let roll_right = rocketsim::Mat3A::from_cols(
            rocketsim::Vec3A::X,
            -rocketsim::Vec3A::Z,
            rocketsim::Vec3A::Y,
        );
        assert!((rotator_from_mat(roll_right)[2] - std::f32::consts::FRAC_PI_2).abs() < 1e-5);
    }

    #[test]
    fn a_falling_ball_reports_a_bounce() {
        let mut sim = Sim::new();
        sim.debug_drop_ball(3.0);

        let mut bounce = None;
        for _ in 0..240 {
            sim.step(&[Input::default()]);
            bounce = sim.events().iter().find_map(|event| match *event {
                SimEvent::BallBounce { speed, .. } => Some(speed),
                _ => None,
            });
            if bounce.is_some() {
                break;
            }
        }

        let speed = bounce.expect("a 3 m drop should hit the floor and bounce");
        assert!(speed >= BALL_BOUNCE_MIN_SPEED);
        assert!(speed < 20.0, "impact speed {speed} m/s is implausible");
    }

    #[test]
    fn a_car_hitting_the_ball_reports_an_impact() {
        let mut sim = Sim::new();
        let car = sim.car();
        // Put the ball on the floor in front of the car and drive it back into
        // the car. A stationary car adds no hit impulse, so this also checks
        // the ball-velocity-change fallback.
        let pos = car.pos + car.forward * 2.5 + Vec3::Y * BALL_RADIUS;
        sim.debug_set_ball(pos, -car.forward * 20.0);

        let mut impact = None;
        for _ in 0..90 {
            sim.step(&[Input::default()]);
            impact = sim.events().iter().find_map(|event| match *event {
                SimEvent::CarHitBall { strength, .. } => Some(strength),
                _ => None,
            });
            if impact.is_some() {
                break;
            }
        }

        let strength = impact.expect("a ball driven into the car should register a hit");
        assert!(strength >= CAR_HIT_MIN_SPEED);
    }

    #[test]
    fn a_resting_ball_stays_silent() {
        let mut sim = Sim::new();
        for _ in 0..240 {
            sim.step(&[Input::default()]);
            assert!(
                !sim.events()
                    .iter()
                    .any(|event| matches!(event, SimEvent::BallBounce { .. })),
                "a resting ball must not trigger bounce sounds"
            );
        }
    }

    #[test]
    fn rlbot_snapshot_has_two_cars_and_a_prediction() {
        let mut sim = Sim::with_opponent();
        let inputs = vec![Input::default(); sim.num_cars()];
        for _ in 0..4 {
            sim.step(&inputs);
        }

        let snapshot = sim.rlbot_snapshot(true);
        assert_eq!(snapshot.cars.len(), 2);
        assert_eq!(snapshot.pads.len(), sim.num_pads());
        assert_eq!(snapshot.prediction.len(), PREDICTION_TICKS);
        assert!(snapshot.ball.diameter > 180.0);
        // The prediction starts at the current ball position.
        assert_eq!(
            snapshot.prediction[0].physics.pos,
            snapshot.ball.physics.pos
        );

        let field = sim.rlbot_field();
        assert_eq!(field.pads.len(), 34);
        assert_eq!(field.goals.len(), 2);
        // RLBot's goal direction points away from the opening, into the net.
        assert_eq!(field.goals[0].direction, [0.0, -1.0, 0.0]);
        assert_eq!(field.goals[1].direction, [0.0, 1.0, 0.0]);
        // The pad list must be ordered by y, then x.
        assert!(field
            .pads
            .windows(2)
            .all(|pair| { (pair[0].pos[1], pair[0].pos[0]) <= (pair[1].pos[1], pair[1].pos[0]) }));
    }
}
