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
use rlbot_bridge::{
    BallSnapshot, BodyState, CarSnapshot, FieldSpec, GameSnapshot, GoalSpec,
    InputState as BridgeInput, PadSpec, PadState, Phase, PredictionSlice,
};
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};

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
        Self {
            arena,
            last_inputs: vec![Input::default(); cars.len()],
            cars,
            kickoff_until,
            prediction_arena,
            pad_order,
            score: [0, 0],
        }
    }

    /// Advance one tick (1/120 s), scoring and resetting on a goal.
    ///
    /// `inputs` must be in car order; index 0 is the human car. Missing entries
    /// are treated as neutral, which keeps a bot-less run working.
    pub fn step(&mut self, inputs: &[Input]) {
        for (index, &car) in self.cars.iter().enumerate() {
            let input = inputs.get(index).copied().unwrap_or_default();
            self.last_inputs[index] = input;
            self.arena.set_car_controls(car, controls_from(&input));
        }
        self.arena.step_tick();

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

    /// Reset the score and put the cars and ball back on their kickoff spots.
    pub fn reset(&mut self) {
        self.score = [0, 0];
        self.reset_positions_after_goal();
    }

    fn reset_positions_after_goal(&mut self) {
        self.arena.reset_to_random_kickoff(None);
        self.last_inputs.fill(Input::default());
        self.kickoff_until = self.arena.tick_count() + KICKOFF_TICKS;
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
