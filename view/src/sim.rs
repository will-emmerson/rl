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

/// The simulation: a RocketSim Soccar arena with one Blue car.
pub struct Sim {
    arena: Arena,
    car: usize,
    /// Goals scored, indexed `[blue, orange]`.
    score: [u32; 2],
}

impl Sim {
    /// Load the arena meshes and start a Soccar game with one car.
    ///
    /// # Panics
    ///
    /// If the collision meshes in `./collision_meshes/` are missing or invalid.
    pub fn new() -> Self {
        // RocketSim needs the dumped arena collision meshes before it can build
        // an arena. See the module docs.
        if rocketsim::init_from_default(true).is_err() {
            panic!(
                "RocketSim could not load its arena collision meshes.\n\
                 Dump them with RLArenaCollisionDumper and put the `.cmf` files in\n\
                 `./collision_meshes/` (relative to where you run the viewer)."
            );
        }

        // Fixed RNG seed: RocketSim draws kickoffs/respawns from it, and an
        // unseeded arena makes the whole run - and so the self-driving capture -
        // vary from launch to launch.
        let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_rng_seed(0));
        let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
        arena.reset_to_random_kickoff(None);

        Self {
            arena,
            car,
            score: [0, 0],
        }
    }

    /// Advance one tick (1/120 s), scoring and resetting on a goal.
    pub fn step(&mut self, input: &Input) {
        self.arena.set_car_controls(self.car, controls_from(input));
        self.arena.step_tick();

        // RocketSim reports a goal but does not score it or reset itself, so we
        // do both here. Blue defends -Y, so a ball at +Y is in Orange's net.
        if self.arena.is_ball_scored() {
            if self.arena.get_ball_state().pos.y > 0.0 {
                self.score[0] += 1;
            } else {
                self.score[1] += 1;
            }
            self.arena.reset_to_random_kickoff(None);
        }
    }

    /// Reset the score and put the car and ball back on their kickoff spots.
    pub fn reset(&mut self) {
        self.score = [0, 0];
        self.arena.reset_to_random_kickoff(None);
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

    /// The car's state, in the viewer's frame.
    pub fn car(&self) -> CarView {
        let car = self.arena.get_car_state(self.car);
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

    /// Debug-only helper for eyeballing wall detail: park the car flat on the
    /// viewer's +X wall, facing +Z, halfway up. Used by an env-var path in
    /// `main` for free-cam screenshots; it is not part of normal play.
    pub fn debug_park_on_wall(&mut self) {
        let mut state = *self.arena.get_car_state(self.car);
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
        self.arena.set_car_state(self.car, state);
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
}
