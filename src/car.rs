//! The car.
//!
//! Deliberately *not* a rigid body with wheels. Rocket League's cars are not
//! wheel-driven in the ordinary sense — they are a bespoke driving model with a
//! speed-dependent steering curve, an analog powerslide, a box hitbox, and a
//! full 3D attitude once airborne (air control and flips).
//!
//! The steering, jump, air-control and flip models are ported from RocketSim
//! (`RLConst.h` + `Car::_UpdateWheels` / `_UpdateJump` / `_UpdateAirTorque` /
//! `_UpdateDoubleJumpOrFlip`); the engine/brake numbers are still tuned for feel.
//!
//! The driving model runs in the frame of whatever surface the car is on, so the
//! same steering and forces apply on the floor, the walls and the ceiling (see
//! `step_grounded` / `resolve_arena`) rather than only in the ground plane.

use crate::arena::Arena;
use crate::math::{piecewise_linear, rotate_about, tan_small, StateHasher, Vec3};
use crate::world::{Input, GRAVITY};

// RocketSim's internal Octane hitbox (`CAR_CONFIG_OCTANE`), in uu converted to
// metres. These differ from the numbers a Google search will give you (and from
// `GetLocalCollisionExtent`): RocketSim deliberately uses the smaller values
// because they reproduce Rocket League's inertia matrix, whereas the published
// ones do not. The hitbox is a bit *wider and taller* than the old 1.18 x 0.84
// x 0.36 figure. It is still noticeably smaller than the ball.
pub const CAR_HALF_LENGTH: f64 = 0.602535;
pub const CAR_HALF_WIDTH: f64 = 0.433497;
pub const CAR_HALF_HEIGHT: f64 = 0.1932955;
pub const CAR_MASS: f64 = 180.0;

/// Octane wheelbase: the front wheels sit +51.25 uu ahead of the car origin and
/// the rear wheels -33.75 uu behind (RocketSim `FRONT_WHEELS_OFFSET` /
/// `BACK_WHEELS_OFFSET`), so the wheelbase is 85 uu = 0.85 m. It sets the
/// bicycle-model yaw, `omega = v * tan(steer) / wheelbase`.
pub const CAR_WHEELBASE: f64 = 0.85;

/// Engine acceleration is not a RocketSim constant — RL drives the car through
/// a `btRaycastVehicle` wheel torque (`THROTTLE_TORQUE_AMOUNT`), not a linear
/// acceleration. This stays a tuned-for-feel value.
pub const CAR_ENGINE_ACCEL: f64 = 8.0;
/// `CAR_MAX_SPEED = 2300 uu/s`. Rocket League clamps the car's *total* speed to
/// this every tick, so it is a hard ceiling (see `Car::clamp_limits`). The
/// supersonic threshold `SUPERSONIC_START_SPEED` is 2200 uu/s.
pub const CAR_MAX_SPEED: f64 = 23.0;
/// `CAR_MAX_ANG_SPEED = 5.5 rad/s`: the car can never spin faster than this.
pub const CAR_MAX_ANG_SPEED: f64 = 5.5;
/// `BOOST_ACCEL_GROUND = 2975/3 uu/s^2`.
pub const CAR_BOOST_ACCEL_GROUND: f64 = 9.916_666_666_666_666;
/// `BOOST_ACCEL_AIR = 3175/3 uu/s^2`. Boost pulls slightly harder in the air.
pub const CAR_BOOST_ACCEL_AIR: f64 = 10.583_333_333_333_332;
/// Rocket League has no separate boosted speed cap — boost accelerates up to
/// `CAR_MAX_SPEED` like everything else.
pub const CAR_MAX_BOOST_SPEED: f64 = CAR_MAX_SPEED;
pub const CAR_OVERSPEED_DECEL: f64 = 2.0;
pub const CAR_REVERSE_ACCEL: f64 = 6.0;
pub const CAR_MAX_REVERSE_SPEED: f64 = 10.0;
pub const CAR_BRAKE_DECEL: f64 = 18.0;

/// Lateral velocity retained per tick. High grip keeps the velocity glued to
/// the heading; the handbrake keeps more of it, which is what makes a drift.
pub const CAR_GRIP_KEEP: f64 = 0.90;
pub const CAR_DRIFT_KEEP: f64 = 0.985;
pub const CAR_ROLLING_KEEP: f64 = 0.995;

/// Restitution for a car *bouncing* off a surface it is not landing on - a
/// sideways wall hit, or the floor while inverted. Not a RocketSim figure: RL
/// resolves those contacts with its suspension and tyre model, which this does
/// not have, so this is tuned for feel.
pub const CAR_SURFACE_RESTITUTION: f64 = 0.2;
/// How far a grounded car may drift clear of its surface before it is treated
/// as having left it. A grounded car is glued to the surface, so this is only a
/// sanity valve against numerical drift.
pub const CAR_SURFACE_SNAP: f64 = 0.10;
/// Minimum speed to stay glued to an *overhanging* surface (the ceiling, and the
/// upper part of the wall-to-ceiling fillet), in m/s. A surface can only push,
/// never pull, so with its normal pointing away from gravity it cannot hold the
/// car: the car stays only while it is moving fast enough to carry itself, and
/// peels off and falls once it slows. A floor or a wall has no such requirement.
/// Tuned for feel - RL resolves this through its tyre model.
pub const CAR_STICK_SPEED: f64 = 4.0;

/// Powerslide is an *analog* 0..1 value in Rocket League: it ramps in at
/// `POWERSLIDE_RISE_RATE` and out at `POWERSLIDE_FALL_RATE` per second, rather
/// than switching on the handbrake input.
pub const CAR_POWERSLIDE_RISE_RATE: f64 = 5.0;
pub const CAR_POWERSLIDE_FALL_RATE: f64 = 2.0;

pub const CAR_BOOST_MAX: f64 = 100.0;
/// `BOOST_USED_PER_SECOND = BOOST_MAX / 3`.
pub const CAR_BOOST_DRAIN: f64 = 33.333_333_333_333_336;

// Rocket League's jump, from RocketSim (`RLConst.h`). A jump is an immediate
// impulse plus a held force; a tap and a hold are different heights.

/// `JUMP_IMMEDIATE_FORCE = 875/3 uu/s`: an immediate upward impulse (delta-v)
/// applied when a jump starts — on the ground or as a double jump.
pub const CAR_JUMP_IMMEDIATE_SPEED: f64 = 2.916_666_666_666_667;
/// `JUMP_ACCEL = 4375/3 uu/s^2`: the held upward force, applied while the button
/// is down (up to `CAR_JUMP_MAX_TIME`).
pub const CAR_JUMP_ACCEL: f64 = 14.583_333_333_333_332;
/// `JUMP_MIN_TIME = 0.025 s`: a jump always burns at least this long.
pub const CAR_JUMP_MIN_TIME: f64 = 0.025;
/// `JUMP_MAX_TIME = 0.2 s`: holding the button longer than this adds nothing.
pub const CAR_JUMP_MAX_TIME: f64 = 0.2;
/// `JUMP_RESET_TIME_PAD = 1/40 s`: a small grace before the jump is re-armed.
pub const CAR_JUMP_RESET_TIME_PAD: f64 = 0.025;
/// `DOUBLEJUMP_MAX_DELAY = 1.25 s`: the double jump or flip must happen within
/// this of the first jump finishing.
pub const CAR_DOUBLEJUMP_MAX_DELAY: f64 = 1.25;
/// `JUMP_PRE_MIN_ACCEL_SCALE = 0.62`: the held force is reduced for the first
/// `CAR_JUMP_MIN_TIME`.
const CAR_JUMP_PRE_MIN_ACCEL_SCALE: f64 = 0.62;

// Air orientation control (RocketSim `_UpdateAirTorque` / `RLConst.h`).

/// `CAR_TORQUE_SCALE = 2*pi/(2^16)*1000`; the angular-acceleration scale the
/// air-control torque values are expressed in.
const CAR_TORQUE_SCALE: f64 = 0.095_873_799_242_852_57;
/// `CAR_AIR_CONTROL_TORQUE = (pitch, yaw, roll)`, in Rocket League's units.
const CAR_AIR_CONTROL_TORQUE: Vec3 = Vec3::new(130.0, 95.0, 400.0);
/// `CAR_AIR_CONTROL_DAMPING = (pitch, yaw, roll)`.
const CAR_AIR_CONTROL_DAMPING: Vec3 = Vec3::new(30.0, 20.0, 50.0);
/// `THROTTLE_AIR_ACCEL = 200/3 uu/s^2`: throttle still nudges the car forward
/// in the air.
const CAR_THROTTLE_AIR_ACCEL: f64 = 0.666_666_666_666_666_7;

// Flips / dodges (RocketSim `_UpdateDoubleJumpOrFlip`).

/// `(|pitch| + |yaw| + |roll|)` must reach this to flip rather than double jump.
const CAR_DODGE_DEADZONE: f64 = 0.5;
/// `FLIP_INITIAL_VEL_SCALE = 500 uu/s`: the base dodge velocity.
const CAR_FLIP_INITIAL_VEL: f64 = 5.0;
/// `FLIP_TORQUE_X` / `FLIP_TORQUE_Y`: the flip torque about the body forward and
/// right axes. Big enough to saturate `CAR_MAX_ANG_SPEED` almost immediately.
const CAR_FLIP_TORQUE_X: f64 = 260.0;
const CAR_FLIP_TORQUE_Y: f64 = 224.0;
/// `FLIP_TORQUE_TIME = 0.65 s`: how long the flip torque is applied.
const CAR_FLIP_TORQUE_TIME: f64 = 0.65;
/// `FLIP_PITCHLOCK_EXTRA_TIME = 0.3 s`: pitch air control stays locked for this
/// long after the flip torque ends.
const CAR_FLIP_PITCHLOCK_EXTRA_TIME: f64 = 0.3;
/// `FLIP_Z_DAMP_START` / `FLIP_Z_DAMP_END` / `FLIP_Z_DAMP_120`: between these
/// times the flip bleeds off downward speed (`pow(1-0.35, 1)` per 120 Hz tick).
const CAR_FLIP_Z_DAMP_START: f64 = 0.15;
const CAR_FLIP_Z_DAMP_END: f64 = 0.21;
const CAR_FLIP_Z_DAMP_KEEP: f64 = 0.65;
/// `FLIP_FORWARD/SIDE/BACKWARD_IMPULSE_MAX_SPEED_SCALE` and
/// `FLIP_BACKWARD_IMPULSE_SCALE_X`: how the dodge velocity scales with the
/// car's forward speed.
const CAR_FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE: f64 = 1.0;
const CAR_FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE: f64 = 1.9;
const CAR_FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE: f64 = 2.5;
const CAR_FLIP_BACKWARD_IMPULSE_SCALE_X: f64 = 16.0 / 15.0;

/// `CARBALL_COLLISION_RESTITUTION = 0.0`. A car-to-ball hit is *inelastic*:
/// the ball leaves at the relative contact speed, not twice it. Rocket League
/// then adds a separate "extra impulse" on top to make shots pop — see
/// `World::resolve_car_ball`.
pub const CAR_BALL_RESTITUTION: f64 = 0.0;
/// `CARBALL_COLLISION_FRICTION = 2.0`.
pub const CAR_BALL_FRICTION: f64 = 2.0;

// Rocket League's separate "extra" car-to-ball impulse (RocketSim `_OnHit`).
// Applied by the world after the ordinary collision; see
// `World::apply_extra_impulse`.
/// `BALL_CAR_EXTRA_IMPULSE_MAXDELTAVEL_UU = 4600 uu/s` — the closing speed fed
/// to the factor curve is clamped to this.
pub const BALL_CAR_EXTRA_IMPULSE_MAX_DELTA_VEL: f64 = 46.0;
/// `BALL_CAR_EXTRA_IMPULSE_Z_SCALE = 0.35` (soccar): squashes the vertical part
/// of the hit direction.
pub const BALL_CAR_EXTRA_IMPULSE_Z_SCALE: f64 = 0.35;
/// `BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE = 0.65`: how much of the car-forward
/// component of the hit direction is kept.
pub const BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE: f64 = 0.65;

#[derive(Clone, Copy, Debug)]
pub struct Car {
    /// Centre of the hitbox.
    pub pos: Vec3,
    /// Velocity. `y` is zero on the ground and carries the jump arc in the air.
    pub velocity: Vec3,
    /// Body forward axis, world space. Unit and, on the ground, horizontal.
    pub forward: Vec3,
    /// Body up axis, world space. Unit. `forward x up` is the body right.
    pub up: Vec3,
    /// Angular velocity in world space (rad/s). Ground steering sets its `y`.
    pub ang_vel: Vec3,
    /// The ground steering rate (rad/s); also mirrored into `ang_vel.y`.
    pub yaw_rate: f64,
    pub boost: f64,
    /// True while the car is off the ground, including the whole jump arc.
    pub airborne: bool,
    /// True once a ground jump has started, until the next landing.
    pub has_jumped: bool,
    /// True while the held jump force is still being applied.
    pub is_jumping: bool,
    /// Seconds since the jump started, reset on landing.
    pub jump_time: f64,
    /// True once the mid-air double jump has been used this airtime.
    pub has_double_jumped: bool,
    /// True once a flip has been used this airtime.
    pub has_flipped: bool,
    /// True while the flip torque is still being applied (< `CAR_FLIP_TORQUE_TIME`).
    pub is_flipping: bool,
    /// Body-frame dodge direction `(forward, right, 0)`, unit or zero. `x > 0`
    /// is a frontflip, `y > 0` a sideflip to the car's right.
    pub flip_dir: Vec3,
    /// Seconds since the flip started.
    pub flip_time: f64,
    /// Seconds of airtime since the jump *finished*, which gates the double
    /// jump / flip (`CAR_DOUBLEJUMP_MAX_DELAY`).
    pub air_time_since_jump: f64,
    /// Analog powerslide amount, 0..1.
    pub handbrake_val: f64,
    /// Whether jump was held on the previous tick. Public only for consistency
    /// with the other fields; it is really an implementation detail.
    pub jump_held: bool,
    /// Ticks remaining during which the car-to-ball "extra impulse" is blocked.
    /// Rocket League refuses to apply it on two consecutive ticks (RocketSim's
    /// `BallHitInfo::tickCountWhenExtraImpulseApplied`), which stops a resting
    /// contact from dumping in velocity every tick. Decremented once per step.
    pub extra_impulse_cooldown: u8,
}

impl Default for Car {
    fn default() -> Self {
        Self {
            pos: Vec3::new(0.0, CAR_HALF_HEIGHT, -14.0),
            velocity: Vec3::ZERO,
            forward: Vec3::new(0.0, 0.0, 1.0),
            up: Vec3::UP,
            ang_vel: Vec3::ZERO,
            yaw_rate: 0.0,
            boost: CAR_BOOST_MAX,
            airborne: false,
            has_jumped: false,
            is_jumping: false,
            jump_time: 0.0,
            has_double_jumped: false,
            has_flipped: false,
            is_flipping: false,
            flip_dir: Vec3::ZERO,
            flip_time: 0.0,
            air_time_since_jump: 0.0,
            handbrake_val: 0.0,
            jump_held: false,
            extra_impulse_cooldown: 0,
        }
    }
}

impl Car {
    /// Local axes of the hitbox: forward, up, right. Full 3D, so a flipping car
    /// presents its hitbox at an angle.
    pub fn axes(&self) -> [Vec3; 3] {
        [self.forward, self.up, self.right()]
    }

    pub fn half_extents(&self) -> Vec3 {
        Vec3::new(CAR_HALF_LENGTH, CAR_HALF_HEIGHT, CAR_HALF_WIDTH)
    }

    /// Body right axis (`forward x up`).
    pub fn right(&self) -> Vec3 {
        self.forward.cross(self.up)
    }

    /// Facing direction for the ground-plane maths and the viewers: the body
    /// forward flattened to the horizontal, or the full forward when the car is
    /// on a wall or the ceiling and that flattening would be degenerate.
    pub fn heading(&self) -> Vec3 {
        let h = self.forward.xz();
        if h.length_squared() > 1e-12 {
            h.normalized()
        } else {
            self.forward
        }
    }

    /// Angular velocity vector, used so the car can flick the ball while
    /// rotating (in the air, this includes pitch and roll).
    pub fn angular_velocity(&self) -> Vec3 {
        self.ang_vel
    }

    pub fn speed(&self) -> f64 {
        self.velocity.length()
    }

    /// Signed speed along the body forward axis.
    pub fn forward_speed(&self) -> f64 {
        self.velocity.dot(self.forward)
    }

    pub fn step(&mut self, dt: f64, input: &Input, arena: &Arena) {
        // Tick down the extra-impulse lockout before anything can re-arm it.
        self.extra_impulse_cooldown = self.extra_impulse_cooldown.saturating_sub(1);

        // Powerslide ramps in and out rather than switching instantly.
        if input.handbrake {
            self.handbrake_val += CAR_POWERSLIDE_RISE_RATE * dt;
        } else {
            self.handbrake_val -= CAR_POWERSLIDE_FALL_RATE * dt;
        }
        self.handbrake_val = self.handbrake_val.clamp(0.0, 1.0);

        // Jump is edge-triggered: `input.jump` is a held level, and holding it
        // must not re-fire the instant the car touches down again.
        let jump_pressed = input.jump && !self.jump_held;
        self.jump_held = input.jump;

        if self.airborne {
            self.step_airborne(dt, input);
        } else {
            self.step_grounded(dt, input, arena);
        }

        self.update_jump(dt, input, jump_pressed);

        self.pos += self.velocity * dt;
        self.resolve_arena(arena);
    }

    /// The driving model. Runs while the car is on a surface, which is not
    /// necessarily the floor: on a wall or the ceiling the same steering,
    /// engine, brake and boost apply in that surface's own frame.
    fn step_grounded(&mut self, dt: f64, input: &Input, arena: &Arena) {
        // Rotate the attitude (and the velocity with it) so `up` matches the
        // surface normal. Crossing from the floor onto a wall turns the whole
        // car by the angle the surface turned, which is what carries its speed
        // up the wall instead of stopping it dead.
        let normal = arena.normal_at(self.pos);
        self.align_up_to(normal);

        let up = self.up;
        let mut f = self.tangent_forward();
        self.forward = f;
        let r = f.cross(up);
        let mut v_forward = self.velocity.dot(f);
        let mut v_lateral = self.velocity.dot(r);

        // --- yaw ---
        // RL sets the front-wheel steer angle from the forward speed
        // (`STEER_ANGLE_FROM_SPEED_CURVE`), scales it by the steering input, and
        // the yaw that produces is the bicycle-model v * tan(delta) / wheelbase.
        // Because it is proportional to v, the car cannot pivot on the spot at a
        // standstill, and because v is signed, reversing mirrors the turn. On a
        // wall the steer axis is the surface normal, not world +Y.
        let speed_uu = v_forward.abs() * 100.0;
        let mut steer_angle = steer_angle_from_speed(speed_uu);
        if self.handbrake_val > 0.0 {
            // Powerslide moves the usable steer angle towards its own curve.
            steer_angle +=
                (powerslide_steer_angle_from_speed(speed_uu) - steer_angle) * self.handbrake_val;
        }
        steer_angle *= input.steer;
        self.yaw_rate = v_forward * tan_small(steer_angle) / CAR_WHEELBASE;
        self.forward = rotate_about(f, up, self.yaw_rate * dt);
        self.forward = self.tangent_forward();
        self.up = up;
        self.ang_vel = up * self.yaw_rate;
        f = self.forward;
        let r = f.cross(up);

        // --- gravity along the slope ---
        // A grounded car is held to the surface, so only gravity's component
        // *along* the surface counts: none on the flat floor or the ceiling, a
        // full pull down a vertical wall.
        let gravity = Vec3::new(0.0, GRAVITY, 0.0);
        let g_tangent = gravity - up * gravity.dot(up);
        v_forward += g_tangent.dot(f) * dt;
        v_lateral += g_tangent.dot(r) * dt;

        // --- longitudinal ---
        let boosting = input.boost && self.boost > 0.0;
        if input.throttle > 0.0 {
            v_forward += CAR_ENGINE_ACCEL * input.throttle * dt;
        } else if input.throttle < 0.0 {
            if v_forward > 0.5 {
                v_forward = (v_forward - CAR_BRAKE_DECEL * dt).max(0.0);
            } else {
                v_forward -= CAR_REVERSE_ACCEL * -input.throttle * dt;
            }
        } else {
            v_forward *= CAR_ROLLING_KEEP;
        }

        if boosting {
            v_forward += CAR_BOOST_ACCEL_GROUND * dt;
            self.boost = (self.boost - CAR_BOOST_DRAIN * dt).max(0.0);
        } else if v_forward > CAR_MAX_SPEED {
            // Bleed off speed carried from a boost.
            v_forward = (v_forward - CAR_OVERSPEED_DECEL * dt).max(CAR_MAX_SPEED);
        }

        let cap = if boosting {
            CAR_MAX_BOOST_SPEED
        } else {
            CAR_MAX_SPEED
        };
        v_forward = v_forward.clamp(-CAR_MAX_REVERSE_SPEED, cap);

        // --- lateral grip ---
        // Grip blends towards the looser drift value as the powerslide builds.
        let keep = CAR_GRIP_KEEP + (CAR_DRIFT_KEEP - CAR_GRIP_KEEP) * self.handbrake_val;
        v_lateral *= keep;

        self.velocity = f * v_forward + r * v_lateral;
    }

    /// Airborne: gravity, air control / flips, boost, and whatever momentum the
    /// car left the ground with.
    fn step_airborne(&mut self, dt: f64, input: &Input) {
        // The same gravity the ball feels.
        self.velocity.y += GRAVITY * dt;

        // Air-orientation control and any in-progress flip both drive the
        // angular velocity (RocketSim `_UpdateAirTorque`).
        self.apply_air_control(dt, input);

        // No wheel traction in the air, so the ground yaw rate is zero; boost
        // and the air throttle still push along the (possibly tilted) forward.
        self.yaw_rate = 0.0;
        if input.boost && self.boost > 0.0 {
            self.velocity += self.forward * (CAR_BOOST_ACCEL_AIR * dt);
            self.boost = (self.boost - CAR_BOOST_DRAIN * dt).max(0.0);
        }
        if input.throttle != 0.0 {
            self.velocity += self.forward * (input.throttle * CAR_THROTTLE_AIR_ACCEL * dt);
        }

        // Advance the attitude from the angular velocity. Done here (not in
        // `step`) so the grounded tick that *starts* a jump does not rotate the
        // heading twice.
        self.integrate_orientation(dt);
    }

    /// Rocket League's air control and in-flight flip torque
    /// (RocketSim `_UpdateAirTorque`).
    ///
    /// Input signs: `pitch` is +1 nose-up / -1 nose-down, `yaw` +1 turns (and
    /// sideflips) to the car's right, `roll` +1 rolls right. These are chosen so
    /// the behaviour matches Rocket League; the sim's world is left-handed, so
    /// the torque *axes* differ from RocketSim's cross products even though the
    /// magnitudes (`CAR_AIR_CONTROL_TORQUE`) and timing are RL's.
    fn apply_air_control(&mut self, dt: f64, input: &Input) {
        // The flip torque only runs for `CAR_FLIP_TORQUE_TIME`.
        if self.is_flipping {
            self.is_flipping = self.has_flipped && self.flip_time < CAR_FLIP_TORQUE_TIME;
        }

        let mut do_air_control = false;

        if self.is_flipping {
            let dodge = self.flip_dir;
            if dodge.length_squared() > 0.0 {
                let mut pitch_scale = 1.0;
                if dodge.x != 0.0 && input.pitch != 0.0 && same_sign(dodge.x, input.pitch) {
                    // Flip cancel: pitching against the flip eases its pitch
                    // and lets air control back in.
                    pitch_scale = 1.0 - input.pitch.abs().min(1.0);
                    do_air_control = true;
                }
                // A forward dodge flips the nose down; a rightward dodge rolls
                // the car to its right.
                let pitch_axis = -self.right();
                let roll_axis = self.forward;
                self.ang_vel += pitch_axis * (dodge.x * CAR_FLIP_TORQUE_Y * pitch_scale) * dt
                    + roll_axis * (dodge.y * CAR_FLIP_TORQUE_X) * dt;
            } else {
                do_air_control = true; // the flip has stalled
            }
        } else {
            do_air_control = true;
        }

        if !do_air_control {
            return;
        }

        let dir_pitch = self.right();
        let dir_yaw = -self.up;
        let dir_roll = self.forward;

        // Pitch air control is locked during a flip and briefly afterwards.
        let pitch_scale = if self.is_flipping
            || (self.has_flipped
                && self.flip_time < CAR_FLIP_TORQUE_TIME + CAR_FLIP_PITCHLOCK_EXTRA_TIME)
        {
            0.0
        } else {
            1.0
        };

        let torque = dir_pitch * (input.pitch * pitch_scale * CAR_AIR_CONTROL_TORQUE.x)
            + dir_yaw * (input.yaw * CAR_AIR_CONTROL_TORQUE.y)
            + dir_roll * (input.roll * CAR_AIR_CONTROL_TORQUE.z);

        // Damping always opposes existing rotation on each axis, so it is
        // independent of the axis orientation above.
        let damp_pitch = dir_pitch.dot(self.ang_vel)
            * CAR_AIR_CONTROL_DAMPING.x
            * (1.0 - (input.pitch * pitch_scale).abs());
        let damp_yaw =
            dir_yaw.dot(self.ang_vel) * CAR_AIR_CONTROL_DAMPING.y * (1.0 - input.yaw.abs());
        let damp_roll = dir_roll.dot(self.ang_vel) * CAR_AIR_CONTROL_DAMPING.z;
        let damping = dir_yaw * damp_yaw + dir_pitch * damp_pitch + dir_roll * damp_roll;

        self.ang_vel += (torque - damping) * (CAR_TORQUE_SCALE * dt);
    }

    /// Rocket League's jump, double jump and flip (RocketSim `_UpdateJump` +
    /// `_UpdateDoubleJumpOrFlip`): an immediate upward impulse on press, a held
    /// force for up to `CAR_JUMP_MAX_TIME`, and then either a double jump or a
    /// flip within `CAR_DOUBLEJUMP_MAX_DELAY` of the first jump finishing.
    fn update_jump(&mut self, dt: f64, input: &Input, jump_pressed: bool) {
        let on_ground = !self.airborne;

        // Once back on the ground, re-arm the jump — but wait out a small pad
        // so a minimum-time jump is not reset before we actually leave.
        if on_ground && !self.is_jumping {
            if self.has_jumped && self.jump_time < CAR_JUMP_MIN_TIME + CAR_JUMP_RESET_TIME_PAD {
                // Hold the reset; we might still be leaving the ground.
            } else {
                self.has_jumped = false;
                self.jump_time = 0.0;
            }
        }

        // Continue or finish an in-progress jump, or start a new one.
        if self.is_jumping {
            self.is_jumping = self.jump_time < CAR_JUMP_MIN_TIME
                || (input.jump && self.jump_time < CAR_JUMP_MAX_TIME);
        } else if on_ground && jump_pressed {
            self.is_jumping = true;
            self.jump_time = 0.0;
            // Jump along the car's own up axis, which is the surface normal on
            // the ground: straight up off the floor, straight out of a wall,
            // straight down off the ceiling.
            self.velocity += self.up * CAR_JUMP_IMMEDIATE_SPEED;
            self.airborne = true;
        }

        if self.is_jumping {
            self.has_jumped = true;
            // Reduced force before the minimum time, full force after.
            let scale = if self.jump_time < CAR_JUMP_MIN_TIME {
                CAR_JUMP_PRE_MIN_ACCEL_SCALE
            } else {
                1.0
            };
            self.velocity += self.up * (CAR_JUMP_ACCEL * scale * dt);
        }

        // The double jump / flip.
        if on_ground {
            self.has_double_jumped = false;
            self.air_time_since_jump = 0.0;
            self.has_flipped = false;
            self.is_flipping = false;
            self.flip_time = 0.0;
        } else {
            if self.has_jumped && !self.is_jumping {
                self.air_time_since_jump += dt;
            } else {
                self.air_time_since_jump = 0.0;
            }
            if jump_pressed
                && !self.has_double_jumped
                && !self.has_flipped
                && self.air_time_since_jump < CAR_DOUBLEJUMP_MAX_DELAY
            {
                let magnitude = input.pitch.abs() + input.yaw.abs() + input.roll.abs();
                if magnitude >= CAR_DODGE_DEADZONE {
                    self.start_flip(input);
                } else {
                    self.velocity += self.up * CAR_JUMP_IMMEDIATE_SPEED;
                    self.has_double_jumped = true;
                }
            }
        }

        if self.is_jumping || self.has_jumped {
            self.jump_time += dt;
        }

        // Flip timing, plus the downward-speed damping that makes a flip read
        // as a dodge rather than a launch.
        if self.is_flipping {
            self.flip_time += dt;
            if self.flip_time <= CAR_FLIP_TORQUE_TIME
                && self.flip_time >= CAR_FLIP_Z_DAMP_START
                && (self.velocity.y < 0.0 || self.flip_time < CAR_FLIP_Z_DAMP_END)
            {
                self.velocity.y *= CAR_FLIP_Z_DAMP_KEEP;
            }
        } else if self.has_flipped {
            self.flip_time += dt;
        }
    }

    /// Begin a flip/dodge (RocketSim `_UpdateDoubleJumpOrFlip`): an impulse in
    /// the dodge direction, plus the body-frame torque stored for
    /// `apply_air_control` to apply over the next few ticks.
    fn start_flip(&mut self, input: &Input) {
        // `(forward, right)` components of the stick, per RocketSim.
        let mut dodge = Vec3::new(-input.pitch, input.yaw + input.roll, 0.0);
        if dodge.x.abs() < 0.1 && dodge.y.abs() < 0.1 {
            dodge = Vec3::ZERO;
        } else {
            dodge = dodge.normalized();
        }

        self.flip_dir = dodge;
        self.flip_time = 0.0;
        self.has_flipped = true;
        self.is_flipping = true;

        // Tiny components are dropped before the direction becomes velocity.
        let dir_x = if dodge.x.abs() < 0.1 { 0.0 } else { dodge.x };
        let dir_y = if dodge.y.abs() < 0.1 { 0.0 } else { dodge.y };
        if dir_x == 0.0 && dir_y == 0.0 {
            return; // pure roll: the car spins but does not dodge
        }

        let forward_speed = self.forward_speed();
        let abs_forward = forward_speed.abs();
        let forward_ratio = abs_forward / CAR_MAX_SPEED;

        let should_dodge_backwards = if abs_forward < 1.0 {
            dir_x < 0.0
        } else {
            (dir_x >= 0.0) != (forward_speed >= 0.0)
        };

        let max_speed_scale_x = if should_dodge_backwards {
            CAR_FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE
        } else {
            CAR_FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE
        };
        let mut dodge_vel = Vec3::new(
            dir_x * CAR_FLIP_INITIAL_VEL,
            dir_y * CAR_FLIP_INITIAL_VEL,
            0.0,
        );
        dodge_vel.x *= ((max_speed_scale_x - 1.0) * forward_ratio) + 1.0;
        dodge_vel.y *= ((CAR_FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE - 1.0) * forward_ratio) + 1.0;
        if should_dodge_backwards {
            dodge_vel.x *= CAR_FLIP_BACKWARD_IMPULSE_SCALE_X;
        }

        // Push along the car's own forward/right axes, so a flip launched off a
        // wall aims the same way it would from the floor. On the ground those
        // axes are already horizontal, so this matches the old flattened dodge.
        self.velocity += self.forward * dodge_vel.x + self.right() * dodge_vel.y;
    }

    /// Integrate the body attitude from the angular velocity and re-orthonormalise.
    fn integrate_orientation(&mut self, dt: f64) {
        let w = self.ang_vel;
        let f = (self.forward + w.cross(self.forward) * dt).normalized();
        let u_raw = self.up + w.cross(self.up) * dt;
        let ortho = u_raw - f * u_raw.dot(f);
        let up = if ortho.length_squared() > 1e-12 {
            ortho.normalized()
        } else {
            // Degenerate: `up` became parallel to `forward`. Rebuild it from a
            // reference axis so the frame stays valid.
            let fallback = (Vec3::UP - f * f.y).normalized();
            if fallback.length_squared() > 1e-12 {
                fallback
            } else {
                (Vec3::new(1.0, 0.0, 0.0) - f * f.x).normalized()
            }
        };
        self.forward = f;
        self.up = up;
    }

    /// Rotate the car's attitude - and its velocity with it - so its up axis
    /// becomes `normal` (the surface normal, pointing into the arena). The
    /// rotation is the shortest arc from the current up, taken from the cross
    /// and dot products directly so no `acos`/`sin`/`cos` is needed and the step
    /// stays deterministic.
    fn align_up_to(&mut self, normal: Vec3) {
        let from = self.up;
        let axis = from.cross(normal);
        let sin = axis.length();
        let cos = from.dot(normal);
        if sin < 1e-9 {
            // Already aligned, or exactly opposite (which a car on a surface
            // never should be).
            self.up = normal;
            return;
        }
        let axis = axis / sin;
        let rotate = |v: Vec3| v * cos + axis.cross(v) * sin + axis * (axis.dot(v) * (1.0 - cos));
        self.forward = rotate(self.forward);
        self.velocity = rotate(self.velocity);
        self.ang_vel = rotate(self.ang_vel);
        self.up = normal;
    }

    /// The car's forward axis projected into the plane perpendicular to its up
    /// axis, i.e. the direction it is pointing along the surface.
    fn tangent_forward(&self) -> Vec3 {
        let f = self.forward - self.up * self.forward.dot(self.up);
        if f.length_squared() > 1e-12 {
            f.normalized()
        } else {
            // Forward collapsed onto the surface normal. There is no facing to
            // recover, so pick any tangent rather than returning a zero vector.
            let alt = Vec3::new(0.0, 0.0, 1.0) - self.up * self.up.z;
            if alt.length_squared() > 1e-12 {
                alt.normalized()
            } else {
                (Vec3::new(1.0, 0.0, 0.0) - self.up * self.up.x).normalized()
            }
        }
    }

    /// Put the car flat on its wheels, yaw-only. The model has no suspension or
    /// auto-roll, so a car that lands badly (inverted, or sideways off a wall)
    /// is simply set back on the ground rather than left to settle.
    fn snap_upright(&mut self) {
        let h = self.forward.xz();
        self.forward = if h.length_squared() > 1e-12 {
            h.normalized()
        } else {
            Vec3::new(0.0, 0.0, 1.0)
        };
        self.up = Vec3::UP;
        self.ang_vel = Vec3::ZERO;
    }

    /// Resolve the car against the arena surface, after its position has been
    /// integrated. The car is a box, but the contact is treated as a point at
    /// its centre held `CAR_HALF_HEIGHT` off the surface - enough for driving on
    /// the flat parts and around the rounded joins.
    ///
    /// A grounded car is glued to whatever surface it is on. An airborne one
    /// lands if it arrives wheels-down for that surface, and otherwise bounces.
    fn resolve_arena(&mut self, arena: &Arena) {
        let distance = arena.inside_distance(self.pos);
        let normal = arena.normal_at(self.pos);
        let penetration = distance + CAR_HALF_HEIGHT;

        if !self.airborne {
            // A surface can only push, never pull. An overhanging surface (the
            // ceiling, and the upper part of the rim fillet) has its normal
            // pointing away from gravity, so it cannot hold the car: the car
            // stays only while it is fast enough, and otherwise peels off and
            // falls. Everywhere else the car is simply glued on.
            let overhanging = normal.y < 0.0;
            let glued =
                penetration > -CAR_SURFACE_SNAP && !(overhanging && self.speed() < CAR_STICK_SPEED);
            if glued {
                // Sit exactly on the surface and remove any normal drift, so a
                // curved surface never accumulates a gap or a penetration.
                self.pos += normal * penetration;
                let vn = self.velocity.dot(normal);
                self.velocity -= normal * vn;
                return;
            }
            // Overhanging and too slow, or drifted well clear: fall.
            self.airborne = true;
        }

        if penetration <= 0.0 {
            return;
        }

        // Airborne contact: push out of the surface first.
        self.pos += normal * penetration;
        let vn = self.velocity.dot(normal);
        if vn >= 0.0 {
            return;
        }

        if self.up.dot(normal) > 0.5 {
            // Arriving wheels-down for this surface: land on it.
            self.airborne = false;
            self.align_up_to(normal);
            self.velocity -= normal * self.velocity.dot(normal);
        } else {
            // Sideways or inverted: bounce. If it is the floor, also snap
            // upright so the car can land properly next time.
            self.velocity -= normal * ((1.0 + CAR_SURFACE_RESTITUTION) * vn);
            if normal.y > 0.5 {
                self.snap_upright();
            }
        }
    }

    /// Rocket League clamps both the car's *total* speed and its angular speed
    /// every tick (RocketSim's `Car::_FinishPhysicsTick`), so neither boost nor
    /// a collision can leave it overspeed. Applied once per tick, after
    /// collisions, by `World::step`.
    pub fn clamp_limits(&mut self) {
        let speed = self.velocity.length();
        if speed > CAR_MAX_SPEED {
            self.velocity = self.velocity * (CAR_MAX_SPEED / speed);
        }
        let spin = self.ang_vel.length();
        if spin > CAR_MAX_ANG_SPEED {
            self.ang_vel = self.ang_vel * (CAR_MAX_ANG_SPEED / spin);
        }
    }

    /// Bicycle-model turn radius at the current speed under full steering lock:
    /// `wheelbase / tan(steer_angle(v))`. Finite even at a standstill, because
    /// RL's wheels can hold full lock; the car just cannot yaw without rolling.
    pub fn turn_radius(&self) -> f64 {
        let delta = steer_angle_from_speed(self.forward_speed().abs() * 100.0);
        CAR_WHEELBASE / tan_small(delta)
    }

    pub fn hash_into(&self, h: &mut StateHasher) {
        self.pos.hash_into(h);
        self.velocity.hash_into(h);
        self.forward.hash_into(h);
        self.up.hash_into(h);
        self.ang_vel.hash_into(h);
        h.fold(self.yaw_rate);
        h.fold(self.boost);
        h.fold_u64(self.airborne as u64);
        h.fold_u64(self.has_jumped as u64);
        h.fold_u64(self.is_jumping as u64);
        h.fold(self.jump_time);
        h.fold_u64(self.has_double_jumped as u64);
        h.fold_u64(self.has_flipped as u64);
        h.fold_u64(self.is_flipping as u64);
        self.flip_dir.hash_into(h);
        h.fold(self.flip_time);
        h.fold(self.air_time_since_jump);
        h.fold(self.handbrake_val);
        h.fold_u64(self.jump_held as u64);
        h.fold_u64(self.extra_impulse_cooldown as u64);
    }
}

/// True if `a` and `b` share a sign (used by the flip-cancel check).
fn same_sign(a: f64, b: f64) -> bool {
    (a >= 0.0) == (b >= 0.0)
}

/// Rocket League's `STEER_ANGLE_FROM_SPEED_CURVE`: the maximum front-wheel
/// steer angle (radians) as a function of forward speed (uu/s). The angle falls
/// off sharply with speed, which is what makes the turn radius grow.
fn steer_angle_from_speed(speed_uu: f64) -> f64 {
    const POINTS: [(f64, f64); 6] = [
        (0.0, 0.53356),
        (500.0, 0.31930),
        (1000.0, 0.18203),
        (1500.0, 0.10570),
        (1750.0, 0.08507),
        (3000.0, 0.03454),
    ];
    piecewise_linear(&POINTS, speed_uu)
}

/// Rocket League's `POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE`, blended in as the
/// powerslide builds.
fn powerslide_steer_angle_from_speed(speed_uu: f64) -> f64 {
    const POINTS: [(f64, f64); 2] = [(0.0, 0.39235), (2500.0, 0.12610)];
    piecewise_linear(&POINTS, speed_uu)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steer_curve_matches_the_rocketsim_knots() {
        assert!((steer_angle_from_speed(0.0) - 0.53356).abs() < 1e-12);
        assert!((steer_angle_from_speed(1000.0) - 0.18203).abs() < 1e-12);
        assert!((steer_angle_from_speed(3000.0) - 0.03454).abs() < 1e-12);
        // Clamped beyond the final knot.
        assert!((steer_angle_from_speed(9000.0) - 0.03454).abs() < 1e-12);
    }

    #[test]
    fn turn_radius_follows_the_bicycle_model() {
        // radius = 0.85 / tan(delta(v)), with delta from the RL curve.
        let radius = |v: f64| {
            Car {
                velocity: Vec3::new(0.0, 0.0, 1.0) * v,
                ..Default::default()
            }
            .turn_radius()
        };
        assert!((radius(5.0) - 2.57).abs() < 0.05, "r(5) = {}", radius(5.0));
        assert!(
            (radius(10.0) - 4.62).abs() < 0.05,
            "r(10) = {}",
            radius(10.0)
        );
        assert!(
            (radius(22.0) - 12.69).abs() < 0.1,
            "r(22) = {}",
            radius(22.0)
        );
    }

    #[test]
    fn orientation_stays_orthonormal_under_spin() {
        let mut car = Car {
            airborne: true,
            ang_vel: Vec3::new(3.0, 4.0, 5.0),
            ..Default::default()
        };
        for _ in 0..600 {
            car.integrate_orientation(1.0 / 120.0);
        }
        assert!((car.forward.length() - 1.0).abs() < 1e-9);
        assert!((car.up.length() - 1.0).abs() < 1e-9);
        assert!(car.forward.dot(car.up).abs() < 1e-9);
    }
}
