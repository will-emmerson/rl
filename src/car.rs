//! The car.
//!
//! Deliberately *not* a rigid body with wheels. Rocket League's cars are not
//! wheel-driven in the ordinary sense — they are a bespoke driving model with a
//! speed-dependent steering curve, an analog powerslide, and a box hitbox.
//! Modelling it directly is both simpler and closer to the real thing than
//! bolting four springs onto a box and hoping.
//!
//! The steering and jump models here are ported from RocketSim
//! (`RLConst.h` + `Car::_UpdateWheels` / `_UpdateJump` /
//! `_UpdateDoubleJumpOrFlip`); the engine/brake numbers are still tuned for feel.

use crate::arena::Arena;
use crate::math::{
    heading_right, piecewise_linear, rotate_y, tan_small, StateHasher, Vec3,
};
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
/// this every tick, so it is a hard ceiling (see `Car::clamp_speed`). The
/// supersonic threshold `SUPERSONIC_START_SPEED` is 2200 uu/s.
pub const CAR_MAX_SPEED: f64 = 23.0;
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
/// `DOUBLEJUMP_MAX_DELAY = 1.25 s`: the double jump must happen within this of
/// the first jump finishing.
pub const CAR_DOUBLEJUMP_MAX_DELAY: f64 = 1.25;
/// `JUMP_PRE_MIN_ACCEL_SCALE = 0.62`: the held force is reduced for the first
/// `CAR_JUMP_MIN_TIME`.
const CAR_JUMP_PRE_MIN_ACCEL_SCALE: f64 = 0.62;

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
    /// Unit heading in the XZ plane. +Z at spawn.
    pub heading: Vec3,
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
    /// Seconds of airtime since the jump *finished*, which gates the double
    /// jump (`CAR_DOUBLEJUMP_MAX_DELAY`).
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
            heading: Vec3::new(0.0, 0.0, 1.0),
            yaw_rate: 0.0,
            boost: CAR_BOOST_MAX,
            airborne: false,
            has_jumped: false,
            is_jumping: false,
            jump_time: 0.0,
            has_double_jumped: false,
            air_time_since_jump: 0.0,
            handbrake_val: 0.0,
            jump_held: false,
            extra_impulse_cooldown: 0,
        }
    }
}

impl Car {
    /// Local axes of the hitbox: forward, up, right.
    pub fn axes(&self) -> [Vec3; 3] {
        let f = self.heading.normalized();
        [f, Vec3::UP, heading_right(f)]
    }

    pub fn half_extents(&self) -> Vec3 {
        Vec3::new(CAR_HALF_LENGTH, CAR_HALF_HEIGHT, CAR_HALF_WIDTH)
    }

    pub fn right(&self) -> Vec3 {
        heading_right(self.heading)
    }

    /// Angular velocity vector, used so the car can flick the ball while
    /// rotating.
    pub fn angular_velocity(&self) -> Vec3 {
        Vec3::new(0.0, self.yaw_rate, 0.0)
    }

    pub fn speed(&self) -> f64 {
        self.velocity.length()
    }

    /// Signed speed along the heading.
    pub fn forward_speed(&self) -> f64 {
        self.velocity.dot(self.heading)
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
            self.step_grounded(dt, input);
        }

        self.update_jump(dt, input, jump_pressed);

        self.pos += self.velocity * dt;
        self.resolve_vertical(arena);
        self.collide_walls(arena);
    }

    /// The driving model. Runs only while the car is on the ground.
    fn step_grounded(&mut self, dt: f64, input: &Input) {
        let f = self.heading;
        let r = self.right();
        let mut v_forward = self.velocity.dot(f);
        let mut v_lateral = self.velocity.dot(r);

        // --- yaw ---
        // RL sets the front-wheel steer angle from the forward speed
        // (`STEER_ANGLE_FROM_SPEED_CURVE`), scales it by the steering input, and
        // the yaw that produces is the bicycle-model v * tan(delta) / wheelbase.
        // Because it is proportional to v, the car cannot pivot on the spot at a
        // standstill, and because v is signed, reversing mirrors the turn.
        let speed_uu = v_forward.abs() * 100.0;
        let mut steer_angle = steer_angle_from_speed(speed_uu);
        if self.handbrake_val > 0.0 {
            // Powerslide moves the usable steer angle towards its own curve.
            steer_angle +=
                (powerslide_steer_angle_from_speed(speed_uu) - steer_angle) * self.handbrake_val;
        }
        steer_angle *= input.steer;
        self.yaw_rate = v_forward * tan_small(steer_angle) / CAR_WHEELBASE;
        self.heading = rotate_y(f, self.yaw_rate * dt);
        let f = self.heading;
        let r = self.right();

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

    /// Airborne: gravity, plus whatever momentum the car left the ground with.
    fn step_airborne(&mut self, dt: f64, input: &Input) {
        // The same gravity the ball feels.
        self.velocity.y += GRAVITY * dt;

        // No traction in the air. The engine and tyres do nothing and there is
        // no air control yet, so the car keeps its momentum. Boost still works,
        // as it does in RL, capped at the top speed.
        self.yaw_rate = 0.0;
        if input.boost && self.boost > 0.0 {
            self.velocity += self.heading * (CAR_BOOST_ACCEL_AIR * dt);
            self.boost = (self.boost - CAR_BOOST_DRAIN * dt).max(0.0);
            self.clamp_horizontal_speed(CAR_MAX_BOOST_SPEED);
        }
    }

    /// Rocket League's jump model (RocketSim `_UpdateJump` +
    /// `_UpdateDoubleJumpOrFlip`), minus flips and air control: an immediate
    /// upward impulse on press, a held force for up to `CAR_JUMP_MAX_TIME`, and
    /// a single mid-air double jump within `CAR_DOUBLEJUMP_MAX_DELAY` of the
    /// first jump finishing. A tap is a low hop; a hold is much higher.
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
            self.velocity.y += CAR_JUMP_IMMEDIATE_SPEED;
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
            self.velocity.y += CAR_JUMP_ACCEL * scale * dt;
        }

        // The double jump (no flips yet: the sim has no pitch/roll input).
        if on_ground {
            self.has_double_jumped = false;
            self.air_time_since_jump = 0.0;
        } else {
            if self.has_jumped && !self.is_jumping {
                self.air_time_since_jump += dt;
            } else {
                self.air_time_since_jump = 0.0;
            }
            if jump_pressed
                && !self.has_double_jumped
                && self.air_time_since_jump < CAR_DOUBLEJUMP_MAX_DELAY
            {
                self.velocity.y += CAR_JUMP_IMMEDIATE_SPEED;
                self.has_double_jumped = true;
            }
        }

        if self.is_jumping || self.has_jumped {
            self.jump_time += dt;
        }
    }

    /// Land on the floor, or stop at the ceiling. The box rests on the ground,
    /// so on the ground its centre sits at `CAR_HALF_HEIGHT`.
    fn resolve_vertical(&mut self, arena: &Arena) {
        if self.pos.y <= CAR_HALF_HEIGHT {
            self.pos.y = CAR_HALF_HEIGHT;
            self.velocity.y = self.velocity.y.max(0.0);
            self.airborne = false;
        }

        let ceiling = arena.height - CAR_HALF_HEIGHT;
        if self.pos.y > ceiling {
            self.pos.y = ceiling;
            self.velocity.y = self.velocity.y.min(0.0);
        }
    }

    fn clamp_horizontal_speed(&mut self, max: f64) {
        let horizontal = self.velocity.xz();
        let speed = horizontal.length();
        if speed > max {
            let scale = max / speed;
            self.velocity.x = horizontal.x * scale;
            self.velocity.z = horizontal.z * scale;
        }
    }

    /// Rocket League clamps the car's *total* speed to `CAR_MAX_SPEED` every
    /// tick (RocketSim's `Car::_FinishPhysicsTick`), so a car cannot be pushed
    /// past its own top speed by a collision with the ball. Applied once per
    /// tick, after collisions, by `World::step`.
    pub fn clamp_speed(&mut self) {
        let speed = self.velocity.length();
        if speed > CAR_MAX_SPEED {
            self.velocity = self.velocity * (CAR_MAX_SPEED / speed);
        }
    }

    fn collide_walls(&mut self, arena: &Arena) {
        // Footprint of the oriented box projected onto the world axes.
        let f = self.heading;
        let r = self.right();
        let half_x = f.x.abs() * CAR_HALF_LENGTH + r.x.abs() * CAR_HALF_WIDTH;
        let half_z = f.z.abs() * CAR_HALF_LENGTH + r.z.abs() * CAR_HALF_WIDTH;

        let limit_x = arena.half_x - half_x;
        if self.pos.x > limit_x {
            self.pos.x = limit_x;
            self.velocity.x = self.velocity.x.min(0.0);
        } else if self.pos.x < -limit_x {
            self.pos.x = -limit_x;
            self.velocity.x = self.velocity.x.max(0.0);
        }

        let limit_z = arena.half_z - half_z;
        if self.pos.z > limit_z {
            self.pos.z = limit_z;
            self.velocity.z = self.velocity.z.min(0.0);
        } else if self.pos.z < -limit_z {
            self.pos.z = -limit_z;
            self.velocity.z = self.velocity.z.max(0.0);
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
        self.heading.hash_into(h);
        h.fold(self.yaw_rate);
        h.fold(self.boost);
        h.fold_u64(self.airborne as u64);
        h.fold_u64(self.has_jumped as u64);
        h.fold_u64(self.is_jumping as u64);
        h.fold(self.jump_time);
        h.fold_u64(self.has_double_jumped as u64);
        h.fold(self.air_time_since_jump);
        h.fold(self.handbrake_val);
        h.fold_u64(self.jump_held as u64);
        h.fold_u64(self.extra_impulse_cooldown as u64);
    }
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
        let radius = |v: f64| Car {
            velocity: Vec3::new(0.0, 0.0, 1.0) * v,
            ..Default::default()
        }
        .turn_radius();
        assert!((radius(5.0) - 2.57).abs() < 0.05, "r(5) = {}", radius(5.0));
        assert!((radius(10.0) - 4.62).abs() < 0.05, "r(10) = {}", radius(10.0));
        assert!((radius(22.0) - 12.69).abs() < 0.1, "r(22) = {}", radius(22.0));
    }
}
