//! The car.
//!
//! Deliberately *not* a rigid body with wheels. Rocket League's cars are not
//! wheel-driven in the ordinary sense — they are a bespoke driving model with
//! a speed-dependent turning radius, a powerslide state, and a box hitbox.
//! Modelling it directly is both simpler and closer to the real thing than
//! bolting four springs onto a box and hoping.

use crate::arena::Arena;
use crate::math::{heading_right, rotate_y, StateHasher, Vec3};
use crate::world::Input;

// Roughly the Octane hitbox (118.01 x 84.2 x 36.16 uu -> metres), which is
// noticeably *smaller* than the ball. That size relationship is genuine: RL's
// ball is bigger than a car.
pub const CAR_HALF_LENGTH: f64 = 0.59;
pub const CAR_HALF_WIDTH: f64 = 0.42;
pub const CAR_HALF_HEIGHT: f64 = 0.18;
pub const CAR_MASS: f64 = 180.0;

pub const CAR_ENGINE_ACCEL: f64 = 8.0;
pub const CAR_MAX_SPEED: f64 = 22.0;
pub const CAR_BOOST_ACCEL: f64 = 10.0;
pub const CAR_MAX_BOOST_SPEED: f64 = 23.0;
pub const CAR_OVERSPEED_DECEL: f64 = 2.0;
pub const CAR_REVERSE_ACCEL: f64 = 6.0;
pub const CAR_MAX_REVERSE_SPEED: f64 = 10.0;
pub const CAR_BRAKE_DECEL: f64 = 18.0;

/// Yaw rate the car can hold, in rad/s. This is the single knob that decides
/// how the car handles.
///
/// The rate is bounded directly rather than derived from a lateral
/// acceleration budget. A budget (radius = v^2 / a) makes the rate rise as
/// 1/v once the radius floor is reached, which is how the car ended up able
/// to spin on the spot at a crawl.
///
/// The consequence of holding the rate is a minimum turn radius of
/// `CAR_STEER_ENGAGE_SPEED / CAR_MAX_YAW_RATE` (~3.9 m) below the engagement
/// speed, growing linearly with speed above it: ~7.7 m at 10 m/s and ~17 m at
/// 22 m/s.
pub const CAR_MAX_YAW_RATE: f64 = 1.3;
/// Steering authority ramps in over this speed. Below it the car turns
/// proportionally less, and at a dead stop it does not turn at all.
pub const CAR_STEER_ENGAGE_SPEED: f64 = 5.0;

/// Lateral velocity retained per tick. High grip keeps the velocity glued to
/// the heading; the handbrake keeps more of it, which is what makes a drift.
pub const CAR_GRIP_KEEP: f64 = 0.90;
pub const CAR_DRIFT_KEEP: f64 = 0.985;
pub const CAR_ROLLING_KEEP: f64 = 0.995;

pub const CAR_BOOST_MAX: f64 = 100.0;
pub const CAR_BOOST_DRAIN: f64 = 33.3;

pub const CAR_BALL_RESTITUTION: f64 = 0.62;
pub const CAR_BALL_FRICTION: f64 = 0.5;

#[derive(Clone, Copy, Debug)]
pub struct Car {
    /// Centre of the hitbox.
    pub pos: Vec3,
    /// Ground-plane velocity (y is always 0 for the car).
    pub velocity: Vec3,
    /// Unit heading in the XZ plane. +Z at spawn, matching the Godot scene.
    pub heading: Vec3,
    pub yaw_rate: f64,
    pub boost: f64,
}

impl Default for Car {
    fn default() -> Self {
        Self {
            pos: Vec3::new(0.0, CAR_HALF_HEIGHT, -14.0),
            velocity: Vec3::ZERO,
            heading: Vec3::new(0.0, 0.0, 1.0),
            yaw_rate: 0.0,
            boost: CAR_BOOST_MAX,
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
        let f = self.heading;
        let r = self.right();
        let mut v_forward = self.velocity.dot(f);
        let mut v_lateral = self.velocity.dot(r);

        // --- yaw ---
        // Bounded directly, and scaled in below the engagement speed so the
        // car cannot pivot on the spot. The sign follows the direction of
        // travel, so reversing mirrors the turn like a real car.
        let direction = if v_forward >= 0.0 { 1.0 } else { -1.0 };
        let engagement = (v_forward.abs() / CAR_STEER_ENGAGE_SPEED).min(1.0);
        self.yaw_rate = input.steer * CAR_MAX_YAW_RATE * engagement * direction;
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
            v_forward += CAR_BOOST_ACCEL * dt;
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
        v_lateral *= if input.handbrake {
            CAR_DRIFT_KEEP
        } else {
            CAR_GRIP_KEEP
        };

        self.velocity = f * v_forward + r * v_lateral;
        self.pos += self.velocity * dt;
        self.pos.y = CAR_HALF_HEIGHT;

        self.collide_walls(arena);
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

    /// Nominal turn radius at the current speed, which is `v / yaw_rate`.
    /// Infinite when stopped, because the car cannot turn without rolling.
    pub fn turn_radius(&self) -> f64 {
        let v = self.forward_speed().abs();
        let rate = CAR_MAX_YAW_RATE * (v / CAR_STEER_ENGAGE_SPEED).min(1.0);
        if rate > 1e-9 {
            v / rate
        } else {
            f64::INFINITY
        }
    }

    pub fn hash_into(&self, h: &mut StateHasher) {
        self.pos.hash_into(h);
        self.velocity.hash_into(h);
        self.heading.hash_into(h);
        h.fold(self.yaw_rate);
        h.fold(self.boost);
    }
}
