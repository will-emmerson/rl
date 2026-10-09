//! The fixed-step world.
//!
//! The simulation is *defined* at 120 Hz. Every per-tick constant is expressed
//! for that rate, and `dt` is a compile-time constant, so nothing about the
//! step depends on how the host chooses to schedule it. Advance the world by
//! calling `step` an integral number of times; never pass a variable `dt`.

use crate::arena::Arena;
use crate::ball::Ball;
use crate::car::{Car, CAR_BALL_FRICTION, CAR_BALL_RESTITUTION, CAR_MASS};
use crate::math::{StateHasher, Vec3};

pub const TICK_HZ: u32 = 120;
pub const DT: f64 = 1.0 / TICK_HZ as f64;
/// Community-cited Rocket League gravity (~650 uu/s^2), in m/s^2.
pub const GRAVITY: f64 = -6.5;

/// One tick of player intent, already normalised to [-1, 1].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Input {
    /// -1 brake/reverse .. +1 accelerate.
    pub throttle: f64,
    /// -1 right .. +1 left.
    pub steer: f64,
    pub boost: bool,
    pub handbrake: bool,
    /// Jump, carried as a *held* level rather than an edge. The car detects the
    /// press itself, so holding the button cannot re-fire the jump the instant
    /// it lands.
    pub jump: bool,
}

impl Input {
    pub fn new(throttle: f64, steer: f64, boost: bool, handbrake: bool, jump: bool) -> Self {
        Self {
            throttle: throttle.clamp(-1.0, 1.0),
            steer: steer.clamp(-1.0, 1.0),
            boost,
            handbrake,
            jump,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Snapshot {
    pub tick: u64,
    pub car_pos: Vec3,
    pub car_velocity: Vec3,
    pub car_heading: Vec3,
    pub car_boost: f64,
    pub ball_pos: Vec3,
    pub ball_velocity: Vec3,
    pub ball_spin: Vec3,
}

#[derive(Clone, Debug)]
pub struct World {
    pub arena: Arena,
    pub ball: Ball,
    pub car: Car,
    pub tick: u64,
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}

impl World {
    pub fn new() -> Self {
        Self {
            arena: Arena::default(),
            ball: Ball::default(),
            car: Car::default(),
            tick: 0,
        }
    }

    /// Advance exactly one tick. The order of operations here is part of the
    /// simulation's contract — changing it changes results.
    pub fn step(&mut self, input: &Input) {
        self.car.step(DT, input, &self.arena);
        self.ball.step(DT, GRAVITY, &self.arena);
        self.resolve_car_ball();
        self.tick += 1;
    }

    pub fn step_n(&mut self, input: &Input, ticks: u32) {
        for _ in 0..ticks {
            self.step(input);
        }
    }

    fn resolve_car_ball(&mut self) {
        // Copy what the contact solver needs so the borrows stay disjoint.
        let axes = self.car.axes();
        let half = self.car.half_extents();
        let car_pos = self.car.pos;
        let car_vel = self.car.velocity;
        let car_spin = self.car.angular_velocity();

        let contact = self.ball.resolve_obb(
            car_pos,
            axes,
            half,
            car_vel,
            car_spin,
            CAR_MASS,
            CAR_BALL_RESTITUTION,
            CAR_BALL_FRICTION,
        );

        if let Some((normal, impulse)) = contact {
            if impulse > 0.0 {
                // Equal and opposite on the car. It weighs 120x the ball, so
                // it barely notices, which is exactly how RL feels.
                self.car.velocity -= normal * (impulse / CAR_MASS);
            }
        }
    }

    pub fn state_hash(&self) -> u64 {
        let mut h = StateHasher::new();
        h.fold_u64(self.tick);
        self.car.hash_into(&mut h);
        self.ball.hash_into(&mut h);
        h.0
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            tick: self.tick,
            car_pos: self.car.pos,
            car_velocity: self.car.velocity,
            car_heading: self.car.heading,
            car_boost: self.car.boost,
            ball_pos: self.ball.pos,
            ball_velocity: self.ball.vel,
            ball_spin: self.ball.spin,
        }
    }

    /// Distance between the two hitboxes' centres, handy for assertions.
    pub fn car_ball_distance(&self) -> f64 {
        self.car.pos.distance(self.ball.pos)
    }
}
