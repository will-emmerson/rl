//! The fixed-step world.
//!
//! The simulation is *defined* at 120 Hz. Every per-tick constant is expressed
//! for that rate, and `dt` is a compile-time constant, so nothing about the
//! step depends on how the host chooses to schedule it. Advance the world by
//! calling `step` an integral number of times; never pass a variable `dt`.

use crate::arena::Arena;
use crate::ball::Ball;
use crate::car::{
    Car, BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE, BALL_CAR_EXTRA_IMPULSE_MAX_DELTA_VEL,
    BALL_CAR_EXTRA_IMPULSE_Z_SCALE, CAR_BALL_FRICTION, CAR_BALL_RESTITUTION, CAR_MASS,
};
use crate::math::{piecewise_linear, StateHasher, Vec3};

pub const TICK_HZ: u32 = 120;
pub const DT: f64 = 1.0 / TICK_HZ as f64;
/// Community-cited Rocket League gravity (~650 uu/s^2), in m/s^2.
pub const GRAVITY: f64 = -6.5;

/// One tick of player intent, already normalised to [-1, 1].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Input {
    /// -1 brake/reverse .. +1 accelerate. Also the air *throttle*.
    pub throttle: f64,
    /// -1 right .. +1 left.
    pub steer: f64,
    /// Air orientation control, following Rocket League's `CarControls`: pitch
    /// (-1 nose down / frontflip), yaw, roll. Ignored on the ground.
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
    pub boost: bool,
    pub handbrake: bool,
    /// Jump, carried as a *held* level rather than an edge. The car detects the
    /// press itself, so holding the button cannot re-fire the jump the instant
    /// it lands.
    pub jump: bool,
}

impl Input {
    /// Driving/meta constructor. The air-orientation axes default to centred;
    /// set `pitch`/`yaw`/`roll` on the returned value to fly the car.
    pub fn new(throttle: f64, steer: f64, boost: bool, handbrake: bool, jump: bool) -> Self {
        Self {
            throttle: throttle.clamp(-1.0, 1.0),
            steer: steer.clamp(-1.0, 1.0),
            pitch: 0.0,
            yaw: 0.0,
            roll: 0.0,
            boost,
            handbrake,
            jump,
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
            pitch: pitch.clamp(-1.0, 1.0),
            yaw: yaw.clamp(-1.0, 1.0),
            roll: roll.clamp(-1.0, 1.0),
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
    /// The Soccar boost pads, in a fixed order (see `boost_pad::soccar`).
    pub boost_pads: Vec<crate::boost_pad::BoostPad>,
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
            boost_pads: crate::boost_pad::soccar(),
            tick: 0,
        }
    }

    /// Advance exactly one tick. The order of operations here is part of the
    /// simulation's contract — changing it changes results.
    pub fn step(&mut self, input: &Input) {
        self.car.step(DT, input, &self.arena);
        self.ball.step(DT, GRAVITY, &self.arena);
        self.resolve_car_ball();
        self.update_boost_pads();
        // Rocket League clamps both bodies at the end of the tick, *after*
        // collisions and the extra ball impulse (RocketSim's
        // `_FinishPhysicsTick`), so a collision cannot leave a body overspeed.
        self.car.clamp_limits();
        self.ball.clamp_speeds();
        self.tick += 1;
    }

    /// Tick every pad's cooldown and hand out boost for any the car is over.
    fn update_boost_pads(&mut self) {
        let car_pos = self.car.pos;
        // Rocket League does not take a pad while the tank is full, so it stays
        // available for the next pass.
        let can_take = self.car.boost < crate::car::CAR_BOOST_MAX;
        let mut gained = 0.0;
        for pad in &mut self.boost_pads {
            pad.tick(DT);
            if can_take && pad.available() && pad.covers(car_pos) {
                gained += pad.amount;
                pad.cooldown = pad.cooldown_duration;
            }
        }
        if gained > 0.0 {
            self.car.boost = (self.car.boost + gained).min(crate::car::CAR_BOOST_MAX);
        }
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

        // Rocket League computes its extra hit impulse inside the collision
        // callback, i.e. from the pre-solver velocities, so capture them here.
        let ball_pos_before = self.ball.pos;
        let ball_vel_before = self.ball.vel;

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
                // Equal and opposite on the car. The ball is now a sixth of the
                // car rather than a 120th, so the car does recoil noticeably.
                self.car.velocity -= normal * (impulse / CAR_MASS);
            }
            self.apply_extra_impulse(car_pos, car_vel, ball_pos_before, ball_vel_before);
        }
    }

    /// Rocket League's "extra" car-to-ball impulse (RocketSim `Ball::_OnHit`).
    ///
    /// The plain car-ball collision is inelastic, so on its own the ball would
    /// leave at only `m_car/(m_car+m_ball)` of the closing speed. RL adds this
    /// separate velocity kick on top of it, scaled by the closing speed, which
    /// is why car hits pop the ball far faster than momentum alone allows. It
    /// is applied to the ball only, and at most every other tick.
    fn apply_extra_impulse(
        &mut self,
        car_pos: Vec3,
        car_vel: Vec3,
        ball_pos: Vec3,
        ball_vel: Vec3,
    ) {
        if self.car.extra_impulse_cooldown > 0 {
            return;
        }

        let rel_vel = ball_vel - car_vel;
        let rel_speed = rel_vel.length().min(BALL_CAR_EXTRA_IMPULSE_MAX_DELTA_VEL);
        if rel_speed <= 0.0 {
            return;
        }

        // Direction from the car to the ball, with the vertical squashed, then
        // with part of the car-forward component removed: a flat hit therefore
        // pushes the ball more forward than straight up.
        let delta = ball_pos - car_pos;
        let flattened = Vec3::new(delta.x, delta.y * BALL_CAR_EXTRA_IMPULSE_Z_SCALE, delta.z);
        let mut hit_dir = flattened.normalized();
        let car_forward = self.car.forward;
        hit_dir = (hit_dir
            - car_forward
                * (hit_dir.dot(car_forward) * (1.0 - BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE)))
            .normalized();

        let factor = extra_impulse_factor(rel_speed);
        self.ball.vel += hit_dir * (rel_speed * factor);

        // Block the next tick, matching `tickCountWhenExtraImpulseApplied`.
        self.car.extra_impulse_cooldown = 2;
    }

    pub fn state_hash(&self) -> u64 {
        let mut h = StateHasher::new();
        h.fold_u64(self.tick);
        self.car.hash_into(&mut h);
        self.ball.hash_into(&mut h);
        for pad in &self.boost_pads {
            pad.hash_into(&mut h);
        }
        h.0
    }

    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            tick: self.tick,
            car_pos: self.car.pos,
            car_velocity: self.car.velocity,
            car_heading: self.car.heading(),
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

/// Rocket League's `BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE`, with the speed axis
/// converted from uu/s to m/s.
fn extra_impulse_factor(rel_speed: f64) -> f64 {
    const POINTS: [(f64, f64); 4] = [(0.0, 0.65), (5.0, 0.65), (23.0, 0.55), (46.0, 0.30)];
    piecewise_linear(&POINTS, rel_speed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extra_impulse_factor_matches_the_rocketsim_curve() {
        // End points and a midpoint, in m/s (uu/s / 100).
        assert!((extra_impulse_factor(0.0) - 0.65).abs() < 1e-12);
        assert!((extra_impulse_factor(5.0) - 0.65).abs() < 1e-12);
        assert!((extra_impulse_factor(23.0) - 0.55).abs() < 1e-12);
        assert!((extra_impulse_factor(46.0) - 0.30).abs() < 1e-12);
        // 14 m/s sits halfway between the 5 and 23 m/s knots.
        assert!((extra_impulse_factor(14.0) - 0.60).abs() < 1e-12);
        // Clamped outside the range.
        assert!((extra_impulse_factor(100.0) - 0.30).abs() < 1e-12);
    }
}
