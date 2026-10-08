//! Physics behaviour tests.
//!
//! This is what a hand-rolled engine buys you: no window, no render loop, no
//! editor — just `cargo test` and exact numbers.

use rl_sim::arena::Arena;
use rl_sim::ball::BALL_RADIUS;
use rl_sim::car::{CAR_LATERAL_ACCEL, CAR_MIN_TURN_RADIUS};
use rl_sim::math::Vec3;
use rl_sim::{Ball, Car, Input, World, DT, GRAVITY};

/// Deterministic pseudo-random stream, owned by the test not the simulation.
struct Lcg(u64);

impl Lcg {
    fn next_f64(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn signed(&mut self) -> f64 {
        self.next_f64() * 2.0 - 1.0
    }
}

#[test]
fn ball_settles_on_the_floor() {
    let arena = Arena::default();
    let mut ball = Ball {
        pos: Vec3::new(0.0, 6.0, 14.0),
        ..Default::default()
    };
    for _ in 0..1200 {
        ball.step(DT, GRAVITY, &arena);
    }
    assert!((ball.pos.y - BALL_RADIUS).abs() < 0.02, "y={}", ball.pos.y);
    assert!(ball.vel.y.abs() < 0.05, "still moving: {:?}", ball.vel);
}

#[test]
fn head_on_floor_bounce_is_specular() {
    // Straight down, no spin => no tangential contact velocity, so the bounce
    // must reverse Y and leave X/Z exactly alone.
    let arena = Arena::default();
    let mut ball = Ball {
        pos: Vec3::new(0.0, BALL_RADIUS + 0.05, 0.0),
        vel: Vec3::new(0.0, -10.0, 0.0),
        ..Default::default()
    };
    ball.step(DT, GRAVITY, &arena);

    assert!(ball.vel.y > 6.0 && ball.vel.y < 6.5, "vy={}", ball.vel.y);
    assert!(ball.vel.x.abs() < 1e-12, "vx drifted: {}", ball.vel.x);
    assert!(ball.vel.z.abs() < 1e-12, "vz drifted: {}", ball.vel.z);
    assert!(ball.spin.length() < 1e-12, "spin appeared");
}

#[test]
fn angled_bounce_trades_sliding_for_spin() {
    // An angled bounce is NOT a mirror: friction turns some sliding into
    // rotation, so the ball leaves slower and spinning. This is the "curve
    // off the wall" mechanism.
    let arena = Arena::default();
    let mut ball = Ball {
        pos: Vec3::new(0.0, BALL_RADIUS + 0.05, 0.0),
        vel: Vec3::new(5.0, -10.0, 0.0),
        ..Default::default()
    };
    ball.step(DT, GRAVITY, &arena);

    assert!(ball.vel.y > 0.0, "should have bounced up");
    assert!(ball.vel.x < 5.0, "tangential speed should drop");
    assert!(ball.spin.length() > 1.0, "should have gained spin");
}

#[test]
fn spin_curves_the_ball_in_flight() {
    // Identical launches, one ball spinning about +Y. Magnus is the only
    // difference, so any lateral divergence is the curve.
    let arena = Arena::default();
    let mut plain = Ball {
        pos: Vec3::new(0.0, 6.0, -20.0),
        vel: Vec3::new(0.0, 0.0, 20.0),
        ..Default::default()
    };
    let mut spun = plain;
    spun.spin = Vec3::new(0.0, 40.0, 0.0);

    for _ in 0..120 {
        plain.step(DT, GRAVITY, &arena);
        spun.step(DT, GRAVITY, &arena);
    }
    assert!(plain.pos.x.abs() < 1e-9, "unspun ball drifted");
    let curve = (spun.pos.x - plain.pos.x).abs();
    assert!(curve > 1.0, "expected visible curve, got {curve} m");
}

fn yaw_rate_for_speed(speed: f64) -> f64 {
    let arena = Arena::default();
    let heading = Vec3::new(0.0, 0.0, 1.0);
    let mut car = Car {
        velocity: heading * speed,
        ..Default::default()
    };
    car.step(DT, &Input::new(0.0, 1.0, false, false), &arena);
    car.yaw_rate
}

#[test]
fn car_accelerates_then_saturates() {
    let mut world = World::new();
    let drive = Input::new(1.0, 0.0, false, false);
    for _ in 0..120 {
        world.step(&drive);
    }
    let after_1s = world.car.speed();
    assert!((7.0..=9.0).contains(&after_1s), "got {after_1s} after 1 s");

    let mut peak: f64 = 0.0;
    for _ in 0..400 {
        world.step(&drive);
        peak = peak.max(world.car.speed());
    }
    assert!((peak - 22.0).abs() < 0.05, "peaked at {peak}");
}

#[test]
fn turn_radius_grows_with_speed() {
    // A Rocket League car cannot corner as tightly at speed as it can crawling.
    let slow = yaw_rate_for_speed(5.0);
    let fast = yaw_rate_for_speed(22.0);
    assert!(slow > fast, "slow={slow} fast={fast}");
    assert!(
        (slow - 5.0 / CAR_MIN_TURN_RADIUS).abs() < 0.05,
        "slow={slow}"
    );
    let expected_fast = 22.0 / (22.0 * 22.0 / CAR_LATERAL_ACCEL);
    assert!((fast - expected_fast).abs() < 0.05, "fast={fast}");
}

#[test]
fn sustained_turn_has_a_sane_radius() {
    let mut world = World::new();
    world.car.velocity = world.car.heading * 15.0;
    let steer = Input::new(1.0, 1.0, false, false);

    let mut yaw = 0.0;
    let mut speed = 0.0;
    for _ in 0..240 {
        world.step(&steer);
        yaw += world.car.yaw_rate.abs() * DT;
        speed += world.car.speed();
    }
    let avg_speed = speed / 240.0;
    let avg_yaw_rate = yaw / (240.0 * DT);
    let radius = avg_speed / avg_yaw_rate;
    assert!(
        (3.0..=30.0).contains(&radius),
        "radius {radius} m (speed {avg_speed}, yaw rate {avg_yaw_rate})"
    );
}

#[test]
fn car_never_leaves_the_arena() {
    let mut world = World::new();
    let mut rng = Lcg(12345);
    for _ in 0..4000 {
        let input = Input::new(rng.signed(), rng.signed(), rng.next_f64() < 0.3, false);
        world.step(&input);
    }
    let a = &world.arena;
    assert!(
        world.car.pos.x.abs() <= a.half_x && world.car.pos.z.abs() <= a.half_z,
        "car escaped to {:?}",
        world.car.pos
    );
}

#[test]
fn head_on_shot_launches_the_ball_faster_than_the_car() {
    let (impact, ball_speed) = rl_sim::head_on_shot();
    assert!(
        impact > 10.0,
        "car should be moving at impact, was {impact}"
    );
    assert!(
        ball_speed > impact,
        "ball ({ball_speed}) should leave faster than the car ({impact})"
    );
    assert!(ball_speed < 60.0, "implausibly fast ball: {ball_speed}");
}

#[test]
fn long_run_stays_finite_and_bounded() {
    // Fuzz for blow-ups: pinches between car and wall are exactly the case
    // that breaks naïve solvers, so aim for them deliberately.
    let mut world = World::new();
    let mut rng = Lcg(0xC0FFEE);
    let mut peak_ball: f64 = 0.0;
    let mut peak_car: f64 = 0.0;

    for tick in 0..12000 {
        // Bias towards charging the walls with the ball in front.
        let aiming = if (tick / 300) % 2 == 0 { 1.0 } else { -1.0 };
        let steer = if tick % 600 < 300 { 0.0 } else { aiming * 0.7 };
        let input = Input::new(1.0, steer, rng.next_f64() < 0.5, rng.next_f64() < 0.1);
        world.step(&input);

        assert!(world.car.pos.is_finite(), "car NaN at tick {tick}");
        assert!(world.ball.pos.is_finite(), "ball NaN at tick {tick}");
        assert!(world.ball.vel.is_finite(), "ball vel NaN at tick {tick}");

        peak_ball = peak_ball.max(world.ball.vel.length());
        peak_car = peak_car.max(world.car.speed());
    }

    let a = &world.arena;
    assert!(
        world.ball.pos.x.abs() <= a.half_x && world.ball.pos.z.abs() <= a.half_z,
        "ball escaped to {:?}",
        world.ball.pos
    );
    assert!(
        world.ball.pos.y >= -0.01 && world.ball.pos.y <= a.height + 0.01,
        "ball left vertically: y={}",
        world.ball.pos.y
    );
    assert!(peak_ball < 150.0, "ball blew up: {peak_ball} m/s");
    assert!(peak_car < 25.0, "car exceeded its cap: {peak_car} m/s");
    eprintln!("fuzz peaks: ball {peak_ball:.1} m/s, car {peak_car:.1} m/s");
}
