//! Physics behaviour tests.
//!
//! This is what a hand-rolled engine buys you: no window, no render loop, no
//! editor — just `cargo test` and exact numbers.

use rl::arena::Arena;
use rl::ball::{BALL_MASS, BALL_MAX_ANG_SPEED, BALL_MAX_SPEED, BALL_RADIUS};
use rl::car::{CAR_HALF_HEIGHT, CAR_MASS, CAR_MAX_SPEED};
use rl::math::Vec3;
use rl::{Ball, Car, Input, World, DT, GRAVITY};

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

    assert!(ball.vel.y > 5.5 && ball.vel.y < 6.5, "vy={}", ball.vel.y);
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
    // difference, so any lateral divergence is the curve. Spin is capped at
    // `BALL_MAX_ANG_SPEED` (6 rad/s) in Rocket League, so that is what we use.
    let arena = Arena::default();
    let mut plain = Ball {
        pos: Vec3::new(0.0, 6.0, -20.0),
        vel: Vec3::new(0.0, 0.0, 20.0),
        ..Default::default()
    };
    let mut spun = plain;
    spun.spin = Vec3::new(0.0, BALL_MAX_ANG_SPEED, 0.0);

    for _ in 0..120 {
        plain.step(DT, GRAVITY, &arena);
        spun.step(DT, GRAVITY, &arena);
    }
    assert!(plain.pos.x.abs() < 1e-9, "unspun ball drifted");
    let curve = (spun.pos.x - plain.pos.x).abs();
    assert!(curve > 0.3, "expected a visible curve, got {curve} m");
}

fn yaw_rate_for_speed(speed: f64) -> f64 {
    let arena = Arena::default();
    let heading = Vec3::new(0.0, 0.0, 1.0);
    let mut car = Car {
        velocity: heading * speed,
        ..Default::default()
    };
    car.step(DT, &Input::new(0.0, 1.0, false, false, false), &arena);
    car.yaw_rate
}

#[test]
fn car_accelerates_then_saturates() {
    let mut world = World::new();
    let drive = Input::new(1.0, 0.0, false, false, false);
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
    assert!((peak - CAR_MAX_SPEED).abs() < 0.05, "peaked at {peak}");
}

#[test]
fn steering_follows_the_rocketsim_curve() {
    // The turn radius comes from RL's steer-angle-vs-speed curve through the
    // bicycle model, so it grows with speed and matches the game: ~2.6 m at
    // 5 m/s, ~4.6 m at 10, ~12.7 m at 22.
    let radius = |v: f64| v / yaw_rate_for_speed(v).abs();
    let (r5, r10, r22) = (radius(5.0), radius(10.0), radius(22.0));
    assert!((r5 - 2.57).abs() < 0.1, "r(5) = {r5}");
    assert!((r10 - 4.62).abs() < 0.1, "r(10) = {r10}");
    assert!((r22 - 12.69).abs() < 0.3, "r(22) = {r22}");
    assert!(r5 < r10 && r10 < r22, "radius should grow with speed");

    // Peak yaw rate sits around 8 m/s and is bounded (~2.3 rad/s).
    let mut peak: f64 = 0.0;
    let mut v = 1.0;
    while v <= 23.0 {
        peak = peak.max(yaw_rate_for_speed(v).abs());
        v += 0.5;
    }
    assert!(peak > 2.0 && peak < 2.5, "peak yaw rate {peak} rad/s");
}

#[test]
fn yaw_rate_vanishes_at_a_standstill() {
    // The bicycle model is proportional to v, so a stationary car cannot turn
    // however hard the stick is held — no pivot-on-the-spot.
    assert!(yaw_rate_for_speed(0.0).abs() < 1e-12);
    // And the rate grows once the car is rolling.
    let slow = yaw_rate_for_speed(2.0).abs();
    let faster = yaw_rate_for_speed(8.0).abs();
    assert!(slow > 0.0 && slow < faster, "slow {slow}, faster {faster}");
}

#[test]
fn sustained_turn_has_a_sane_radius() {
    let mut world = World::new();
    world.car.velocity = world.car.heading * 15.0;
    let steer = Input::new(1.0, 1.0, false, false, false);

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
    eprintln!(
        "sustained turn: ~{avg_speed:.1} m/s, yaw {avg_yaw_rate:.2} rad/s ({:.0} deg/s), radius {radius:.1} m",
        avg_yaw_rate.to_degrees()
    );
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
        let input = Input::new(
            rng.signed(),
            rng.signed(),
            rng.next_f64() < 0.3,
            false,
            false,
        );
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
    let (impact, ball_speed) = rl::head_on_shot();
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
        let input = Input::new(
            1.0,
            steer,
            rng.next_f64() < 0.5,
            rng.next_f64() < 0.1,
            false,
        );
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

#[test]
fn jump_lifts_the_car_then_it_lands() {
    let mut world = World::new();
    // One tick of jump, then release it.
    world.step(&Input::new(0.0, 0.0, false, false, true));
    assert!(world.car.airborne, "should be airborne right after jumping");
    assert!(world.car.velocity.y > 0.0, "should be moving up");

    let mut peak = world.car.pos.y;
    for _ in 0..600 {
        world.step(&Input::new(0.0, 0.0, false, false, false));
        peak = peak.max(world.car.pos.y);
    }

    assert!(
        peak > CAR_HALF_HEIGHT + 0.5,
        "jump peaked at {peak}, barely off the ground"
    );
    assert!(!world.car.airborne, "should have come back down");
    assert!(
        (world.car.pos.y - CAR_HALF_HEIGHT).abs() < 1e-9,
        "resting at y={}",
        world.car.pos.y
    );
    assert!(world.car.velocity.y.abs() < 1e-9, "still moving vertically");
}

#[test]
fn holding_jump_does_not_re_jump_on_landing() {
    // Jump is edge-triggered. Level-triggered, a held button would bounce the
    // car forever; here it must fire exactly once.
    let mut world = World::new();
    let hold = Input::new(0.0, 0.0, false, false, true);

    let mut takeoffs = 0;
    let mut was_airborne = false;
    for _ in 0..900 {
        world.step(&hold);
        if world.car.airborne && !was_airborne {
            takeoffs += 1;
        }
        was_airborne = world.car.airborne;
    }

    assert_eq!(takeoffs, 1, "held jump re-fired {takeoffs} times");
    assert!(!world.car.airborne, "should have settled on the ground");
}

#[test]
fn double_jump_reaches_higher_than_a_single_jump() {
    let peak_with = |rejump_at: Option<u32>| {
        let mut world = World::new();
        let mut peak = world.car.pos.y;
        for tick in 0..400 {
            let jump = tick == 0 || rejump_at == Some(tick);
            world.step(&Input::new(0.0, 0.0, false, false, jump));
            peak = peak.max(world.car.pos.y);
        }
        peak
    };

    // Re-jump a quarter of a second in, still on the way up.
    let single = peak_with(None);
    let double = peak_with(Some(30));
    assert!(
        double > single + 0.3,
        "double jump peaked at {double}, single at {single}"
    );
}

// --- Parity checks against RocketSim's `RLConst.h` ---

#[test]
fn holding_jump_goes_higher_than_tapping_it() {
    // RL's jump is an immediate impulse plus a held force, so the button time
    // matters: a tap is a low hop (~0.8 m above rest), a full hold is much
    // higher (~2.5 m).
    let peak = |hold: bool| {
        let mut world = World::new();
        let mut top = world.car.pos.y;
        for tick in 0..600 {
            let jump = tick == 0 || (hold && tick < 30);
            world.step(&Input::new(0.0, 0.0, false, false, jump));
            top = top.max(world.car.pos.y);
        }
        top
    };
    let tap = peak(false);
    let held = peak(true);
    assert!(held > tap + 1.0, "hold {held} m vs tap {tap} m");
}

#[test]
fn arena_matches_rocket_league_soccar() {
    // ARENA_EXTENT_X/Y and ARENA_HEIGHT in uu, over 100.
    let a = Arena::default();
    assert!((a.half_x - 40.96).abs() < 1e-12, "half_x = {}", a.half_x);
    assert!((a.half_z - 51.20).abs() < 1e-12, "half_z = {}", a.half_z);
    assert!((a.height - 20.48).abs() < 1e-12, "height = {}", a.height);
    assert!(
        (BALL_RADIUS - 0.9125).abs() < 1e-12,
        "radius = {BALL_RADIUS}"
    );
}

#[test]
fn ball_speed_and_spin_are_capped() {
    // BALL_MAX_SPEED = 6000 uu/s and BALL_MAX_ANG_SPEED = 6 rad/s are hard
    // clamps in the game; a pinch must not be able to exceed them.
    let arena = Arena::default();
    let mut ball = Ball {
        pos: Vec3::new(0.0, 6.0, 0.0),
        vel: Vec3::new(0.0, 0.0, 500.0),
        spin: Vec3::new(0.0, 200.0, 0.0),
        ..Default::default()
    };
    ball.step(DT, GRAVITY, &arena);
    assert!(
        ball.vel.length() <= BALL_MAX_SPEED + 1e-9,
        "{}",
        ball.vel.length()
    );
    assert!(
        ball.spin.length() <= BALL_MAX_ANG_SPEED + 1e-9,
        "{}",
        ball.spin.length()
    );
}

#[test]
fn car_speed_is_capped_by_the_world() {
    // RL clamps the car's total speed to CAR_MAX_SPEED every tick, so neither
    // boost nor a collision can leave it overspeed.
    let mut world = World::new();
    world.car.velocity = world.car.heading * 100.0;
    world.step(&Input::default());
    assert!(
        world.car.speed() <= CAR_MAX_SPEED + 1e-9,
        "car at {} m/s",
        world.car.speed()
    );
}

#[test]
fn car_hit_adds_the_extra_impulse() {
    // A pure inelastic car-ball collision would leave the ball at
    // m_car/(m_car+m_ball) * closing speed. RL adds a separate extra impulse on
    // top, so the ball must beat that bound.
    let (impact, ball_speed) = rl::head_on_shot();
    let inelastic_bound = (CAR_MASS / (CAR_MASS + BALL_MASS)) * impact;
    eprintln!(
        "head-on: car {impact:.1} m/s, inelastic bound {inelastic_bound:.1} m/s, ball {ball_speed:.1} m/s"
    );
    assert!(
        ball_speed > inelastic_bound + 1.0,
        "ball {ball_speed} did not beat the inelastic bound {inelastic_bound} (impact {impact})"
    );
}
