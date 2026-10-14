//! Physics behaviour tests.
//!
//! This is what a hand-rolled engine buys you: no window, no render loop, no
//! editor — just `cargo test` and exact numbers.

use rl::arena::{
    Arena, GOAL_DEPTH, GOAL_HALF_WIDTH, GOAL_HEIGHT, GOAL_NET_CENTER_HEIGHT, GOAL_SCORE_BASE_Z,
};
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
    world.car.velocity = world.car.heading() * 15.0;
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
    // The goals are recesses in the end walls, so the car may sit inside one -
    // but it must stay within the playable space, goals included.
    assert!(
        a.inside_distance(world.car.pos) <= 1e-3,
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

// --- Air control and flips ---

/// Jump once, let the jump finish, and return a world that is still airborne.
fn airborne_world() -> World {
    let mut world = World::new();
    world.step(&Input::new(0.0, 0.0, false, false, true));
    for _ in 0..20 {
        world.step(&Input::new(0.0, 0.0, false, false, false));
    }
    assert!(world.car.airborne, "should still be in the air");
    world
}

#[test]
fn frontflip_pushes_the_car_forward() {
    let mut world = airborne_world();
    let before = world.car.forward_speed();
    // Stick forward (throttle +1) is `pitch = -1`: a frontflip.
    let mut input = Input::new(1.0, 0.0, false, false, true);
    input.pitch = -1.0;
    world.step(&input);
    assert!(
        world.car.has_flipped,
        "should have flipped, not double jumped"
    );
    assert!(
        world.car.forward_speed() > before + 2.0,
        "frontflip should add forward speed: {before} -> {}",
        world.car.forward_speed()
    );
}

#[test]
fn frontflip_pitches_the_nose_down() {
    let mut world = airborne_world();
    let mut input = Input::new(0.0, 0.0, false, false, true);
    input.pitch = -1.0;
    world.step(&input);
    for _ in 0..20 {
        world.step(&Input::new(0.0, 0.0, false, false, false));
    }
    assert!(
        world.car.forward.y < -0.1,
        "frontflip should pitch the nose down, forward = {:?}",
        world.car.forward
    );
}

#[test]
fn air_control_rolls_the_car() {
    let mut world = airborne_world();
    let mut input = Input::new(0.0, 0.0, false, false, false);
    input.roll = 1.0;
    for _ in 0..30 {
        world.step(&input);
    }
    // A pure roll must not yaw, and the up axis must tilt away from world up.
    assert!(
        world.car.ang_vel.y.abs() < 0.5,
        "roll should not yaw: {:?}",
        world.car.ang_vel
    );
    assert!(
        world.car.up.dot(Vec3::UP) < 0.95,
        "car should have rolled, up = {:?}",
        world.car.up
    );
}

#[test]
fn flip_needs_directional_input() {
    // A second jump press with the stick centred is a double jump, not a flip.
    let mut world = airborne_world();
    world.step(&Input::new(0.0, 0.0, false, false, true));
    assert!(world.car.has_double_jumped, "centred press = double jump");
    assert!(!world.car.has_flipped, "...not a flip");
}

// --- Curved arena: driving on walls and the ceiling ---

/// Aim the car at the +X wall and drive flat out, returning the world.
fn drive_at_the_wall(speed: f64) -> World {
    let mut world = World::new();
    world.car.forward = Vec3::new(1.0, 0.0, 0.0);
    world.car.up = Vec3::UP;
    world.car.velocity = Vec3::new(speed, 0.0, 0.0);
    world
}

#[test]
fn car_drives_up_a_wall() {
    // Straight into the wall from the floor. The rounded floor-to-wall join
    // must turn the car onto the wall: its up axis rotates from vertical to
    // horizontal and it climbs clear of the floor.
    let mut world = drive_at_the_wall(12.0);
    let drive = Input::new(1.0, 0.0, false, false, false);
    let a = Arena::default();

    let mut on_wall = false;
    for tick in 0..1200 {
        world.step(&drive);
        // Grounded on the wall: out near the wall face, tilted onto it.
        if world.car.pos.x > a.half_x - 0.5 && world.car.up.y < 0.3 && world.car.pos.y > 3.0 {
            on_wall = true;
            eprintln!("on wall after {tick} ticks at {:?}", world.car.pos);
            break;
        }
    }
    assert!(
        on_wall,
        "never climbed the wall: pos {:?} up {:?}",
        world.car.pos, world.car.up
    );
}

#[test]
fn car_rolls_from_the_wall_onto_the_ceiling() {
    // Keep driving once on the wall: the rounded wall-to-ceiling join must
    // carry the car over the rim so its up ends up pointing down.
    let mut world = drive_at_the_wall(18.0);
    let drive = Input::new(1.0, 0.0, false, false, false);
    let a = Arena::default();

    let mut on_ceiling = false;
    for tick in 0..3000 {
        world.step(&drive);
        if world.car.up.y < -0.5 && world.car.pos.y > a.height - 1.0 {
            on_ceiling = true;
            eprintln!("on ceiling after {tick} ticks at {:?}", world.car.pos);
            break;
        }
    }
    assert!(
        on_ceiling,
        "never reached the ceiling: pos {:?} up {:?}",
        world.car.pos, world.car.up
    );
}

#[test]
fn car_falls_off_the_ceiling_when_it_slows() {
    // A ceiling cannot hold the car (it can only push, never pull), so once the
    // car stops powering itself it must lose speed and fall back down rather
    // than sticking to the roof forever.
    let a = Arena::default();
    let mut world = drive_at_the_wall(18.0);
    let drive = Input::new(1.0, 0.0, false, false, false);

    let mut on_ceiling = false;
    for _ in 0..3000 {
        world.step(&drive);
        if world.car.up.y < -0.5 && world.car.pos.y > a.height - 1.0 {
            on_ceiling = true;
            break;
        }
    }
    assert!(on_ceiling, "never reached the ceiling");

    // Coast: the car should fall off and end up back on the floor, upright.
    let idle = Input::default();
    for _ in 0..2400 {
        world.step(&idle);
    }
    assert!(
        world.car.pos.y < a.height * 0.5,
        "stuck to the ceiling at {:?}",
        world.car.pos
    );
    assert!(
        world.car.up.y > 0.5 && (world.car.pos.y - CAR_HALF_HEIGHT).abs() < 0.1,
        "should have settled on the floor: pos {:?} up {:?}",
        world.car.pos,
        world.car.up
    );
}

#[test]
fn car_slides_down_a_wall_when_it_stops() {
    // Parked on a vertical wall with no throttle, the slope gravity must pull
    // the car back down to the floor.
    let a = Arena::default();
    let mut world = World::new();
    world.car.pos = Vec3::new(a.half_x - CAR_HALF_HEIGHT, 12.0, 0.0);
    world.car.up = Vec3::new(-1.0, 0.0, 0.0);
    world.car.forward = Vec3::new(0.0, 1.0, 0.0);
    world.car.velocity = Vec3::ZERO;
    world.car.airborne = false;

    let idle = Input::default();
    for _ in 0..1500 {
        world.step(&idle);
    }
    assert!(
        world.car.up.y > 0.9 && world.car.pos.y < 1.0,
        "did not slide back to the floor: pos {:?} up {:?}",
        world.car.pos,
        world.car.up
    );
}

#[test]
fn car_does_not_stick_in_a_corner() {
    // Drive hard into a vertical corner. With the old box arena the car wedged
    // there; with the chamfer it should keep moving and never blow up.
    let a = Arena::default();
    let mut world = World::new();
    // Aim at the +X/-Z corner.
    let dir = Vec3::new(1.0, 0.0, -1.0).normalized();
    world.car.forward = dir;
    world.car.up = Vec3::UP;
    world.car.velocity = dir * 15.0;
    let drive = Input::new(1.0, 0.0, false, false, false);

    let mut min_after_corner = f64::MAX;
    for _ in 0..1200 {
        world.step(&drive);
        assert!(world.car.pos.is_finite(), "car went non-finite");
        // Track how slowly it moves once it is out at the corner.
        if world.car.pos.x > a.half_x - 2.0 && world.car.pos.z < -a.half_z + 2.0 {
            min_after_corner = min_after_corner.min(world.car.speed());
        }
    }
    assert!(
        a.inside_distance(world.car.pos) <= 1e-3,
        "car left the playable space: {:?}",
        world.car.pos
    );
    if min_after_corner != f64::MAX {
        assert!(
            min_after_corner > 0.5,
            "car stuck at the corner ({min_after_corner} m/s)"
        );
    }
}

// --- Boost pads ---

#[test]
fn big_boost_pad_refills_and_then_cools_down() {
    let mut world = World::new();
    let pad = *world.boost_pads.iter().find(|p| p.is_big).unwrap();
    world.car.pos = Vec3::new(pad.pos.x, CAR_HALF_HEIGHT, pad.pos.z);
    world.car.velocity = Vec3::ZERO;
    world.car.boost = 0.0;

    world.step(&Input::default());
    assert!(
        (world.car.boost - 100.0).abs() < 1e-9,
        "boost {}",
        world.car.boost
    );
    assert!(
        !world
            .boost_pads
            .iter()
            .find(|p| p.is_big)
            .unwrap()
            .available(),
        "the pad should be on cooldown"
    );

    // Sitting on the cooling pad does not refill again.
    world.car.boost = 0.0;
    world.step(&Input::default());
    assert!(
        world.car.boost < 1.0,
        "a cooling pad refilled the car: {}",
        world.car.boost
    );
}

#[test]
fn small_boost_pad_gives_twelve() {
    let mut world = World::new();
    let pad = *world.boost_pads.iter().find(|p| !p.is_big).unwrap();
    world.car.pos = Vec3::new(pad.pos.x, CAR_HALF_HEIGHT, pad.pos.z);
    world.car.velocity = Vec3::ZERO;
    world.car.boost = 0.0;

    world.step(&Input::default());
    assert!(
        (world.car.boost - 12.0).abs() < 1e-9,
        "boost {}",
        world.car.boost
    );
}

#[test]
fn a_full_tank_does_not_consume_a_pad() {
    let mut world = World::new();
    let pad = *world.boost_pads.iter().find(|p| p.is_big).unwrap();
    world.car.pos = Vec3::new(pad.pos.x, CAR_HALF_HEIGHT, pad.pos.z);
    world.car.velocity = Vec3::ZERO;
    world.car.boost = 100.0;

    world.step(&Input::default());
    assert!(
        world
            .boost_pads
            .iter()
            .find(|p| p.is_big)
            .unwrap()
            .available(),
        "a full tank consumed the pad"
    );
    assert!((world.car.boost - 100.0).abs() < 1e-9);
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

// --- Goals ---

#[test]
fn goal_dimensions_match_the_game() {
    // RLBot "Useful Game Values" (the page RocketSim cites): goal
    // center-to-post 892.755 uu, height 642.775 uu, depth 880 uu. RocketSim
    // `SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y` = 5124.25 uu.
    assert!(
        (2.0 * GOAL_HALF_WIDTH - 17.8551).abs() < 1e-9,
        "mouth = {}",
        2.0 * GOAL_HALF_WIDTH
    );
    assert!(
        (GOAL_HEIGHT - 6.42775).abs() < 1e-9,
        "height = {GOAL_HEIGHT}"
    );
    assert!((GOAL_DEPTH - 8.80).abs() < 1e-9, "depth = {GOAL_DEPTH}");
    assert!(
        (GOAL_SCORE_BASE_Z - 51.2425).abs() < 1e-9,
        "score line = {GOAL_SCORE_BASE_Z}"
    );
}

#[test]
fn the_end_wall_is_open_only_where_the_goal_is() {
    // The collision field must carry the ball through the mouth and keep the
    // end wall solid around it - under the crossbar and between the posts.
    let a = Arena::default();
    let just_short = a.half_z - 0.01;
    // In the mouth: no wall, so the ball passes.
    let in_mouth = Vec3::new(0.0, 3.0, just_short);
    assert!(
        a.inside_distance(in_mouth) + BALL_RADIUS < 0.0,
        "the goal mouth is blocked"
    );
    // Above the crossbar, and beside a post: solid wall.
    let above = Vec3::new(0.0, GOAL_HEIGHT + 0.2, just_short);
    assert!(
        a.inside_distance(above) + BALL_RADIUS > 0.0,
        "open above the crossbar"
    );
    let beside = Vec3::new(GOAL_HALF_WIDTH + 0.2, 3.0, just_short);
    assert!(
        a.inside_distance(beside) + BALL_RADIUS > 0.0,
        "open beside a post"
    );
}

#[test]
fn the_goal_net_is_a_curved_scoop() {
    // The net is not a flat back wall: it bulges back at mid-height so a car can
    // drive in low and ride the curve up and out. Depth at a few heights down the
    // centre line, walked out from the goal line to the first surface.
    let a = Arena::default();
    let depth_at = |h: f64| {
        let mut z = a.half_z + 0.001;
        while z < a.half_z + GOAL_DEPTH + 0.5 {
            if a.inside_distance(Vec3::new(0.0, h, z)) > 0.0 {
                return z - a.half_z;
            }
            z += 0.001;
        }
        f64::INFINITY
    };
    let low = depth_at(0.3);
    let mid = depth_at(GOAL_NET_CENTER_HEIGHT);
    let high = depth_at(GOAL_HEIGHT - 1.0);
    assert!(mid > low + 1.0, "mid {mid:.2} m vs low {low:.2} m");
    assert!(mid > high + 1.0, "mid {mid:.2} m vs high {high:.2} m");
    // And the deepest point is the full goal depth.
    assert!((mid - GOAL_DEPTH).abs() < 0.1, "deepest {mid:.2} m");
}

#[test]
fn a_car_drives_up_the_goal_scoop() {
    // Drive into the goal along the floor at speed: the curved net must carry the
    // car up the back rather than stopping it flat, i.e. it clears the mouth
    // height. (This is the "in at the bottom, out the top" shape.)
    let mut world = World::new();
    let half_z = world.arena.half_z;
    world.car.pos = Vec3::new(0.0, CAR_HALF_HEIGHT, -half_z + 4.0);
    world.car.forward = Vec3::new(0.0, 0.0, -1.0);
    world.car.up = Vec3::UP;
    world.car.velocity = Vec3::new(0.0, 0.0, -18.0);
    let drive = Input::new(1.0, 0.0, false, false, false);
    let mut peak = 0.0_f64;
    for _ in 0..240 {
        world.step(&drive);
        peak = peak.max(world.car.pos.y);
    }
    assert!(peak > 2.0, "car did not climb the scoop: peak {peak:.2} m");
}

#[test]
fn the_goal_scoop_carries_the_car_back_out_at_any_speed() {
    // In at the bottom of the mouth, up the curved back and back out over the lip -
    // at any entry speed. Before the recess's corners were filleted (`GOAL_FILLET`)
    // a car rolling up the back jammed where the scoop meets the flat back wall and
    // the flat wall meets the roof.
    for speed in [5.0, 9.0, 13.0] {
        let mut world = World::new();
        let half_z = world.arena.half_z;
        world.car.pos = Vec3::new(0.0, CAR_HALF_HEIGHT, half_z - 3.0);
        world.car.forward = Vec3::new(0.0, 0.0, 1.0);
        world.car.up = Vec3::UP;
        world.car.velocity = Vec3::new(0.0, 0.0, speed);
        let drive = Input::new(1.0, 0.0, false, false, false);
        let mut peak = 0.0_f64;
        let mut went_in = false;
        let mut out = false;
        for _ in 0..400 {
            world.step(&drive);
            peak = peak.max(world.car.pos.y);
            let behind = world.car.pos.z - half_z;
            if behind > 2.0 {
                went_in = true;
            }
            if went_in && behind < -1.0 {
                out = true;
                break;
            }
        }
        assert!(went_in, "speed {speed}: the car never entered the goal");
        assert!(peak > 2.0, "speed {speed}: the car never climbed the scoop");
        assert!(out, "speed {speed}: the car jammed in the goal");
    }
}

#[test]
fn ball_scores_in_the_goal_and_returns_to_kickoff() {
    let mut world = World::new();
    let half_z = world.arena.half_z;
    // Fire the ball at the -Z (blue) goal from just shy of the line.
    world.ball.pos = Vec3::new(0.0, 1.0, -half_z + 2.0);
    world.ball.vel = Vec3::new(0.0, 0.0, -25.0);
    let mut scored = false;
    for _ in 0..240 {
        world.step(&Input::default());
        if world.score != [0, 0] {
            scored = true;
            break;
        }
    }
    assert!(scored, "the ball never scored");
    // Blue defends -Z, so a ball in the -Z net scores for orange (index 1).
    assert_eq!(world.score, [0, 1], "score {:?}", world.score);
    // The ball is back on the centre spot, at rest, the instant it scores.
    assert!(
        world.ball.pos.x.abs() < 1e-9 && world.ball.pos.z.abs() < 1e-9,
        "ball not on the centre spot: {:?}",
        world.ball.pos
    );
    assert_eq!(world.ball.vel, Vec3::ZERO, "ball still moving");
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
    world.car.velocity = world.car.heading() * 100.0;
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
