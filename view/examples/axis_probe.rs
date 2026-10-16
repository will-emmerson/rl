//! Diagnostic: what the *player sees* for RocketSim's positive steer/yaw/dodge,
//! with the viewer's frame bridge (`view = (-x, z, y)`) applied.
//!
//!     cargo run -p rl_view --example axis_probe
//!
//! The player's right is `forward x up` (a right-handed frame: facing north with
//! up, your right is east). Everything is reported as a component along that, so
//! a positive number means "the car moved to the player's right". Not used by
//! the game; kept to pin the viewer's input signs against RocketSim instead of
//! reasoning about handedness by hand.

use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Team};

type V = (f32, f32, f32);

/// RocketSim vector -> viewer vector, the same bridge `sim.rs` uses.
fn to_view(x: f32, y: f32, z: f32) -> V {
    (-x, z, y)
}
fn sub(a: V, b: V) -> V {
    (a.0 - b.0, a.1 - b.1, a.2 - b.2)
}
fn dot(a: V, b: V) -> f32 {
    a.0 * b.0 + a.1 * b.1 + a.2 * b.2
}
fn cross(a: V, b: V) -> V {
    (
        a.1 * b.2 - a.2 * b.1,
        a.2 * b.0 - a.0 * b.2,
        a.0 * b.1 - a.1 * b.0,
    )
}

fn controls(steer: f32, yaw: f32, jump: bool) -> CarControls {
    CarControls {
        throttle: 0.0,
        steer,
        pitch: 0.0,
        yaw,
        roll: 0.0,
        jump,
        boost: false,
        handbrake: false,
    }
}

fn fresh() -> (Arena, usize) {
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_rng_seed(0));
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(None);
    (arena, car)
}

/// The car's forward and up, in the viewer's frame.
fn basis(arena: &Arena, car: usize) -> (V, V) {
    let s = arena.get_car_state(car);
    let f = s.get_forward_dir();
    let u = s.get_up_dir();
    (to_view(f.x, f.y, f.z), to_view(u.x, u.y, u.z))
}

fn main() {
    assert!(
        rocketsim::init_from_default(true).is_ok(),
        "needs collision_meshes/soccar next to the CWD"
    );

    // 1. Ground steer, positive: integrate the nose's motion along the player's
    //    right over a normal turn.
    {
        let (mut arena, car) = fresh();
        let mut c = controls(1.0, 0.0, false);
        c.throttle = 1.0;
        let (mut f_prev, _) = basis(&arena, car);
        let mut acc = 0.0;
        for _ in 0..180 {
            arena.set_car_controls(car, c);
            arena.step_tick();
            let (f, u) = basis(&arena, car);
            acc += dot(sub(f, f_prev), cross(f, u));
            f_prev = f;
        }
        println!("steer +1 (ground): nose along player-right = {acc:+.3}  (right if > 0)");
    }

    // 2. Air yaw, positive.
    {
        let (mut arena, car) = fresh();
        arena.set_car_controls(car, controls(0.0, 0.0, true));
        arena.step_tick();
        for _ in 0..10 {
            arena.set_car_controls(car, controls(0.0, 0.0, false));
            arena.step_tick();
        }
        let (mut f_prev, _) = basis(&arena, car);
        let mut acc = 0.0;
        for _ in 0..30 {
            arena.set_car_controls(car, controls(0.0, 1.0, false));
            arena.step_tick();
            let (f, u) = basis(&arena, car);
            acc += dot(sub(f, f_prev), cross(f, u));
            f_prev = f;
        }
        println!("yaw   +1 (air):    nose along player-right = {acc:+.3}  (right if > 0)");
    }

    // 3. Dodge, positive yaw + jump.
    {
        let (mut arena, car) = fresh();
        arena.set_car_controls(car, controls(0.0, 0.0, true));
        arena.step_tick();
        for _ in 0..10 {
            arena.set_car_controls(car, controls(0.0, 0.0, false));
            arena.step_tick();
        }
        let (f0, u0) = basis(&arena, car);
        let v = arena.get_car_state(car).vel;
        let v0 = to_view(v.x, v.y, v.z);
        arena.set_car_controls(car, controls(0.0, 1.0, true));
        arena.step_tick();
        let v = arena.get_car_state(car).vel;
        let v1 = to_view(v.x, v.y, v.z);
        println!(
            "dodge yaw +1:      leap along player-right = {:+.2}  (right if > 0)",
            dot(sub(v1, v0), cross(f0, u0))
        );
    }

    // 4. Air roll, positive: does the car's roof tip toward the player's right?
    {
        let (mut arena, car) = fresh();
        arena.set_car_controls(car, controls(0.0, 0.0, true));
        arena.step_tick();
        for _ in 0..10 {
            arena.set_car_controls(car, controls(0.0, 0.0, false));
            arena.step_tick();
        }
        let (f0, u0) = basis(&arena, car);
        let right = cross(f0, u0); // the player's right, fixed at the start
        let mut roll_c = controls(0.0, 0.0, false);
        roll_c.roll = 1.0;
        let mut u_prev = u0;
        let mut acc = 0.0;
        for _ in 0..30 {
            arena.set_car_controls(car, roll_c);
            arena.step_tick();
            let u = basis(&arena, car).1;
            acc += dot(sub(u, u_prev), right);
            u_prev = u;
        }
        println!("roll  +1 (air):    roof along player-right = {acc:+.3}  (rolls right if > 0)");
        let s = arena.get_car_state(car);
        let f = s.get_forward_dir();
        println!(
            "                  roll rate about forward = {:+.3}  (rolls right if > 0)",
            s.ang_vel.dot(f)
        );
    }
}
