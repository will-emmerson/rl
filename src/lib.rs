//! `rl` — a hand-rolled, deterministic Rocket League style simulation.
//!
//! The whole thing is deliberately dependency-free and free of transcendental
//! maths, so that a given input stream produces a bit-identical state stream
//! on any conforming platform. See `docs/PHYSICS.md` for the reasoning, the
//! model, and an honest account of what "deterministic" does and does not buy
//! you here.
//!
//! ```no_run
//! use rl::{Input, World};
//!
//! let mut world = World::new();
//! let input = Input::new(1.0, 0.0, false, false, false); // full throttle
//! for _ in 0..240 {
//!     world.step(&input);
//! }
//! println!("car at {:?}", world.car.pos);
//! ```

pub mod arena;
pub mod ball;
pub mod car;
pub mod math;
pub mod world;

pub use arena::Arena;
pub use ball::Ball;
pub use car::Car;
pub use math::{heading_forward, heading_right, Vec3};
pub use world::{Input, Snapshot, World, DT, GRAVITY, TICK_HZ};

/// Drive the car straight into the ball from a standing start.
///
/// Returns `(car speed at impact, ball speed shortly after)`.
pub fn head_on_shot() -> (f64, f64) {
    let mut world = World::new();
    let drive = Input::new(1.0, 0.0, false, false, false);
    let mut last_car_speed = 0.0;
    let mut impact = 0.0;

    for _ in 0..1200 {
        world.step(&drive);
        if world.ball.vel.length() > 0.5 {
            impact = last_car_speed;
            break;
        }
        last_car_speed = world.car.speed();
    }

    // Let the ball fly clear of the car before measuring.
    for _ in 0..30 {
        world.step(&drive);
    }
    (impact, world.ball.vel.length())
}
