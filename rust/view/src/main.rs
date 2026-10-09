//! Bevy 3D viewer for `rl_sim`.
//!
//! The simulation is untouched and engine-free: this crate owns the render
//! scene, feeds input into `rl_sim::World`, and copies the resulting state onto
//! transforms. Nothing here can affect the physics, so the determinism
//! guarantees in `docs/PHYSICS.md` still hold with the viewer attached.
//!
//! Run it:
//!   cargo run --release -p rl_view              # play
//!   cargo run --release -p rl_view -- --screenshot shot.png

use std::f32::consts::PI;

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};

use rl_sim::ball::BALL_RADIUS;
use rl_sim::car::{CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH};
use rl_sim::{Input as SimInput, World as SimWorld};

/// Arena dimensions, matching `rl_sim::Arena::default()`.
const ARENA_HALF_X: f32 = 40.0;
const ARENA_HALF_Z: f32 = 25.0;
const ARENA_HEIGHT: f32 = 12.0;

/// The camera never gets closer to a wall than this.
const CAMERA_WALL_MARGIN: f32 = 0.8;
/// Preferred chase position: this far behind, and this far above, the car.
const CAMERA_DISTANCE: f32 = 5.5;
const CAMERA_HEIGHT: f32 = 2.6;
/// Where the camera escapes to when a wall blocks the chase position.
const CAMERA_SQUEEZE_HEIGHT: f32 = 5.0;
const CAMERA_SQUEEZE_INSET: f32 = 2.5;
/// How far the aim point may lean from the car towards the ball, and how high
/// above the car it sits.
const CAMERA_AIM_LEAN: f32 = 6.0;
const CAMERA_AIM_HEIGHT: f32 = 0.6;

// ---------------------------------------------------------------------------
// Resources / components
// ---------------------------------------------------------------------------

#[derive(Resource)]
struct Sim(SimWorld);

/// How the app was started.
#[derive(Resource)]
enum Mode {
    Play,
    /// Run until the simulation reaches `capture_at_tick`, dump a PNG, exit.
    Capture {
        path: String,
        capture_at_tick: u64,
    },
}

#[derive(Component)]
enum Vis {
    Car,
    Ball,
}

#[derive(Component)]
struct ChaseCam;

#[derive(Component)]
struct HudText;

fn main() {
    let mut mode = Mode::Play;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        if arg == "--screenshot" {
            let path = args.next().unwrap_or_else(|| "screenshot.png".into());
            // Gate on the simulation tick, not the frame count: the render
            // frame rate varies with load, the tick does not.
            let capture_at_tick = args.next().and_then(|v| v.parse().ok()).unwrap_or(250);
            mode = Mode::Capture {
                path,
                capture_at_tick,
            };
        }
    }

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(Sim(SimWorld::new()))
        .insert_resource(mode)
        .insert_resource(Time::<Fixed>::from_hz(rl_sim::TICK_HZ as f64))
        .add_systems(Startup, setup_scene)
        .add_systems(FixedUpdate, step_simulation)
        .add_systems(
            Update,
            (
                reset_on_key,
                sync_visuals,
                chase_camera,
                update_hud,
                screenshot_driver,
                quit_on_escape,
            ),
        )
        .run();
}

// ---------------------------------------------------------------------------
// Scene
// ---------------------------------------------------------------------------

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mode: Res<Mode>,
) {
    // --- arena ---
    commands.spawn((
        Mesh3d(
            meshes.add(
                Plane3d::default()
                    .mesh()
                    .size(ARENA_HALF_X * 2.0, ARENA_HALF_Z * 2.0),
            ),
        ),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.17, 0.40, 0.29),
            perceptual_roughness: 0.9,
            ..default()
        })),
    ));

    let wall_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.55, 0.58, 0.65),
        perceptual_roughness: 0.7,
        ..default()
    });
    // Walls are 1 m thick, pushed back so their inner faces sit exactly on the
    // simulation's bounds.
    for (half_x, half_y, half_z, x, y, z) in [
        (
            ARENA_HALF_X,
            ARENA_HEIGHT / 2.0,
            0.5,
            0.0,
            ARENA_HEIGHT / 2.0,
            -ARENA_HALF_Z - 0.5,
        ),
        (
            ARENA_HALF_X,
            ARENA_HEIGHT / 2.0,
            0.5,
            0.0,
            ARENA_HEIGHT / 2.0,
            ARENA_HALF_Z + 0.5,
        ),
        (
            0.5,
            ARENA_HEIGHT / 2.0,
            ARENA_HALF_Z,
            -ARENA_HALF_X - 0.5,
            ARENA_HEIGHT / 2.0,
            0.0,
        ),
        (
            0.5,
            ARENA_HEIGHT / 2.0,
            ARENA_HALF_Z,
            ARENA_HALF_X + 0.5,
            ARENA_HEIGHT / 2.0,
            0.0,
        ),
    ] {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(half_x * 2.0, half_y * 2.0, half_z * 2.0))),
            MeshMaterial3d(wall_material.clone()),
            Transform::from_xyz(x, y, z),
        ));
    }

    // Translucent ceiling, so a lofted ball reads as "up there" rather than
    // vanishing off the top of the screen.
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(ARENA_HALF_X * 2.0, 0.5, ARENA_HALF_Z * 2.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.6, 0.75, 0.92, 0.10),
            alpha_mode: AlphaMode::Blend,
            cull_mode: None,
            ..default()
        })),
        Transform::from_xyz(0.0, ARENA_HEIGHT + 0.25, 0.0),
    ));

    // --- ball ---
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(BALL_RADIUS as f32).mesh().uv(32, 18))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.95, 0.96, 0.99),
            perceptual_roughness: 0.25,
            ..default()
        })),
        Transform::default(),
        Vis::Ball,
    ));

    // --- car ---
    // Local axes are (+Z forward, +Y up, +X right), matching the simulation.
    let car = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(
                CAR_HALF_WIDTH as f32 * 2.0,
                CAR_HALF_HEIGHT as f32 * 2.0,
                CAR_HALF_LENGTH as f32 * 2.0,
            ))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.10, 0.60, 0.95),
                metallic: 0.3,
                perceptual_roughness: 0.35,
                ..default()
            })),
            Transform::default(),
            Vis::Car,
        ))
        .id();

    // A vertical fin at the front. The chase camera sits directly behind the
    // car, where a low nose is completely hidden, so the fin is what actually
    // tells you which way you are pointing.
    commands.entity(car).with_children(|parent| {
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.10, 0.34, 0.42))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.98, 0.78, 0.16),
                emissive: LinearRgba::rgb(0.45, 0.28, 0.0),
                ..default()
            })),
            Transform::from_xyz(
                0.0,
                CAR_HALF_HEIGHT as f32 + 0.12,
                CAR_HALF_LENGTH as f32 - 0.22,
            ),
        ));
    });

    // --- lighting ---
    commands.insert_resource(GlobalAmbientLight {
        color: Color::srgb(0.82, 0.88, 1.0),
        brightness: 620.0,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 30_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_rotation_x(-PI / 3.5) * Quat::from_rotation_y(0.6)),
    ));
    // A second, shadowless light from the other side, so walls facing away
    // from the sun are not solid black.
    commands.spawn((
        DirectionalLight {
            illuminance: 7_000.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_rotation_x(-PI / 6.0) * Quat::from_rotation_y(-2.4)),
    ));

    // --- camera ---
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.2, -22.0).looking_at(Vec3::new(0.0, 0.8, 0.0), Vec3::Y),
        ChaseCam,
    ));

    // --- hud ---
    let hint = match &*mode {
        Mode::Play => "WASD drive  space boost  shift drift  R reset  esc quit",
        Mode::Capture { .. } => "capture mode",
    };
    commands.spawn((
        Text::default(),
        Node {
            position_type: PositionType::Absolute,
            top: px(12),
            left: px(12),
            ..default()
        },
        children![TextSpan::new(""), TextSpan::new(""), TextSpan::new(hint)],
        HudText,
    ));
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

fn axis(keys: &ButtonInput<KeyCode>, positive: &[KeyCode], negative: &[KeyCode]) -> f64 {
    let p = positive.iter().any(|k| keys.pressed(*k));
    let n = negative.iter().any(|k| keys.pressed(*k));
    match (p, n) {
        (true, false) => 1.0,
        (false, true) => -1.0,
        _ => 0.0,
    }
}

fn read_input(keys: &ButtonInput<KeyCode>) -> SimInput {
    SimInput::new(
        axis(
            keys,
            &[KeyCode::KeyW, KeyCode::ArrowUp],
            &[KeyCode::KeyS, KeyCode::ArrowDown],
        ),
        axis(
            keys,
            &[KeyCode::KeyA, KeyCode::ArrowLeft],
            &[KeyCode::KeyD, KeyCode::ArrowRight],
        ),
        keys.pressed(KeyCode::Space),
        keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight),
    )
}

fn step_simulation(keys: Res<ButtonInput<KeyCode>>, mut sim: ResMut<Sim>, mode: Res<Mode>) {
    let input = match &*mode {
        Mode::Play => read_input(&keys),
        // Self-driving, so a capture is reproducible rather than dependent on
        // the frame rate. See `capture_input` for the schedule.
        Mode::Capture { .. } => capture_input(sim.0.tick),
    };
    sim.0.step(&input);
}

/// The self-driving script used by `--screenshot`, as a function of the tick.
///
/// Two interesting moments fall out of it: around tick 250 the car is mid-drive
/// with the ball in front (the shot in the README), and by tick 1200 it has
/// driven into one wall and reversed back into the other, which squeezes the
/// camera against a wall - the case that used to stick it to the car.
fn capture_input(tick: u64) -> SimInput {
    if tick < 420 {
        SimInput::new(1.0, 0.0, false, false)
    } else if tick < 1000 {
        SimInput::new(-1.0, 0.0, false, false)
    } else {
        SimInput::new(0.0, 0.0, false, false)
    }
}

fn reset_on_key(keys: Res<ButtonInput<KeyCode>>, mut sim: ResMut<Sim>) {
    if keys.just_pressed(KeyCode::KeyR) {
        sim.0 = SimWorld::new();
    }
}

// ---------------------------------------------------------------------------
// Presentation
// ---------------------------------------------------------------------------

fn sync_visuals(sim: Res<Sim>, mut query: Query<(&mut Transform, &Vis)>) {
    for (mut transform, vis) in &mut query {
        match vis {
            Vis::Car => {
                let car = &sim.0.car;
                let heading = car.heading;
                transform.translation =
                    Vec3::new(car.pos.x as f32, car.pos.y as f32, car.pos.z as f32);
                transform.rotation =
                    Quat::from_rotation_y((heading.x as f32).atan2(heading.z as f32));
            }
            Vis::Ball => {
                let ball = &sim.0.ball;
                transform.translation =
                    Vec3::new(ball.pos.x as f32, ball.pos.y as f32, ball.pos.z as f32);
                // Rolling looks wrong without spin, and the angle is free.
                let spin = ball.spin;
                transform.rotate(Quat::from_scaled_axis(
                    Vec3::new(spin.x as f32, spin.y as f32, spin.z as f32) * (1.0 / 60.0),
                ));
            }
        }
    }
}

/// Keep a point inside the arena, leaving `margin` clearance from every face.
fn clamp_inside(p: Vec3, margin: f32) -> Vec3 {
    Vec3::new(
        p.x.clamp(-ARENA_HALF_X + margin, ARENA_HALF_X - margin),
        p.y.clamp(margin, ARENA_HEIGHT - margin),
        p.z.clamp(-ARENA_HALF_Z + margin, ARENA_HALF_Z - margin),
    )
}

/// How much of the segment `from -> to` stays inside the arena, as a fraction.
///
/// Assumes `from` is already inside, in which case the arena being a box means
/// this is just clipping against three axis-aligned slabs. The result is where
/// a camera on a boom from `from` towards `to` would hit a wall.
fn free_fraction(from: Vec3, to: Vec3, margin: f32) -> f32 {
    let min = [-ARENA_HALF_X + margin, margin, -ARENA_HALF_Z + margin];
    let max = [
        ARENA_HALF_X - margin,
        ARENA_HEIGHT - margin,
        ARENA_HALF_Z - margin,
    ];
    let p = from.to_array();
    let d = (to - from).to_array();

    let mut t = 1.0_f32;
    for axis in 0..3 {
        if d[axis].abs() > 1e-6 {
            let limit = if d[axis] > 0.0 {
                (max[axis] - p[axis]) / d[axis]
            } else {
                (min[axis] - p[axis]) / d[axis]
            };
            t = t.min(limit);
        }
    }
    t.clamp(0.0, 1.0)
}

fn chase_camera(sim: Res<Sim>, time: Res<Time>, mut query: Query<&mut Transform, With<ChaseCam>>) {
    let car = sim.0.car.pos;
    let heading = sim.0.car.heading;
    let ball = sim.0.ball.pos;

    let forward = Vec3::new(heading.x as f32, 0.0, heading.z as f32).normalize_or_zero();
    let car_pos = Vec3::new(car.x as f32, car.y as f32, car.z as f32);
    let ball_pos = Vec3::new(ball.x as f32, ball.y as f32, ball.z as f32);

    // Pull the camera in along the line of sight rather than clamping it to
    // the wall plane. A car backed against a wall has *no* room behind it, and
    // clamping put the camera on top of the car, which is what made it feel
    // stuck. Clipping a boom from the car finds the farthest position that is
    // actually clear.
    let pivot = clamp_inside(car_pos + Vec3::Y * 1.2, CAMERA_WALL_MARGIN);
    let wanted = car_pos - forward * CAMERA_DISTANCE + Vec3::Y * CAMERA_HEIGHT;
    let reach = free_fraction(pivot, wanted, CAMERA_WALL_MARGIN);
    let mut desired = pivot.lerp(wanted, reach);

    let squeeze = 1.0 - reach;
    if squeeze > 0.001 {
        // No room behind the car, so rise and step *sideways* along the wall.
        // Sideways rather than inwards, because a car backed against a wall is
        // usually facing the middle — stepping that way puts the car behind the
        // camera. Sideways rather than straight up because `look_at` then has
        // no usable up vector.
        let side = Vec3::new(forward.z, 0.0, -forward.x);
        let step = if wall_clearance(car_pos + side * CAMERA_SQUEEZE_INSET)
            >= wall_clearance(car_pos - side * CAMERA_SQUEEZE_INSET)
        {
            side
        } else {
            -side
        };
        let escape = car_pos + Vec3::Y * CAMERA_SQUEEZE_HEIGHT + step * CAMERA_SQUEEZE_INSET;
        desired = desired.lerp(escape, squeeze);
    }

    // Both the current position and the target are inside a convex box, so the
    // interpolation can never leave the arena. The clamp is belt and braces.
    let desired = clamp_inside(desired, CAMERA_WALL_MARGIN);

    // Lean the aim towards the ball normally, but square up on the car when the
    // camera has been forced overhead.
    //
    // This is a clamped offset from the car, not a weighted average of the two
    // positions. Blending the raw positions lets the aim point drift behind the
    // camera whenever the ball is, and the camera then stares at the floor with
    // the car somewhere off screen.
    let lean_factor = 0.35 * (1.0 - squeeze);
    let lean = (ball_pos - car_pos)
        .with_y(0.0)
        .clamp_length_max(CAMERA_AIM_LEAN)
        * lean_factor;
    let aim = car_pos + lean + Vec3::Y * CAMERA_AIM_HEIGHT;

    let weight = 1.0 - (-8.0 * time.delta_secs()).exp();
    for mut transform in &mut query {
        transform.translation = transform.translation.lerp(desired, weight);
        // Directly overhead, world up is parallel to the view direction and
        // `look_at` has no valid basis. The car's forward is a sane stand-in.
        let to_aim = aim - transform.translation;
        let up = if to_aim.with_y(0.0).length() >= 0.5 {
            Vec3::Y
        } else if forward.length_squared() > 0.5 {
            forward
        } else {
            Vec3::X
        };
        transform.look_at(aim, up);
    }
}

/// Shortest distance from a point to any of the four walls.
fn wall_clearance(p: Vec3) -> f32 {
    (ARENA_HALF_X - p.x.abs()).min(ARENA_HALF_Z - p.z.abs())
}

fn update_hud(sim: Res<Sim>, hud: Single<Entity, With<HudText>>, mut writer: TextUiWriter) {
    let entity = *hud;
    let car = &sim.0.car;
    let ball = &sim.0.ball;
    *writer.text(entity, 1) = format!(
        "speed {:>5.1} m/s   boost {:>5.1}   tick {:>6}\n",
        car.speed(),
        car.boost,
        sim.0.tick
    );
    *writer.text(entity, 2) = format!(
        "ball {:>5.1} m/s   height {:>5.2} m   spin {:>5.1} rad/s\n",
        ball.vel.length(),
        ball.pos.y,
        ball.spin.length()
    );
}

fn quit_on_escape(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

fn screenshot_driver(
    mut commands: Commands,
    mode: Res<Mode>,
    sim: Res<Sim>,
    mut frames_since_capture: Local<u32>,
    mut captured: Local<bool>,
    mut exit: MessageWriter<AppExit>,
) {
    let Mode::Capture {
        path,
        capture_at_tick,
    } = &*mode
    else {
        return;
    };

    if !*captured && sim.0.tick >= *capture_at_tick {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path.clone()));
        info!("capturing screenshot at tick {} to {path}", sim.0.tick);
        *captured = true;
    }

    // Leave a generous margin for the async capture + PNG encode to finish.
    if *captured {
        *frames_since_capture += 1;
        if *frames_since_capture > 90 {
            exit.write(AppExit::Success);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamp_inside_leaves_the_wall_margin() {
        let p = clamp_inside(Vec3::new(100.0, -5.0, -100.0), CAMERA_WALL_MARGIN);
        assert!((p.x - (ARENA_HALF_X - CAMERA_WALL_MARGIN)).abs() < 1e-6);
        assert!((p.y - CAMERA_WALL_MARGIN).abs() < 1e-6);
        assert!((p.z - (-ARENA_HALF_Z + CAMERA_WALL_MARGIN)).abs() < 1e-6);
    }

    #[test]
    fn open_ice_gives_the_camera_its_full_boom() {
        let pivot = clamp_inside(Vec3::new(0.0, 1.2, 0.0), CAMERA_WALL_MARGIN);
        let wanted = Vec3::new(0.0, 2.6, -CAMERA_DISTANCE);
        assert!((free_fraction(pivot, wanted, CAMERA_WALL_MARGIN) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn a_wall_behind_the_car_blocks_the_boom() {
        // Backed against the -Z wall facing away, so the preferred position is
        // outside the arena. This is the case that used to pin the camera to
        // the wall (and therefore onto the car itself).
        let car = Vec3::new(0.0, 0.18, -(ARENA_HALF_Z - 0.6));
        let pivot = clamp_inside(car + Vec3::Y * 1.2, CAMERA_WALL_MARGIN);
        let wanted = car - Vec3::new(0.0, 0.0, 1.0) * CAMERA_DISTANCE + Vec3::Y * CAMERA_HEIGHT;

        let reach = free_fraction(pivot, wanted, CAMERA_WALL_MARGIN);
        assert!(reach < 0.1, "boom should be blocked, got {reach}");

        // And whatever it does, it must stay inside the arena.
        let blocked = clamp_inside(pivot.lerp(wanted, reach), CAMERA_WALL_MARGIN);
        assert!(blocked.z >= -ARENA_HALF_Z + CAMERA_WALL_MARGIN - 1e-6);
    }
}
