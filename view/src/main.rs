//! Bevy 3D viewer for `rl`.
//!
//! The simulation is untouched and engine-free: this crate owns the render
//! scene, feeds input into `rl::World`, and copies the resulting state onto
//! transforms. Nothing here can affect the physics, so the determinism
//! guarantees in `docs/PHYSICS.md` still hold with the viewer attached.
//!
//! Run it:
//!   cargo run -p rl_view                          # play (dev profile is playable)
//!   cargo run -p rl_view -- --screenshot shot.png # render one frame, then exit
//!   cargo run -p rl_view -- --screenshot shot.png --cam 0,12,-30 --look 0,1,0
//!                                                 # instant free-cam shot, no sim
//!   cargo run -p rl_view -- --gamepad-debug       # log every raw pad event

use std::f32::consts::PI;

use bevy::app::AppExit;
use bevy::asset::RenderAssetUsages;
use bevy::camera::Hdr;
use bevy::image::{ImageAddressMode, ImageSampler, ImageSamplerDescriptor};
use bevy::input::gamepad::{RawGamepadAxisChangedEvent, RawGamepadButtonChangedEvent};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::post_process::bloom::{Bloom, BloomCompositeMode, BloomPrefilter};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, Face, TextureDimension, TextureFormat};
use bevy::render::view::screenshot::{save_to_disk, Screenshot};

use rl::arena::{GOAL_HALF_WIDTH, GOAL_HEIGHT};
use rl::ball::BALL_RADIUS;
use rl::car::{CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH, CAR_WHEELBASE};

mod sim;
use sim::Input as SimInput;

/// Arena dimensions, matching `rl::Arena::default()`: the goal line, frame and net
/// sit at `ARENA_HALF_Z`.
const ARENA_HALF_Z: f32 = 51.20;

/// World size of one tile of the arena grid texture, in metres. Inside a tile
/// the minor lines are spaced `GRID_TILE / GRID_CELLS` apart, and the tile border
/// is a heavier major line, so the surfaces read as a 2 m grid with a 10 m major
/// grid - enough to eyeball distances and the curve of the arena corners.
const GRID_TILE: f32 = 10.0;
const GRID_CELLS: u32 = 5;
/// Texture resolution per tile, and the pixels per minor cell.
const GRID_TEXELS: u32 = 500;
const GRID_CELL_PX: u32 = GRID_TEXELS / GRID_CELLS;

/// Visual wheel radius, in metres. A fraction of the hitbox half height, so the
/// tyre reaches the hitbox floor.
const WHEEL_RADIUS: f32 = 0.52 * CAR_HALF_HEIGHT as f32;
/// Full-lock angle for the visual front wheels, in radians. RL's real front-wheel
/// angle is speed-dependent; this is a fixed stand-in.
const MAX_WHEEL_STEER: f32 = 0.5;

/// Preferred chase position: this far behind, and this far above, the car.
const CAMERA_DISTANCE: f32 = 5.5;
const CAMERA_HEIGHT: f32 = 2.6;
/// The camera may leave the arena through the walls *and* the roof - the shell is
/// one-sided, so it can still see back in - but it never drops below the pitch,
/// which would film the game from underneath. This is the margin it keeps off the
/// floor.
const CAMERA_FLOOR_MARGIN: f32 = 0.4;
/// How high above the car the straight-ahead car-cam aim point sits.
const CAMERA_AIM_HEIGHT: f32 = 0.6;
/// How far left/right the look control swivels the camera, in radians.
const CAMERA_LOOK_YAW: f32 = 1.5;
/// How far the pitch control orbits the camera up and down around the car, in
/// radians. Push up raises the camera into a top-down overview; pull down drops
/// it towards a flat, horizon-skimming view.
const CAMERA_LOOK_PITCH: f32 = 0.9;
/// The orbit angle is clamped to this range, in radians above the horizontal:
/// `0` is level with the car, `PI/2` is straight overhead. This is what keeps
/// the car in frame - the view can never tip past the car into the ground.
const CAMERA_PITCH_MIN: f32 = 0.10;
const CAMERA_PITCH_MAX: f32 = 1.35;
/// Ball cam sits *lower* than car cam (a shallow orbit angle): looking out at a
/// far ball from a high boom drops the near car off the bottom of the view, so
/// ball cam uses a low boom that keeps both the car and the ball in frame.
const CAMERA_BALL_PITCH: f32 = 0.20;
/// Ball cam's orbit angle is allowed to swing *below* the tangent plane, because
/// "away from the ball" can point out of the arena: on a wall the ball is inside,
/// so the boom points outside the wall and the camera sits outside the cage, looking
/// back in. Car cam's `CAMERA_PITCH_MIN` clamp, by contrast, keeps it above the car.
const CAMERA_BALL_PITCH_MIN: f32 = -1.40;
/// Vertical look is inverted: pushing the stick up looks down (and so raises the
/// camera into the overview). Flip to `false` for the standard mapping.
const CAMERA_LOOK_INVERT_Y: bool = true;
/// How fast the look angle catches up to the swivel target, per second (higher
/// is snappier). Only the swivel is smoothed: the look-behind click is a hard
/// 180° flip, both in and out.
const CAMERA_LOOK_RESPONSE: f32 = 8.0;

// ---------------------------------------------------------------------------
// Resources / components
// ---------------------------------------------------------------------------

#[derive(Resource)]
struct Sim(sim::Sim);

/// How the app was started.
#[derive(Resource)]
enum Mode {
    Play,
    /// Run until the simulation reaches `capture_at_tick`, dump a PNG, exit.
    ///
    /// When `free_cam` is set the camera is pinned and the simulation is never
    /// advanced: the frame is captured once the renderer has warmed up (see
    /// `FREE_SHOT_MIN_FRAMES`). That is the fast path for eyeballing the scene
    /// from a chosen angle, and `capture_at_tick` is ignored.
    Capture {
        path: String,
        capture_at_tick: u64,
        free_cam: Option<FreeCam>,
    },
}

/// A pinned camera pose: where the eye sits and the point it aims at, both in
/// world space. Set by `--cam` / `--look`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct FreeCam {
    pos: Vec3,
    look: Vec3,
}

/// Warm-up before a free-cam capture fires. The 3D scene is not drawn for the
/// first fraction of a second (pipeline compilation and window bring-up), so a
/// shot grabbed sooner comes out as the bare clear colour - the same reason the
/// driven capture has to run for a while before its frame is representative.
///
/// Gate on both a wall-clock floor and a minimum frame count, so the wait holds
/// whether the app is throttled (few frames, lots of time) or running flat out
/// (many frames, little time). This is still far quicker than driving to a tick.
const FREE_SHOT_MIN_FRAMES: u32 = 20;
const FREE_SHOT_MIN_SECS: f32 = 1.0;
/// Where `--cam` looks from when it is given without `--look`, and what `--look`
/// aims at when it is given without `--cam`: a high overview behind the kickoff.
const FREE_CAM_DEFAULT_POS: Vec3 = Vec3::new(0.0, 12.0, -30.0);
const FREE_CAM_DEFAULT_LOOK: Vec3 = Vec3::new(0.0, 1.0, 0.0);

/// Set by `--gamepad-debug`: log every raw gamepad event.
///
/// Detecting a pad and mapping its controls cannot be exercised from a test, so
/// this is the escape hatch: run it, press things, and read the log to see which
/// `GamepadButton` / `GamepadAxis` each physical control produces.
#[derive(Resource, Default)]
struct GamepadDebug(bool);

/// Ball cam toggle. On (the default, as in Rocket League) the chase aim leans
/// toward the ball; off it looks straight ahead along the car. Toggled with `Y`
/// on the pad or `C` on the keyboard.
#[derive(Resource)]
struct BallCam(bool);

#[derive(Component)]
enum Vis {
    Car,
    Ball,
    /// A boost pad's glowing part, indexed into `World::boost_pads`. Hidden while
    /// the pad is on cooldown. The small pads' translucent base disc has no `Vis`
    /// at all, so it stays on the pitch whether or not the boost is available.
    BoostPad(usize),
}

#[derive(Component)]
struct ChaseCam;

#[derive(Component)]
struct HudText;

/// A visual wheel. `steers` marks the front pair, which also yaw with the
/// steering input; every wheel rolls about its axle.
#[derive(Component)]
struct Wheel {
    steers: bool,
}

/// The steering input from the most recent tick, so the visual front wheels can
/// follow the stick. The simulation does not expose the last input, so the viewer
/// keeps its own copy.
#[derive(Resource, Default)]
struct LastSteer(f64);

fn main() {
    let mut gamepad_debug = false;
    // A capture is asked for by `--screenshot PATH [TICK]`. `--cam` / `--look`
    // turn it into an instant free-camera shot (see `resolve_free_cam`).
    let mut shot: Option<(String, Option<u64>)> = None;
    let mut cam: Option<Vec3> = None;
    let mut look: Option<Vec3> = None;

    let mut args = std::env::args().skip(1).peekable();
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--screenshot" => {
                let path = args.next().unwrap_or_else(|| "screenshot.png".into());
                // The tick is an optional bare number. Only consume it when it
                // really is one, so a following flag is not eaten as the tick.
                let capture_at_tick = match args.peek().and_then(|v| v.parse::<u64>().ok()) {
                    Some(tick) => {
                        args.next();
                        Some(tick)
                    }
                    None => None,
                };
                shot = Some((path, capture_at_tick));
            }
            "--cam" => cam = Some(parse_vec3_arg(&mut args, "--cam")),
            "--look" => look = Some(parse_vec3_arg(&mut args, "--look")),
            "--gamepad-debug" => gamepad_debug = true,
            _ => {}
        }
    }

    let free_cam = resolve_free_cam(cam, look);
    if free_cam.is_some() && shot.is_none() {
        eprintln!("note: --cam/--look only pin the camera for a --screenshot capture");
    }

    let mode = match shot {
        Some((path, capture_at_tick)) => Mode::Capture {
            path,
            // Gate on the simulation tick, not the frame count: the render frame
            // rate varies with load, the tick does not. A free-cam shot ignores
            // the tick and captures after a short warm-up instead.
            capture_at_tick: capture_at_tick.unwrap_or(250),
            free_cam,
        },
        None => Mode::Play,
    };

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(Sim(sim::Sim::new()))
        .insert_resource(mode)
        .insert_resource(GamepadDebug(gamepad_debug))
        .insert_resource(BallCam(true))
        .init_resource::<LastSteer>()
        .insert_resource(Time::<Fixed>::from_hz(rl::TICK_HZ as f64))
        .add_systems(Startup, setup_scene)
        .add_systems(FixedUpdate, step_simulation.run_if(simulating))
        .add_systems(
            Update,
            (
                toggle_camera_mode,
                reset_on_key,
                report_gamepads,
                report_gamepad_events,
                sync_visuals,
                spin_wheels,
                chase_camera.run_if(simulating),
                update_hud,
                screenshot_driver,
                quit_on_escape,
            ),
        )
        .run();
}

/// Combine `--cam` / `--look` into a pinned camera pose. Either flag alone is
/// enough; the missing half falls back to a default overview. `None` means
/// neither was given, i.e. keep the chase camera.
fn resolve_free_cam(cam: Option<Vec3>, look: Option<Vec3>) -> Option<FreeCam> {
    if cam.is_none() && look.is_none() {
        return None;
    }
    Some(FreeCam {
        pos: cam.unwrap_or(FREE_CAM_DEFAULT_POS),
        look: look.unwrap_or(FREE_CAM_DEFAULT_LOOK),
    })
}

/// Read the `x,y,z` value for `flag`, exiting with a clear message if it is
/// missing or malformed.
fn parse_vec3_arg(args: &mut impl Iterator<Item = String>, flag: &str) -> Vec3 {
    let raw = args
        .next()
        .unwrap_or_else(|| panic!("{flag} needs an x,y,z value"));
    parse_vec3(&raw).unwrap_or_else(|| panic!("{flag}: bad value {raw:?}, want x,y,z"))
}

/// Parse a comma-separated `x,y,z` triple (`12,6,-30` or `12, 6, -30`).
fn parse_vec3(s: &str) -> Option<Vec3> {
    let mut parts = s.split(',');
    let x = parts.next()?.trim().parse::<f32>().ok()?;
    let y = parts.next()?.trim().parse::<f32>().ok()?;
    let z = parts.next()?.trim().parse::<f32>().ok()?;
    parts.next().is_none().then_some(Vec3::new(x, y, z))
}

/// True when `--cam` has pinned the camera for an instant shot. Then there is no
/// simulation to advance and the chase camera must not fight the fixed pose.
fn is_free_shot(mode: &Mode) -> bool {
    matches!(
        mode,
        Mode::Capture {
            free_cam: Some(_),
            ..
        }
    )
}

/// Run condition for the systems that only make sense while simulating.
fn simulating(mode: Res<Mode>) -> bool {
    !is_free_shot(&mode)
}

// ---------------------------------------------------------------------------
// Scene
// ---------------------------------------------------------------------------

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    sim: Res<Sim>,
    mode: Res<Mode>,
) {
    // --- arena ---
    // The arena is drawn from RocketSim's own collision mesh (`sim::
    // arena_collision_mesh`), so the visible surface *is* the surface the ball
    // and car collide with - the real RL arena, not an approximation. A
    // repeating grid texture makes the scale readable, and vertex colours pick
    // out the pitch green from the grey walls.
    commands.spawn((
        Mesh3d(meshes.add(arena_surface_mesh())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            base_color_texture: Some(images.add(arena_grid_texture())),
            perceptual_roughness: 0.85,
            // The shell is a cage the camera can see through from outside: the
            // mesh is wound so its front faces inward, so back-face culling hides
            // it from outside and the camera may leave the arena (see
            // `chase_camera`) and still watch the car through the wall.
            cull_mode: Some(Face::Back),
            ..default()
        })),
    ));

    // --- ball ---
    // A football (truncated icosahedron): the black pentagons are what make the
    // spin readable, and it reads as a ball rather than a wireframe globe.
    commands.spawn((
        Mesh3d(meshes.add(soccer_ball_mesh(BALL_RADIUS as f32))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            perceptual_roughness: 0.35,
            ..default()
        })),
        Transform::default(),
        Vis::Ball,
    ));

    // --- boost pads ---
    // The six big pads are Rocket League-style translucent orbs; the 28 small
    // ones are flat discs on the floor. Both are hidden while on cooldown (see
    // `sync_visuals`).
    let pad_big_mesh = meshes.add(Sphere::new(0.8).mesh().uv(24, 18));
    let pad_small_mesh = meshes.add(Cylinder::new(0.55, 0.08));
    // The small pad is two pieces: a faint translucent disc (the part you can
    // see the pitch grid through) plus a small opaque core that blooms. They have
    // to be separate meshes because one material cannot be both - a surface that
    // blooms is near-white and hides whatever is behind it, whatever its alpha.
    // The core is opaque so it takes part in the opaque pass and the translucent
    // disc then depth-tests and blends over it, leaving an annulus of see-through
    // disc around a glowing middle.
    let pad_small_core_mesh = meshes.add(Cylinder::new(0.22, 0.1));
    let pad_big_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(1.0, 0.72, 0.15, 0.4),
        // Emissive is what makes the orb glow: it is set well above the bloom
        // threshold below, so the pad blooms where nothing lit does. The orb is
        // blended, so the emissive has to be strong to survive `alpha < 1` and
        // still throw a halo, while staying weak enough that the pitch shows
        // through it.
        emissive: LinearRgba::rgb(14.0, 9.5, 2.0),
        alpha_mode: AlphaMode::Blend,
        perceptual_roughness: 1.0,
        // Both cap faces of the sphere are drawn, so the far wall of the orb
        // shows through the near one and the sphere reads as a volume, not a
        // flat disc.
        cull_mode: None,
        ..default()
    });
    let pad_small_mat = materials.add(StandardMaterial {
        // Kept faint so it reads as translucent, with the pitch grid showing
        // straight through it. All of the small pad's glow comes from the core
        // below, not from here.
        base_color: Color::srgba(1.0, 0.85, 0.28, 0.35),
        emissive: LinearRgba::rgb(0.5, 0.36, 0.08),
        alpha_mode: AlphaMode::Blend,
        perceptual_roughness: 1.0,
        ..default()
    });
    let pad_small_core_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.85, 0.45),
        // Above the bloom threshold, so the core throws the halo the disc cannot.
        // Kept modest on purpose: a hotter core washes the grid out of the ring.
        emissive: LinearRgba::rgb(10.0, 6.7, 1.7),
        perceptual_roughness: 1.0,
        ..default()
    });
    for i in 0..sim.0.num_pads() {
        let pad = sim.0.pad(i);
        if pad.is_big {
            // The orb *is* the pad, so the whole thing goes on cooldown.
            commands.spawn((
                Mesh3d(pad_big_mesh.clone()),
                MeshMaterial3d(pad_big_mat.clone()),
                Transform::from_xyz(pad.pos.x, pad.pos.y, pad.pos.z),
                Vis::BoostPad(i),
            ));
        } else {
            // The translucent disc is permanent scenery: it carries no `Vis`, so
            // `sync_visuals` never hides it and the marking stays put. Disc sits
            // on the floor (half its height above y = 0).
            commands.spawn((
                Mesh3d(pad_small_mesh.clone()),
                MeshMaterial3d(pad_small_mat.clone()),
                Transform::from_xyz(pad.pos.x, 0.04, pad.pos.z),
            ));
            // Only the glow comes and goes. Poking just above the disc so it is
            // not coplanar; shares the pad index, so `sync_visuals` hides it on
            // cooldown.
            commands.spawn((
                Mesh3d(pad_small_core_mesh.clone()),
                MeshMaterial3d(pad_small_core_mat.clone()),
                Transform::from_xyz(pad.pos.x, 0.05, pad.pos.z),
                Vis::BoostPad(i),
            ));
        }
    }

    // --- goals ---
    // A team-coloured frame (posts and crossbar) at each goal line. The recess
    // behind it - the mouth and the net scoop - is part of RocketSim's collision
    // mesh, so `arena_surface_mesh` already draws it; only the coloured frame is
    // added here. Sizes come from `rl::arena`.
    let goal_frame_blue = materials.add(StandardMaterial {
        base_color: Color::srgb(0.20, 0.45, 0.95),
        emissive: LinearRgba::rgb(0.10, 0.22, 0.50),
        perceptual_roughness: 0.5,
        ..default()
    });
    let goal_frame_orange = materials.add(StandardMaterial {
        base_color: Color::srgb(1.00, 0.45, 0.12),
        emissive: LinearRgba::rgb(0.50, 0.20, 0.04),
        perceptual_roughness: 0.5,
        ..default()
    });
    let w = GOAL_HALF_WIDTH as f32;
    let h = GOAL_HEIGHT as f32;
    // Blue defends -Z, orange +Z (matching `rl::arena::Goal`).
    for (sign, frame) in [(-1.0_f32, goal_frame_blue), (1.0_f32, goal_frame_orange)] {
        let z_line = sign * ARENA_HALF_Z;
        // Frame: two posts and a crossbar, at the goal line.
        for x in [-w, w] {
            commands.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.16, h, 0.16))),
                MeshMaterial3d(frame.clone()),
                Transform::from_xyz(x, h * 0.5, z_line),
            ));
        }
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(2.0 * w + 0.16, 0.16, 0.16))),
            MeshMaterial3d(frame.clone()),
            Transform::from_xyz(0.0, h, z_line),
        ));
    }

    // --- car ---
    // Local axes are (+Z forward, +Y up, +X right), matching the simulation, with
    // the origin at the hitbox centre. The body is a three-box sedan - bonnet,
    // cabin, boot - built by extruding Z-Y profiles across the width, so the roof
    // reaches the hitbox ceiling, the nose and tail reach its ends, and the sill
    // sits clear of the ground with the wheels filling the arches. The collision
    // box is unchanged; this is only what you see.
    let (hw, hh, hl) = (
        CAR_HALF_WIDTH as f32,
        CAR_HALF_HEIGHT as f32,
        CAR_HALF_LENGTH as f32,
    );
    let body_color = linear_rgba(0.10, 0.60, 0.95);
    let glass_color = linear_rgba(0.04, 0.09, 0.15);
    let underside_color = linear_rgba(0.07, 0.07, 0.08);
    // One material for the whole body: the colours come from the vertices.
    let car_mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        metallic: 0.35,
        perceptual_roughness: 0.4,
        // Both sides, rather than relying on the winding of every extruded strip.
        cull_mode: None,
        ..default()
    });
    let tyre_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.05, 0.05, 0.06),
        perceptual_roughness: 0.9,
        ..default()
    });
    let rim_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.74, 0.77, 0.80),
        metallic: 0.6,
        perceptual_roughness: 0.4,
        ..default()
    });
    let light_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.93, 0.62),
        emissive: LinearRgba::rgb(1.1, 0.95, 0.5),
        ..default()
    });
    let tail_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.85, 0.10, 0.10),
        emissive: LinearRgba::rgb(0.6, 0.05, 0.05),
        ..default()
    });

    // A bare node: the parts hang off it so `sync_visuals` can drive the whole
    // car from the hitbox transform.
    let car = commands
        .spawn((Transform::default(), Visibility::default(), Vis::Car))
        .id();

    commands.entity(car).with_children(|parent| {
        // The sill is the underside of the bodywork. It sits at the wheel centre,
        // so the wheels stand proud of it - flush with the sill they would look
        // like castors, and the car would appear to have no ground clearance.
        let sill = -0.09_f32;
        let beltline = 0.10_f32; // the bonnet/boot top, where the glass begins

        // Lower body: one slab the full hitbox length, its nose and tail sloping
        // in below the beltline. The bonnet and boot are just the parts of its top
        // in front of and behind the cabin.
        parent.spawn((
            Mesh3d(meshes.add(extrude_profile(
                &[
                    (hl, sill),
                    (hl, -0.02),
                    (0.94 * hl, beltline),
                    (-0.94 * hl, beltline),
                    (-hl, -0.02),
                    (-hl, sill),
                ],
                0.83 * hw,
                body_color,
                &[
                    body_color,
                    body_color,
                    body_color,
                    body_color,
                    body_color,
                    underside_color,
                ],
            ))),
            MeshMaterial3d(car_mat.clone()),
            Transform::default(),
        ));
        // Cabin: the greenhouse, sitting on the beltline and narrower than the
        // body so its flanks show as glass, with a raked windscreen and rear
        // window and a solid roof on the hitbox ceiling.
        parent.spawn((
            Mesh3d(meshes.add(extrude_profile(
                &[
                    (0.33 * hl, beltline),
                    (0.27 * hl, hh),
                    (-0.27 * hl, hh),
                    (-0.33 * hl, beltline),
                ],
                0.69 * hw,
                glass_color,
                &[glass_color, body_color, glass_color, body_color],
            ))),
            MeshMaterial3d(car_mat.clone()),
            Transform::default(),
        ));

        // Wheels. Each is a node that steers (the front pair only) and rolls
        // about its axle, so tyre, rim and spoke cross turn together - a plain
        // cylinder looks identical however it is spun, so the rim and spokes are
        // what make the rotation visible. They sit flush with the body sides, on
        // the sim's wheelbase.
        let front_axle = 0.46_f32;
        let rear_axle = front_axle - CAR_WHEELBASE as f32;
        let wheel_w = 0.24 * hw;
        for (x, z, steers) in [
            (-0.72 * hw, front_axle, true),
            (0.72 * hw, front_axle, true),
            (-0.72 * hw, rear_axle, false),
            (0.72 * hw, rear_axle, false),
        ] {
            parent
                .spawn((
                    Transform::from_xyz(x, -hh + WHEEL_RADIUS, z),
                    Visibility::default(),
                    Wheel { steers },
                ))
                .with_children(|wheel| {
                    let axle = Quat::from_rotation_z(PI / 2.0);
                    wheel.spawn((
                        Mesh3d(meshes.add(Cylinder::new(WHEEL_RADIUS, wheel_w))),
                        MeshMaterial3d(tyre_mat.clone()),
                        Transform::from_rotation(axle),
                    ));
                    wheel.spawn((
                        Mesh3d(meshes.add(Cylinder::new(0.62 * WHEEL_RADIUS, wheel_w * 1.12))),
                        MeshMaterial3d(rim_mat.clone()),
                        Transform::from_rotation(axle),
                    ));
                    for angle in [0.0, PI / 2.0] {
                        wheel.spawn((
                            Mesh3d(meshes.add(Cuboid::new(
                                wheel_w * 1.16,
                                1.4 * WHEEL_RADIUS,
                                0.04,
                            ))),
                            MeshMaterial3d(rim_mat.clone()),
                            Transform::from_rotation(Quat::from_rotation_x(angle)),
                        ));
                    }
                });
        }

        // Headlights at the nose and tail lights at the back, so the two ends
        // read apart.
        for x in [-0.55 * hw, 0.55 * hw] {
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.30 * hw, 0.055, 0.05))),
                MeshMaterial3d(light_mat.clone()),
                Transform::from_xyz(x, -0.055, hl + 0.005),
            ));
        }
        for x in [-0.55 * hw, 0.55 * hw] {
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.30 * hw, 0.055, 0.05))),
                MeshMaterial3d(tail_mat.clone()),
                Transform::from_xyz(x, -0.055, -hl - 0.005),
            ));
        }
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
    // HDR plus bloom, so the emissive boost pads glow.
    //
    // The prefilter threshold is the whole trick, and it has to sit *above*
    // everything else in the scene. HDR linear values here are already exposure-
    // scaled (the camera uses the default `Exposure`, EV100 9.7), so a lit white
    // surface is only a couple of units - the ball peaks around 2-3 - while an
    // unlit wall is well under 1. A threshold near 1 therefore caught the ball,
    // the car body and the walls too, and the additive composite smeared the
    // whole frame into a glow. At 6.0 only the big orbs clear it once blended
    // (`alpha * emissive` is ~5.6, just inside the knee below); the small discs
    // are held under the threshold on purpose so they stay translucent (see
    // `pad_small_mat`). The soft knee reaches down to
    // `threshold * (1 - softness)` = 3.0, still clear of the ball.
    // The chase camera normally starts behind the kickoff and then follows the
    // car. A `--cam` shot pins it to a fixed pose instead, and the chase system
    // is skipped (see `simulating`).
    let camera_pose = match &*mode {
        Mode::Capture {
            free_cam: Some(free),
            ..
        } => Transform::from_translation(free.pos).looking_at(free.look, Vec3::Y),
        _ => Transform::from_xyz(0.0, 3.2, -22.0).looking_at(Vec3::new(0.0, 0.8, 0.0), Vec3::Y),
    };
    commands.spawn((
        Camera3d::default(),
        Hdr,
        Bloom {
            intensity: 0.4,
            prefilter: BloomPrefilter {
                threshold: 6.0,
                threshold_softness: 0.5,
            },
            // Additive, so the pads *flare* rather than merely brighten. This is
            // the recommended composite mode once a non-energy-conserving
            // prefilter is used.
            composite_mode: BloomCompositeMode::Additive,
            ..Bloom::NATURAL
        },
        camera_pose,
        ChaseCam,
    ));

    // --- hud ---
    // Only the readouts; the control bindings live in the README, not on screen.
    commands.spawn((
        Text::default(),
        Node {
            position_type: PositionType::Absolute,
            top: px(12),
            left: px(12),
            ..default()
        },
        children![TextSpan::new(""), TextSpan::new(""), TextSpan::new("")],
        HudText,
    ));
}

// ---------------------------------------------------------------------------
// Arena surface and grid
// ---------------------------------------------------------------------------

/// The arena's collision surface as a single triangle mesh, read straight from
/// RocketSim (`sim::arena_collision_mesh`) - the same geometry RocketSim collides
/// against, so the drawn surface and the collision surface are one and the same.
/// A grid UV and the pitch-vs-wall colour are baked into the vertices.
///
/// The source triangles have no guaranteed winding, so each is oriented here so
/// its front face looks inward (toward the arena centre); the vertex normals are
/// the area-weighted average of those, and the grid UV comes from projecting the
/// vertex onto whichever axis plane it faces most directly.
fn arena_surface_mesh() -> Mesh {
    let (verts, tris) = sim::arena_collision_mesh();

    // A point inside the shell, used to orient every face inward.
    let mut lo = Vec3::splat(f32::MAX);
    let mut hi = Vec3::splat(f32::MIN);
    for v in &verts {
        lo = lo.min(*v);
        hi = hi.max(*v);
    }
    let centre = (lo + hi) * 0.5;

    let floor_color = linear_rgba(0.17, 0.40, 0.29);
    let wall_color = linear_rgba(0.60, 0.63, 0.70);

    let mut normals = vec![Vec3::ZERO; verts.len()];
    let mut indices = Vec::with_capacity(tris.len() * 3);
    for t in &tris {
        let (ia, ib, ic) = (t[0] as usize, t[1] as usize, t[2] as usize);
        let (a, b, c) = (verts[ia], verts[ib], verts[ic]);
        let n = (b - a).cross(c - a);
        let inward = centre - (a + b + c) / 3.0;
        let flip = n.dot(inward) < 0.0;
        let face = if flip { -n } else { n };
        let (i1, i2) = if flip { (ic, ib) } else { (ib, ic) };
        indices.extend_from_slice(&[ia as u32, i1 as u32, i2 as u32]);
        normals[ia] += face;
        normals[ib] += face;
        normals[ic] += face;
    }

    let mut normal_out = Vec::with_capacity(verts.len());
    let mut uvs = Vec::with_capacity(verts.len());
    let mut colors = Vec::with_capacity(verts.len());
    for (v, n) in verts.iter().zip(&normals) {
        let n = n.normalize_or_zero();
        normal_out.push(n.to_array());

        // Grid UV: project onto the plane the vertex faces most directly.
        let an = n.abs();
        let (u, w) = if an.y >= an.x && an.y >= an.z {
            (v.x, v.z)
        } else if an.x >= an.z {
            (v.z, v.y)
        } else {
            (v.x, v.y)
        };
        uvs.push([u / GRID_TILE, w / GRID_TILE]);

        // Green where the surface faces up (the pitch and its base fillet), grey
        // on the walls and ceiling.
        let t = ((n.y - 0.3) / 0.5).clamp(0.0, 1.0);
        colors.push(lerp_rgba(wall_color, floor_color, t));
    }

    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(
        Mesh::ATTRIBUTE_POSITION,
        verts.iter().map(Vec3::to_array).collect::<Vec<_>>(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normal_out)
    .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U32(indices))
}

/// A repeating grid: white background (so the vertex colour shows through) with a
/// darker minor line every `GRID_TILE / GRID_CELLS` metres and a heavier major
/// line on the tile border.
fn arena_grid_texture() -> Image {
    let size = GRID_TEXELS as usize;
    let cell = GRID_CELL_PX as usize;
    let mut data = vec![0u8; size * size * 4];
    for y in 0..size {
        for x in 0..size {
            let major = x < 3 || y < 3 || x + 3 >= size || y + 3 >= size;
            let minor = x % cell < 2 || y % cell < 2;
            let value = if major {
                55
            } else if minor {
                196
            } else {
                255
            };
            let i = (y * size + x) * 4;
            data[i] = value;
            data[i + 1] = value;
            data[i + 2] = value;
            data[i + 3] = 255;
        }
    }
    let mut image = Image::new(
        Extent3d {
            width: GRID_TEXELS,
            height: GRID_TEXELS,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    );
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::Repeat,
        address_mode_v: ImageAddressMode::Repeat,
        ..default()
    });
    image
}

/// A football: the truncated icosahedron - 12 black pentagons and 20 white
/// hexagons - inscribed at `radius`. The panels are what make the ball's spin
/// readable, and this reads as a ball rather than a wireframe globe.
///
/// The construction is the textbook one: cut every vertex of an icosahedron a
/// third of the way along each incident edge. Each cut vertex leaves a pentagon,
/// and what is left of each triangular face is a hexagon. The corner points are
/// shared, so each panel is emitted as its own fan of triangles with its own
/// colour.
fn soccer_ball_mesh(radius: f32) -> Mesh {
    let phi = (1.0 + 5.0_f32.sqrt()) * 0.5;
    let raw = [
        Vec3::new(-1.0, phi, 0.0),
        Vec3::new(1.0, phi, 0.0),
        Vec3::new(-1.0, -phi, 0.0),
        Vec3::new(1.0, -phi, 0.0),
        Vec3::new(0.0, -1.0, phi),
        Vec3::new(0.0, 1.0, phi),
        Vec3::new(0.0, -1.0, -phi),
        Vec3::new(0.0, 1.0, -phi),
        Vec3::new(phi, 0.0, -1.0),
        Vec3::new(phi, 0.0, 1.0),
        Vec3::new(-phi, 0.0, -1.0),
        Vec3::new(-phi, 0.0, 1.0),
    ];
    let v: Vec<Vec3> = raw.iter().map(|p| p.normalize()).collect();

    // The icosahedron's edge is its shortest vertex-to-vertex distance, so two
    // vertices are neighbours exactly when they are that far apart.
    let mut edge = f32::MAX;
    for i in 0..v.len() {
        for j in i + 1..v.len() {
            edge = edge.min(v[i].distance(v[j]));
        }
    }
    let adjacent = |i: usize, j: usize| (v[i].distance(v[j]) - edge).abs() < 1e-4;

    // The point a third of the way from `i` towards its neighbour `j`.
    let cut = |i: usize, j: usize| v[i] + (v[j] - v[i]) / 3.0;

    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut colors = Vec::new();
    let mut indices = Vec::new();
    let black = linear_rgba(0.06, 0.06, 0.07);
    let white = linear_rgba(0.95, 0.96, 0.99);

    // Pentagons: one per icosahedron vertex, its five corners the cuts towards
    // that vertex's five neighbours, sorted into a ring.
    for i in 0..v.len() {
        let n = v[i];
        let seed = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        let t1 = (seed - n * seed.dot(n)).normalize();
        let t2 = n.cross(t1);
        let angle = |p: Vec3| p.dot(t2).atan2(p.dot(t1));
        let mut corners: Vec<Vec3> = (0..v.len())
            .filter(|&j| j != i && adjacent(i, j))
            .map(|j| cut(i, j))
            .collect();
        corners.sort_by(|a, b| angle(*a).total_cmp(&angle(*b)));
        push_panel(
            &corners,
            black,
            &mut positions,
            &mut normals,
            &mut colors,
            &mut indices,
        );
    }

    // Hexagons: one per icosahedron face, its six corners the two cuts on each
    // of the face's three edges, in ring order.
    for i in 0..v.len() {
        for j in i + 1..v.len() {
            if !adjacent(i, j) {
                continue;
            }
            for k in j + 1..v.len() {
                if adjacent(i, k) && adjacent(j, k) {
                    let corners = [
                        cut(i, j),
                        cut(j, i),
                        cut(j, k),
                        cut(k, j),
                        cut(k, i),
                        cut(i, k),
                    ];
                    push_panel(
                        &corners,
                        white,
                        &mut positions,
                        &mut normals,
                        &mut colors,
                        &mut indices,
                    );
                }
            }
        }
    }

    // Put the corners on the requested radius.
    let max_r = positions
        .iter()
        .fold(0.0_f32, |m, p| m.max(Vec3::new(p[0], p[1], p[2]).length()));
    let scale = radius / max_r;
    for p in &mut positions {
        p[0] *= scale;
        p[1] *= scale;
        p[2] *= scale;
    }

    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
    .with_inserted_indices(Indices::U32(indices))
}

/// Append one convex panel as a triangle fan, in `color`. The ring is wound so
/// its front face points outwards, and the normals are radial, so the ball still
/// shades smoothly across the panel seams.
fn push_panel(
    points: &[Vec3],
    color: [f32; 4],
    positions: &mut Vec<[f32; 3]>,
    normals: &mut Vec<[f32; 3]>,
    colors: &mut Vec<[f32; 4]>,
    indices: &mut Vec<u32>,
) {
    let mut face_normal = Vec3::ZERO;
    let mut centroid = Vec3::ZERO;
    for k in 0..points.len() {
        face_normal += points[k].cross(points[(k + 1) % points.len()]);
        centroid += points[k];
    }
    let ring: Vec<Vec3> = if face_normal.dot(centroid) >= 0.0 {
        points.to_vec()
    } else {
        points.iter().rev().copied().collect()
    };

    let base = positions.len() as u32;
    for p in &ring {
        positions.push(p.to_array());
        normals.push(p.normalize().to_array());
        colors.push(color);
    }
    for k in 1..ring.len() - 1 {
        indices.extend_from_slice(&[base, base + k as u32, base + k as u32 + 1]);
    }
}

/// Extrude a closed, convex 2D profile - a list of `(z, y)` points - across X by
/// ±`half_w`, giving the prism used for the car's body panels. `strip_colors[i]`
/// colours the quad on the edge from point `i` to point `i + 1`; both end caps
/// take `cap_color`. The profile must be listed so that the edge normal
/// `(dy, -dz)` points outwards (the order the car profiles use).
fn extrude_profile(
    profile: &[(f32, f32)],
    half_w: f32,
    cap_color: [f32; 4],
    strip_colors: &[[f32; 4]],
) -> Mesh {
    let n = profile.len();
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut colors = Vec::new();
    let mut indices = Vec::new();

    // The two end caps, fanned from the first point (the profiles are convex).
    for side in [-1.0_f32, 1.0] {
        let base = positions.len() as u32;
        for &(z, y) in profile {
            positions.push([side * half_w, y, z]);
            normals.push([side, 0.0, 0.0]);
            colors.push(cap_color);
        }
        for k in 1..n as u32 - 1 {
            indices.extend_from_slice(&[base, base + k, base + k + 1]);
        }
    }

    // One quad per profile edge, spanning the width.
    for i in 0..n {
        let (z0, y0) = profile[i];
        let (z1, y1) = profile[(i + 1) % n];
        let (dz, dy) = (z1 - z0, y1 - y0);
        let len = (dz * dz + dy * dy).sqrt();
        let (nz, ny) = (dy / len, -dz / len);
        let base = positions.len() as u32;
        for (x, z, y) in [
            (-half_w, z0, y0),
            (half_w, z0, y0),
            (half_w, z1, y1),
            (-half_w, z1, y1),
        ] {
            positions.push([x, y, z]);
            normals.push([0.0, ny, nz]);
            colors.push(strip_colors[i]);
        }
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
    .with_inserted_indices(Indices::U32(indices))
}

/// An sRGB colour as a linear `[f32; 4]`, for a vertex colour attribute.
fn linear_rgba(r: f32, g: f32, b: f32) -> [f32; 4] {
    let c = Color::srgb(r, g, b).to_linear();
    [c.red, c.green, c.blue, c.alpha]
}

/// Per-component linear-space blend, `t = 0` giving `a` and `t = 1` giving `b`.
fn lerp_rgba(a: [f32; 4], b: [f32; 4], t: f32) -> [f32; 4] {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
        a[3] + (b[3] - a[3]) * t,
    ]
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

/// Whichever of the two is further from zero, so the keyboard and the pad can
/// be used interchangeably without cancelling each other out.
fn strongest(a: f64, b: f64) -> f64 {
    if a.abs() >= b.abs() {
        a
    } else {
        b
    }
}

/// Reads one analogue trigger, in `0.0..=1.0`.
///
/// gilrs maps a pad through a database entry keyed by its device id. This pad
/// has no entry (see `examples/gamepad_probe.rs`), so it falls back to gilrs'
/// built-in default mapping, which exposes the xpad triggers as the *axes*
/// `LeftZ` / `RightZ` (evdev ABS_Z / ABS_RZ) rather than as trigger buttons.
/// Those axes run 0..=255 and are normalised to `-1.0..=1.0`, so they rest near
/// `-1.0` and reach `+1.0` when fully pulled. Pads that *do* have a database
/// entry report the triggers as buttons instead, so accept either and take the
/// larger.
fn trigger(pad: &Gamepad, axis: GamepadAxis, button: GamepadButton) -> f64 {
    let from_axis = pad
        .get(axis)
        .map(|v| ((v as f64 + 1.0) * 0.5).clamp(0.0, 1.0))
        .unwrap_or(0.0);
    let from_button = pad.get(button).unwrap_or(0.0) as f64;

    // The axis' resting value only maps to about 0, not exactly 0, so trim the
    // slack and stop the car creeping at idle.
    let value = strongest(from_axis, from_button);
    if value < 0.05 {
        0.0
    } else {
        value
    }
}

fn read_input(keys: &ButtonInput<KeyCode>, gamepad: Option<&Gamepad>) -> SimInput {
    let keyboard_throttle = axis(
        keys,
        &[KeyCode::KeyW, KeyCode::ArrowUp],
        &[KeyCode::KeyS, KeyCode::ArrowDown],
    );
    let keyboard_steer = axis(
        keys,
        &[KeyCode::KeyA, KeyCode::ArrowLeft],
        &[KeyCode::KeyD, KeyCode::ArrowRight],
    );
    // Air roll, on dedicated keys: E rolls right, Q rolls left.
    let keyboard_roll = axis(keys, &[KeyCode::KeyE], &[KeyCode::KeyQ]);
    let keyboard_jump = keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::ControlRight);

    // Triggers are analogue, so they give proportional throttle rather than the
    // keyboard's full-on. Button layout matches Rocket League: A jumps, B boosts
    // and X is the handbrake. The pad's *stick* axes carry steer and air pitch
    // (throttle is the triggers), which is how RL binds it.
    //
    // NOTE: a driver-default (unmapped) X-Box pad gets gilrs' built-in mapping,
    // where evdev's `BTN_NORTH`/`BTN_WEST` are the aliases of `BTN_X`/`BTN_Y`.
    // So the *physical X* button arrives as `North` and the *physical Y* as
    // `West`. Drift is therefore `North` (physical X, as the README says) and
    // the camera toggle is `West` (physical Y). On an SDL-mapped pad these two
    // are the other way round.
    let (pad_throttle, pad_steer, pad_pitch, pad_boost, pad_drift, pad_jump) = match gamepad {
        Some(pad) => (
            trigger(pad, GamepadAxis::RightZ, GamepadButton::RightTrigger2)
                - trigger(pad, GamepadAxis::LeftZ, GamepadButton::LeftTrigger2),
            // Sticks are positive to the right; `steer` is positive to the left.
            -pad.get(GamepadAxis::LeftStickX).unwrap_or(0.0) as f64,
            // Sticks are positive up here (see `look_input`); pitch +1 is nose
            // up, so the sign flips.
            -pad.get(GamepadAxis::LeftStickY).unwrap_or(0.0) as f64,
            pad.get(GamepadButton::East).unwrap_or(0.0) > 0.5,
            pad.get(GamepadButton::North).unwrap_or(0.0) > 0.5,
            pad.get(GamepadButton::South).unwrap_or(0.0) > 0.5,
        ),
        None => (0.0, 0.0, 0.0, false, false, false),
    };

    let throttle = strongest(keyboard_throttle, pad_throttle);
    let steer = strongest(keyboard_steer, pad_steer);
    let drift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight) || pad_drift;

    // Air orientation. RL flies with the same stick it drives with: stick up is a
    // frontflip (`pitch -1`) and stick left yaws left (`yaw = -steer`). On the
    // keyboard the W/S keys double as pitch; the pad uses the left stick's
    // vertical axis, which is why pitch is *not* taken from the throttle.
    let keyboard_pitch = -keyboard_throttle;
    let pitch = strongest(keyboard_pitch, pad_pitch);
    let yaw = -steer;

    // On the pad, air roll is the handbrake/drift button plus the stick, as in
    // Rocket League; the keyboard uses Q/E.
    let pad_roll = if pad_drift { -pad_steer } else { 0.0 };
    let roll = strongest(keyboard_roll, pad_roll);

    SimInput::with_air(
        throttle,
        steer,
        pitch,
        yaw,
        roll,
        keys.pressed(KeyCode::Space) || pad_boost,
        drift,
        keyboard_jump || pad_jump,
    )
}

fn step_simulation(
    keys: Res<ButtonInput<KeyCode>>,
    gamepads: Query<&Gamepad>,
    mut sim: ResMut<Sim>,
    mut last_steer: ResMut<LastSteer>,
    mode: Res<Mode>,
) {
    let input = match &*mode {
        Mode::Play => read_input(&keys, gamepads.iter().next()),
        // Self-driving, so a capture is reproducible rather than dependent on
        // the frame rate. See `capture_input` for the schedule.
        Mode::Capture { .. } => capture_input(sim.0.tick()),
    };
    last_steer.0 = input.steer;
    sim.0.step(&input);
}

/// Logs gamepad connect/disconnect.
///
/// Detecting the pad is the one part of controller support that cannot be
/// tested from code, so it says so loudly on stdout.
fn report_gamepads(gamepads: Query<Entity, With<Gamepad>>, mut seen: Local<usize>) {
    let connected = gamepads.iter().count();
    if connected != *seen {
        *seen = connected;
        if connected == 0 {
            info!("no gamepad detected");
        } else {
            info!("gamepad detected ({connected} connected)");
        }
    }
}

/// Stable index for the axes `report_gamepad_events` tracks, so it can throttle
/// the log without a map. Non-standard axes share the last slot.
fn axis_slot(axis: GamepadAxis) -> usize {
    match axis {
        GamepadAxis::LeftStickX => 0,
        GamepadAxis::LeftStickY => 1,
        GamepadAxis::LeftZ => 2,
        GamepadAxis::RightStickX => 3,
        GamepadAxis::RightStickY => 4,
        GamepadAxis::RightZ => 5,
        GamepadAxis::Other(_) => 6,
    }
}

/// Logs raw gamepad events, for `--gamepad-debug`.
///
/// Bevy forwards gilrs events almost verbatim, so this is the ground truth for
/// "what does this physical control send?". The messages are drained every frame
/// whether or not logging is on, so the queue can never grow, and axis logging is
/// throttled so a twitchy stick cannot flood the terminal.
fn report_gamepad_events(
    debug: Res<GamepadDebug>,
    mut buttons: MessageReader<RawGamepadButtonChangedEvent>,
    mut axes: MessageReader<RawGamepadAxisChangedEvent>,
    mut last: Local<[f32; 7]>,
) {
    let verbose = debug.0;
    for ev in buttons.read() {
        if verbose {
            info!("pad button {:?} = {:+.2}", ev.button, ev.value);
        }
    }
    for ev in axes.read() {
        let slot = axis_slot(ev.axis);
        if (ev.value - last[slot]).abs() > 0.02 {
            last[slot] = ev.value;
            if verbose {
                info!("pad axis {:?} = {:+.2}", ev.axis, ev.value);
            }
        }
    }
}

/// The self-driving script used by `--screenshot`, as a function of the tick.
///
/// Two interesting moments fall out of it: around tick 250 the car is mid-drive
/// with the ball in front (the shot in the README), and by tick 1200 it has
/// driven into one wall and reversed back into the other - a wall play the camera
/// now films from outside the see-through cage.
fn capture_input(tick: u64) -> SimInput {
    if tick < 420 {
        SimInput::new(1.0, 0.0, false, false, false)
    } else if tick < 1000 {
        SimInput::new(-1.0, 0.0, false, false, false)
    } else {
        SimInput::new(0.0, 0.0, false, false, false)
    }
}

/// `Y` on the pad (or `C`) toggles between ball cam and straight-ahead car cam.
/// `West` is the *physical Y* button on a driver-default pad - see `read_input`.
fn toggle_camera_mode(
    keys: Res<ButtonInput<KeyCode>>,
    gamepads: Query<&Gamepad>,
    mut ball_cam: ResMut<BallCam>,
) {
    let pad_toggle = gamepads
        .iter()
        .any(|pad| pad.just_pressed(GamepadButton::West));
    if pad_toggle || keys.just_pressed(KeyCode::KeyC) {
        ball_cam.0 = !ball_cam.0;
    }
}

fn reset_on_key(keys: Res<ButtonInput<KeyCode>>, gamepads: Query<&Gamepad>, mut sim: ResMut<Sim>) {
    let pad_reset = gamepads
        .iter()
        .any(|pad| pad.just_pressed(GamepadButton::Start));
    if keys.just_pressed(KeyCode::KeyR) || pad_reset {
        sim.0.reset();
    }
}

// ---------------------------------------------------------------------------
// Presentation
// ---------------------------------------------------------------------------

fn sync_visuals(sim: Res<Sim>, mut query: Query<(&mut Transform, &Vis, &mut Visibility)>) {
    for (mut transform, vis, mut visibility) in &mut query {
        match vis {
            Vis::Car => {
                let car = sim.0.car();
                let (f, u) = (car.forward, car.up);
                transform.translation = car.pos;
                // Full 3D attitude, so flips and air control are visible. The
                // basis columns are (right, up, forward).
                transform.rotation = Quat::from_mat3(&Mat3::from_cols(u.cross(f), u, f));
            }
            Vis::Ball => {
                let ball = sim.0.ball();
                transform.translation = ball.pos;
                // Rolling looks wrong without spin, and the angle is free.
                transform.rotate(Quat::from_scaled_axis(ball.spin * (1.0 / 60.0)));
            }
            Vis::BoostPad(index) => {
                // Hidden while on cooldown, so only the glow leaves when the pad
                // is taken (the small pads' disc is not tagged, so it stays).
                *visibility = if sim.0.pad(*index).available {
                    Visibility::Inherited
                } else {
                    Visibility::Hidden
                };
            }
        }
    }
}

/// Roll and steer the visual wheels. View-only: the simulation has no wheels, so
/// the roll integrates the distance the car has travelled over the wheel radius,
/// and the steer angle comes from the last input.
fn spin_wheels(
    time: Res<Time>,
    sim: Res<Sim>,
    steer: Res<LastSteer>,
    mut rolled: Local<f32>,
    mut wheels: Query<(&mut Transform, &Wheel)>,
) {
    let speed = sim.0.car().forward_speed;
    *rolled = (*rolled + speed * time.delta_secs() / WHEEL_RADIUS) % std::f32::consts::TAU;
    let steer_angle = steer.0 as f32 * MAX_WHEEL_STEER;
    for (mut transform, wheel) in &mut wheels {
        let yaw = if wheel.steers { steer_angle } else { 0.0 };
        transform.rotation = Quat::from_rotation_y(yaw) * Quat::from_rotation_x(*rolled);
    }
}

/// Look input as two scalars in `-1..=1`, merged from the pad's right stick and
/// the keyboard (`I`/`K` pitch, `J`/`L` swivel). `x` is positive to the right;
/// `y` is the pitch input, positive to look up. The vertical axis is inverted
/// (`CAMERA_LOOK_INVERT_Y`), so pushing the stick up looks down.
fn look_input(keys: &ButtonInput<KeyCode>, gamepad: Option<&Gamepad>) -> (f64, f64) {
    let key_x = axis(keys, &[KeyCode::KeyL], &[KeyCode::KeyJ]);
    let key_y = axis(keys, &[KeyCode::KeyI], &[KeyCode::KeyK]);
    let (pad_x, pad_y) = match gamepad {
        Some(pad) => (
            pad.get(GamepadAxis::RightStickX).unwrap_or(0.0) as f64,
            pad.get(GamepadAxis::RightStickY).unwrap_or(0.0) as f64,
        ),
        None => (0.0, 0.0),
    };
    let y = strongest(key_y, pad_y);
    let y = if CAMERA_LOOK_INVERT_Y { -y } else { y };
    (strongest(key_x, pad_x), y)
}

/// Whether the look-behind control is held. In Rocket League that is pushing the
/// right stick in - the "click", R3.
fn look_behind(gamepad: Option<&Gamepad>) -> bool {
    gamepad.is_some_and(|pad| pad.pressed(GamepadButton::RightThumb))
}

/// How far the camera is yawed away from straight-behind, in radians. Left and
/// right swivel; `behind` turns the camera right around to the front of the car,
/// which is Rocket League's look-behind.
fn look_yaw_offset(look_x: f64, behind: bool) -> f32 {
    if behind {
        PI
    } else {
        -(look_x as f32) * CAMERA_LOOK_YAW
    }
}

/// The camera's resting orbit angle: the angle above the horizontal at which
/// the preferred chase position sits.
fn resting_pitch() -> f32 {
    CAMERA_HEIGHT.atan2(CAMERA_DISTANCE)
}

/// The orbit angle for an (already inverted) vertical look input, clamped to the
/// safe range. Push up raises the camera towards a top-down view; pull down
/// lowers it towards a flat one. `base` is the resting angle for the mode
/// (`resting_pitch` for car cam, `CAMERA_BALL_PITCH` for ball cam).
fn pitch_for_look(base: f32, look_y: f32) -> f32 {
    (base - look_y * CAMERA_LOOK_PITCH).clamp(CAMERA_PITCH_MIN, CAMERA_PITCH_MAX)
}

/// The camera's offsets from the car: where it sits (`boom`) and the point it
/// looks at (`aim`), from a yaw offset (`look_yaw_offset`) and an orbit angle
/// (`pitch`, in radians above the car's own up axis).
///
/// The boom is built in the car's own frame - rotated about the car's `up` axis
/// and lifted along it - so the camera follows the car onto a wall and the
/// ceiling instead of collapsing to "straight below" once the forward axis goes
/// vertical. On the ground `up` is world up, which is exactly the old behaviour:
/// at zero yaw and the resting pitch, `boom` is `-forward * CAMERA_DISTANCE +
/// up * CAMERA_HEIGHT` and `aim` is `up * CAMERA_AIM_HEIGHT`.
fn camera_frame(forward: Vec3, up: Vec3, yaw: f32, pitch: f32) -> (Vec3, Vec3) {
    let back = Quat::from_axis_angle(up, yaw) * -forward;
    let radius = (CAMERA_DISTANCE * CAMERA_DISTANCE + CAMERA_HEIGHT * CAMERA_HEIGHT).sqrt();
    let boom = back * (radius * pitch.cos()) + up * (radius * pitch.sin());
    let aim = up * CAMERA_AIM_HEIGHT;
    (boom, aim)
}

/// Ball cam's resting boom: the yaw (about the car's `up`) and pitch that point
/// the boom straight *away from the ball*, so the camera sits on the far side of
/// the car from the ball and looks through the car at it.
///
/// "Away from the ball" is measured in full 3D, not only in the car's tangent
/// plane. On the floor it puts the camera behind the car; on a wall it lifts the
/// boom clean out of the arena, so the camera sits *outside* the wall and looks
/// back in through the see-through cage with the car in front of the ball. The
/// tangent-plane bearing the car uses elsewhere would keep the boom inside the
/// wall plane and never get there. `CAMERA_BALL_PITCH` is folded in so the boom
/// still lifts off a flat floor, where "away from the ball" is level.
fn ball_cam_angle(car_up: Vec3, forward: Vec3, to_ball: Vec3) -> (f32, f32) {
    let away = (-to_ball).normalize_or_zero();
    let along_up = away.dot(car_up).clamp(-1.0, 1.0);
    let tangent = away - car_up * along_up;
    let yaw = if tangent.length_squared() > 1e-6 {
        let n = tangent.normalize();
        let back = -forward;
        car_up.dot(back.cross(n)).atan2(back.dot(n))
    } else {
        // Directly off the surface: no bearing to speak of, so trail the car.
        0.0
    };
    (yaw, along_up.asin() + CAMERA_BALL_PITCH)
}

#[allow(clippy::too_many_arguments)]
fn chase_camera(
    keys: Res<ButtonInput<KeyCode>>,
    gamepads: Query<&Gamepad>,
    sim: Res<Sim>,
    ball_cam: Res<BallCam>,
    time: Res<Time>,
    mut query: Query<&mut Transform, With<ChaseCam>>,
    mut look_yaw: Local<f32>,
    mut behind_last: Local<bool>,
) {
    let car = sim.0.car();

    // Follow the car's *full* forward axis, not just its horizontal part, so the
    // camera trails it up a wall and over onto the ceiling. On the flat ground the
    // two are the same, because a grounded car's forward is horizontal.
    let forward = car.forward.normalize_or_zero();
    // The car's up axis is the surface normal on the ground, a wall's normal on a
    // wall, and straight down on the ceiling. That is the frame the camera boom
    // is built in (see `camera_frame`).
    let car_up = if car.up.length_squared() > 0.5 {
        car.up.normalize()
    } else {
        Vec3::Y
    };
    let car_pos = car.pos;
    let ball_pos = sim.0.ball().pos;

    let pad = gamepads.iter().next();
    let behind = look_behind(pad);
    let (look_x, look_y) = look_input(&keys, pad);

    // Look-behind is a hard 180° flip in Rocket League: it jumps around the
    // instant the stick is clicked, and jumps back on release. `behind_last`
    // catches both edges so neither transition swivels. The ordinary swivel, by
    // contrast, is smoothed so the camera eases around after the stick.
    let was_behind = *behind_last;
    *behind_last = behind;
    let snap = behind != was_behind;

    // The boom's resting direction. In ball cam it points straight away from the
    // ball (so the camera looks *through* the car at it, and on a wall sits outside
    // the wall doing so); car cam keeps the boom behind the car, trailing its
    // heading. See `ball_cam_angle` / `camera_frame`.
    let to_ball = ball_pos - car_pos;
    let (resting_yaw, resting_pitch) = if ball_cam.0 {
        ball_cam_angle(car_up, forward, to_ball)
    } else {
        (0.0, resting_pitch())
    };

    // Smooth the yaw *angle*, not the camera position: easing the angle swings
    // the camera *around* the car rather than cutting through it. Ease by the
    // shortest way round, so the ball passing behind the car does not spin the
    // camera the long way.
    let smooth = if snap {
        1.0
    } else {
        1.0 - (-CAMERA_LOOK_RESPONSE * time.delta_secs()).exp()
    };
    let target_yaw = look_yaw_offset(look_x, behind) + resting_yaw;
    let mut delta = target_yaw - *look_yaw;
    delta = (delta + PI).rem_euclid(std::f32::consts::TAU) - PI;
    *look_yaw += delta * smooth;
    let pitch = if ball_cam.0 {
        // Ball cam may swing below the tangent plane (out of the arena); see
        // `CAMERA_BALL_PITCH_MIN`.
        (resting_pitch - look_y as f32 * CAMERA_LOOK_PITCH)
            .clamp(CAMERA_BALL_PITCH_MIN, CAMERA_PITCH_MAX)
    } else {
        pitch_for_look(resting_pitch, look_y as f32)
    };
    let (boom, aim_offset) = camera_frame(forward, car_up, *look_yaw, pitch);

    // The arena is a see-through cage - its shell is drawn one-sided (see
    // `arena_surface_mesh`) - so the camera is free to sit outside it and look
    // back in through the wall or the roof. That is what keeps the car in view
    // when it drives up a wall or across the ceiling, instead of the boom stopping
    // dead against the shell as it used to. The only guard is the floor, which the
    // camera never drops below.
    let mut desired = car_pos + boom;
    desired.y = desired.y.max(CAMERA_FLOOR_MARGIN);

    // Where the camera looks. Ball cam frames the ball; car cam (or while the
    // look controls are in use) squares up on the car, so a swivel keeps the car
    // centred instead of dragging the aim off toward the ball.
    //
    // This is an absolute aim point, not a weighted average of the car and ball
    // positions: blending the raw positions lets the aim drift behind the camera
    // whenever the ball is, and the camera then stares at the floor with the car
    // off screen.
    let look_amount = if behind {
        1.0
    } else {
        (look_x.abs().max(look_y.abs()) as f32).clamp(0.0, 1.0)
    };
    // Ball cam frames the ball (the car sits between the camera and the ball);
    // car cam, look-behind, or a swivel square up on the car.
    let frame_ball = ball_cam.0 && !behind && look_amount < 0.5;
    let car_aim = car_pos + aim_offset;
    let aim = if frame_ball {
        // Aim at the ball so it stays framed; the low ball-cam boom keeps the
        // car in the bottom of the view.
        ball_pos
    } else {
        car_aim
    };

    // Ease the camera into place to damp jitter, except on a look-behind flip,
    // where the whole point is to jump straight to the other side of the car.
    let weight = if snap {
        1.0
    } else {
        1.0 - (-8.0 * time.delta_secs()).exp()
    };
    for mut transform in &mut query {
        transform.translation = transform.translation.lerp(desired, weight);
        // Directly overhead - or, on a wall, directly below - world up is
        // parallel to the view direction and `look_at` has no valid basis. Fall
        // back to the car's own up axis, which stays horizontal on a wall.
        let to_aim = aim - transform.translation;
        let view = to_aim.normalize_or_zero();
        let up = if view.cross(Vec3::Y).length() > 0.2 {
            Vec3::Y
        } else if view.cross(car_up).length() > 0.2 {
            car_up
        } else {
            Vec3::X
        };
        transform.look_at(aim, up);
    }
}

fn update_hud(
    sim: Res<Sim>,
    ball_cam: Res<BallCam>,
    hud: Single<Entity, With<HudText>>,
    mut writer: TextUiWriter,
) {
    let entity = *hud;
    let car = sim.0.car();
    let ball = sim.0.ball();
    let score = sim.0.score();
    *writer.text(entity, 1) = format!("BLUE {:>2} - {:<2} ORANGE\n", score[0], score[1]);
    *writer.text(entity, 2) = format!(
        "speed {:>5.1} m/s   boost {:>5.1}   tick {:>6}   cam {}\n",
        car.speed,
        car.boost,
        sim.0.tick(),
        if ball_cam.0 { "ball" } else { "car" }
    );
    *writer.text(entity, 3) = format!(
        "ball {:>5.1} m/s   height {:>5.2} m   spin {:>5.1} rad/s\n",
        ball.speed,
        ball.pos.y,
        ball.spin.length()
    );
}

fn quit_on_escape(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

#[allow(clippy::too_many_arguments)]
fn screenshot_driver(
    mut commands: Commands,
    mode: Res<Mode>,
    sim: Res<Sim>,
    time: Res<Time>,
    mut elapsed: Local<u32>,
    mut frames_since_capture: Local<u32>,
    mut captured: Local<bool>,
    mut exit: MessageWriter<AppExit>,
) {
    let Mode::Capture {
        path,
        capture_at_tick,
        free_cam,
    } = &*mode
    else {
        return;
    };

    *elapsed += 1;

    // A driven shot waits for the simulation to reach the moment; a free-cam
    // shot has no tick to reach, so it fires once the scene has had its warm-up
    // - and the simulation never runs at all.
    let ready = match free_cam {
        Some(_) => *elapsed >= FREE_SHOT_MIN_FRAMES && time.elapsed_secs() >= FREE_SHOT_MIN_SECS,
        None => sim.0.tick() >= *capture_at_tick,
    };

    if !*captured && ready {
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path.clone()));
        info!("capturing screenshot to {path} (tick {})", sim.0.tick());
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
    fn the_boom_is_not_clipped_against_the_arena() {
        // The arena is a see-through cage, so a car backed against a wall gets
        // its full boom even though that puts the camera outside the arena: the
        // shell is drawn one-sided and the car stays visible through it.
        let car = Vec3::new(0.0, 0.18, -(ARENA_HALF_Z - 0.6));
        let (boom, _) = camera_frame(Vec3::new(0.0, 0.0, 1.0), Vec3::Y, 0.0, resting_pitch());
        let desired = car + boom;
        assert!(
            desired.z < -ARENA_HALF_Z,
            "the camera should sit outside the wall, got {desired:?}"
        );
    }

    #[test]
    fn camera_frame_without_look_is_the_plain_chase_offset() {
        // The look control must not change the resting view: at zero yaw and the
        // resting pitch the boom is exactly the old
        // `-forward * CAMERA_DISTANCE + Y * CAMERA_HEIGHT`, and the aim sits
        // CAMERA_AIM_HEIGHT above the car.
        let forward = Vec3::new(0.0, 0.0, 1.0);
        let (boom, aim) = camera_frame(forward, Vec3::Y, 0.0, resting_pitch());
        let expected = -forward * CAMERA_DISTANCE + Vec3::Y * CAMERA_HEIGHT;
        assert!((boom - expected).length() < 1e-5, "boom {boom:?}");
        assert!(
            (aim - Vec3::Y * CAMERA_AIM_HEIGHT).length() < 1e-5,
            "aim {aim:?}"
        );
    }

    #[test]
    fn look_yaw_swings_the_camera_around_the_car() {
        let forward = Vec3::new(0.0, 0.0, 1.0);
        let radius = (CAMERA_DISTANCE * CAMERA_DISTANCE + CAMERA_HEIGHT * CAMERA_HEIGHT).sqrt();
        let (resting, _) = camera_frame(forward, Vec3::Y, 0.0, resting_pitch());

        for look_x in [-1.0, -0.5, 0.5, 1.0] {
            let (swung, _) = camera_frame(
                forward,
                Vec3::Y,
                look_yaw_offset(look_x, false),
                resting_pitch(),
            );
            assert!(
                (swung.length() - radius).abs() < 1e-4,
                "look_x {look_x}: distance {} vs {radius}",
                swung.length()
            );
            assert!(
                (swung.x - resting.x).abs() > 0.1,
                "look_x {look_x} did not swing: {swung:?}"
            );
        }
    }

    #[test]
    fn look_behind_swings_the_camera_to_the_front_of_the_car() {
        let forward = Vec3::new(0.0, 0.0, 1.0);
        let (resting, _) = camera_frame(
            forward,
            Vec3::Y,
            look_yaw_offset(0.0, false),
            resting_pitch(),
        );
        assert!(
            resting.z < 0.0,
            "the resting boom sits behind the car: {resting:?}"
        );

        let (behind, _) = camera_frame(
            forward,
            Vec3::Y,
            look_yaw_offset(0.0, true),
            resting_pitch(),
        );
        assert!(behind.z > 0.0, "look-behind must cross the car: {behind:?}");
    }

    #[test]
    fn pitch_orbits_the_camera_up_and_over_the_car() {
        // Push up (look_y is inverted, so negative) raises the camera towards a
        // top-down view; pull down drops it flatter. Either way the aim stays on
        // the car, so it can never stare past it into the ground.
        let forward = Vec3::new(0.0, 0.0, 1.0);
        let height = |look_y: f32| {
            camera_frame(
                forward,
                Vec3::Y,
                0.0,
                pitch_for_look(resting_pitch(), look_y),
            )
            .0
            .y
        };

        assert!(
            height(-1.0) > height(0.0),
            "pushing up should raise the camera: {} vs {}",
            height(-1.0),
            height(0.0)
        );
        assert!(
            height(1.0) < height(0.0),
            "pulling down should lower the camera: {} vs {}",
            height(1.0),
            height(0.0)
        );

        // The orbit is bounded: a hard push can never take the camera below the
        // car, which is where "looking into the ground" began.
        assert!(
            height(-5.0) > 0.0,
            "clamped push-up must stay above the car"
        );
        assert!((pitch_for_look(resting_pitch(), -5.0) - CAMERA_PITCH_MAX).abs() < 1e-6);
        assert!((pitch_for_look(resting_pitch(), 5.0) - CAMERA_PITCH_MIN).abs() < 1e-6);

        // Throughout the range the aim sits just above the car, never down at the
        // ground.
        for look_y in [-1.0, -0.5, 0.0, 0.5, 1.0] {
            let (_, aim) = camera_frame(
                forward,
                Vec3::Y,
                0.0,
                pitch_for_look(resting_pitch(), look_y),
            );
            assert!(
                (aim - Vec3::Y * CAMERA_AIM_HEIGHT).length() < 1e-6,
                "aim drifted off the car at look_y {look_y}: {aim:?}"
            );
        }
    }

    #[test]
    fn the_boom_is_built_in_the_cars_own_frame() {
        // On a wall the car's forward is vertical and its up is horizontal. The
        // boom must lie in the wall plane - lifted off the wall along the car's
        // up - rather than collapsing to "straight down", which is what let ball
        // cam track the ball on a wall.
        let forward = Vec3::new(0.0, 1.0, 0.0); // up the +X wall
        let up = Vec3::new(-1.0, 0.0, 0.0); // wall normal, into the arena
        let (boom, aim) = camera_frame(forward, up, 0.0, resting_pitch());
        assert!(boom.y < -1.0, "boom should sit down the wall: {boom:?}");
        assert!(boom.x < -1.0, "boom should lift off the wall: {boom:?}");
        assert!(boom.z.abs() < 1e-4, "boom left the wall plane: {boom:?}");
        assert!(
            (aim - up * CAMERA_AIM_HEIGHT).length() < 1e-5,
            "aim {aim:?}"
        );
    }

    #[test]
    fn ball_cam_swings_the_boom_outside_the_wall() {
        // Car high on the +X wall, ball in-field. Ball cam puts the camera on the
        // far side of the car from the ball - *outside* the wall - so the car is in
        // front of the ball looking back in through the cage.
        let car_up = Vec3::new(-1.0, 0.0, 0.0);
        let forward = Vec3::new(0.0, 1.0, 0.0);
        let half_x = rl::Arena::default().half_x as f32;
        let car = Vec3::new(half_x - 0.2, 8.0, 0.0);
        let ball = Vec3::new(20.0, 2.0, 3.0);
        let (yaw, pitch) = ball_cam_angle(car_up, forward, ball - car);
        let (boom, _) = camera_frame(forward, car_up, yaw, pitch);
        let camera = car + boom;
        assert!(
            camera.x > half_x,
            "the camera should sit outside the +X wall, got {camera:?}"
        );
        assert!(
            boom.dot(ball - car) < 0.0,
            "the boom should point away from the ball: {boom:?}"
        );
    }

    #[test]
    fn ball_cam_on_the_floor_still_trails_the_car() {
        // Ball ahead on the ground: the boom stays behind and above the car, i.e.
        // the plain resting chase offset.
        let forward = Vec3::new(0.0, 0.0, 1.0);
        let car = Vec3::new(0.0, 0.2, 0.0);
        let ball = Vec3::new(0.0, 0.2, 10.0);
        let (yaw, pitch) = ball_cam_angle(Vec3::Y, forward, ball - car);
        let (boom, _) = camera_frame(forward, Vec3::Y, yaw, pitch);
        assert!(boom.z < -1.0, "boom should be behind the car: {boom:?}");
        assert!(boom.y > 1.0, "boom should be above the car: {boom:?}");
        assert!(yaw.abs() < 1e-4, "yaw {yaw}");
    }

    #[test]
    fn the_vertical_axis_is_inverted() {
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyL);
        let (x, _) = look_input(&keys, None);
        assert!(x > 0.5, "L should swivel right, got {x}");

        // Pushing up looks down, so the aim lift comes out negative.
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyI);
        let (_, y) = look_input(&keys, None);
        assert!(y < -0.5, "I should look down (inverted), got {y}");
    }

    #[test]
    fn the_drive_stick_doubles_as_air_control() {
        // Stick up (W) is throttle on the ground and a frontflip (pitch -1) in
        // the air; stick left (A) is steer left and yaw left. Roll is Q/E.
        let press = |code| {
            let mut keys = ButtonInput::<KeyCode>::default();
            keys.press(code);
            read_input(&keys, None)
        };

        let up = press(KeyCode::KeyW);
        assert!(up.throttle > 0.5 && up.pitch < -0.5, "W: {up:?}");

        let down = press(KeyCode::KeyS);
        assert!(down.throttle < -0.5 && down.pitch > 0.5, "S: {down:?}");

        let left = press(KeyCode::KeyA);
        assert!(left.steer > 0.5 && left.yaw < -0.5, "A: {left:?}");

        let roll_right = press(KeyCode::KeyE);
        assert!(roll_right.roll > 0.5, "E: {roll_right:?}");
    }

    #[test]
    fn parse_vec3_reads_a_triple() {
        assert_eq!(parse_vec3("1,2,3"), Some(Vec3::new(1.0, 2.0, 3.0)));
        assert_eq!(
            parse_vec3(" 12, 6, -30 "),
            Some(Vec3::new(12.0, 6.0, -30.0))
        );
        // Wrong arity or a non-number is rejected, not silently defaulted.
        assert_eq!(parse_vec3("1,2"), None);
        assert_eq!(parse_vec3("1,2,3,4"), None);
        assert_eq!(parse_vec3("a,2,3"), None);
    }

    #[test]
    fn a_lone_cam_or_look_fills_the_other_half() {
        assert!(resolve_free_cam(None, None).is_none());

        let from_cam = resolve_free_cam(Some(Vec3::new(1.0, 2.0, 3.0)), None).unwrap();
        assert_eq!(from_cam.pos, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(from_cam.look, FREE_CAM_DEFAULT_LOOK);

        let from_look = resolve_free_cam(None, Some(Vec3::new(4.0, 5.0, 6.0))).unwrap();
        assert_eq!(from_look.pos, FREE_CAM_DEFAULT_POS);
        assert_eq!(from_look.look, Vec3::new(4.0, 5.0, 6.0));
    }

    #[test]
    fn only_a_free_cam_capture_skips_the_simulation() {
        assert!(!is_free_shot(&Mode::Play));
        assert!(!is_free_shot(&Mode::Capture {
            path: "x.png".into(),
            capture_at_tick: 250,
            free_cam: None,
        }));
        assert!(is_free_shot(&Mode::Capture {
            path: "x.png".into(),
            capture_at_tick: 0,
            free_cam: Some(FreeCam {
                pos: FREE_CAM_DEFAULT_POS,
                look: Vec3::ONE,
            }),
        }));
    }
}
