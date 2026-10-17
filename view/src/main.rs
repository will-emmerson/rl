//! Bevy 3D viewer for the RocketSim simulation in `sim.rs`.
//!
//! This crate owns the render scene, feeds input into the simulation, and copies
//! the resulting state onto transforms; nothing in the draw path feeds back into
//! the physics.
//!
//! Run it:
//!   cargo run -p rl_view                          # play (dev profile is playable)
//!   cargo run -p rl_view -- --screenshot shot.png # render one frame, then exit
//!   cargo run -p rl_view -- --screenshot shot.png --cam 0,12,-30 --look 0,1,0
//!                                                 # instant free-cam shot, no sim
//!   cargo run -p rl_view -- --gamepad-debug       # log every raw pad event
//!   cargo run -p rl_view -- --camera-config cam.toml # use a different camera file

use std::f32::consts::PI;
use std::path::{Path, PathBuf};
use std::time::SystemTime;

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
use bevy::window::PrimaryWindow;
use serde::Deserialize;

mod sim;
use sim::{
    Input as SimInput, BALL_RADIUS, CAR_HALF_HEIGHT, CAR_HALF_LENGTH, CAR_HALF_WIDTH,
    CAR_WHEELBASE, GOAL_HALF_WIDTH, GOAL_HEIGHT,
};

/// Arena dimensions, matching RocketSim's soccar arena: the goal line, frame and
/// net sit at `ARENA_HALF_Z`.
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
const WHEEL_RADIUS: f32 = 0.52 * CAR_HALF_HEIGHT;
/// Full-lock angle for the visual front wheels, in radians. RL's real front-wheel
/// angle is speed-dependent; this is a fixed stand-in.
const MAX_WHEEL_STEER: f32 = 0.5;

/// The camera settings file, relative to where the viewer is run. Missing is
/// fine (RL's defaults are used); a malformed file is a hard error. Override the
/// path with `--camera-config PATH`. The file is hot-reloaded on save.
const CAMERA_CONFIG_PATH: &str = "camera.toml";
/// How often the settings file is checked for changes. Five seconds is enough
/// for hand-tuning and avoids reloading on every keystroke while it is being
/// edited; a file caught mid-edit normally fails to parse and the previous
/// settings stay in place.
const CAMERA_RELOAD_CHECK_SECS: f32 = 5.0;
/// The camera may leave the arena through the walls *and* the roof - the shell is
/// one-sided, so it can still see back in - but it never drops below the pitch,
/// which would film the game from underneath. This is the margin it keeps off the
/// floor.
const CAMERA_FLOOR_MARGIN: f32 = 0.4;
/// Vertical look at full deflection, in radians. It is added to the configured
/// camera angle; positive looks up, negative looks down.
const CAMERA_LOOK_PITCH: f32 = 0.9;
/// Absolute cap on the view pitch, keeping it short of straight up/down so
/// `looking_to` always has a usable up vector.
const CAMERA_PITCH_LIMIT: f32 = 1.45;
/// Swivel rate per unit of RL's `swivel_speed`, in radians per second. RL exposes
/// 1..=10; at the default 5 the camera turns at ~172°/s with the stick held.
const SWIVEL_RAD_PER_SPEED: f32 = 0.6;
/// How quickly the swivel springs back to centre when the stick is released,
/// per unit of `swivel_speed`, in inverse seconds.
const SWIVEL_RETURN_PER_SPEED: f32 = 2.0;

// ---------------------------------------------------------------------------
// Resources / components
// ---------------------------------------------------------------------------

#[derive(Resource)]
struct Sim(sim::Sim);

/// Rocket League's camera settings, in the viewer's units (metres, degrees).
///
/// The names and defaults mirror the in-game camera panel so a config file reads
/// like the settings screen: distance/height are the boom, `angle` is the view
/// pitch (not the boom), and `stiffness` controls how tightly the camera follows
/// the car. RL stores distance/height in Unreal units - multiply by 100 for the
/// `uu` values shown in game (270 uu = 2.7 m).
///
/// `fov` is the **horizontal** field of view, as in RL. Bevy's projection takes a
/// vertical FOV, so [`vertical_fov`] converts it using the window aspect.
#[derive(Resource, Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct CameraSettings {
    /// Horizontal field of view, in degrees (RL 80..=110).
    fov: f32,
    /// How far behind the car the camera sits, in metres (RL 1.0..=4.0).
    distance: f32,
    /// How far above the car the camera sits, in metres (RL 0.4..=2.0).
    height: f32,
    /// View pitch in degrees, independent of the boom (RL -15..=5). Negative
    /// looks down; the camera position does not move when this changes.
    angle: f32,
    /// How tightly the camera follows the car, 0 (loose) ..= 1 (locked).
    stiffness: f32,
    /// Right-stick swivel rate (RL 1..=10).
    swivel_speed: f32,
    /// How quickly the camera swings between car cam and ball cam (RL 1..=2).
    transition_speed: f32,
    /// Invert the horizontal swivel axis (RL's "Invert Swivel").
    invert_swivel: bool,
    /// Invert the vertical look axis. Not an RL camera-panel setting; kept so
    /// the viewer's pad convention can stay as it was.
    invert_look_y: bool,
}

impl Default for CameraSettings {
    /// Rocket League's default camera: 110° FOV, 270 uu distance, 100 uu height,
    /// -4° angle, 0.5 stiffness, 5.0 swivel speed, 1.0 transition speed.
    fn default() -> Self {
        Self {
            fov: 110.0,
            distance: 2.7,
            height: 1.0,
            angle: -4.0,
            stiffness: 0.5,
            swivel_speed: 5.0,
            transition_speed: 1.0,
            invert_swivel: false,
            invert_look_y: true,
        }
    }
}

impl CameraSettings {
    /// Clamp each numeric field to the range RL allows, warning when a hand-edited
    /// file falls outside it. Clamping (rather than rejecting) keeps a typo from
    /// taking the whole viewer down while still making the mistake visible.
    fn clamped(mut self) -> Self {
        let default = Self::default();
        clamp_setting(&mut self.fov, 80.0, 110.0, default.fov, "fov");
        clamp_setting(&mut self.distance, 1.0, 4.0, default.distance, "distance");
        clamp_setting(&mut self.height, 0.4, 2.0, default.height, "height");
        clamp_setting(&mut self.angle, -15.0, 5.0, default.angle, "angle");
        clamp_setting(
            &mut self.stiffness,
            0.0,
            1.0,
            default.stiffness,
            "stiffness",
        );
        clamp_setting(
            &mut self.swivel_speed,
            1.0,
            10.0,
            default.swivel_speed,
            "swivel_speed",
        );
        clamp_setting(
            &mut self.transition_speed,
            1.0,
            2.0,
            default.transition_speed,
            "transition_speed",
        );
        self
    }
}

/// Where the settings came from and when it last changed, so the file can be
/// hot-reloaded while the viewer runs.
#[derive(Resource)]
struct CameraConfig {
    path: PathBuf,
    modified: Option<SystemTime>,
}

fn clamp_setting(value: &mut f32, lo: f32, hi: f32, default: f32, name: &str) {
    if !value.is_finite() {
        warn!("camera: {name} must be finite; using {default}");
        *value = default;
        return;
    }
    let clamped = value.clamp(lo, hi);
    if clamped != *value {
        warn!(
            "camera: {name}={} is outside {lo}..={hi}; using {clamped}",
            *value
        );
        *value = clamped;
    }
}

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
    let mut camera_config: Option<PathBuf> = None;

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
            "--camera-config" => {
                camera_config = Some(PathBuf::from(
                    args.next()
                        .unwrap_or_else(|| panic!("--camera-config needs a path")),
                ));
            }
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

    let (camera_settings, camera_config) = initial_camera_settings(camera_config);

    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(Sim(sim::Sim::new()))
        .insert_resource(mode)
        .insert_resource(camera_settings)
        .insert_resource(camera_config)
        .insert_resource(GamepadDebug(gamepad_debug))
        .insert_resource(BallCam(true))
        .init_resource::<LastSteer>()
        .insert_resource(Time::<Fixed>::from_hz(sim::TICK_HZ))
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
                (reload_camera_settings, apply_camera_fov).chain(),
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

/// Load `--camera-config PATH`, or `camera.toml` if it exists. An explicit path
/// must load; a missing default file silently falls back to RL's defaults.
fn initial_camera_settings(explicit: Option<PathBuf>) -> (CameraSettings, CameraConfig) {
    let required = explicit.is_some();
    let path = explicit.unwrap_or_else(|| PathBuf::from(CAMERA_CONFIG_PATH));
    let modified = file_modified(&path);
    let settings = match read_camera_settings(&path) {
        Ok(settings) => settings,
        // No config file is the normal case: use the built-in RL defaults.
        Err(_) if !required && !path.exists() => CameraSettings::default(),
        Err(e) => panic!("camera config: {e}"),
    };
    (settings, CameraConfig { path, modified })
}

fn read_camera_settings(path: &Path) -> Result<CameraSettings, String> {
    let raw = std::fs::read_to_string(path)
        .map_err(|e| format!("could not read {}: {e}", path.display()))?;
    toml::from_str::<CameraSettings>(&raw)
        .map(CameraSettings::clamped)
        .map_err(|e| format!("could not parse {}: {e}", path.display()))
}

fn file_modified(path: &Path) -> Option<SystemTime> {
    std::fs::metadata(path)
        .and_then(|meta| meta.modified())
        .ok()
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
    settings: Res<CameraSettings>,
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
        Mesh3d(meshes.add(soccer_ball_mesh(BALL_RADIUS))),
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
    // added here. Sizes come from RocketSim (`sim`).
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
    let w = GOAL_HALF_WIDTH;
    let h = GOAL_HEIGHT;
    // Blue defends -Z, orange +Z (matching the sim).
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
    let (hw, hh, hl) = (CAR_HALF_WIDTH, CAR_HALF_HEIGHT, CAR_HALF_LENGTH);
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
        let rear_axle = front_axle - CAR_WHEELBASE;
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
    // is skipped (see `simulating`). Start the chase camera at its resting pose
    // rather than easing in from an arbitrary one.
    let camera_pose = match &*mode {
        Mode::Capture {
            free_cam: Some(free),
            ..
        } => Transform::from_translation(free.pos).looking_at(free.look, Vec3::Y),
        _ => {
            let car = sim.0.car();
            let heading = base_heading(false, car.forward, Vec3::ZERO).unwrap_or(Vec3::Z);
            let position = chase_position(car.pos, heading, settings.distance, settings.height);
            let view = view_direction(heading, settings.angle.to_radians());
            Transform::from_translation(position).looking_to(view, Vec3::Y)
        }
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
    // Air roll, on dedicated keys: Q rolls left, E rolls right (positive `roll`
    // is left, the same sign convention as `steer` and `yaw`).
    let keyboard_roll = axis(keys, &[KeyCode::KeyQ], &[KeyCode::KeyE]);
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
    // frontflip (`pitch -1`) and stick left yaws left. On the keyboard the W/S
    // keys double as pitch; the pad uses the left stick's vertical axis, which is
    // why pitch is *not* taken from the throttle.
    //
    // `yaw` keeps `steer`'s sign, so the same key that steers left also yaws and
    // dodges left: RocketSim's positive steer/yaw/roll all go to the car's left
    // (measured by `examples/axis_probe.rs`), and the viewer's `steer` is already
    // positive to the left.
    let keyboard_pitch = -keyboard_throttle;
    let pitch = strongest(keyboard_pitch, pad_pitch);
    let yaw = steer;

    // On the pad, air roll is the handbrake/drift button plus the stick, as in
    // Rocket League: drift + stick left rolls left (so it keeps `pad_steer`'s
    // sign). The keyboard uses Q/E.
    let pad_roll = if pad_drift { pad_steer } else { 0.0 };
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
/// `y` is the pitch input, positive to look up. `invert_y` flips the vertical
/// axis (the viewer's default), so pushing the stick up looks down.
fn look_input(
    keys: &ButtonInput<KeyCode>,
    gamepad: Option<&Gamepad>,
    invert_y: bool,
) -> (f64, f64) {
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
    let y = if invert_y { -y } else { y };
    (strongest(key_x, pad_x), y)
}

/// Whether the look-behind control is held. In Rocket League that is pushing the
/// right stick in - the "click", R3.
fn look_behind(gamepad: Option<&Gamepad>) -> bool {
    gamepad.is_some_and(|pad| pad.pressed(GamepadButton::RightThumb))
}

/// A horizontal direction's yaw, where `0` points along `+Z` and positive angles
/// turn toward `+X` (Bevy's positive rotation about `Y`).
fn yaw_of(dir: Vec3) -> f32 {
    dir.x.atan2(dir.z)
}

fn dir_from_yaw(yaw: f32) -> Vec3 {
    Vec3::new(yaw.sin(), 0.0, yaw.cos())
}

/// Yaw of the camera's view heading after swivel and look-behind. Positive
/// `swivel` looks right: for a heading along `+Z`, right is `-X`.
fn heading_yaw(base_yaw: f32, swivel: f32, behind: bool) -> f32 {
    base_yaw - swivel + if behind { PI } else { 0.0 }
}

/// Shortest signed way round from one angle to another.
fn shortest_angle(delta: f32) -> f32 {
    (delta + PI).rem_euclid(std::f32::consts::TAU) - PI
}

/// Horizontal direction the camera looks along: the car's heading in car cam, or
/// the bearing to the ball in ball cam. `None` means the input is vertical (a car
/// pointing straight up a wall), so the caller keeps its previous heading rather
/// than snapping to an arbitrary one.
fn base_heading(ball_cam: bool, car_forward: Vec3, to_ball: Vec3) -> Option<Vec3> {
    let dir = if ball_cam {
        Vec3::new(to_ball.x, 0.0, to_ball.z)
    } else {
        Vec3::new(car_forward.x, 0.0, car_forward.z)
    };
    (dir.length_squared() > 1e-6).then(|| dir.normalize())
}

/// Camera position for a view heading, RL-style: a fixed distance behind and
/// height above the car, along world up. Building the boom from a horizontal
/// heading rather than the car's own frame is what keeps the camera upright when
/// the car drives up a wall or rolls in the air.
fn chase_position(car_pos: Vec3, heading: Vec3, distance: f32, height: f32) -> Vec3 {
    car_pos - heading * distance + Vec3::Y * height
}

/// Unit view direction from a horizontal heading and a pitch in radians
/// (negative looks down).
fn view_direction(heading: Vec3, pitch: f32) -> Vec3 {
    (heading * pitch.cos() + Vec3::Y * pitch.sin()).normalize()
}

/// Convert RL's horizontal FOV (degrees) to the vertical FOV Bevy's perspective
/// projection takes.
fn vertical_fov(horizontal_degrees: f32, aspect_ratio: f32) -> f32 {
    let half_horizontal = horizontal_degrees.to_radians() * 0.5;
    2.0 * (half_horizontal.tan() / aspect_ratio.max(0.1)).atan()
}

/// Hot-reload `camera.toml` when it changes. Parsing is cheap and this checks
/// every few seconds, so tuning the camera does not need a restart. A failed
/// parse leaves the last good settings in place.
fn reload_camera_settings(
    time: Res<Time>,
    mut config: ResMut<CameraConfig>,
    mut settings: ResMut<CameraSettings>,
    mut since_check: Local<f32>,
) {
    *since_check += time.delta_secs();
    if *since_check < CAMERA_RELOAD_CHECK_SECS {
        return;
    }
    *since_check = 0.0;
    let Some(modified) = file_modified(&config.path) else {
        return;
    };
    if config.modified == Some(modified) {
        return;
    }
    config.modified = Some(modified);
    match read_camera_settings(&config.path) {
        Ok(new_settings) => {
            info!("camera: reloaded {}", config.path.display());
            *settings = new_settings;
        }
        Err(e) => error!("camera: reload failed: {e}"),
    }
}

/// Apply RL's horizontal FOV to the camera as Bevy's vertical FOV.
fn apply_camera_fov(
    settings: Res<CameraSettings>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut cameras: Query<&mut Projection, With<ChaseCam>>,
) {
    let Ok(window) = windows.single() else {
        return;
    };
    let aspect = window.width() / window.height().max(1.0);
    let fov = vertical_fov(settings.fov, aspect);
    for mut projection in &mut cameras {
        if let Projection::Perspective(perspective) = &mut *projection {
            perspective.fov = fov;
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn chase_camera(
    keys: Res<ButtonInput<KeyCode>>,
    gamepads: Query<&Gamepad>,
    sim: Res<Sim>,
    ball_cam: Res<BallCam>,
    settings: Res<CameraSettings>,
    time: Res<Time>,
    mut query: Query<&mut Transform, With<ChaseCam>>,
    mut heading: Local<Option<f32>>,
    mut swivel: Local<f32>,
    mut behind_last: Local<bool>,
) {
    let car = sim.0.car();
    let ball_pos = sim.0.ball().pos;
    let car_pos = car.pos;
    let dt = time.delta_secs();

    let pad = gamepads.iter().next();
    let behind = look_behind(pad);
    // Look-behind is a hard 180° flip in RL: snap the pose on both edges rather
    // than easing the boom through the car.
    let snap = behind != *behind_last;
    *behind_last = behind;
    let (mut look_x, look_y) = look_input(&keys, pad, settings.invert_look_y);
    if settings.invert_swivel {
        look_x = -look_x;
    }

    // The heading is horizontal in both modes: car cam trails the car's heading,
    // ball cam looks along the bearing to the ball. This is the whole difference
    // from the old car-frame boom - the camera never rolls with the car, and on a
    // wall the ball still steers the camera around the field, as in RL.
    let to_ball = ball_pos - car_pos;
    let base = base_heading(ball_cam.0, car.forward, to_ball);
    let target_yaw = base.map(yaw_of);

    // Ease the heading angle, not the camera position: swinging the angle walks
    // the camera *around* the car instead of cutting through it. Stiffness
    // tightens the follow; transition speed shortens the swing when toggling
    // between ball and car cam.
    let stiffness = settings.stiffness;
    let yaw_rate = (4.0 + 16.0 * stiffness) * settings.transition_speed;
    let yaw_weight = if stiffness >= 0.999 {
        1.0
    } else {
        1.0 - (-yaw_rate * dt).exp()
    };
    let current_yaw = match (*heading, target_yaw) {
        (Some(current), Some(target)) => current + shortest_angle(target - current) * yaw_weight,
        (Some(current), None) => current,
        (None, Some(target)) => target,
        (None, None) => 0.0,
    };
    *heading = Some(current_yaw);

    // Swivel is a rate, as in RL: holding the stick turns the camera at
    // `swivel_speed`, and releasing it springs back to centre. Look-behind is a
    // hard 180° flip, so swivel is ignored while it is held.
    let look_x = look_x as f32;
    if !behind && look_x.abs() > 0.05 {
        *swivel += look_x * settings.swivel_speed * SWIVEL_RAD_PER_SPEED * dt;
        *swivel = swivel.clamp(-PI, PI);
    } else {
        let recenter = settings.swivel_speed * SWIVEL_RETURN_PER_SPEED * dt;
        *swivel *= (-recenter).exp();
    }

    let view_heading = dir_from_yaw(heading_yaw(current_yaw, *swivel, behind));

    // RL's camera angle pitches the *view*; it does not move the boom. Keeping
    // the camera at `distance`/`height` and tilting the view is what leaves the
    // car low in the frame instead of centred like a look-at camera.
    let target = chase_position(car_pos, view_heading, settings.distance, settings.height);
    let mut desired = target;
    // The shell is a see-through cage, so the camera may sit outside it and look
    // back in. The only guard is the floor; it never films from underneath.
    desired.y = desired.y.max(CAMERA_FLOOR_MARGIN);

    let follow_rate = 3.0 + 17.0 * stiffness;
    let weight = if snap || stiffness >= 0.999 {
        1.0
    } else {
        1.0 - (-follow_rate * dt).exp()
    };

    let pitch = (settings.angle.to_radians() + look_y as f32 * CAMERA_LOOK_PITCH)
        .clamp(-CAMERA_PITCH_LIMIT, CAMERA_PITCH_LIMIT);
    let view = view_direction(view_heading, pitch);

    for mut transform in &mut query {
        // Stiffness damps the position follow; the heading is damped above.
        transform.translation = transform.translation.lerp(desired, weight);
        transform.translation.y = transform.translation.y.max(CAMERA_FLOOR_MARGIN);
        // World up, always: the camera stays level when the car rolls or drives
        // on a wall, which is how RL's camera behaves.
        transform.rotation = Transform::default().looking_to(view, Vec3::Y).rotation;
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
    fn camera_defaults_match_rocket_league() {
        let s = CameraSettings::default();
        assert_eq!(s.fov, 110.0);
        assert_eq!(s.distance, 2.7); // 270 uu
        assert_eq!(s.height, 1.0); // 100 uu
        assert_eq!(s.angle, -4.0);
        assert_eq!(s.stiffness, 0.5);
        assert_eq!(s.swivel_speed, 5.0);
        assert_eq!(s.transition_speed, 1.0);
        assert!(!s.invert_swivel);
    }

    #[test]
    fn camera_settings_load_from_toml_and_clamp_to_rl_ranges() {
        let parsed: CameraSettings = toml::from_str(
            "fov = 90\n\
             distance = 3.5\n\
             height = 0.75\n\
             angle = -6\n\
             stiffness = 0.7\n\
             swivel_speed = 7\n\
             transition_speed = 1.5\n\
             invert_swivel = true\n\
             invert_look_y = false",
        )
        .unwrap();
        assert_eq!(parsed.fov, 90.0);
        assert_eq!(parsed.distance, 3.5);
        assert_eq!(parsed.height, 0.75);
        assert_eq!(parsed.angle, -6.0);
        assert_eq!(parsed.stiffness, 0.7);
        assert_eq!(parsed.swivel_speed, 7.0);
        assert_eq!(parsed.transition_speed, 1.5);
        assert!(parsed.invert_swivel);
        assert!(!parsed.invert_look_y);

        // Out-of-range values are clamped to the game's limits.
        let clamped: CameraSettings = toml::from_str("distance = 9.0\nfov = 10.0").unwrap();
        let clamped = clamped.clamped();
        assert_eq!(clamped.distance, 4.0);
        assert_eq!(clamped.fov, 80.0);
    }

    #[test]
    fn camera_settings_reject_unknown_keys() {
        assert!(toml::from_str::<CameraSettings>("distnace = 2.0").is_err());
    }

    #[test]
    fn the_camera_boom_is_rl_distance_and_height() {
        let car = Vec3::new(1.0, 0.17, 2.0);
        let heading = base_heading(false, Vec3::Z, Vec3::ZERO).unwrap();
        let position = chase_position(car, heading, 2.7, 1.0);
        let expected = car - Vec3::Z * 2.7 + Vec3::Y * 1.0;
        assert!((position - expected).length() < 1e-5, "{position:?}");
    }

    #[test]
    fn the_camera_is_not_clipped_against_the_arena() {
        // The arena is a see-through cage, so a car backed against a wall keeps
        // its full boom even when that puts the camera outside the shell.
        let car = Vec3::new(0.0, 0.18, -(ARENA_HALF_Z - 0.6));
        let position = chase_position(car, Vec3::Z, 2.7, 1.0);
        assert!(
            position.z < -ARENA_HALF_Z,
            "the camera should sit outside the wall, got {position:?}"
        );
    }

    #[test]
    fn ball_cam_steers_by_the_horizontal_bearing_to_the_ball() {
        let to_ball = Vec3::new(3.0, 7.0, 4.0);
        let heading = base_heading(true, Vec3::Z, to_ball).unwrap();
        assert!(
            heading.y.abs() < 1e-6,
            "heading must stay level: {heading:?}"
        );
        let flat_ball = Vec3::new(to_ball.x, 0.0, to_ball.z).normalize();
        assert!(heading.dot(flat_ball) > 0.999);

        // Behind the car relative to the ball, and still a fixed height up: no
        // 3D orbit and no climbing the car's wall normal.
        let camera = chase_position(Vec3::ZERO, heading, 2.7, 1.0);
        assert!(
            camera.dot(to_ball) < 0.0,
            "camera should be on the far side of the car: {camera:?}"
        );
        assert!((camera.y - 1.0).abs() < 1e-6, "camera height {camera:?}");
    }

    #[test]
    fn the_camera_follows_a_wall_without_rolling() {
        // Nose straight up a wall: there is no horizontal heading to follow, so
        // the caller keeps its previous one rather than snapping.
        assert!(base_heading(false, Vec3::Y, Vec3::ZERO).is_none());

        // Nosed up at an angle: only the horizontal component is used, and the
        // boom stays world-up, so the camera is level while the car is sideways.
        let heading = base_heading(false, Vec3::new(0.0, 1.0, 0.2), Vec3::ZERO).unwrap();
        assert!(heading.y.abs() < 1e-6);
        assert!((heading - Vec3::Z).length() < 1e-5);
        let camera = chase_position(Vec3::new(0.0, 5.0, 0.0), heading, 2.7, 1.0);
        assert!(
            (camera.y - 6.0).abs() < 1e-5,
            "camera should stay world-up: {camera:?}"
        );
    }

    #[test]
    fn look_behind_flips_the_heading_180_degrees() {
        let base_yaw = yaw_of(Vec3::Z);
        let front = dir_from_yaw(heading_yaw(base_yaw, 0.0, false));
        let behind = dir_from_yaw(heading_yaw(base_yaw, 0.0, true));
        assert!(front.dot(behind) < -0.999, "{front:?} vs {behind:?}");
    }

    #[test]
    fn positive_swivel_looks_right() {
        // Looking along +Z, the camera's right is -X in Bevy's frame.
        let heading = dir_from_yaw(heading_yaw(yaw_of(Vec3::Z), 0.5, false));
        assert!(heading.x < 0.0, "{heading:?}");
    }

    #[test]
    fn camera_angle_pitches_the_view_not_the_boom() {
        // RL's default -4° angle looks slightly down, while the boom itself is
        // still distance/height only.
        let view = view_direction(Vec3::Z, -4.0_f32.to_radians());
        assert!(view.y < 0.0, "default angle should look down: {view:?}");

        let camera = chase_position(Vec3::ZERO, Vec3::Z, 2.7, 1.0);
        assert!((camera.y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn horizontal_fov_converts_to_bevy_vertical_fov() {
        // 110° horizontal on 16:9 is ~77.6° vertical.
        let vertical = vertical_fov(110.0, 16.0 / 9.0).to_degrees();
        assert!((vertical - 77.6).abs() < 0.2, "vertical FOV {vertical}");
        assert!(vertical > 45.0, "RL's default must be wider than Bevy's");
    }

    #[test]
    fn the_vertical_axis_is_inverted() {
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyL);
        let (x, _) = look_input(&keys, None, true);
        assert!(x > 0.5, "L should swivel right, got {x}");

        // Pushing up looks down, so the pitch input comes out negative.
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyI);
        let (_, y) = look_input(&keys, None, true);
        assert!(y < -0.5, "I should look down (inverted), got {y}");

        // The same key is not inverted when the setting is off.
        let (_, y) = look_input(&keys, None, false);
        assert!(y > 0.5, "I should look up when not inverted, got {y}");
    }

    #[test]
    fn the_drive_stick_doubles_as_air_control() {
        // Stick up (W) is throttle on the ground and a frontflip (pitch -1) in
        // the air; stick left (A) is steer left *and* yaw left (yaw keeps
        // steer's sign, so the dodge leans the way the stick points). Roll is
        // Q/E.
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
        assert!(left.steer > 0.5 && left.yaw > 0.5, "A: {left:?}");

        let right = press(KeyCode::KeyD);
        assert!(right.steer < -0.5 && right.yaw < -0.5, "D: {right:?}");

        let roll_left = press(KeyCode::KeyQ);
        assert!(roll_left.roll > 0.5, "Q: {roll_left:?}");

        let roll_right = press(KeyCode::KeyE);
        assert!(roll_right.roll < -0.5, "E: {roll_right:?}");
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
