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
//!   cargo run -p rl_view -- --controls-config keys.toml # different bindings file
//!   cargo run -p rl_view -- --fullscreen --stats  # native-res fullscreen + perf overlay
//!   cargo run -p rl_view -- --fullscreen --stats --quality low  # cheaper visuals
//!   cargo run -p rl_view -- --rlbot                # listen for one RLBot v5 bot
//!   cargo run -p rl_view -- --rlbot --rlbot-config Nexto
//!                                                 # ...and start the bot too

use std::collections::VecDeque;
use std::f32::consts::PI;
use std::path::{Path, PathBuf};
use std::time::SystemTime;

use bevy::app::AppExit;
use bevy::asset::RenderAssetUsages;
use bevy::camera::Hdr;
use bevy::image::{ImageAddressMode, ImageSampler, ImageSamplerDescriptor};
use bevy::input::gamepad::{RawGamepadAxisChangedEvent, RawGamepadButtonChangedEvent};
use bevy::light::{DirectionalLightShadowMap, NotShadowCaster};
use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::post_process::bloom::{Bloom, BloomCompositeMode, BloomPrefilter};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, Face, TextureDimension, TextureFormat};
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use bevy::window::{
    MonitorSelection, PresentMode, PrimaryWindow, WindowMode, WindowPlugin, WindowResolution,
};
use serde::Deserialize;

mod sim;
use sim::{
    BallView, CarView, Input as SimInput, BALL_RADIUS, CAR_HALF_HEIGHT, CAR_HALF_LENGTH,
    CAR_HALF_WIDTH, CAR_WHEELBASE, GOAL_DEPTH, GOAL_HALF_WIDTH, GOAL_HEIGHT,
};

mod rlbot;
use rlbot::{LaunchOptions, Rlbot, RlbotOptions};

mod controls;
use controls::{button_axis, key_axis, seed_trigger_axes, strongest, Controls};

/// Soccar arena half-extents, in metres. The long axis is Z in the viewer.
/// RocketSim's `consts::arena::get_aabb(Soccar)` gives 4096 x 5120 x 2048 uu;
/// the goal line, frame and net sit at `ARENA_HALF_Z`.
const ARENA_HALF_X: f32 = 40.96;
const ARENA_HALF_Z: f32 = 51.20;
const ARENA_HEIGHT: f32 = 20.48;

/// Structural ribs standing `GRID_DEPTH` proud of the inside of the shell
/// (visual only). A little thickness is what keeps them visible when the camera
/// is looking along the wall from a car driving on it; `GRID_BEAM` is their
/// width. They are shallow enough not to foul the visual car.
const GRID_DEPTH: f32 = 0.03;
const GRID_BEAM: f32 = 0.14;

/// World size of one repeat of the arena glass texture, in metres. The texture
/// is a hex lattice, and the same texture is used at two scales: a fine mesh on
/// the walls, where a chase camera can be 20 cm away, and large hexagonal panels
/// on the roof, matching RL's ceiling. Four metres puts a wall hex at ~44 cm;
/// the roof uses [`CEILING_TEX_WORLD`].
const WALL_TEX_WORLD: f32 = 4.0;
/// Roof repeat, in metres. Large hexes, like the ones in RL's dome.
const CEILING_TEX_WORLD: f32 = 20.0;

/// The pitch is a single quad laid just above the collision floor. The offset
/// only has to beat z-fighting, not fill a hole: the collision floor is beneath
/// it and is painted the same green.
const PITCH_OFFSET: f32 = 0.01;

/// Mowed-square size, in metres. It divides both pitch dimensions exactly (8
/// squares across, 10 along), so the checkerboard lines up with the arena.
const PITCH_TILE: f32 = 10.24;
/// White pitch markings, in metres. They are geometry: a 16 cm line is a thin
/// quad, so the whole pitch is still only a few hundred triangles and needs no
/// texture at all.
const PITCH_LINE_WIDTH: f32 = 0.16;
/// How far the touchline sits inside the wall.
const PITCH_INSET: f32 = 1.2;
const CENTER_CIRCLE_RADIUS: f32 = 9.0;
/// Goal box: how far it reaches into the pitch, and how far past each post.
const GOAL_BOX_DEPTH: f32 = 8.0;
const GOAL_BOX_MARGIN: f32 = 2.5;
// Corner arcs painted on the pitch, in metres. RL's markings turn the corner
// where the touchline meets the goal line.
const PITCH_CORNER_RADIUS: f32 = 2.0;
/// How far the team wash reaches in from each end wall, in metres. The goal
/// box sits inside this, so the ends read blue/orange across the full width
/// without losing the green pitch in midfield.
const TEAM_WASH_DEPTH: f32 = 24.0;
/// Strength of that wash at the end wall. The goal box is stronger still.
const TEAM_WASH_STRENGTH: f32 = 0.45;

/// The curved wall base: RL's floor-to-wall fillet is a 2.5 m quarter circle,
/// and the coloured boards follow it rather than standing on the flat wall. A
/// board placed at the wall plane would float over the ramp and foul the car.
const BOARD_FILLET_RADIUS: f32 = 2.5;
/// The pitch is a rounded rectangle in plan too: the straight walls stop 12.8 m
/// short of the corner, which is where the corner arc begins.
const BOARD_CORNER_RADIUS: f32 = 12.8;
/// The team-coloured band on the base boards, in metres above the floor. It sits
/// on the upper part of the fillet, leaving a green lip at the foot of the wall.
const BOARD_BOTTOM: f32 = 0.28;
const BOARD_TOP: f32 = 1.50;
/// A bright trim line and a dark capping rail above the team band.
const BOARD_TRIM_TOP: f32 = 1.57;
const BOARD_RAIL_TOP: f32 = 1.68;
/// How far the boards float off the collision surface, in metres. Enough to
/// beat z-fighting without letting the car clip them.
const BOARD_LIFT: f32 = 0.018;

/// Floodlight masts at the four corners. They stand just outside the shell, so
/// they read through the glass but never intrude on the play volume.
const LIGHT_TOWER_OFFSET: f32 = 0.8;
const LIGHT_TOWER_HEIGHT: f32 = 25.0;

/// Visual wheel radius, in metres. A fraction of the hitbox half height, so the
/// tyre reaches the hitbox floor.
const WHEEL_RADIUS: f32 = 0.52 * CAR_HALF_HEIGHT;
/// Full-lock angle for the visual front wheels, in radians. RL's real front-wheel
/// angle is speed-dependent; this is a fixed stand-in.
const MAX_WHEEL_STEER: f32 = 0.5;
/// Free-spin speed of the wheels at full throttle while airborne, in m/s. Only a
/// visual: the rolling maths converts it to a wheel angle like a ground speed.
const AIR_WHEEL_SPIN_SPEED: f32 = 6.0;

/// RL's ball indicator: a thin white ring projected on the pitch below the ball.
/// It is how you find the ball when it is high or against the ceiling. The ring
/// is a flat torus just above the pitch markings, slightly wider than the ball
/// so it still peeks out when the ball is resting on the pitch.
const BALL_INDICATOR_INNER: f32 = 0.95;
const BALL_INDICATOR_OUTER: f32 = 1.15;
const BALL_INDICATOR_Y: f32 = 0.04;

/// The ball trail: a burst of long, thin streaks around the ball, aligned with
/// its velocity, like RL's hard-hit trail. Hidden below `MIN_SPEED`, full
/// strength at `FULL_SPEED`; lengths are in game time, so slow motion stretches
/// them and pause freezes them.
const BALL_TRAIL_STREAKS: usize = 18;
const BALL_TRAIL_MIN_SPEED: f32 = 15.0;
const BALL_TRAIL_FULL_SPEED: f32 = 40.0;
/// How far the streaks reach, in seconds of ball travel.
const BALL_TRAIL_SECS: f32 = 0.35;
const BALL_TRAIL_MIN_LENGTH: f32 = 1.5;
const BALL_TRAIL_MAX_LENGTH: f32 = 18.0;
/// Half-width of a streak at the head, in metres. Thin lines, not a ribbon.
const BALL_TRAIL_WIDTH: f32 = 0.12;
/// How far the streaks are spread around the ball, in metres.
const BALL_TRAIL_SPREAD: f32 = 1.25;
const BALL_TRAIL_ALPHA: f32 = 0.4;
/// How quickly the trail fades in/out as the ball crosses the speed gate.
const BALL_TRAIL_RESPONSE: f32 = 12.0;

/// Boost pad visuals. The pickup hitboxes (RLBot's useful game values: 144 uu
/// radius for a small pad, 208 uu for a big one) are much larger than the glow,
/// so these are the visible sizes. A small pad is a hexagonal plate with a ring
/// and four flame panels; a big pad is a hovering orb over a dark cross-shaped
/// platform, with a beam rising from the centre.
const SMALL_PAD_RADIUS: f32 = 0.52;
const SMALL_PAD_PANEL_HEIGHT: f32 = 0.44;
const SMALL_PAD_PANEL_RADIUS: f32 = 0.34;
const BIG_PAD_RADIUS: f32 = 0.80;
const BIG_BEAM_HEIGHT: f32 = 1.0;
/// Big pad orb: radius and centre height above the pitch. It hovers clear of the
/// platform, as RL's "pill" does.
const BIG_ORB_RADIUS: f32 = 0.64;
const BIG_ORB_HEIGHT: f32 = 1.08;

/// Pads are never quite still in RL. The big orbs bob and turn; the small pads'
/// light cones breathe. Periods in seconds, spin in radians per second.
const BOOST_BOB_PERIOD: f32 = 2.6;
const BOOST_BOB_AMPLITUDE: f32 = 0.045;
const BOOST_ORB_SPIN: f32 = 0.55;
const BOOST_PULSE_PERIOD: f32 = 2.2;
const BOOST_PULSE_AMOUNT: f32 = 0.04;

/// The camera settings file, relative to where the viewer is run. Missing is
/// fine (RL's defaults are used); a malformed file is a hard error. Override the
/// path with `--camera-config PATH`. The file is hot-reloaded on save.
const CAMERA_CONFIG_PATH: &str = "camera.toml";
/// The keyboard/gamepad bindings file. Like `camera.toml`, it is optional and
/// hot-reloaded; `--controls-config PATH` overrides the path.
const CONTROLS_CONFIG_PATH: &str = "controls.toml";
/// How often the settings files are checked for changes. Five seconds is enough
/// for hand-tuning and avoids reloading on every keystroke while it is being
/// edited; a file caught mid-edit normally fails to parse and the previous
/// settings stay in place.
const SETTINGS_RELOAD_CHECK_SECS: f32 = 5.0;
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
/// An axis is only trusted to imply the car's heading while its horizontal
/// projection is at least this long. Below it the axis is close to vertical and
/// its direction is mostly numerical noise; the other axis takes over.
const CAR_HEADING_MIN_HORIZONTAL: f32 = 0.1;
/// How upright an airborne car must be before the camera follows its heading.
/// Below this (about 18° of pitch or roll) the car is mid-flip, and the camera
/// holds its yaw: RL's car cam does not swing through a dodge's pitch/roll.
const CAR_UPRIGHT_MIN_UP_Y: f32 = 0.95;

/// Simulation playback controls (pause and slow motion). These are viewer-level
/// keys like F3, not gameplay bindings, so they stay out of `controls.toml`.
///
/// The speeds are a ladder rather than a fixed step: 0.1x rungs around normal
/// speed make slow motion adjustable, while the fast end is coarser so reaching
/// 10x does not take dozens of key presses. A fixed step cannot do both, and a
/// 0.5x step bottoms out at 0.5x after one press.
const PLAYBACK_SPEEDS: [f32; 21] = [
    0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 2.0, 2.5, 3.0, 4.0,
    5.0, 10.0,
];
const PLAYBACK_DEFAULT_SPEED: f32 = 1.0;

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

/// Where the camera settings came from and when they last changed, so the file
/// can be hot-reloaded while the viewer runs.
#[derive(Resource)]
struct CameraConfig {
    path: PathBuf,
    modified: Option<SystemTime>,
}

/// Where the input bindings came from and when they last changed.
#[derive(Resource)]
struct ControlsConfig {
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

/// Simulation playback state: `P` freezes the physics, and `+`/`-` scale how
/// many simulation ticks a real second contains.
///
/// Pause is a run condition on `step_simulation`, *not* `Time<Virtual>::pause`,
/// so the render clock keeps running: the chase camera, the settings hot-reload
/// and the HUD stay responsive while the car is frozen. Game speed, on the other
/// hand, *is* `Time<Virtual>`'s relative speed, because that is the clock
/// `Time<Fixed>` accumulates - scaling it changes the sim tick rate without
/// duplicating the fixed-timestep logic here.
#[derive(Resource)]
struct Playback {
    paused: bool,
    /// Index into [`PLAYBACK_SPEEDS`]. Keeping an index rather than an `f32`
    /// makes the ladder exact (no float drift from repeated `+= 0.5`) and makes
    /// the rungs trivial to test.
    speed_index: usize,
}

impl Default for Playback {
    fn default() -> Self {
        Self {
            paused: false,
            speed_index: Self::default_speed_index(),
        }
    }
}

impl Playback {
    fn default_speed_index() -> usize {
        PLAYBACK_SPEEDS
            .iter()
            .position(|&speed| speed == PLAYBACK_DEFAULT_SPEED)
            .expect("PLAYBACK_SPEEDS contains the default 1x speed")
    }

    fn speed(&self) -> f32 {
        PLAYBACK_SPEEDS[self.speed_index]
    }

    /// One rung slower, stopping at the slowest rung.
    fn slower(&mut self) {
        self.speed_index = self.speed_index.saturating_sub(1);
    }

    /// One rung faster, stopping at the fastest rung.
    fn faster(&mut self) {
        self.speed_index = (self.speed_index + 1).min(PLAYBACK_SPEEDS.len() - 1);
    }

    /// Back to 1x without touching the paused flag.
    fn reset_speed(&mut self) {
        self.speed_index = Self::default_speed_index();
    }
}

/// The last two completed fixed ticks, kept for render interpolation.
///
/// Drawing the newest simulation state directly makes motion advance in whole
/// ticks: at 120 Hz a 60 fps display gets a steady 2 ticks per frame, but at
/// 1.1x it alternates 2/2/2/2/3 and visibly beats. Blending the last two ticks
/// by the fixed clock's overstep gives the renderer a continuous pose without
/// changing the physics or introducing a variable timestep.
#[derive(Resource)]
struct TickHistory {
    previous_cars: Vec<CarView>,
    current_cars: Vec<CarView>,
    previous_ball: BallView,
    current_ball: BallView,
}

impl TickHistory {
    fn new(sim: &sim::Sim) -> Self {
        let mut cars = Vec::with_capacity(sim.num_cars());
        fill_car_views(sim, &mut cars);
        Self {
            previous_cars: cars.clone(),
            current_cars: cars,
            previous_ball: sim.ball(),
            current_ball: sim.ball(),
        }
    }

    /// Advance the history by one simulation tick. The two vectors are swapped,
    /// not cloned, so the steady state does not allocate.
    fn push(&mut self, sim: &sim::Sim) {
        std::mem::swap(&mut self.previous_cars, &mut self.current_cars);
        self.previous_ball = self.current_ball;
        fill_car_views(sim, &mut self.current_cars);
        self.current_ball = sim.ball();
    }

    /// Collapse the history onto the current state after a reset, so the render
    /// does not blend across the teleport back to kickoff.
    fn snap(&mut self, sim: &sim::Sim) {
        fill_car_views(sim, &mut self.current_cars);
        self.current_ball = sim.ball();
        self.previous_cars.clear();
        self.previous_cars.extend_from_slice(&self.current_cars);
        self.previous_ball = self.current_ball;
    }
}

fn fill_car_views(sim: &sim::Sim, cars: &mut Vec<CarView>) {
    cars.clear();
    cars.extend((0..sim.num_cars()).map(|index| sim.car_at(index)));
}

/// The pose the renderer and camera should use for this frame, sampled between
/// the last two ticks. `ball` is `None` only before the first sample.
#[derive(Resource, Default)]
struct RenderViews {
    cars: Vec<CarView>,
    ball: Option<BallView>,
}

/// One streak in the ball trail. The values are fixed per streak so the pattern
/// does not shimmer from frame to frame.
#[derive(Clone, Copy)]
struct TrailStreak {
    /// Position around the velocity axis, in radians.
    angle: f32,
    /// Distance from the ball's centre, in metres.
    radius: f32,
    /// Fraction of the speed-derived length this streak uses.
    length_scale: f32,
    /// Multiplier on the overall trail opacity.
    alpha_scale: f32,
}

/// The trail mesh and its per-streak pattern. The mesh is rebuilt each frame
/// from the ball's velocity; the entity is hidden while the trail is inactive.
#[derive(Resource)]
struct BallTrail {
    mesh: Handle<Mesh>,
    /// 0..=1, eased from the ball's speed so the trail fades rather than pops.
    strength: f32,
    streaks: Vec<TrailStreak>,
}

/// Marks the single entity that draws the ball trail.
#[derive(Component)]
struct BallTrailVisual;

#[derive(Component)]
enum Vis {
    /// A simulated car, indexed in `Sim`/player order. Index 0 is the human.
    Car(usize),
    Ball,
    /// RL's ball indicator: a ring on the pitch directly below the ball.
    BallIndicator,
    /// The glowing part of a boost pad, indexed into `World::boost_pads`.
    /// Hidden while the pad is on cooldown. For a small pad that is its ring,
    /// core and flame panels; the hexagonal marker stays on the pitch. For a big
    /// pad it is the orb, light beam and base ring, leaving only the dark
    /// platform standing.
    BoostPad(usize),
}

#[derive(Component)]
struct ChaseCam;

#[derive(Component)]
struct HudText;

/// Visual-cost preset for `--quality`. `High` is the viewer's normal look;
/// `Low` is for integrated GPUs, where 4x MSAA + shadows + bloom at 1080p is
/// enough to miss vsync every so often.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum QualityPreset {
    Low,
    Medium,
    #[default]
    High,
}

/// The resolved render settings used by `setup_scene`. `--msaa` can override the
/// preset's MSAA level without changing its shadow/bloom choices.
#[derive(Resource, Clone, Copy)]
struct RenderQuality {
    msaa: Msaa,
    shadows: bool,
    bloom: bool,
    shadow_map_size: usize,
}

impl RenderQuality {
    fn new(preset: QualityPreset, msaa_override: Option<Msaa>) -> Self {
        let (msaa, shadows, bloom, shadow_map_size) = match preset {
            QualityPreset::Low => (Msaa::Off, false, false, 1024),
            QualityPreset::Medium => (Msaa::Sample2, true, true, 1024),
            QualityPreset::High => (Msaa::Sample4, true, true, 2048),
        };
        Self {
            msaa: msaa_override.unwrap_or(msaa),
            shadows,
            bloom,
            shadow_map_size,
        }
    }
}

/// Rolling frame-time window for the `--stats` overlay. A 5 s window keeps the
/// average responsive while still giving the 1% low a useful number of frames
/// to choose from (300 at 60 fps).
const PERF_WINDOW_SECS: f32 = 5.0;
/// How often the overlay's numbers are recomputed. Updating every frame would
/// sort the window constantly and make the text unreadable.
const PERF_REFRESH_SECS: f32 = 0.25;
/// Ignore the first second: pipeline compilation and window bring-up make the
/// first frames far slower than steady state, which would poison max/1% low.
const PERF_WARMUP_SECS: f32 = 1.0;

/// Frame timing for the performance overlay, switched on with `--stats` or F3.
/// Frames are measured from `Time<Real>`, i.e. wall-clock time between frames,
/// so the numbers are what the display actually receives (vsync included).
#[derive(Resource)]
struct PerfStats {
    visible: bool,
    /// Frame times in milliseconds, covering roughly `PERF_WINDOW_SECS`.
    frames: VecDeque<f32>,
    window_ms: f32,
    since_refresh: f32,
    fps: f32,
    avg_ms: f32,
    max_ms: f32,
    low_fps: f32,
    /// Physical window size, shown next to the timings to confirm the render
    /// resolution (fullscreen picks the monitor's native size).
    width: u32,
    height: u32,
}

impl Default for PerfStats {
    fn default() -> Self {
        Self {
            visible: false,
            frames: VecDeque::new(),
            window_ms: 0.0,
            since_refresh: 0.0,
            fps: 0.0,
            avg_ms: 0.0,
            max_ms: 0.0,
            low_fps: 0.0,
            width: 0,
            height: 0,
        }
    }
}

/// The two goal totals in the top-centre scoreboard.
#[derive(Component)]
struct BlueScoreText;

#[derive(Component)]
struct OrangeScoreText;

/// The boost meter's numbered readout and its fill bar, bottom right as in RL.
#[derive(Component)]
struct BoostMeterText;

#[derive(Component)]
struct BoostMeterFill;

/// A visual wheel. `steers` marks the front pair, which also yaw with the
/// steering input; every wheel rolls about its axle. `car` indexes `Sim` so the
/// opponent's wheels use the opponent's speed and steering, not the human's.
#[derive(Component)]
struct Wheel {
    steers: bool,
    car: usize,
}

/// Whether each car was actually boosting on the most recent tick (the button is
/// held *and* there is boost to spend). Indexed by car; drives the exhaust flames.
#[derive(Resource, Default)]
struct Boosting([bool; 2]);

/// A car's exhaust flame, shown only while that car is boosting.
#[derive(Component)]
struct BoostFlame {
    car: usize,
}

/// A big pad's orb pivot. It bobs and spins as one, carrying its shell and core.
#[derive(Component)]
struct BoostOrb {
    base_y: f32,
    phase: f32,
}

/// A small pad's flame panel, breathing slowly so the pad reads as live even
/// when nothing is happening.
#[derive(Component)]
struct BoostPulse {
    phase: f32,
}

fn main() {
    let mut gamepad_debug = false;
    // A capture is asked for by `--screenshot PATH [TICK]`. `--cam` / `--look`
    // turn it into an instant free-camera shot (see `resolve_free_cam`).
    let mut shot: Option<(String, Option<u64>)> = None;
    let mut cam: Option<Vec3> = None;
    let mut look: Option<Vec3> = None;
    let mut camera_config: Option<PathBuf> = None;
    let mut controls_config: Option<PathBuf> = None;
    // Window presentation. Fullscreen is borderless, so it uses the monitor's
    // native resolution without changing the desktop video mode.
    let mut fullscreen = false;
    let mut resolution: Option<(u32, u32)> = None;
    let mut no_vsync = false;
    // Render-cost preset and an optional MSAA override for it.
    let mut quality = QualityPreset::default();
    let mut msaa_override = None;
    // Start with the frame-time overlay visible (F3 toggles it either way).
    let mut stats = false;
    let mut rlbot_enabled = false;
    let mut rlbot_port = rlbot_bridge::DEFAULT_PORT;
    // `None` means "use the bot config, or the built-in default".
    let mut rlbot_agent_id: Option<String> = None;
    let mut rlbot_name: Option<String> = None;
    // A bot to start, either explicitly or from a `bot.toml`.
    let mut rlbot_command: Option<String> = None;
    let mut rlbot_config: Option<String> = None;
    let mut rlbot_dir: Option<PathBuf> = None;

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
            "--controls-config" => {
                controls_config = Some(PathBuf::from(
                    args.next()
                        .unwrap_or_else(|| panic!("--controls-config needs a path")),
                ));
            }
            "--fullscreen" => fullscreen = true,
            "--resolution" => {
                let value = args
                    .next()
                    .unwrap_or_else(|| panic!("--resolution needs WIDTHxHEIGHT"));
                resolution = Some(parse_resolution(&value));
            }
            "--no-vsync" => no_vsync = true,
            "--quality" => {
                let value = args
                    .next()
                    .unwrap_or_else(|| panic!("--quality needs low, medium or high"));
                quality = parse_quality(&value);
            }
            "--msaa" => {
                let value = args
                    .next()
                    .unwrap_or_else(|| panic!("--msaa needs off, 2, 4 or 8"));
                msaa_override = Some(parse_msaa(&value));
            }
            "--stats" => stats = true,
            "--gamepad-debug" => gamepad_debug = true,
            "--rlbot" => rlbot_enabled = true,
            "--rlbot-port" => {
                let value = args
                    .next()
                    .unwrap_or_else(|| panic!("--rlbot-port needs a port"));
                rlbot_port = value
                    .parse()
                    .unwrap_or_else(|_| panic!("--rlbot-port: bad port {value:?}"));
            }
            "--rlbot-agent-id" => {
                rlbot_agent_id = Some(
                    args.next()
                        .unwrap_or_else(|| panic!("--rlbot-agent-id needs an id")),
                );
            }
            "--rlbot-name" => {
                rlbot_name = Some(
                    args.next()
                        .unwrap_or_else(|| panic!("--rlbot-name needs a name")),
                );
            }
            "--rlbot-command" => {
                rlbot_command = Some(
                    args.next()
                        .unwrap_or_else(|| panic!("--rlbot-command needs a command")),
                );
            }
            "--rlbot-config" => {
                rlbot_config = Some(
                    args.next()
                        .unwrap_or_else(|| panic!("--rlbot-config needs a path or bot name")),
                );
            }
            "--rlbot-dir" => {
                rlbot_dir = Some(PathBuf::from(
                    args.next()
                        .unwrap_or_else(|| panic!("--rlbot-dir needs a path")),
                ));
            }
            _ => {}
        }
    }

    if rlbot_command.is_some() && rlbot_config.is_some() {
        panic!("use --rlbot-command or --rlbot-config, not both");
    }
    if fullscreen && resolution.is_some() {
        eprintln!(
            "note: --resolution is ignored with --fullscreen; borderless fullscreen \
             uses the monitor's native resolution"
        );
    }
    if rlbot_dir.is_some() && rlbot_command.is_none() && rlbot_config.is_none() {
        panic!("--rlbot-dir needs --rlbot-command or --rlbot-config");
    }

    // A bot config supplies the agent id, display name and launch command;
    // explicit flags still win, and the built-in defaults fill any gaps.
    let bot_config = rlbot_config.as_deref().map(|spec| {
        let path = rlbot::resolve_bot_config(spec);
        rlbot::load_bot_config(&path)
    });
    let rlbot_agent_id = rlbot_agent_id
        .or_else(|| bot_config.as_ref().map(|config| config.agent_id.clone()))
        .unwrap_or_else(|| "rlbot/opponent".to_string());
    let rlbot_name = rlbot_name
        .or_else(|| bot_config.as_ref().map(|config| config.name.clone()))
        .unwrap_or_else(|| "RLBot".to_string());
    let launch = match (rlbot_command, bot_config) {
        (Some(command), _) => Some(LaunchOptions {
            command,
            working_dir: rlbot_dir,
            environment: Vec::new(),
        }),
        (None, Some(mut config)) => {
            if rlbot_dir.is_some() {
                config.launch.working_dir = rlbot_dir;
            }
            Some(config.launch)
        }
        (None, None) => None,
    };
    if launch.is_some() {
        rlbot_enabled = true;
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
    let (controls, controls_config) = initial_controls(controls_config);

    let mut sim = if rlbot_enabled {
        sim::Sim::with_opponent()
    } else {
        sim::Sim::new()
    };
    // Debug-only wall pose for free-cam screenshots (the sim is skipped in that
    // mode, so the car stays where it is put).
    if std::env::var_os("RL_WALL_DEBUG").is_some() {
        sim.debug_park_on_wall();
    }
    let tick_history = TickHistory::new(&sim);

    let rlbot = if rlbot_enabled {
        Rlbot::start(
            &sim,
            RlbotOptions {
                port: rlbot_port,
                agent_id: rlbot_agent_id,
                bot_name: rlbot_name,
                launch,
            },
        )
    } else {
        Rlbot::disabled()
    };

    let mut window = Window::default();
    if fullscreen {
        // Borderless rather than exclusive: it still renders at the monitor's
        // native resolution, but avoids a desktop video-mode switch (and works
        // on Wayland, where exclusive fullscreen is not available).
        window.mode = WindowMode::BorderlessFullscreen(MonitorSelection::Primary);
    }
    if let Some((width, height)) = resolution {
        window.resolution = WindowResolution::new(width, height);
    }
    if no_vsync {
        // `AutoNoVsync` falls back to vsync when a tearing-free mode is the only
        // one available; `Immediate` would panic in that case.
        window.present_mode = PresentMode::AutoNoVsync;
    }

    let render_quality = RenderQuality::new(quality, msaa_override);

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(window),
            ..default()
        }))
        .insert_resource(Sim(sim))
        .insert_resource(tick_history)
        .init_resource::<RenderViews>()
        .insert_resource(render_quality)
        .insert_resource(DirectionalLightShadowMap {
            size: render_quality.shadow_map_size,
        })
        .insert_resource(rlbot)
        .insert_resource(mode)
        .insert_resource(camera_settings)
        .insert_resource(camera_config)
        .insert_resource(controls)
        .insert_resource(controls_config)
        .insert_resource(GamepadDebug(gamepad_debug))
        .insert_resource(BallCam(true))
        .insert_resource(PerfStats {
            visible: stats,
            ..default()
        })
        .init_resource::<Boosting>()
        .init_resource::<Playback>()
        .insert_resource(Time::<Fixed>::from_hz(sim::TICK_HZ))
        .add_systems(
            PreUpdate,
            seed_gamepad_triggers.after(bevy::input::InputSystems),
        )
        .add_systems(Startup, setup_scene)
        .add_systems(
            FixedUpdate,
            step_simulation.run_if(simulating).run_if(not_paused),
        )
        .add_systems(
            Update,
            (
                toggle_camera_mode,
                control_playback,
                poll_rlbot_while_paused.run_if(paused),
                reset_on_key,
                report_gamepads,
                report_gamepad_events,
                sample_views,
                sync_visuals.after(sample_views),
                animate_boost_pads,
                spin_wheels,
                update_ball_trail.after(sample_views),
                (reload_camera_settings, apply_camera_fov).chain(),
                reload_controls,
                chase_camera.run_if(simulating).after(sample_views),
                track_performance.before(update_hud),
                update_hud,
                update_boost_meter,
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

/// Load `--controls-config PATH`, or `controls.toml` if it exists. An explicit
/// path must load; a missing default file silently falls back to the built-in
/// bindings.
fn initial_controls(explicit: Option<PathBuf>) -> (Controls, ControlsConfig) {
    let required = explicit.is_some();
    let path = explicit.unwrap_or_else(|| PathBuf::from(CONTROLS_CONFIG_PATH));
    let modified = file_modified(&path);
    let controls = match read_controls(&path) {
        Ok(controls) => controls,
        Err(_) if !required && !path.exists() => Controls::default(),
        Err(e) => panic!("controls config: {e}"),
    };
    (controls, ControlsConfig { path, modified })
}

fn read_controls(path: &Path) -> Result<Controls, String> {
    let raw = std::fs::read_to_string(path)
        .map_err(|e| format!("could not read {}: {e}", path.display()))?;
    toml::from_str::<Controls>(&raw).map_err(|e| format!("could not parse {}: {e}", path.display()))
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

/// Parse `--resolution WIDTHxHEIGHT` (physical pixels).
fn parse_resolution(raw: &str) -> (u32, u32) {
    let (width, height) = raw
        .split_once('x')
        .or_else(|| raw.split_once('X'))
        .unwrap_or_else(|| {
            panic!("--resolution: bad value {raw:?}, want WIDTHxHEIGHT (e.g. 1920x1080)")
        });
    let width = width
        .trim()
        .parse::<u32>()
        .unwrap_or_else(|_| panic!("--resolution: bad value {raw:?}, want WIDTHxHEIGHT"));
    let height = height
        .trim()
        .parse::<u32>()
        .unwrap_or_else(|_| panic!("--resolution: bad value {raw:?}, want WIDTHxHEIGHT"));
    if width == 0 || height == 0 {
        panic!("--resolution: bad value {raw:?}, want WIDTHxHEIGHT");
    }
    (width, height)
}

/// Parse `--quality low|medium|high`.
fn parse_quality(raw: &str) -> QualityPreset {
    match raw.to_ascii_lowercase().as_str() {
        "low" => QualityPreset::Low,
        "medium" | "med" => QualityPreset::Medium,
        "high" => QualityPreset::High,
        _ => panic!("--quality: bad value {raw:?}, want low, medium or high"),
    }
}

/// Parse `--msaa off|2|4|8` (`0`/`none` are accepted as aliases for `off`).
fn parse_msaa(raw: &str) -> Msaa {
    match raw.to_ascii_lowercase().as_str() {
        "off" | "none" | "0" => Msaa::Off,
        "2" => Msaa::Sample2,
        "4" => Msaa::Sample4,
        "8" => Msaa::Sample8,
        _ => panic!("--msaa: bad value {raw:?}, want off, 2, 4 or 8"),
    }
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

/// Run condition for `step_simulation`: paused means the clock keeps ticking but
/// no physics step is taken. The fixed schedule still consumes its accumulated
/// time, so unpausing cannot replay a backlog of ticks.
fn not_paused(playback: Res<Playback>) -> bool {
    !playback.paused
}

fn paused(playback: Res<Playback>) -> bool {
    playback.paused
}

// ---------------------------------------------------------------------------
// Scene
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    sim: Res<Sim>,
    mode: Res<Mode>,
    settings: Res<CameraSettings>,
    quality: Res<RenderQuality>,
) {
    // --- arena ---
    // The arena is drawn from RocketSim's own collision mesh (`sim::
    // arena_collision_mesh`), so the visible surface *is* the surface the ball
    // and car collide with - the real RL arena, not an approximation. Vertex
    // colours separate the pitch green from the slate walls, the darker roof and
    // a team-coloured band low on each end wall.
    commands.spawn((
        Mesh3d(meshes.add(arena_surface_mesh())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            base_color_texture: Some(images.add(arena_detail_texture())),
            perceptual_roughness: 0.55,
            // RL's walls and roof are dark glass with a bright lattice, not an
            // opaque box. The texture's alpha carries the difference: the hex
            // panels are almost clear, the lattice is nearly solid. That keeps
            // the cage readable from inside while still letting the camera see
            // through it from the walls and roof.
            alpha_mode: AlphaMode::Blend,
            // The shell is a cage the camera can see through from outside: the
            // mesh is wound so its front faces inward, so back-face culling hides
            // it from outside and the camera may leave the arena (see
            // `chase_camera`) and still watch the car through the wall.
            cull_mode: Some(Face::Back),
            ..default()
        })),
        // The shell is glass, but the shadow pass ignores alpha and would treat it
        // as a solid box: the goal recess in particular threw a black slab across
        // the goal box. Only opaque objects (cars, ball, goal frame) cast shadows.
        NotShadowCaster,
    ));

    // --- pitch ---
    // Mowed squares, markings and goal boxes are vertex-coloured geometry laid a
    // centimetre above the collision floor: no texture, a few hundred triangles.
    commands.spawn((
        Mesh3d(meshes.add(pitch_mesh())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            perceptual_roughness: 0.95,
            ..default()
        })),
    ));

    // --- wall base boards ---
    // A low, team-coloured band skirting the whole arena, following the curved
    // wall base. This is the single biggest cue that the cage has a floor: the
    // glass above it is nearly clear, so without a solid base the walls appear
    // to float. Blue takes the -Z half, orange the +Z half, as in RL.
    commands.spawn((
        Mesh3d(meshes.add(arena_boards_mesh())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::WHITE,
            perceptual_roughness: 0.7,
            metallic: 0.15,
            // The band wraps the fillet, so it is seen from odd angles while
            // driving on a wall; do not let a winding slip hide part of it.
            cull_mode: None,
            ..default()
        })),
        NotShadowCaster,
    ));

    // --- wall and ceiling grid ---
    // Shallow ribs on the inside of the shell, like the trusses and fence panels
    // in RL. They are one mesh and one material; the 3 cm depth is what keeps
    // them readable at the grazing angle you get while driving on a wall.
    commands.spawn((
        Mesh3d(meshes.add(arena_grid_mesh())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.46, 0.50, 0.56),
            metallic: 0.4,
            perceptual_roughness: 0.5,
            // A touch of self-light so the roof grid still reads where the key
            // light does not reach it. Well below the bloom threshold.
            emissive: LinearRgba::rgb(0.035, 0.040, 0.050),
            ..default()
        })),
        NotShadowCaster,
    ));

    // --- ceiling lights and corner floodlights ---
    // RL's roof is not blank: it has light banks and corner floodlights. Both are
    // emissive geometry rather than real lights, so the cost stays at a couple of
    // draw calls and the lighting rig does not change. They are also what gives
    // the transparent roof something to read against.
    let ceiling_lights_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.95, 0.97, 1.0),
        // Bright on the HDR target but below the bloom prefilter, so the strips
        // read as white light without blooming the whole roof.
        emissive: LinearRgba::rgb(3.2, 3.5, 4.0),
        perceptual_roughness: 0.5,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(ceiling_lights_mesh())),
        MeshMaterial3d(ceiling_lights_mat),
        NotShadowCaster,
    ));

    let tower_frame_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.46, 0.49, 0.55),
        metallic: 0.6,
        perceptual_roughness: 0.45,
        ..default()
    });
    let tower_lamp_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.97, 0.88),
        // Just clears the bloom threshold (6.0) so the lamp heads glow, like the
        // big boost pads, without washing the arena out.
        emissive: LinearRgba::rgb(7.5, 7.0, 6.0),
        perceptual_roughness: 0.35,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(light_tower_frame_mesh())),
        MeshMaterial3d(tower_frame_mat),
        NotShadowCaster,
    ));
    commands.spawn((
        Mesh3d(meshes.add(light_tower_lamp_mesh())),
        MeshMaterial3d(tower_lamp_mat),
        NotShadowCaster,
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

    // RL's ball indicator: a white ring on the pitch under the ball. It is
    // always visible, at any ball height, which is what makes it useful in ball
    // cam when the ball is above the frame.
    commands.spawn((
        Mesh3d(
            meshes.add(
                Torus::new(BALL_INDICATOR_INNER, BALL_INDICATOR_OUTER)
                    .mesh()
                    .minor_resolution(6)
                    .major_resolution(48),
            ),
        ),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(1.0, 1.0, 1.0, 0.65),
            unlit: true,
            alpha_mode: AlphaMode::Blend,
            cull_mode: None,
            ..default()
        })),
        Transform::from_xyz(0.0, BALL_INDICATOR_Y, 0.0),
        Vis::BallIndicator,
        NotShadowCaster,
    ));

    // --- ball trail ---
    // The mesh starts as a hidden degenerate quad; `update_ball_trail` replaces
    // it with a camera-facing ribbon made from the ball's recent positions.
    let mut trail_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    // Start with a degenerate (zero-area) quad rather than empty buffers. Bevy's
    // render asset allocator does not like a mesh with no vertices, and the
    // entity is hidden until the first real trail is built anyway.
    trail_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vec![[0.0f32; 3]; 4]);
    trail_mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, vec![[1.0f32, 1.0, 1.0, 0.0]; 4]);
    trail_mesh.insert_indices(Indices::U32(vec![0, 1, 2, 1, 3, 2]));
    let trail_mesh = meshes.add(trail_mesh);
    commands.spawn((
        Mesh3d(trail_mesh.clone()),
        MeshMaterial3d(materials.add(StandardMaterial {
            // Unlit and additive: the vertex colours carry the fade, and the
            // trail must not pick up the stadium lights.
            base_color: Color::WHITE,
            alpha_mode: AlphaMode::Add,
            unlit: true,
            cull_mode: None,
            ..default()
        })),
        Transform::default(),
        Visibility::Hidden,
        BallTrailVisual,
        NotShadowCaster,
        // The mesh is rebuilt every frame and spans wherever the ball is; a
        // stale AABB would cull it, and one mesh is not worth culling anyway.
        bevy::camera::visibility::NoFrustumCulling,
    ));
    commands.insert_resource(BallTrail {
        mesh: trail_mesh,
        strength: 0.0,
        streaks: trail_streaks(),
    });

    // --- boost pads ---
    // RL has two distinct pickups. A small pad is a light hexagonal plate with a
    // glowing centre ring and four translucent flame panels; a big pad is a
    // fiery orb hovering over a dark cross-shaped platform, fed by a light beam.
    // Both are a handful of shared meshes and materials, animated by
    // `animate_boost_pads`.
    //
    // The flames and the beam are unlit and additive, with a vertical colour
    // gradient baked into their vertex colours: bright and opaque at the bottom,
    // fading to nothing at the top. That is what makes them read as light.
    let pad_base_mesh = meshes.add(Cylinder::new(SMALL_PAD_RADIUS, 0.05).mesh().resolution(6));
    let pad_top_mesh = meshes.add(
        Cylinder::new(SMALL_PAD_RADIUS * 0.88, 0.02)
            .mesh()
            .resolution(6),
    );
    let pad_puck_mesh = meshes.add(Cylinder::new(0.22, 0.10).mesh().resolution(24));
    let pad_core_mesh = meshes.add(Cylinder::new(0.14, 0.012).mesh().resolution(24));
    let pad_ring_mesh = meshes.add(
        Torus::new(0.16, 0.24)
            .mesh()
            .minor_resolution(6)
            .major_resolution(24),
    );
    let pad_panel_mesh = meshes.add(with_vertical_gradient(
        Cuboid::new(0.18, SMALL_PAD_PANEL_HEIGHT, 0.025).into(),
        [1.0, 0.55, 0.06, 0.70],
        [1.0, 0.20, 0.00, 0.00],
    ));

    let big_hub_mesh = meshes.add(Cylinder::new(BIG_PAD_RADIUS, 0.14).mesh().resolution(24));
    let big_arm_mesh = meshes.add(Cuboid::new(2.6, 0.12, 0.62));
    let big_arm_glow_mesh = meshes.add(Cuboid::new(2.0, 0.012, 0.26));
    let big_ring_mesh = meshes.add(
        Torus::new(0.56, 0.74)
            .mesh()
            .minor_resolution(8)
            .major_resolution(32),
    );
    let big_disc_mesh = meshes.add(Cylinder::new(0.54, 0.012).mesh().resolution(32));
    let big_beam_mesh = meshes.add(with_vertical_gradient(
        ConicalFrustum {
            radius_top: 0.10,
            radius_bottom: 0.24,
            height: BIG_BEAM_HEIGHT,
        }
        .into(),
        [1.0, 0.95, 0.60, 1.00],
        [1.0, 0.50, 0.05, 0.10],
    ));
    let big_orb_mesh = meshes.add(boost_orb_mesh(BIG_ORB_RADIUS));
    let big_orb_core_mesh = meshes.add(Sphere::new(0.28).mesh().uv(16, 12));

    let pad_plate_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.09, 0.10, 0.12),
        metallic: 0.55,
        perceptual_roughness: 0.45,
        ..default()
    });
    let pad_top_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.55, 0.58, 0.62),
        metallic: 0.35,
        perceptual_roughness: 0.45,
        ..default()
    });
    let pad_ring_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.40, 0.03),
        emissive: LinearRgba::rgb(2.6, 0.7, 0.03),
        perceptual_roughness: 0.6,
        ..default()
    });
    let pad_flame_mat = materials.add(StandardMaterial {
        // Unlit and additive: a flame must not pick up the white stadium key
        // light. The mesh's vertex colours supply both the gradient and the
        // alpha, and they replace `base_color` entirely (see the shader's
        // `VERTEX_COLORS` path), so the colour here is unused.
        base_color: Color::WHITE,
        alpha_mode: AlphaMode::Add,
        unlit: true,
        // Seen from outside on approach and from inside once the car is on the
        // pad, so neither face may be culled.
        cull_mode: None,
        ..default()
    });
    let pad_core_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.55, 0.12),
        // In the bloom's soft knee (the knee reaches down to 3.0), so the pad
        // reads as a light source without washing out to white the way a hotter
        // core would.
        emissive: LinearRgba::rgb(5.0, 2.0, 0.25),
        perceptual_roughness: 1.0,
        ..default()
    });
    let big_orb_mat = materials.add(StandardMaterial {
        // The shell carries a hot-to-charred gradient in its vertex colours, so
        // `base_color` is ignored. Only front faces are drawn: the back wall of
        // a double-sided transparent sphere shows the bright bottom half through
        // the top and flattens the orb into a disc.
        emissive: LinearRgba::rgb(0.8, 0.28, 0.02),
        alpha_mode: AlphaMode::Blend,
        perceptual_roughness: 1.0,
        ..default()
    });
    let big_orb_core_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.70, 0.25),
        // Opaque and hot. The shell is opaque now, so this only shows through
        // the gaps in the displaced silhouette; it mostly adds a warm underglow.
        emissive: LinearRgba::rgb(6.0, 3.2, 0.5),
        perceptual_roughness: 1.0,
        ..default()
    });

    for i in 0..sim.0.num_pads() {
        let pad = sim.0.pad(i);
        // Stagger the animation so the field does not pulse in lockstep.
        let phase = i as f32 * 0.7;
        if pad.is_big {
            // The dark cross platform is arena scenery and stays put. Everything
            // that glows is one hidden group, so it all leaves when the pad is
            // taken.
            commands.spawn((
                Mesh3d(big_hub_mesh.clone()),
                MeshMaterial3d(pad_plate_mat.clone()),
                Transform::from_xyz(pad.pos.x, 0.07, pad.pos.z),
            ));
            for yaw in [0.0, PI * 0.5] {
                commands.spawn((
                    Mesh3d(big_arm_mesh.clone()),
                    MeshMaterial3d(pad_plate_mat.clone()),
                    Transform::from_xyz(pad.pos.x, 0.06, pad.pos.z)
                        .with_rotation(Quat::from_rotation_y(yaw)),
                ));
            }
            commands
                .spawn((
                    Transform::from_xyz(pad.pos.x, 0.0, pad.pos.z),
                    Visibility::default(),
                    Vis::BoostPad(i),
                ))
                .with_children(|root| {
                    for yaw in [0.0, PI * 0.5] {
                        root.spawn((
                            Mesh3d(big_arm_glow_mesh.clone()),
                            MeshMaterial3d(pad_ring_mat.clone()),
                            Transform::from_xyz(0.0, 0.126, 0.0)
                                .with_rotation(Quat::from_rotation_y(yaw)),
                        ));
                    }
                    root.spawn((
                        Mesh3d(big_disc_mesh.clone()),
                        MeshMaterial3d(pad_core_mat.clone()),
                        Transform::from_xyz(0.0, 0.148, 0.0),
                    ));
                    root.spawn((
                        Mesh3d(big_ring_mesh.clone()),
                        MeshMaterial3d(pad_ring_mat.clone()),
                        Transform::from_xyz(0.0, 0.15, 0.0),
                    ));
                    root.spawn((
                        Mesh3d(big_beam_mesh.clone()),
                        MeshMaterial3d(pad_flame_mat.clone()),
                        // The frustum is centred on its own height, and a cone
                        // points +Y (the same convention as the exhaust flame).
                        Transform::from_xyz(0.0, 0.15 + BIG_BEAM_HEIGHT * 0.5, 0.0),
                    ));
                    root.spawn((
                        Transform::from_xyz(0.0, BIG_ORB_HEIGHT, 0.0),
                        Visibility::default(),
                        BoostOrb {
                            base_y: BIG_ORB_HEIGHT,
                            phase,
                        },
                    ))
                    .with_children(|orb| {
                        orb.spawn((
                            Mesh3d(big_orb_mesh.clone()),
                            MeshMaterial3d(big_orb_mat.clone()),
                        ));
                        orb.spawn((
                            Mesh3d(big_orb_core_mesh.clone()),
                            MeshMaterial3d(big_orb_core_mat.clone()),
                        ));
                    });
                });
        } else {
            // The hexagonal plate and its dark puck are a permanent marker: they
            // carry no `Vis`, so they stay on the pitch when the boost is taken.
            commands.spawn((
                Mesh3d(pad_base_mesh.clone()),
                MeshMaterial3d(pad_plate_mat.clone()),
                Transform::from_xyz(pad.pos.x, 0.025, pad.pos.z),
            ));
            commands.spawn((
                Mesh3d(pad_top_mesh.clone()),
                MeshMaterial3d(pad_top_mat.clone()),
                Transform::from_xyz(pad.pos.x, 0.06, pad.pos.z),
            ));
            commands.spawn((
                Mesh3d(pad_puck_mesh.clone()),
                MeshMaterial3d(pad_plate_mat.clone()),
                Transform::from_xyz(pad.pos.x, 0.09, pad.pos.z),
            ));
            // Only the glow is collected. The ring, core and flame panels share
            // one hidden group over the marker.
            commands
                .spawn((
                    Transform::from_xyz(pad.pos.x, 0.0, pad.pos.z),
                    Visibility::default(),
                    Vis::BoostPad(i),
                ))
                .with_children(|root| {
                    root.spawn((
                        Mesh3d(pad_core_mesh.clone()),
                        MeshMaterial3d(pad_core_mat.clone()),
                        Transform::from_xyz(0.0, 0.148, 0.0),
                    ));
                    root.spawn((
                        Mesh3d(pad_ring_mesh.clone()),
                        MeshMaterial3d(pad_ring_mat.clone()),
                        Transform::from_xyz(0.0, 0.15, 0.0),
                    ));
                    // Four flame panels around the ring. Each faces outward and
                    // leans away from the centre, like a crown of fire.
                    for k in 0..4 {
                        let angle = k as f32 * PI * 0.5;
                        let (sin, cos) = angle.sin_cos();
                        root.spawn((
                            Mesh3d(pad_panel_mesh.clone()),
                            MeshMaterial3d(pad_flame_mat.clone()),
                            Transform::from_xyz(
                                SMALL_PAD_PANEL_RADIUS * cos,
                                0.07 + SMALL_PAD_PANEL_HEIGHT * 0.5,
                                SMALL_PAD_PANEL_RADIUS * sin,
                            )
                            .with_rotation(
                                Quat::from_rotation_y(PI * 0.5 - angle)
                                    * Quat::from_rotation_x(0.12),
                            ),
                            BoostPulse { phase },
                        ));
                    }
                });
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
    // The net is one procedural cross-hatch texture on a single back quad per
    // goal. It is blended, so the recess behind it still reads.
    let net_mat = materials.add(StandardMaterial {
        base_color: Color::srgba(0.60, 0.65, 0.72, 0.55),
        base_color_texture: Some(images.add(net_texture())),
        alpha_mode: AlphaMode::Blend,
        cull_mode: None,
        perceptual_roughness: 1.0,
        ..default()
    });
    let net_back_mesh = meshes.add(quad_mesh(2.0 * GOAL_HALF_WIDTH, GOAL_HEIGHT, 20.0, 7.0));

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

        // Net: one back panel at RocketSim's goal depth. The quad faces +Z by
        // default, so turn it to face the pitch. Side panels were tried and
        // dropped: at chase distance the repeating hatch aliases into bright
        // static, which is worse than an open recess.
        commands.spawn((
            Mesh3d(net_back_mesh.clone()),
            MeshMaterial3d(net_mat.clone()),
            Transform::from_xyz(0.0, h * 0.5, sign * (ARENA_HALF_Z + GOAL_DEPTH - 0.15))
                .with_rotation(Quat::from_rotation_y(if sign > 0.0 { PI } else { 0.0 })),
            // The mesh is alpha-blended and mostly empty; the shadow pass still
            // treats it as a solid quad and throws a black slab across the goal
            // box. The frame casts the useful shadow.
            NotShadowCaster,
        ));
    }

    // --- cars ---
    spawn_car(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vis::Car(0),
        0,
        linear_rgba(0.10, 0.60, 0.95),
        Color::srgb(0.08, 0.48, 0.78),
    );

    // The RLBot opponent gets a real second car mesh in orange, plus a
    // translucent disc and a floating marker so it is readable at a distance.
    if sim.0.num_cars() > 1 {
        let opponent = spawn_car(
            &mut commands,
            &mut meshes,
            &mut materials,
            Vis::Car(1),
            1,
            linear_rgba(1.0, 0.36, 0.03),
            Color::srgb(0.90, 0.28, 0.02),
        );
        let opponent_ring = materials.add(StandardMaterial {
            base_color: Color::srgba(1.0, 0.42, 0.04, 0.55),
            emissive: LinearRgba::rgb(2.4, 0.8, 0.08),
            alpha_mode: AlphaMode::Blend,
            unlit: true,
            ..default()
        });
        let marker = materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.45, 0.05),
            emissive: LinearRgba::rgb(4.0, 1.2, 0.1),
            unlit: true,
            ..default()
        });
        commands.entity(opponent).with_children(|parent| {
            parent.spawn((
                Mesh3d(meshes.add(Cylinder::new(1.15, 0.02))),
                MeshMaterial3d(opponent_ring),
                Transform::from_xyz(0.0, -CAR_HALF_HEIGHT + 0.02, 0.0),
            ));
            parent.spawn((
                Mesh3d(meshes.add(Sphere::new(0.16))),
                MeshMaterial3d(marker),
                Transform::from_xyz(0.0, CAR_HALF_HEIGHT + 0.55, 0.0),
            ));
        });
    }

    // --- lighting ---
    // Bright, fairly neutral stadium light: enough contrast to make the pitch
    // read as a lit field, with a cool fill so the slate walls keep their colour
    // instead of going black.
    commands.insert_resource(GlobalAmbientLight {
        color: Color::srgb(0.78, 0.82, 0.90),
        brightness: 180.0,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 9_000.0,
            shadow_maps_enabled: quality.shadows,
            ..default()
        },
        Transform::from_rotation(Quat::from_rotation_x(-PI / 3.5) * Quat::from_rotation_y(0.6)),
    ));
    // A second, shadowless light from the other side, so walls facing away
    // from the sun are not solid black.
    commands.spawn((
        DirectionalLight {
            illuminance: 3_500.0,
            shadow_maps_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_rotation_x(-PI / 6.0) * Quat::from_rotation_y(-2.4)),
    ));

    // --- camera ---
    // HDR, plus bloom when the quality preset keeps it on, so the emissive boost
    // pads glow. `--quality low` drops bloom along with MSAA and shadows.
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
            let heading = dir_from_yaw(
                base_yaw(
                    false,
                    car.forward,
                    car.up,
                    Vec3::ZERO,
                    None,
                    car.is_on_ground,
                )
                // At startup there is no previous heading to keep, so fall
                // back to the nose's horizontal projection (the car is
                // upright at kickoff).
                .unwrap_or_else(|| yaw_of(car.forward)),
            );
            let position = chase_position(car.pos, heading, settings.distance, settings.height);
            let view = view_direction(heading, settings.angle.to_radians());
            Transform::from_translation(position).looking_to(view, Vec3::Y)
        }
    };
    let mut camera = commands.spawn((
        Camera3d::default(),
        quality.msaa,
        Hdr,
        camera_pose,
        ChaseCam,
    ));
    if quality.bloom {
        camera.insert(Bloom {
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
        });
    }

    // --- hud ---
    // Only the readouts; the control bindings live in the README, not on screen.
    commands.spawn((
        Text::default(),
        Node {
            position_type: PositionType::Absolute,
            top: px(64),
            left: px(12),
            ..default()
        },
        children![
            TextSpan::new(""),
            TextSpan::new(""),
            TextSpan::new(""),
            TextSpan::new("")
        ],
        HudText,
    ));

    // RL's scoreboard sits top-centre: a team-coloured panel per side with the
    // goals scored. `update_hud` fills in the numbers.
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: px(12),
            left: px(0),
            right: px(0),
            justify_content: JustifyContent::Center,
            ..default()
        },
        children![(
            Node {
                flex_direction: FlexDirection::Row,
                align_items: AlignItems::Center,
                column_gap: px(8),
                ..default()
            },
            children![
                (
                    Node {
                        width: px(96),
                        height: px(44),
                        justify_content: JustifyContent::Center,
                        align_items: AlignItems::Center,
                        border_radius: BorderRadius::all(px(6)),
                        ..default()
                    },
                    BackgroundColor(Color::srgb(0.20, 0.45, 0.95)),
                    children![(
                        Text::new("0"),
                        TextFont::from_font_size(28.0),
                        TextColor(Color::WHITE),
                        BlueScoreText,
                    )],
                ),
                (
                    Text::new("-"),
                    TextFont::from_font_size(22.0),
                    TextColor(Color::srgba(0.85, 0.9, 1.0, 0.75)),
                ),
                (
                    Node {
                        width: px(96),
                        height: px(44),
                        justify_content: JustifyContent::Center,
                        align_items: AlignItems::Center,
                        border_radius: BorderRadius::all(px(6)),
                        ..default()
                    },
                    BackgroundColor(Color::srgb(1.00, 0.45, 0.12)),
                    children![(
                        Text::new("0"),
                        TextFont::from_font_size(28.0),
                        TextColor(Color::WHITE),
                        OrangeScoreText,
                    )],
                ),
            ],
        )],
    ));

    // RL's boost gauge lives in the bottom-right corner. The fill is a plain
    // node; `update_boost_meter` drives its width from the car's boost.
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            right: px(24),
            bottom: px(24),
            width: px(220),
            flex_direction: FlexDirection::Column,
            row_gap: px(6),
            ..default()
        },
        children![
            (
                Node {
                    width: percent(100),
                    justify_content: JustifyContent::SpaceBetween,
                    align_items: AlignItems::Center,
                    ..default()
                },
                children![
                    (
                        Text::new("BOOST"),
                        TextFont::from_font_size(13.0),
                        TextColor(Color::srgba(0.85, 0.9, 1.0, 0.75)),
                    ),
                    (
                        Text::new("100"),
                        TextFont::from_font_size(20.0),
                        BoostMeterText,
                    ),
                ],
            ),
            (
                Node {
                    width: percent(100),
                    height: px(18),
                    padding: UiRect::all(px(2)),
                    border: UiRect::all(px(2)),
                    border_radius: BorderRadius::all(px(6)),
                    ..default()
                },
                BackgroundColor(Color::srgba(0.02, 0.03, 0.05, 0.75)),
                BorderColor::all(Color::srgba(0.8, 0.85, 0.95, 0.55)),
                children![(
                    Node {
                        width: percent(100),
                        height: percent(100),
                        border_radius: BorderRadius::all(px(4)),
                        ..default()
                    },
                    BackgroundColor(Color::srgb(1.0, 0.62, 0.12)),
                    BoostMeterFill,
                )],
            ),
        ],
    ));
}

// ---------------------------------------------------------------------------
// Car visuals
// ---------------------------------------------------------------------------

/// Build one car's visual hierarchy and return its parent entity.
///
/// The local axes are (+Z forward, +Y up, +X right), matching the simulation,
/// with the origin at the hitbox centre. The body is a low wedge with the cabin
/// set back and a small rear wing - the Octane's silhouette in a dozen triangles -
/// built by extruding Z-Y profiles across the width. The roof reaches the hitbox
/// ceiling, the nose and tail reach its ends, and the sill sits clear of the
/// ground with the wheels filling the arches. The collision box is unchanged;
/// this is only what you see.
fn spawn_car(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    vis: Vis,
    car_index: usize,
    body_color: [f32; 4],
    spoiler_color: Color,
) -> Entity {
    let (hw, hh, hl) = (CAR_HALF_WIDTH, CAR_HALF_HEIGHT, CAR_HALF_LENGTH);
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
    let spoiler_mat = materials.add(StandardMaterial {
        base_color: spoiler_color,
        metallic: 0.35,
        perceptual_roughness: 0.4,
        ..default()
    });
    let flame_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.62, 0.18),
        // Above the bloom threshold, so the exhaust flares rather than just
        // reading as an orange cone - but not so hot that the halo swallows the
        // rear of the car.
        emissive: LinearRgba::rgb(10.0, 3.5, 0.4),
        ..default()
    });

    // A bare node: the parts hang off it so `sync_visuals` can drive the whole
    // car from the hitbox transform.
    let car = commands
        .spawn((Transform::default(), Visibility::default(), vis))
        .id();

    commands.entity(car).with_children(|parent| {
        // The sill is the underside of the bodywork. It sits at the wheel centre,
        // so the wheels stand proud of it - flush with the sill they would look
        // like castors, and the car would appear to have no ground clearance.
        let sill = -0.09_f32;
        let beltline = 0.10_f32; // the bonnet/boot top, where the glass begins

        // Lower body: one slab the full hitbox length. The nose is a long low
        // wedge and the tail falls away quickly, leaving a flat deck between the
        // cabin and the rear wing.
        parent.spawn((
            Mesh3d(meshes.add(extrude_profile(
                &[
                    (hl, sill),
                    (hl, -0.03),
                    (0.90 * hl, 0.005),
                    (0.60 * hl, beltline),
                    (-0.58 * hl, beltline),
                    (-0.84 * hl, 0.015),
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
                    body_color,
                    body_color,
                    underside_color,
                ],
            ))),
            MeshMaterial3d(car_mat.clone()),
            Transform::default(),
        ));
        // Cabin: the greenhouse, sitting on the beltline and narrower than the
        // body so its flanks show as glass, with a raked windscreen, a flat roof
        // on the hitbox ceiling and a fast rear window.
        parent.spawn((
            Mesh3d(meshes.add(extrude_profile(
                &[
                    (0.20 * hl, beltline),
                    (0.08 * hl, hh),
                    (-0.40 * hl, hh),
                    (-0.56 * hl, beltline),
                ],
                0.69 * hw,
                glass_color,
                &[glass_color, body_color, glass_color, body_color],
            ))),
            MeshMaterial3d(car_mat.clone()),
            Transform::default(),
        ));

        // Rear wing: the one silhouette cue that reads as an Octane rather than a
        // generic sedan. A bar on two short supports, all in the body colour.
        parent.spawn((
            Mesh3d(meshes.add(Cuboid::new(1.55 * hw, 0.025, 0.10))),
            MeshMaterial3d(spoiler_mat.clone()),
            Transform::from_xyz(0.0, beltline + 0.06, -0.86 * hl),
        ));
        for x in [-0.6 * hw, 0.6 * hw] {
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.035, 0.06, 0.04))),
                MeshMaterial3d(spoiler_mat.clone()),
                Transform::from_xyz(x, beltline + 0.03, -0.86 * hl),
            ));
        }

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
                    Wheel {
                        steers,
                        car: car_index,
                    },
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

        // Exhaust flames, hidden unless the car is actually boosting (see
        // `sync_visuals`). A cone points +Y; rotating it -90° about X sends the
        // tip backwards (-Z), with the wide base at the car. Two small cones read
        // as twin exhausts without adding much geometry.
        for x in [-0.16, 0.16] {
            parent.spawn((
                Mesh3d(meshes.add(Cone::new(0.06, 0.5))),
                MeshMaterial3d(flame_mat.clone()),
                Transform::from_xyz(x, -0.085, -hl - 0.18)
                    .with_rotation(Quat::from_rotation_x(-PI * 0.5)),
                Visibility::Hidden,
                BoostFlame { car: car_index },
            ));
        }
    });

    car
}

// ---------------------------------------------------------------------------
// Arena surface, pitch and net
// ---------------------------------------------------------------------------

/// The arena's collision surface as a single triangle mesh, read straight from
/// RocketSim (`sim::arena_collision_mesh`) - the same geometry RocketSim collides
/// against, so the drawn surface and the collision surface are one and the same.
/// Region colours are baked into the vertices: green where the surface faces up,
/// slate on the walls, darker slate on the roof, and a team-coloured band low on
/// each end wall so the two directions read at a glance.
///
/// The source triangles have no guaranteed winding, so each is oriented here so
/// its front face looks inward (toward the arena centre); the vertex normals are
/// the area-weighted average of those.
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

    let floor_color = linear_rgba(0.20, 0.46, 0.27);
    // The glass texture darkens the panels and brightens the lattice, so the
    // region colours here are the lattice's colour. The roof is slightly darker
    // than the walls, as in RL, but not black: with transparency the background
    // already supplies the darkness.
    let wall_color = linear_rgba(0.40, 0.45, 0.55);
    let ceiling_color = linear_rgba(0.30, 0.34, 0.42);
    let blue_team = linear_rgba(0.10, 0.30, 0.82);
    let orange_team = linear_rgba(0.92, 0.36, 0.08);

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

        // Chain-link UV: project onto the plane the vertex faces most directly.
        // The floor is covered by the pitch, so it does not matter that it gets
        // the same projection.
        let an = n.abs();
        let (u, w) = if an.y >= an.x && an.y >= an.z {
            (v.x, v.z)
        } else if an.x >= an.z {
            (v.z, v.y)
        } else {
            (v.x, v.y)
        };
        uvs.push([u / WALL_TEX_WORLD, w / WALL_TEX_WORLD]);

        // The roof uses the same lattice at a much larger scale, so the walls
        // read as fine mesh and the roof as big hex panels. The switch happens
        // on the roof fillet, out of the way at the top of the cage.
        let ceiling = smoothstep(0.55, 0.90, -n.y);
        if ceiling > 0.5 {
            // The projection in this branch is (x, z), so the scaling only needs
            // the roof's repeat.
            let i = uvs.len() - 1;
            uvs[i] = [u / CEILING_TEX_WORLD, w / CEILING_TEX_WORLD];
        }

        // Blend floor -> wall -> roof by how directly the surface faces up or
        // down. The fillets between them get a smooth transition for free.
        let floor = smoothstep(0.55, 0.90, n.y);
        let mut color = lerp_rgba(wall_color, floor_color, floor);
        color = lerp_rgba(color, ceiling_color, ceiling);

        // Team colour low on each end wall (and on the goal-recess floor). Only
        // surfaces that actually face along Z take it - the long side walls also
        // reach the ends, and tinting them smeared a band up each side. It fades
        // out toward the corners and the roof.
        let endness = ((v.z.abs() - 0.65 * ARENA_HALF_Z) / (0.35 * ARENA_HALF_Z)).clamp(0.0, 1.0);
        let low = ((7.0 - v.y) / 7.0).clamp(0.0, 1.0);
        // The goal-recess floor is upward-facing, so include it explicitly; it
        // otherwise stays grass-green inside the net.
        let in_recess = v.z.abs() > ARENA_HALF_Z && v.y < 0.5;
        let end_facing = smoothstep(0.55, 0.9, n.z.abs()).max(if in_recess { 1.0 } else { 0.0 });
        let accent = endness * low * end_facing * (1.0 - ceiling);
        let team = if v.z < 0.0 { blue_team } else { orange_team };
        color = lerp_rgba(color, team, 0.75 * accent);
        colors.push(color);
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
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
    .with_inserted_indices(Indices::U32(indices))
}

/// The pitch surface: mowed squares, white markings and team-tinted goal boxes,
/// all vertex-coloured quads a centimetre above the collision floor. No
/// texture - 80 mowed quads, a few line quads and one ring, still only a few
/// hundred triangles.
///
/// The layers are separated by a millimetre so the markings and the goal-box
/// fill never z-fight with the grass beneath them.
fn pitch_mesh() -> Mesh {
    let y_grass = PITCH_OFFSET;
    let y_box = PITCH_OFFSET + 0.001;
    let y_line = PITCH_OFFSET + 0.002;

    // Four green tones, not two. The checkerboard stays, but each square picks
    // one of two shades for its parity and darkens toward the touchlines, so the
    // pitch has the patchy, mown look of RL's turf instead of flat vector art.
    let light = [linear_rgba(0.30, 0.62, 0.35), linear_rgba(0.25, 0.56, 0.30)];
    let dark = [linear_rgba(0.15, 0.38, 0.21), linear_rgba(0.12, 0.33, 0.18)];
    let edge_green = linear_rgba(0.08, 0.25, 0.14);
    let line = linear_rgba(0.88, 0.93, 0.87);
    // Team targets for the end-zone wash and the painted apron outside the
    // touchlines. They are intentionally deeper than the goal-box fill: the box
    // is the strongest team colour, the apron the most graphic, and this wash the
    // transition between the coloured end and the green middle.
    let blue_wash = linear_rgba(0.10, 0.30, 0.50);
    let orange_wash = linear_rgba(0.62, 0.38, 0.12);
    let blue_apron = linear_rgba(0.08, 0.24, 0.54);
    let orange_apron = linear_rgba(0.66, 0.28, 0.05);
    // Goal-box fills, stronger at the goal line and fading toward the pitch so
    // they read as painted turf rather than a flat coloured slab.
    let blue_fill = linear_rgba(0.06, 0.24, 0.66);
    let blue_fill_front = linear_rgba(0.14, 0.39, 0.39);
    let orange_fill = linear_rgba(0.84, 0.30, 0.03);
    let orange_fill_front = linear_rgba(0.56, 0.43, 0.15);

    let mut pitch = PitchMesh::new();

    // Mowed squares. `PITCH_TILE` divides both pitch dimensions exactly, so the
    // checkerboard reaches the walls without a cut-off row.
    let nx = (2.0 * ARENA_HALF_X / PITCH_TILE).round() as i32;
    let nz = (2.0 * ARENA_HALF_Z / PITCH_TILE).round() as i32;
    for ix in 0..nx {
        for iz in 0..nz {
            let x0 = -ARENA_HALF_X + ix as f32 * PITCH_TILE;
            let z0 = -ARENA_HALF_Z + iz as f32 * PITCH_TILE;
            let parity = ((ix + iz) & 1) as usize;
            // A cheap deterministic hash picks one of the two shades for this
            // parity. It must not animate or use the RNG: the pitch is built once.
            let pick = ((ix.rem_euclid(7) * 3 + iz.rem_euclid(5)) & 1) as usize;
            let mut color = if parity == 0 { light[pick] } else { dark[pick] };
            // Darken the apron outside the touchline rather than tinting the
            // whole tile: the boundary sits only 1.2 m in, well inside a tile.
            let cx = (x0 + PITCH_TILE * 0.5).abs();
            let cz = (z0 + PITCH_TILE * 0.5).abs();
            let d = (ARENA_HALF_X - cx).min(ARENA_HALF_Z - cz);
            let edge = (1.0 - d / 12.0).clamp(0.0, 1.0);
            color = lerp_rgba(color, edge_green, 0.35 * smoothstep(0.0, 1.0, edge));
            // Fade each end toward its team colour. Doing it in the tile colours
            // keeps the mow pattern underneath and avoids a transparent overlay
            // layer on the pitch.
            let zc = z0 + PITCH_TILE * 0.5;
            let depth = (ARENA_HALF_Z - zc.abs()).max(0.0);
            let wash = (1.0 - depth / TEAM_WASH_DEPTH).clamp(0.0, 1.0) * TEAM_WASH_STRENGTH;
            let wash_color = if zc < 0.0 { blue_wash } else { orange_wash };
            color = lerp_rgba(color, wash_color, wash);
            pitch.rect(x0, z0, x0 + PITCH_TILE, z0 + PITCH_TILE, y_grass, color);
        }
    }

    let touch = ARENA_HALF_X - PITCH_INSET;
    let goal_line = ARENA_HALF_Z - PITCH_INSET;
    let goal_half_w = GOAL_HALF_WIDTH + GOAL_BOX_MARGIN;
    let goal_half_d = GOAL_BOX_DEPTH * 0.5;
    let goal_centre = goal_line - goal_half_d;

    // Painted apron between the touchline/goal line and the walls. It is split at
    // the halfway line on the sides (blue -Z, orange +Z) and team-coloured end to
    // end behind each goal, which is the strongest colour cue on the floor.
    let y_apron = PITCH_OFFSET + 0.0005;
    for (x0, x1) in [(-ARENA_HALF_X, -touch), (touch, ARENA_HALF_X)] {
        pitch.rect(x0, -ARENA_HALF_Z, x1, 0.0, y_apron, blue_apron);
        pitch.rect(x0, 0.0, x1, ARENA_HALF_Z, y_apron, orange_apron);
    }
    for (z0, z1, color) in [
        (-ARENA_HALF_Z, -goal_line, blue_apron),
        (goal_line, ARENA_HALF_Z, orange_apron),
    ] {
        pitch.rect(-touch, z0, touch, z1, y_apron, color);
    }

    // Team-tinted goal box floor, then the white markings on top. The tint is
    // strongest at the goal line and fades into the grass at the box's front.
    for (sign, back, front) in [
        (-1.0_f32, blue_fill, blue_fill_front),
        (1.0, orange_fill, orange_fill_front),
    ] {
        let zc = sign * goal_centre;
        // Corner order is (x0,z0), (x1,z0), (x1,z1), (x0,z1). The goal line is
        // the high-z edge for +Z and the low-z edge for -Z.
        let colors = if sign > 0.0 {
            [front, front, back, back]
        } else {
            [back, back, front, front]
        };
        pitch.rect_gradient(
            -goal_half_w,
            zc - goal_half_d,
            goal_half_w,
            zc + goal_half_d,
            y_box,
            colors,
        );
    }

    // Touchline and goal line: one inset rectangle around the pitch.
    pitch.rect_outline((0.0, 0.0), (touch, goal_line), y_line, line);

    // Halfway line, then the centre circle and spot. The line runs through the
    // painted apron to the boards, so the blue/orange split reads as deliberate.
    let half_line = PITCH_LINE_WIDTH * 0.5;
    pitch.rect(
        -ARENA_HALF_X,
        -half_line,
        ARENA_HALF_X,
        half_line,
        y_line,
        line,
    );
    pitch.ring(CENTER_CIRCLE_RADIUS, half_line, y_line, line);
    pitch.disc(0.30, y_line, line);

    // Goal box outlines: the outer edge lies on the goal line, on top of the
    // touchline (same colour, so the overlap is invisible).
    for sign in [-1.0_f32, 1.0] {
        pitch.rect_outline(
            (0.0, sign * goal_centre),
            (goal_half_w, goal_half_d),
            y_line,
            line,
        );
    }

    // Corner arcs, where the touchline turns into the goal line.
    let arc_radius = PITCH_CORNER_RADIUS;
    let arc_half = PITCH_LINE_WIDTH * 0.5;
    for (sx, sz) in [(-1.0_f32, -1.0_f32), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
        let centre = (sx * touch, sz * goal_line);
        // Pick the quarter of the circle that curves into the pitch.
        let (a0, a1) = match (sx > 0.0, sz > 0.0) {
            (true, true) => (PI, 1.5 * PI),
            (false, true) => (1.5 * PI, 2.0 * PI),
            (false, false) => (0.0, 0.5 * PI),
            (true, false) => (0.5 * PI, PI),
        };
        pitch.arc(centre, arc_radius, a0, a1, arc_half, y_line, line);
    }

    pitch.into_mesh()
}

/// Accumulates the pitch's upward-facing, vertex-coloured quads, so the mesh
/// builder reads as a list of pitches, lines and rings rather than a wall of
/// buffer arguments.
struct PitchMesh {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    colors: Vec<[f32; 4]>,
    indices: Vec<u32>,
}

impl PitchMesh {
    fn new() -> Self {
        Self {
            positions: Vec::new(),
            normals: Vec::new(),
            colors: Vec::new(),
            indices: Vec::new(),
        }
    }

    /// Append one upward-facing quad. `corners` must be in counter-clockwise
    /// order seen from above.
    fn quad(&mut self, corners: [Vec3; 4], color: [f32; 4]) {
        let base = self.positions.len() as u32;
        for p in corners {
            self.positions.push(p.to_array());
            self.normals.push([0.0, 1.0, 0.0]);
            self.colors.push(color);
        }
        self.indices
            .extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
    }

    fn rect(&mut self, x0: f32, z0: f32, x1: f32, z1: f32, y: f32, color: [f32; 4]) {
        self.quad(pitch_rect(x0, z0, x1, z1, y), color);
    }

    /// Like [`Self::rect`], but with one colour per corner. The corner order is
    /// `(x0,z0), (x1,z0), (x1,z1), (x0,z1)`.
    fn rect_gradient(&mut self, x0: f32, z0: f32, x1: f32, z1: f32, y: f32, colors: [[f32; 4]; 4]) {
        let base = self.positions.len() as u32;
        for (corner, color) in pitch_rect(x0, z0, x1, z1, y).into_iter().zip(colors) {
            self.positions.push(corner.to_array());
            self.normals.push([0.0, 1.0, 0.0]);
            self.colors.push(color);
        }
        self.indices
            .extend_from_slice(&[base, base + 2, base + 1, base, base + 3, base + 2]);
    }

    /// Outline a rectangle with four quads of width `PITCH_LINE_WIDTH`, centred
    /// on its edges. The long edges take the corners, so they meet cleanly
    /// without overlapping quads.
    fn rect_outline(
        &mut self,
        (cx, cz): (f32, f32),
        (hx, hz): (f32, f32),
        y: f32,
        color: [f32; 4],
    ) {
        let h = PITCH_LINE_WIDTH * 0.5;
        for z in [cz - hz, cz + hz] {
            self.rect(cx - hx - h, z - h, cx + hx + h, z + h, y, color);
        }
        for x in [cx - hx, cx + hx] {
            self.rect(x - h, cz - hz + h, x + h, cz + hz - h, y, color);
        }
    }

    /// A flat ring of `radius` and line width `2 * half_width`, as a strip of
    /// quads.
    fn ring(&mut self, radius: f32, half_width: f32, y: f32, color: [f32; 4]) {
        self.arc(
            (0.0, 0.0),
            radius,
            0.0,
            std::f32::consts::TAU,
            half_width,
            y,
            color,
        );
    }

    /// A flat arc of `radius`, from `a0` to `a1` radians, as a strip of quads.
    #[allow(clippy::too_many_arguments)]
    fn arc(
        &mut self,
        (cx, cz): (f32, f32),
        radius: f32,
        a0: f32,
        a1: f32,
        half_width: f32,
        y: f32,
        color: [f32; 4],
    ) {
        const SEGMENTS: usize = 48;
        let (inner, outer) = (radius - half_width, radius + half_width);
        // Keep the tessellation proportional for short corner arcs; a 48-segment
        // quarter circle is wasted geometry.
        let segments = (((a1 - a0).abs() / std::f32::consts::TAU) * SEGMENTS as f32)
            .ceil()
            .max(4.0) as usize;
        for i in 0..segments {
            let t0 = i as f32 / segments as f32;
            let t1 = (i + 1) as f32 / segments as f32;
            let astart = a0 + (a1 - a0) * t0;
            let aend = a0 + (a1 - a0) * t1;
            let (c0, s0) = (astart.cos(), astart.sin());
            let (c1, s1) = (aend.cos(), aend.sin());
            self.quad(
                [
                    Vec3::new(cx + inner * c0, y, cz + inner * s0),
                    Vec3::new(cx + outer * c0, y, cz + outer * s0),
                    Vec3::new(cx + outer * c1, y, cz + outer * s1),
                    Vec3::new(cx + inner * c1, y, cz + inner * s1),
                ],
                color,
            );
        }
    }

    /// A flat disc as a triangle fan.
    fn disc(&mut self, radius: f32, y: f32, color: [f32; 4]) {
        const SEGMENTS: usize = 24;
        let base = self.positions.len() as u32;
        self.positions.push([0.0, y, 0.0]);
        self.normals.push([0.0, 1.0, 0.0]);
        self.colors.push(color);
        for i in 0..SEGMENTS {
            let a = i as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
            self.positions.push([radius * a.cos(), y, radius * a.sin()]);
            self.normals.push([0.0, 1.0, 0.0]);
            self.colors.push(color);
        }
        for i in 0..SEGMENTS {
            let p0 = base + 1 + i as u32;
            let p1 = base + 1 + ((i + 1) % SEGMENTS) as u32;
            self.indices.extend_from_slice(&[base, p1, p0]);
        }
    }

    fn into_mesh(self) -> Mesh {
        Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, self.positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, self.normals)
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, self.colors)
        .with_inserted_indices(Indices::U32(self.indices))
    }
}

/// Corners of an axis-aligned rectangle in the XZ plane, counter-clockwise seen
/// from above (the winding [`PitchMesh::quad`] expects).
fn pitch_rect(x0: f32, z0: f32, x1: f32, z1: f32, y: f32) -> [Vec3; 4] {
    [
        Vec3::new(x0, y, z0),
        Vec3::new(x1, y, z0),
        Vec3::new(x1, y, z1),
        Vec3::new(x0, y, z1),
    ]
}

/// The structural grid on the inside of the walls and ceiling: shallow boxes
/// standing `GRID_DEPTH` proud of the collision surface, one mesh and one
/// material. The bays are sparse on purpose - a 10 m rhythm on the side walls
/// and a coarser ceiling grid - so they read as structure rather than noise.
/// The end-wall grid starts above the crossbar and never crosses the goal mouth.
fn arena_grid_mesh() -> Mesh {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    let half = GRID_BEAM * 0.5;

    // Side walls: nine uprights and three rails per wall.
    for side in [-1.0_f32, 1.0] {
        for k in -4..=4 {
            let z = k as f32 * PITCH_TILE;
            let (center, size) = side_wall_beam(side, 2.74, 17.74, z - half, z + half);
            push_box(&mut positions, &mut normals, &mut indices, center, size);
        }
        for y in [5.12, 10.24, 15.36] {
            let (center, size) = side_wall_beam(side, y - half, y + half, -46.08, 46.08);
            push_box(&mut positions, &mut normals, &mut indices, center, size);
        }
    }

    // Ceiling: a central grid, held clear of the wall fillets.
    for k in -3..=3 {
        let x = k as f32 * PITCH_TILE;
        let (center, size) = ceiling_beam(x - half, x + half, -46.08, 46.08);
        push_box(&mut positions, &mut normals, &mut indices, center, size);
    }
    for k in -4..=4 {
        let z = k as f32 * PITCH_TILE;
        let (center, size) = ceiling_beam(-30.72, 30.72, z - half, z + half);
        push_box(&mut positions, &mut normals, &mut indices, center, size);
    }

    // End walls: a grid above the crossbar only.
    for side in [-1.0_f32, 1.0] {
        for y in [8.0, 11.5, 15.0, 18.0] {
            let (center, size) = end_wall_beam(side, -30.72, 30.72, y - half, y + half);
            push_box(&mut positions, &mut normals, &mut indices, center, size);
        }
        for k in -3..=3 {
            let x = k as f32 * PITCH_TILE;
            let (center, size) = end_wall_beam(side, x - half, x + half, 8.0, 18.0);
            push_box(&mut positions, &mut normals, &mut indices, center, size);
        }
    }

    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_indices(Indices::U32(indices))
}

/// Center and size of a rib on a side wall. It is flush with the shell on the
/// outside and stands `GRID_DEPTH` proud on the inside.
fn side_wall_beam(side: f32, y0: f32, y1: f32, z0: f32, z1: f32) -> (Vec3, Vec3) {
    let center = Vec3::new(
        side * (ARENA_HALF_X - GRID_DEPTH * 0.5),
        (y0 + y1) * 0.5,
        (z0 + z1) * 0.5,
    );
    (center, Vec3::new(GRID_DEPTH, y1 - y0, z1 - z0))
}

/// Center and size of a rib on an end wall.
fn end_wall_beam(side: f32, x0: f32, x1: f32, y0: f32, y1: f32) -> (Vec3, Vec3) {
    let center = Vec3::new(
        (x0 + x1) * 0.5,
        (y0 + y1) * 0.5,
        side * (ARENA_HALF_Z - GRID_DEPTH * 0.5),
    );
    (center, Vec3::new(x1 - x0, y1 - y0, GRID_DEPTH))
}

/// Center and size of a rib on the ceiling.
fn ceiling_beam(x0: f32, x1: f32, z0: f32, z1: f32) -> (Vec3, Vec3) {
    let center = Vec3::new(
        (x0 + x1) * 0.5,
        ARENA_HEIGHT - GRID_DEPTH * 0.5,
        (z0 + z1) * 0.5,
    );
    (center, Vec3::new(x1 - x0, GRID_DEPTH, z1 - z0))
}

/// Append an axis-aligned box as six outward-facing quads.
fn push_box(
    positions: &mut Vec<[f32; 3]>,
    normals: &mut Vec<[f32; 3]>,
    indices: &mut Vec<u32>,
    center: Vec3,
    size: Vec3,
) {
    let h = size * 0.5;
    let (x0, y0, z0) = (center.x - h.x, center.y - h.y, center.z - h.z);
    let (x1, y1, z1) = (center.x + h.x, center.y + h.y, center.z + h.z);
    let faces = [
        (
            [
                Vec3::new(x1, y0, z0),
                Vec3::new(x1, y1, z0),
                Vec3::new(x1, y1, z1),
                Vec3::new(x1, y0, z1),
            ],
            Vec3::X,
        ),
        (
            [
                Vec3::new(x0, y0, z1),
                Vec3::new(x0, y1, z1),
                Vec3::new(x0, y1, z0),
                Vec3::new(x0, y0, z0),
            ],
            Vec3::NEG_X,
        ),
        (
            [
                Vec3::new(x0, y1, z0),
                Vec3::new(x0, y1, z1),
                Vec3::new(x1, y1, z1),
                Vec3::new(x1, y1, z0),
            ],
            Vec3::Y,
        ),
        (
            [
                Vec3::new(x0, y0, z1),
                Vec3::new(x0, y0, z0),
                Vec3::new(x1, y0, z0),
                Vec3::new(x1, y0, z1),
            ],
            Vec3::NEG_Y,
        ),
        (
            [
                Vec3::new(x1, y0, z1),
                Vec3::new(x1, y1, z1),
                Vec3::new(x0, y1, z1),
                Vec3::new(x0, y0, z1),
            ],
            Vec3::Z,
        ),
        (
            [
                Vec3::new(x0, y0, z0),
                Vec3::new(x0, y1, z0),
                Vec3::new(x1, y1, z0),
                Vec3::new(x1, y0, z0),
            ],
            Vec3::NEG_Z,
        ),
    ];
    for (corners, normal) in faces {
        push_detail_quad(positions, normals, indices, corners, normal);
    }
}

/// Append one flat quad with an explicit normal.
fn push_detail_quad(
    positions: &mut Vec<[f32; 3]>,
    normals: &mut Vec<[f32; 3]>,
    indices: &mut Vec<u32>,
    corners: [Vec3; 4],
    normal: Vec3,
) {
    let base = positions.len() as u32;
    for p in corners {
        positions.push(p.to_array());
        normals.push(normal.to_array());
    }
    indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
}

/// One point on the arena's base path, already inset from the wall by the
/// fillet profile. `inward` is the unit horizontal direction toward the pitch.
#[derive(Clone, Copy)]
struct PerimeterPoint {
    pos: Vec2,
    inward: Vec2,
}

/// A straight run or corner arc of the rounded-rectangle perimeter. Sections are
/// sampled independently: the goal mouths are gaps, so there is no single closed
/// polyline. The same section (and therefore the same point count) is sampled at
/// every height of the board profile, which keeps the quads connected.
enum PerimeterSection {
    Line {
        from: Vec2,
        to: Vec2,
        inward: Vec2,
        steps: usize,
    },
    Arc {
        center: Vec2,
        radius: f32,
        a0: f32,
        a1: f32,
        steps: usize,
    },
}

impl PerimeterSection {
    /// Sample this section `inset` metres in from the nominal wall plane. The
    /// corner radius shrinks by the inset, exactly as a parallel offset of a
    /// rounded rectangle does, so every profile level lands on the collision
    /// surface.
    fn sample(&self, inset: f32) -> Vec<PerimeterPoint> {
        match *self {
            Self::Line {
                from,
                to,
                inward,
                steps,
            } => {
                let from = from + inward * inset;
                let to = to + inward * inset;
                (0..=steps)
                    .map(|i| {
                        let t = i as f32 / steps as f32;
                        PerimeterPoint {
                            pos: from.lerp(to, t),
                            inward,
                        }
                    })
                    .collect()
            }
            Self::Arc {
                center,
                radius,
                a0,
                a1,
                steps,
            } => {
                let radius = (radius - inset).max(0.0);
                (0..=steps)
                    .map(|i| {
                        let t = i as f32 / steps as f32;
                        let angle = a0 + (a1 - a0) * t;
                        let (sin, cos) = angle.sin_cos();
                        let outward = Vec2::new(cos, sin);
                        PerimeterPoint {
                            pos: center + outward * radius,
                            inward: -outward,
                        }
                    })
                    .collect()
            }
        }
    }
}

/// The arena perimeter split into the straight side walls, the end-wall pieces
/// beside each goal mouth, and the four corner arcs. `BOARD_CORNER_RADIUS` is
/// where the straight walls stop, matching the collision mesh's rounded corners.
fn perimeter_sections() -> Vec<PerimeterSection> {
    let w = ARENA_HALF_X;
    let l = ARENA_HALF_Z;
    let r = BOARD_CORNER_RADIUS;
    // The tangency points where the straight walls become corner arcs.
    let sx = w - r;
    let sz = l - r;
    let goal = GOAL_HALF_WIDTH;
    // Roughly 4 m of board per segment on the sides, 2.5 m on the ends.
    let side_steps = 20;
    let end_steps = 8;
    let arc_steps = 6;
    vec![
        PerimeterSection::Line {
            from: Vec2::new(w, -sz),
            to: Vec2::new(w, sz),
            inward: Vec2::NEG_X,
            steps: side_steps,
        },
        PerimeterSection::Line {
            from: Vec2::new(-w, sz),
            to: Vec2::new(-w, -sz),
            inward: Vec2::X,
            steps: side_steps,
        },
        // +Z end wall, split at the goal mouth.
        PerimeterSection::Line {
            from: Vec2::new(sx, l),
            to: Vec2::new(goal, l),
            inward: Vec2::NEG_Y,
            steps: end_steps,
        },
        PerimeterSection::Line {
            from: Vec2::new(-goal, l),
            to: Vec2::new(-sx, l),
            inward: Vec2::NEG_Y,
            steps: end_steps,
        },
        // -Z end wall.
        PerimeterSection::Line {
            from: Vec2::new(goal, -l),
            to: Vec2::new(sx, -l),
            inward: Vec2::Y,
            steps: end_steps,
        },
        PerimeterSection::Line {
            from: Vec2::new(-sx, -l),
            to: Vec2::new(-goal, -l),
            inward: Vec2::Y,
            steps: end_steps,
        },
        // Corner arcs, wound so their inward normals meet the wall sections.
        PerimeterSection::Arc {
            center: Vec2::new(sx, sz),
            radius: r,
            a0: 0.0,
            a1: PI * 0.5,
            steps: arc_steps,
        },
        PerimeterSection::Arc {
            center: Vec2::new(sx, -sz),
            radius: r,
            a0: -PI * 0.5,
            a1: 0.0,
            steps: arc_steps,
        },
        PerimeterSection::Arc {
            center: Vec2::new(-sx, sz),
            radius: r,
            a0: PI * 0.5,
            a1: PI,
            steps: arc_steps,
        },
        PerimeterSection::Arc {
            center: Vec2::new(-sx, -sz),
            radius: r,
            a0: PI,
            a1: 1.5 * PI,
            steps: arc_steps,
        },
    ]
}

/// One cross-section of the base boards: height, horizontal inset from the wall
/// plane, and the surface normal's inward/up components. The profile is the
/// collision mesh's 2.5 m floor-to-wall fillet, so the boards lie on the ramp
/// instead of floating over it.
#[derive(Clone, Copy)]
struct BoardLevel {
    y: f32,
    inset: f32,
    normal_h: f32,
    normal_y: f32,
}

/// Number of strips in the team-coloured part of the board. The trim and rail
/// add one strip each above it.
const BOARD_BAND_STEPS: usize = 6;

fn board_levels() -> Vec<BoardLevel> {
    let r = BOARD_FILLET_RADIUS;
    let bottom = (1.0 - BOARD_BOTTOM / r).clamp(-1.0, 1.0).acos();
    let top = (1.0 - BOARD_TOP / r).clamp(-1.0, 1.0).acos();
    let mut levels = Vec::with_capacity(BOARD_BAND_STEPS + 3);
    for i in 0..=BOARD_BAND_STEPS {
        let t = i as f32 / BOARD_BAND_STEPS as f32;
        let theta = bottom + (top - bottom) * t;
        levels.push(BoardLevel {
            y: r * (1.0 - theta.cos()),
            inset: r * (1.0 - theta.sin()),
            normal_h: theta.sin(),
            normal_y: theta.cos(),
        });
    }
    // The trim and rail continue straight up from the top of the fillet, so
    // they keep the top level's inset and normal.
    let inset = r * (1.0 - top.sin());
    let (normal_h, normal_y) = (top.sin(), top.cos());
    levels.push(BoardLevel {
        y: BOARD_TRIM_TOP,
        inset,
        normal_h,
        normal_y,
    });
    levels.push(BoardLevel {
        y: BOARD_RAIL_TOP,
        inset,
        normal_h,
        normal_y,
    });
    levels
}

fn board_vertex(point: PerimeterPoint, level: BoardLevel) -> (Vec3, Vec3) {
    let normal = (Vec3::new(point.inward.x, 0.0, point.inward.y) * level.normal_h
        + Vec3::Y * level.normal_y)
        .normalize_or_zero();
    let pos = Vec3::new(point.pos.x, level.y, point.pos.y) + normal * BOARD_LIFT;
    (pos, normal)
}

/// The team-coloured boards that skirt the arena. They are vertex-coloured quads
/// following the wall fillet: blue on the -Z half, orange on the +Z half, with a
/// slightly darker alternating panel and a dark capping rail.
fn arena_boards_mesh() -> Mesh {
    let sections = perimeter_sections();
    let levels = board_levels();
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut colors = Vec::new();
    let mut indices = Vec::new();

    let blue = linear_rgba(0.10, 0.32, 0.78);
    let orange = linear_rgba(0.95, 0.38, 0.08);
    let trim = linear_rgba(0.74, 0.78, 0.82);
    let rail = linear_rgba(0.05, 0.06, 0.08);

    for (section_index, section) in sections.iter().enumerate() {
        // One polyline per profile level, all with the same point count.
        let paths: Vec<Vec<PerimeterPoint>> = levels
            .iter()
            .map(|level| section.sample(level.inset))
            .collect();
        for band in 0..levels.len() - 1 {
            let lower = &paths[band];
            let upper = &paths[band + 1];
            for i in 0..lower.len() - 1 {
                let color = if band < BOARD_BAND_STEPS {
                    let mid_z = (lower[i].pos.y + lower[i + 1].pos.y) * 0.5;
                    let team = if mid_z < 0.0 { blue } else { orange };
                    let shade = if (i + section_index) & 1 == 0 {
                        1.0
                    } else {
                        0.82
                    };
                    scale_rgba(team, shade)
                } else if band == BOARD_BAND_STEPS {
                    trim
                } else {
                    rail
                };
                let (p0, n0) = board_vertex(lower[i], levels[band]);
                let (p1, n1) = board_vertex(lower[i + 1], levels[band]);
                let (p2, n2) = board_vertex(upper[i + 1], levels[band + 1]);
                let (p3, n3) = board_vertex(upper[i], levels[band + 1]);
                let base = positions.len() as u32;
                for (p, n) in [(p0, n0), (p1, n1), (p2, n2), (p3, n3)] {
                    positions.push(p.to_array());
                    normals.push(n.to_array());
                    colors.push(color);
                }
                indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
            }
        }
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

/// The four floodlight masts, as (position, yaw) pairs. They stand just outside
/// the corners and their local -Z faces the arena centre.
fn light_tower_poses() -> [(Vec3, Quat); 4] {
    let mut poses = [(Vec3::ZERO, Quat::IDENTITY); 4];
    for (n, (sx, sz)) in [(-1.0_f32, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)]
        .into_iter()
        .enumerate()
    {
        let pos = Vec3::new(
            sx * (ARENA_HALF_X + LIGHT_TOWER_OFFSET),
            0.0,
            sz * (ARENA_HALF_Z + LIGHT_TOWER_OFFSET),
        );
        poses[n] = (pos, Quat::from_rotation_y(f32::atan2(sx, sz)));
    }
    poses
}

/// Dark structure of the corner floodlights: base, mast, crossbar and the
/// backing plate for the lamp array.
fn light_tower_frame_mesh() -> Mesh {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    let mast = LIGHT_TOWER_HEIGHT;
    let plate_y = mast - 1.0;
    for (pos, yaw) in light_tower_poses() {
        let place = |local: Vec3| pos + yaw * local;
        let plate_rotation = yaw * Quat::from_rotation_x(-0.28);
        push_oriented_box(
            &mut positions,
            &mut normals,
            &mut indices,
            place(Vec3::new(0.0, 0.3, 0.0)),
            Vec3::new(1.5, 0.6, 1.5),
            yaw,
        );
        push_oriented_box(
            &mut positions,
            &mut normals,
            &mut indices,
            place(Vec3::new(0.0, mast * 0.5, 0.0)),
            Vec3::new(0.46, mast, 0.46),
            yaw,
        );
        push_oriented_box(
            &mut positions,
            &mut normals,
            &mut indices,
            place(Vec3::new(0.0, mast - 1.4, 0.0)),
            Vec3::new(5.2, 0.26, 0.26),
            yaw,
        );
        push_oriented_box(
            &mut positions,
            &mut normals,
            &mut indices,
            place(Vec3::new(0.0, plate_y, 0.0)),
            Vec3::new(4.8, 1.6, 0.16),
            plate_rotation,
        );
    }
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_indices(Indices::U32(indices))
}

/// The six lamp heads on each floodlight, baked into one mesh so the whole set is
/// a single draw call. They are separate geometry because they use an emissive
/// material; the dark frame is the mesh above.
fn light_tower_lamp_mesh() -> Mesh {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    let mast = LIGHT_TOWER_HEIGHT;
    let plate_y = mast - 1.0;
    for (pos, yaw) in light_tower_poses() {
        let plate_center = pos + yaw * Vec3::new(0.0, plate_y, 0.0);
        let plate_rotation = yaw * Quat::from_rotation_x(-0.28);
        for dx in [-1.55_f32, 0.0, 1.55] {
            for dy in [-0.38_f32, 0.38] {
                let center = plate_center + plate_rotation * Vec3::new(dx, dy, -0.14);
                push_oriented_box(
                    &mut positions,
                    &mut normals,
                    &mut indices,
                    center,
                    Vec3::new(1.25, 0.5, 0.12),
                    plate_rotation,
                );
            }
        }
    }
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_indices(Indices::U32(indices))
}

/// Ceiling light banks: four lines of four short strips, so the roof reads as
/// recessed lighting rather than one fluorescent tube per line. Baked into one
/// mesh; the whole roof is a single draw call.
fn ceiling_lights_mesh() -> Mesh {
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    let y = ARENA_HEIGHT - 0.22;
    for x in [-30.72_f32, -10.24, 10.24, 30.72] {
        for z in [-30.0_f32, -10.0, 10.0, 30.0] {
            push_box(
                &mut positions,
                &mut normals,
                &mut indices,
                Vec3::new(x, y, z),
                Vec3::new(0.42, 0.10, 18.0),
            );
        }
    }
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_indices(Indices::U32(indices))
}

/// Append an axis-aligned box, rotated about its centre.
fn push_oriented_box(
    positions: &mut Vec<[f32; 3]>,
    normals: &mut Vec<[f32; 3]>,
    indices: &mut Vec<u32>,
    center: Vec3,
    size: Vec3,
    rotation: Quat,
) {
    let h = size * 0.5;
    let faces = [
        (
            [
                Vec3::new(h.x, -h.y, -h.z),
                Vec3::new(h.x, h.y, -h.z),
                Vec3::new(h.x, h.y, h.z),
                Vec3::new(h.x, -h.y, h.z),
            ],
            Vec3::X,
        ),
        (
            [
                Vec3::new(-h.x, -h.y, h.z),
                Vec3::new(-h.x, h.y, h.z),
                Vec3::new(-h.x, h.y, -h.z),
                Vec3::new(-h.x, -h.y, -h.z),
            ],
            Vec3::NEG_X,
        ),
        (
            [
                Vec3::new(-h.x, h.y, -h.z),
                Vec3::new(-h.x, h.y, h.z),
                Vec3::new(h.x, h.y, h.z),
                Vec3::new(h.x, h.y, -h.z),
            ],
            Vec3::Y,
        ),
        (
            [
                Vec3::new(-h.x, -h.y, h.z),
                Vec3::new(-h.x, -h.y, -h.z),
                Vec3::new(h.x, -h.y, -h.z),
                Vec3::new(h.x, -h.y, h.z),
            ],
            Vec3::NEG_Y,
        ),
        (
            [
                Vec3::new(h.x, -h.y, h.z),
                Vec3::new(h.x, h.y, h.z),
                Vec3::new(-h.x, h.y, h.z),
                Vec3::new(-h.x, -h.y, h.z),
            ],
            Vec3::Z,
        ),
        (
            [
                Vec3::new(-h.x, -h.y, -h.z),
                Vec3::new(-h.x, h.y, -h.z),
                Vec3::new(h.x, h.y, -h.z),
                Vec3::new(h.x, -h.y, -h.z),
            ],
            Vec3::NEG_Z,
        ),
    ];
    for (corners, normal) in faces {
        let corners = corners.map(|c| center + rotation * c);
        push_detail_quad(positions, normals, indices, corners, rotation * normal);
    }
}

/// The arena glass: a tiling hex lattice with the panels almost clear and the
/// lattice lines nearly solid. Untextured transparency reads as a flat tint and
/// disappears against the dark background; the hexes give the walls and roof the
/// structure RL's cage has without adding a single triangle.
///
/// The lattice is built for wrapping: 12 hex columns across the tile and 10 rows
/// down it, with alternating rows offset by half a cell. Sampling considers the
/// neighbouring centres with toroidal deltas, so a hex straddling the edge is
/// completed by its wrapped copy and the texture tiles without a seam.
fn arena_detail_texture() -> Image {
    const SIZE: usize = 512;
    /// Hex columns across the tile. Even, so a whole number of rows fits the
    /// alternating offset and the pattern is invariant after one wrap.
    const COLUMNS: f32 = 12.0;
    const ROWS: f32 = 10.0;
    /// Lattice line width, in texels.
    const LINE: f32 = 2.0;

    let span = SIZE as f32;
    let dx = span / COLUMNS;
    let dy = span / ROWS;
    let radius = dx / 1.5;
    // Distance from a flat-top hex's centre to the middle of an edge.
    let apothem = radius * 1.5_f32.sqrt();
    // Edge normals sit half a step between the vertex directions (0, 60, ...).
    let edge_normals: [[f32; 2]; 6] = std::array::from_fn(|k| {
        let angle = (30.0 + 60.0 * k as f32).to_radians();
        [angle.cos(), angle.sin()]
    });

    let mut data = vec![0u8; SIZE * SIZE * 4];
    for y in 0..SIZE {
        for x in 0..SIZE {
            let px = x as f32 + 0.5;
            let py = y as f32 + 0.5;
            let mut nearest = f32::MAX;

            // The point's own cell plus its eight neighbours covers every hex
            // whose boundary could be nearest; the toroidal deltas below also
            // bring in the copy wrapped from the other side of the tile.
            let column = (px / dx).floor() as i32;
            for i in (column - 1)..=(column + 1) {
                let offset = (i & 1) as f32 * 0.5;
                let row = (py / dy - offset).floor() as i32;
                for j in (row - 1)..=(row + 1) {
                    let cx = i as f32 * dx;
                    let cy = (j as f32 + offset) * dy;
                    // Wrap the delta into [-span/2, span/2) so the copy from the
                    // other side of the tile is considered.
                    let mut ox = px - cx;
                    ox -= span * (ox / span).round();
                    let mut oy = py - cy;
                    oy -= span * (oy / span).round();

                    // Signed distance to the hex boundary: the largest distance
                    // past any of the six edge planes.
                    let mut outside = f32::MIN;
                    for [nx, ny] in edge_normals {
                        outside = outside.max(ox * nx + oy * ny - apothem);
                    }
                    nearest = nearest.min(outside.abs());
                }
            }

            let edge = LINE * 0.5;
            // One-and-a-half texel feather. A hard threshold gives the lines a
            // beaded, stair-stepped edge when the camera is close to a wall; the
            // feather is cheap and the mip chain handles the rest.
            let t = ((edge - nearest) / 1.5 + 0.5).clamp(0.0, 1.0);
            let line_rgb = [218.0_f32, 226.0, 240.0];
            let panel_rgb = [52.0_f32, 62.0, 82.0];
            let rgb = [
                (panel_rgb[0] + (line_rgb[0] - panel_rgb[0]) * t) as u8,
                (panel_rgb[1] + (line_rgb[1] - panel_rgb[1]) * t) as u8,
                (panel_rgb[2] + (line_rgb[2] - panel_rgb[2]) * t) as u8,
            ];
            let alpha = (30.0 + (224.0 - 30.0) * t) as u8;
            let i = (y * SIZE + x) * 4;
            data[i] = rgb[0];
            data[i + 1] = rgb[1];
            data[i + 2] = rgb[2];
            data[i + 3] = alpha;
        }
    }

    // Build the mip chain by hand: runtime images start with one level, and a
    // repeating pattern without mips turns into noise on distant walls.
    let mut mip = data;
    let mut mip_size = SIZE;
    let mut pixels = Vec::new();
    loop {
        pixels.extend_from_slice(&mip);
        if mip_size == 1 {
            break;
        }
        let (next, next_size) = downsample_mip(&mip, mip_size);
        mip = next;
        mip_size = next_size;
    }

    let mut image = Image::new_uninit(
        Extent3d {
            width: SIZE as u32,
            height: SIZE as u32,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    );
    image.texture_descriptor.mip_level_count = SIZE.trailing_zeros() + 1;
    image.data = Some(pixels);
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::Repeat,
        address_mode_v: ImageAddressMode::Repeat,
        ..default()
    });
    image
}

/// Box-filter a square RGBA mip down by half, returning the new pixels and size.
fn downsample_mip(src: &[u8], size: usize) -> (Vec<u8>, usize) {
    let next = size / 2;
    let mut dst = vec![0u8; next * next * 4];
    for y in 0..next {
        for x in 0..next {
            let mut sum = [0u32; 4];
            for dy in 0..2 {
                for dx in 0..2 {
                    let i = ((y * 2 + dy) * size + x * 2 + dx) * 4;
                    for (c, s) in sum.iter_mut().enumerate() {
                        *s += src[i + c] as u32;
                    }
                }
            }
            let o = (y * next + x) * 4;
            for (c, s) in sum.iter().enumerate() {
                dst[o + c] = (s / 4) as u8;
            }
        }
    }
    (dst, next)
}

/// Bake a bottom-to-top RGBA gradient into a mesh's vertex colours. The lit
/// shader path replaces the material's `base_color` with the vertex colour
/// (alpha included), which is exactly what the unlit additive flames and beams
/// want: bright and solid at the base, transparent at the tip.
fn with_vertical_gradient(mut mesh: Mesh, bottom: [f32; 4], top: [f32; 4]) -> Mesh {
    let colors = {
        let Some(VertexAttributeValues::Float32x3(positions)) =
            mesh.attribute(Mesh::ATTRIBUTE_POSITION)
        else {
            return mesh;
        };
        let min = positions.iter().map(|p| p[1]).fold(f32::INFINITY, f32::min);
        let max = positions
            .iter()
            .map(|p| p[1])
            .fold(f32::NEG_INFINITY, f32::max);
        let span = (max - min).max(f32::EPSILON);
        positions
            .iter()
            .map(|p| {
                let t = ((p[1] - min) / span).clamp(0.0, 1.0);
                std::array::from_fn(|i| bottom[i] + (top[i] - bottom[i]) * t)
            })
            .collect::<Vec<[f32; 4]>>()
    };
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    mesh
}

/// The big pad's orb: a low-poly icosphere with low-frequency lumps pushed into
/// it, so the slow spin is visible and the surface reads as a blob of fire rather
/// than a billiard ball. Vertex colours run from molten yellow at the bottom to
/// charred near-black at the top, with a sooty low-frequency pattern. Normals
/// stay radial, which is correct enough for a star-shaped surface and keeps the
/// shading soft.
fn boost_orb_mesh(radius: f32) -> Mesh {
    let mut mesh: Mesh = Sphere::new(radius)
        .mesh()
        .ico(2)
        .expect("icosphere subdivision is valid");
    let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    else {
        return mesh;
    };
    let mut normals = Vec::with_capacity(positions.len());
    let mut colors = Vec::with_capacity(positions.len());
    for position in positions.iter_mut() {
        let dir = Vec3::from_array(*position).normalize_or_zero();
        let lump =
            1.0 + 0.11 * (dir.y * 3.1).sin() * (dir.x * 2.3).cos() + 0.06 * (dir.z * 4.7).cos();
        *position = (dir * radius * lump).to_array();
        normals.push(dir.to_array());

        let heat = ((dir.y + 1.0) * 0.5).powf(1.4);
        let soot = 0.55 + 0.45 * ((dir.x * 5.1).sin() * (dir.z * 4.3).cos()).abs();
        let dim = soot * (1.0 - 0.65 * heat);
        colors.push([
            (1.00 - 0.70 * heat) * dim,
            (0.55 - 0.50 * heat) * dim,
            (0.08 - 0.075 * heat) * dim,
            0.78,
        ]);
    }
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    mesh
}

/// A transparent cross-hatch used for the goal net. Transparent texels stay
/// white rather than black so linear filtering does not leave dark fringes, and
/// the sampler repeats it over each net quad.
fn net_texture() -> Image {
    const SIZE: usize = 64;
    const CELL: usize = 8;
    let mut data = vec![0u8; SIZE * SIZE * 4];
    for y in 0..SIZE {
        for x in 0..SIZE {
            let i = (y * SIZE + x) * 4;
            data[i] = 220;
            data[i + 1] = 226;
            data[i + 2] = 232;
            data[i + 3] = if x % CELL == 0 || y % CELL == 0 {
                210
            } else {
                0
            };
        }
    }
    let mut image = Image::new(
        Extent3d {
            width: SIZE as u32,
            height: SIZE as u32,
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

/// A quad in the XY plane facing +Z, with UVs scaled for a repeating texture.
/// Used for the net panels, which are then rotated into place.
fn quad_mesh(width: f32, height: f32, u_repeat: f32, v_repeat: f32) -> Mesh {
    let (hw, hh) = (width * 0.5, height * 0.5);
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(
        Mesh::ATTRIBUTE_POSITION,
        vec![
            [-hw, -hh, 0.0],
            [hw, -hh, 0.0],
            [hw, hh, 0.0],
            [-hw, hh, 0.0],
        ],
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0, 0.0, 1.0]; 4])
    .with_inserted_attribute(
        Mesh::ATTRIBUTE_UV_0,
        vec![
            [0.0, 0.0],
            [u_repeat, 0.0],
            [u_repeat, v_repeat],
            [0.0, v_repeat],
        ],
    )
    .with_inserted_indices(Indices::U32(vec![0, 1, 2, 0, 2, 3]))
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

/// Scale an RGBA colour's brightness without touching its alpha.
fn scale_rgba(c: [f32; 4], scale: f32) -> [f32; 4] {
    [c[0] * scale, c[1] * scale, c[2] * scale, c[3]]
}

/// Hermite ramp from 0 below `edge0` to 1 above `edge1`, clamped at both ends.
fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------

fn read_input(
    keys: &ButtonInput<KeyCode>,
    gamepad: Option<&Gamepad>,
    controls: &Controls,
) -> SimInput {
    let keyboard = &controls.keyboard;
    let keyboard_throttle = key_axis(
        keys,
        &keyboard.throttle_forward,
        &keyboard.throttle_backward,
    );
    let keyboard_steer = key_axis(keys, &keyboard.steer_left, &keyboard.steer_right);
    // Air roll, on dedicated keys: Q rolls left, E rolls right (positive `roll`
    // is left, the same sign convention as `steer` and `yaw`).
    let keyboard_roll = key_axis(keys, &keyboard.roll_left, &keyboard.roll_right);

    // Triggers are analogue, so they give proportional throttle rather than the
    // keyboard's full-on. The pad's *stick* axes carry steer and air pitch
    // (throttle is the triggers), which is how RL binds it.
    //
    // NOTE: a driver-default (unmapped) X-Box pad gets gilrs' built-in mapping,
    // where evdev's `BTN_NORTH`/`BTN_WEST` are the aliases of `BTN_X`/`BTN_Y`.
    // So the *physical X* button arrives as `North` and the *physical Y* as
    // `West`. Drift is therefore `North` (physical X, as the README says) and
    // the camera toggle is `West` (physical Y). On an SDL-mapped pad these two
    // are the other way round.
    let (pad_throttle, pad_steer, pad_pitch, pad_boost, pad_drift, pad_jump) = match gamepad {
        Some(pad) => {
            let gamepad = &controls.gamepad;
            (
                gamepad.throttle_forward.value(pad) - gamepad.throttle_backward.value(pad),
                // Sticks are positive to the right; `steer` is positive to the left.
                -pad.get(gamepad.steer_axis.0).unwrap_or(0.0) as f64,
                // Sticks are positive up here (see `look_input`); pitch +1 is nose
                // up, so the sign flips.
                -pad.get(gamepad.pitch_axis.0).unwrap_or(0.0) as f64,
                gamepad.boost.pressed(pad),
                gamepad.drift.pressed(pad),
                gamepad.jump.pressed(pad),
            )
        }
        None => (0.0, 0.0, 0.0, false, false, false),
    };

    // Dedicated air-roll buttons take priority over RL's drift + stick roll when
    // held, so a user can bind both and get the button they are pressing.
    let pad_button_roll = gamepad
        .map(|pad| {
            button_axis(
                pad,
                &controls.gamepad.roll_left,
                &controls.gamepad.roll_right,
            )
        })
        .unwrap_or(0.0);

    let throttle = strongest(keyboard_throttle, pad_throttle);
    let steer = strongest(keyboard_steer, pad_steer);
    let drift = keyboard.drift.pressed(keys) || pad_drift;

    // Air orientation. RL flies with the same stick it drives with: stick up is a
    // frontflip (`pitch -1`) and stick left yaws left. On the keyboard the
    // pitch buttons default to the drive keys; the pad uses the left stick's
    // vertical axis, which is why pitch is *not* taken from the throttle.
    //
    // `yaw` keeps `steer`'s sign, so the same key that steers left also yaws and
    // dodges left: RocketSim's positive steer/yaw/roll all go to the car's left
    // (measured by `examples/axis_probe.rs`), and the viewer's `steer` is already
    // positive to the left.
    let keyboard_pitch = -key_axis(keys, &keyboard.pitch_forward, &keyboard.pitch_backward);
    let pitch = strongest(keyboard_pitch, pad_pitch);
    let yaw = steer;

    // On the pad, air roll is the handbrake/drift button plus the stick, as in
    // Rocket League: drift + stick left rolls left (so it keeps `pad_steer`'s
    // sign). Dedicated roll buttons, if configured, win while one is held. The
    // keyboard uses the configured roll keys.
    let pad_roll = if pad_button_roll != 0.0 {
        pad_button_roll
    } else if pad_drift {
        pad_steer
    } else {
        0.0
    };
    let roll = strongest(keyboard_roll, pad_roll);

    SimInput::with_air(
        throttle,
        steer,
        pitch,
        yaw,
        roll,
        keyboard.boost.pressed(keys) || pad_boost,
        drift,
        keyboard.jump.pressed(keys) || pad_jump,
    )
}

/// Seed trigger axes on pads that have just connected.
///
/// See [`seed_trigger_axes`]: a pad's axes start at `0.0` until their first
/// event, which is half-pressed for a trigger that actually rests at `-1.0`.
fn seed_gamepad_triggers(
    mut gamepads: Query<&mut Gamepad, Added<Gamepad>>,
    controls: Res<Controls>,
) {
    for mut pad in &mut gamepads {
        seed_trigger_axes(&mut pad, &controls);
    }
}

#[allow(clippy::too_many_arguments)]
fn step_simulation(
    keys: Res<ButtonInput<KeyCode>>,
    gamepads: Query<&Gamepad>,
    controls: Res<Controls>,
    mut sim: ResMut<Sim>,
    mut history: ResMut<TickHistory>,
    mut boosting: ResMut<Boosting>,
    mut rlbot: ResMut<Rlbot>,
    mode: Res<Mode>,
) {
    rlbot.poll();

    let mut inputs = vec![SimInput::default(); sim.0.num_cars()];
    inputs[0] = match &*mode {
        Mode::Play => read_input(&keys, gamepads.iter().next(), &controls),
        // Self-driving, so a capture is reproducible rather than dependent on
        // the frame rate. See `capture_input` for the schedule.
        Mode::Capture { .. } => capture_input(sim.0.tick()),
    };
    if inputs.len() > 1 {
        let bot_player_id = rlbot.bot_player_id;
        if let Some(input) = rlbot.latest_input(bot_player_id) {
            inputs[1] = rlbot::to_sim_input(input);
        }
    }

    // The button alone is not enough: with an empty tank the car does not boost,
    // so the exhaust should not light either.
    for (index, input) in inputs.iter().enumerate() {
        boosting.0[index] = input.boost && sim.0.car_at(index).boost > 0.0;
    }
    sim.0.step(&inputs);
    // The renderer blends the previous and current tick; see `TickHistory`.
    history.push(&sim.0);

    // Only build (and especially predict) packets when a bot is actually ready
    // to receive them. The bridge opens the TCP port even before that, but an
    // unready client must not make the viewer pay for ball prediction.
    if rlbot.ready_count() > 0 {
        let wants_prediction = rlbot.any_wants_prediction();
        let snapshot = sim.0.rlbot_snapshot(wants_prediction);
        rlbot.broadcast(&snapshot);
    }
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
    controls: Res<Controls>,
    mut ball_cam: ResMut<BallCam>,
) {
    let pad_toggle = gamepads
        .iter()
        .any(|pad| controls.gamepad.toggle_camera.just_pressed(pad));
    if controls.keyboard.toggle_camera.just_pressed(&keys) || pad_toggle {
        ball_cam.0 = !ball_cam.0;
    }
}

fn reset_on_key(
    keys: Res<ButtonInput<KeyCode>>,
    gamepads: Query<&Gamepad>,
    controls: Res<Controls>,
    mut sim: ResMut<Sim>,
    mut history: ResMut<TickHistory>,
) {
    let pad_reset = gamepads
        .iter()
        .any(|pad| controls.gamepad.reset.just_pressed(pad));
    if controls.keyboard.reset.just_pressed(&keys) || pad_reset {
        sim.0.reset();
        history.snap(&sim.0);
    }
}

/// Viewer-level playback keys:
///
/// - `P` pauses/resumes;
/// - `+` (`NumpadAdd`, or shift+`=`) and `-` (`NumpadSubtract`, or the minus
///   key) move one rung along [`PLAYBACK_SPEEDS`];
/// - `=` (or `NumpadEqual`) returns to 1x.
///
/// Only in `Mode::Play`: a capture is driven by `capture_input` and must run to
/// its tick, so pausing one would stall the shot.
fn control_playback(
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<Mode>,
    mut playback: ResMut<Playback>,
    mut time: ResMut<Time<Virtual>>,
) {
    if !matches!(*mode, Mode::Play) {
        return;
    }

    if keys.just_pressed(KeyCode::KeyP) {
        playback.paused = !playback.paused;
        if playback.paused {
            info!("simulation paused (P to resume)");
        } else {
            info!("simulation resumed at {:.1}x", playback.speed());
        }
    }

    let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
    let faster =
        keys.just_pressed(KeyCode::NumpadAdd) || (shift && keys.just_pressed(KeyCode::Equal));
    let slower = keys.just_pressed(KeyCode::NumpadSubtract) || keys.just_pressed(KeyCode::Minus);
    let reset =
        keys.just_pressed(KeyCode::NumpadEqual) || (!shift && keys.just_pressed(KeyCode::Equal));

    let before = playback.speed_index;
    if faster {
        playback.faster();
    } else if slower {
        playback.slower();
    } else if reset {
        playback.reset_speed();
    }

    if playback.speed_index != before {
        let speed = playback.speed();
        // `Time<Virtual>` is what `Time<Fixed>` accumulates, so this changes the
        // number of simulation ticks per real second (and scales the render-side
        // `Time` too, which keeps the camera's smoothing in step with the car).
        time.set_relative_speed(speed);
        info!("game speed {speed:.1}x");
    }
}

/// Keep the bridge's event queue drained while paused.
///
/// `Rlbot::poll` normally runs inside `step_simulation`; while paused that system
/// is skipped, and the bridge's event channel is unbounded, so a long pause
/// would otherwise accumulate the bot's inputs (and hold its latest one).
fn poll_rlbot_while_paused(mut rlbot: ResMut<Rlbot>) {
    rlbot.poll();
}

// ---------------------------------------------------------------------------
// Presentation
// ---------------------------------------------------------------------------

/// Build a car's world rotation from the simulation's basis. The columns are
/// `(right, up, forward)`, so local `Z` is the nose and local `Y` is the roof.
fn car_rotation(car: CarView) -> Quat {
    Quat::from_mat3(&Mat3::from_cols(
        car.up.cross(car.forward),
        car.up,
        car.forward,
    ))
}

/// Blend one car pose between the last two ticks. Position is linear; attitude
/// is slerped, which keeps flips reading as rotations rather than squashes.
fn sample_car(previous: &CarView, current: &CarView, alpha: f32) -> CarView {
    let rotation = car_rotation(*previous).slerp(car_rotation(*current), alpha);
    CarView {
        pos: previous.pos.lerp(current.pos, alpha),
        forward: rotation * Vec3::Z,
        up: rotation * Vec3::Y,
        // Discrete state belongs to the newest tick: using a blend would make
        // the camera switch modes halfway between ticks.
        is_on_ground: current.is_on_ground,
        is_demoed: current.is_demoed,
        speed: previous.speed + (current.speed - previous.speed) * alpha,
        forward_speed: previous.forward_speed
            + (current.forward_speed - previous.forward_speed) * alpha,
        boost: current.boost,
    }
}

fn sample_ball(previous: &BallView, current: &BallView, alpha: f32) -> BallView {
    BallView {
        pos: previous.pos.lerp(current.pos, alpha),
        vel: previous.vel.lerp(current.vel, alpha),
        speed: previous.speed + (current.speed - previous.speed) * alpha,
        spin: current.spin,
    }
}

/// Sample the two most recent ticks for this frame.
///
/// The fixed clock's overstep is the fraction of a tick that has accumulated
/// since the last one, so `previous + alpha * (current - previous)` is the pose
/// at the frame's actual time. Paused frames and captures use `alpha = 1`:
/// a capture must render the exact tick it stopped on, and a paused sim must
/// not drift between the last two ticks.
fn sample_views(
    mode: Res<Mode>,
    playback: Res<Playback>,
    fixed: Res<Time<Fixed>>,
    history: Res<TickHistory>,
    mut views: ResMut<RenderViews>,
) {
    let alpha = if matches!(*mode, Mode::Play) && !playback.paused {
        fixed.overstep_fraction()
    } else {
        1.0
    };

    views.cars.clear();
    views.cars.extend(
        history
            .previous_cars
            .iter()
            .zip(&history.current_cars)
            .map(|(previous, current)| sample_car(previous, current, alpha)),
    );
    views.ball = Some(sample_ball(
        &history.previous_ball,
        &history.current_ball,
        alpha,
    ));
}

/// Deterministic pattern for the trail: evenly spaced angles (golden angle)
/// with hashed radius, length and opacity, so the streaks stay put from frame
/// to frame instead of shimmering.
fn trail_streaks() -> Vec<TrailStreak> {
    const GOLDEN_ANGLE: f32 = 2.399_963_2;

    let hash = |index: usize, salt: u32| {
        let mut x = (index as u32)
            .wrapping_mul(0x9E37_79B9)
            .wrapping_add(salt.wrapping_mul(0x85EB_CA6B));
        x ^= x >> 16;
        x = x.wrapping_mul(0x7FEB_352D);
        x ^= x >> 15;
        x = x.wrapping_mul(0x846C_A68B);
        x ^= x >> 16;
        (x & 0xFFFF) as f32 / 65_535.0
    };

    (0..BALL_TRAIL_STREAKS)
        .map(|index| TrailStreak {
            angle: index as f32 * GOLDEN_ANGLE,
            radius: 0.4 + hash(index, 1) * (BALL_TRAIL_SPREAD - 0.4),
            length_scale: 0.5 + hash(index, 2) * 0.5,
            alpha_scale: 0.45 + hash(index, 3) * 0.55,
        })
        .collect()
}

/// Build the trail geometry: one thin, camera-facing quad per streak, pointing
/// backwards along the ball's velocity. The head is bright white and the tail
/// fades to orange, matching RL's hard-hit streak.
fn trail_mesh_data(
    ball: &BallView,
    view: Vec3,
    strength: f32,
    streaks: &[TrailStreak],
) -> (Vec<[f32; 3]>, Vec<[f32; 4]>, Vec<u32>) {
    let mut positions = Vec::with_capacity(streaks.len() * 4);
    let mut colors = Vec::with_capacity(streaks.len() * 4);
    let mut indices = Vec::with_capacity(streaks.len() * 6);

    let direction = ball.vel.normalize_or_zero();
    if direction.length_squared() < 1e-6 {
        return (positions, colors, indices);
    }

    // A basis perpendicular to the motion, used to distribute the streaks
    // around the ball rather than stacking them on one line.
    let mut across = direction.cross(Vec3::Y).normalize_or_zero();
    if across.length_squared() < 1e-6 {
        across = direction.cross(Vec3::X).normalize_or_zero();
    }
    let up = direction.cross(across).normalize_or_zero();

    let view = view.normalize_or_zero();
    let length = (ball.speed * BALL_TRAIL_SECS).clamp(BALL_TRAIL_MIN_LENGTH, BALL_TRAIL_MAX_LENGTH);

    for (index, streak) in streaks.iter().enumerate() {
        let (sin, cos) = streak.angle.sin_cos();
        let start = ball.pos + (across * cos + up * sin) * streak.radius;
        let end = start - direction * (length * streak.length_scale);

        // Face the camera: the quad's width axis is perpendicular to both the
        // streak and the view.
        let mut side = direction.cross(view).normalize_or_zero();
        if side.length_squared() < 1e-6 {
            side = across;
        }

        let head = BALL_TRAIL_WIDTH;
        let tail = BALL_TRAIL_WIDTH * 0.15;
        let alpha = strength * streak.alpha_scale * BALL_TRAIL_ALPHA;
        let base = index as u32 * 4;
        positions.push((start - side * head).to_array());
        positions.push((start + side * head).to_array());
        positions.push((end - side * tail).to_array());
        positions.push((end + side * tail).to_array());
        // Additive: the tail's zero alpha is what makes the streak fade out
        // instead of ending in a hard edge.
        colors.push([1.0, 0.96, 0.88, alpha]);
        colors.push([1.0, 0.96, 0.88, alpha]);
        colors.push([1.0, 0.42, 0.10, 0.0]);
        colors.push([1.0, 0.42, 0.10, 0.0]);
        indices.extend_from_slice(&[base, base + 1, base + 2, base + 1, base + 3, base + 2]);
    }

    (positions, colors, indices)
}

/// Trail strength for a given ball speed: off below the minimum, full at the
/// top, linear in between so a medium hit gets a medium trail.
fn trail_target(speed: f32) -> f32 {
    ((speed - BALL_TRAIL_MIN_SPEED) / (BALL_TRAIL_FULL_SPEED - BALL_TRAIL_MIN_SPEED))
        .clamp(0.0, 1.0)
}

/// Rebuild the trail whenever the ball is fast enough to deserve one.
///
/// The effect is generated from the ball's instantaneous velocity, not its
/// history: RL's trail is a set of straight streaks aligned with the shot, not
/// a ribbon through the path. Time only eases the opacity, so pause freezes the
/// trail and slow motion fades it in more gently.
fn update_ball_trail(
    time: Res<Time>,
    playback: Res<Playback>,
    views: Res<RenderViews>,
    mut trail: ResMut<BallTrail>,
    mut meshes: ResMut<Assets<Mesh>>,
    cameras: Query<&GlobalTransform, With<Camera3d>>,
    mut visual: Query<&mut Visibility, With<BallTrailVisual>>,
) {
    let Some(ball) = views.ball else {
        return;
    };

    let dt = if playback.paused {
        0.0
    } else {
        time.delta_secs()
    };
    let target = trail_target(ball.speed);
    let response = 1.0 - (-BALL_TRAIL_RESPONSE * dt).exp();
    trail.strength += (target - trail.strength) * response;

    let active =
        trail.strength > 0.01 && ball.vel.length_squared() > 1e-6 && !trail.streaks.is_empty();
    for mut visibility in &mut visual {
        *visibility = if active {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
    }
    if !active {
        return;
    }

    let view = cameras
        .iter()
        .next()
        .map_or(Vec3::Z, |camera| camera.forward().as_vec3());
    let (positions, colors, indices) = trail_mesh_data(&ball, view, trail.strength, &trail.streaks);
    if positions.is_empty() {
        return;
    }
    let Some(mut mesh) = meshes.get_mut(&trail.mesh) else {
        return;
    };
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    mesh.insert_indices(Indices::U32(indices));
    // StandardMaterial's pipeline expects a normal attribute even when it is
    // unlit; without one the mesh is not drawn at all.
    mesh.compute_normals();
}

/// A demolished car is still in the world (and still simulated); hide it rather
/// than drawing an idle car that later teleports back to kickoff.
fn car_visibility(is_demoed: bool) -> Visibility {
    if is_demoed {
        Visibility::Hidden
    } else {
        Visibility::Inherited
    }
}

fn sync_visuals(
    sim: Res<Sim>,
    views: Res<RenderViews>,
    playback: Res<Playback>,
    time: Res<Time>,
    boosting: Res<Boosting>,
    mut query: Query<(&mut Transform, &Vis, &mut Visibility)>,
    mut flames: Query<(&mut Visibility, &BoostFlame), Without<Vis>>,
) {
    for (mut visibility, flame) in &mut flames {
        *visibility = if boosting.0.get(flame.car).copied().unwrap_or(false) {
            Visibility::Inherited
        } else {
            Visibility::Hidden
        };
    }

    // Ball spin is view-only and integrated per frame. Scaling by the virtual
    // delta makes it slow down with the game and stop when paused.
    let spin_dt = if playback.paused {
        0.0
    } else {
        time.delta_secs()
    };

    for (mut transform, vis, mut visibility) in &mut query {
        match vis {
            Vis::Car(index) => {
                let Some(car) = views.cars.get(*index).copied() else {
                    continue;
                };
                // RocketSim keeps a demolished car in the world until it
                // respawns. Hiding the parent also hides its wheels, exhaust,
                // opponent ring and marker, which all inherit the visibility.
                *visibility = car_visibility(car.is_demoed);
                transform.translation = car.pos;
                // Full 3D attitude, so flips and air control are visible.
                transform.rotation = car_rotation(car);
            }
            Vis::Ball => {
                let Some(ball) = views.ball else {
                    continue;
                };
                transform.translation = ball.pos;
                // Rolling looks wrong without spin, and the angle is free.
                transform.rotate(Quat::from_scaled_axis(ball.spin * spin_dt));
            }
            Vis::BallIndicator => {
                let Some(ball) = views.ball else {
                    continue;
                };
                // Project onto the pitch; the height of the ball is irrelevant,
                // which is the point of the indicator.
                transform.translation = Vec3::new(ball.pos.x, BALL_INDICATOR_Y, ball.pos.z);
                *visibility = Visibility::Inherited;
            }
            Vis::BoostPad(index) => {
                // Hidden while on cooldown. For a small pad that is its glow; the
                // marker carries no `Vis` and stays. A big pad's dark platform
                // does the same.
                *visibility = if sim.0.pad(*index).available {
                    Visibility::Inherited
                } else {
                    Visibility::Hidden
                };
            }
        }
    }
}

/// Bob and spin the big pads' orbs, and let the small pads' flame panels breathe.
/// RL's pads are never quite still, and the motion is what separates them from
/// painted plates. Phases are offset per pad (see `setup_scene`) so the field
/// does not pulse in lockstep.
fn animate_boost_pads(
    time: Res<Time>,
    mut orbs: Query<(&mut Transform, &BoostOrb), Without<BoostPulse>>,
    mut pulses: Query<(&mut Transform, &BoostPulse), Without<BoostOrb>>,
) {
    let t = time.elapsed_secs();
    for (mut transform, orb) in &mut orbs {
        let bob = (t * std::f32::consts::TAU / BOOST_BOB_PERIOD + orb.phase).sin();
        transform.translation.y = orb.base_y + bob * BOOST_BOB_AMPLITUDE;
        transform.rotation = Quat::from_rotation_y(t * BOOST_ORB_SPIN + orb.phase);
    }
    for (mut transform, pulse) in &mut pulses {
        let breathe = (t * std::f32::consts::TAU / BOOST_PULSE_PERIOD + pulse.phase).sin();
        transform.scale = Vec3::splat(1.0 + breathe * BOOST_PULSE_AMOUNT);
    }
}

/// How fast the wheels should be rolling, as a signed ground speed. On the
/// ground that is the car's forward speed; in the air there is no contact patch,
/// so they free-spin with the throttle instead (a flip with the stick held still
/// turns them, which is what sells the manoeuvre).
fn wheel_roll_speed(car: CarView, input: SimInput) -> f32 {
    if car.is_on_ground {
        car.forward_speed
    } else {
        input.throttle as f32 * AIR_WHEEL_SPIN_SPEED
    }
}

/// Roll and steer the visual wheels. View-only: the simulation has no wheels, so
/// the roll integrates the distance the car has travelled over the wheel radius,
/// and the steer angle comes from the last input.
fn spin_wheels(
    time: Res<Time>,
    playback: Res<Playback>,
    sim: Res<Sim>,
    mut rolled: Local<[f32; 2]>,
    mut wheels: Query<(&mut Transform, &Wheel)>,
) {
    // Frozen sim means frozen wheels, even if the car was moving when paused.
    let dt = if playback.paused {
        0.0
    } else {
        time.delta_secs()
    };
    for (mut transform, wheel) in &mut wheels {
        let car = sim.0.car_at(wheel.car);
        let input = sim.0.last_input(wheel.car);
        rolled[wheel.car] = (rolled[wheel.car] + wheel_roll_speed(car, input) * dt / WHEEL_RADIUS)
            % std::f32::consts::TAU;
        let steer_angle = input.steer as f32 * MAX_WHEEL_STEER;
        let yaw = if wheel.steers { steer_angle } else { 0.0 };
        transform.rotation = Quat::from_rotation_y(yaw) * Quat::from_rotation_x(rolled[wheel.car]);
    }
}

/// Look input as two scalars in `-1..=1`, merged from the pad's look stick and
/// the keyboard's look keys. `x` is positive to the right; `y` is the pitch
/// input, positive to look up. `invert_y` flips the vertical axis (the viewer's
/// default), so pushing the stick up looks down.
fn look_input(
    keys: &ButtonInput<KeyCode>,
    gamepad: Option<&Gamepad>,
    controls: &Controls,
    invert_y: bool,
) -> (f64, f64) {
    let keyboard = &controls.keyboard;
    let key_x = key_axis(keys, &keyboard.look_right, &keyboard.look_left);
    let key_y = key_axis(keys, &keyboard.look_up, &keyboard.look_down);
    let (pad_x, pad_y) = match gamepad {
        Some(pad) => {
            let gamepad = &controls.gamepad;
            (
                pad.get(gamepad.look_x_axis.0).unwrap_or(0.0) as f64,
                pad.get(gamepad.look_y_axis.0).unwrap_or(0.0) as f64,
            )
        }
        None => (0.0, 0.0),
    };
    let y = strongest(key_y, pad_y);
    let y = if invert_y { -y } else { y };
    (strongest(key_x, pad_x), y)
}

/// Whether the look-behind control is held. In Rocket League that is pushing the
/// right stick in - the "click", R3 - but it can also be bound to a keyboard key.
fn look_behind(
    keys: &ButtonInput<KeyCode>,
    gamepad: Option<&Gamepad>,
    controls: &Controls,
) -> bool {
    controls.keyboard.look_behind.pressed(keys)
        || gamepad.is_some_and(|pad| controls.gamepad.look_behind.pressed(pad))
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

/// Yaw the camera's horizontal heading should follow: the car's heading in car
/// cam, or the bearing to the ball in ball cam. `None` means the input gives no
/// usable heading, so the caller keeps its previous yaw rather than snapping.
///
/// Car cam cannot use the horizontal projection of the nose alone: a front or
/// back flip pitches the nose through vertical, where that projection collapses
/// and then reverses, which would whip the camera through 180°. It cannot use
/// the car's right axis alone either: a side flip or air roll spins that axis.
/// The two axes are orthogonal, so take the yaw each implies and follow the one
/// closest to the heading we already had. During a front/back flip the nose is
/// reversed but the right axis is steady; during a side flip/roll the nose is
/// steady and the right axis spins. This also keeps a nose-up wall stable.
///
/// None of that applies while the car is tilted in the air: a dodge rotates the
/// whole car about a horizontal axis, so *both* axes swing even though its yaw
/// (and RL's camera) does not. `on_ground` is what separates a wall, where the
/// two axes are still meaningful, from a flip; while airborne and tilted the
/// caller holds its heading, and tracking resumes as the car levels out.
fn base_yaw(
    ball_cam: bool,
    car_forward: Vec3,
    car_up: Vec3,
    to_ball: Vec3,
    current: Option<f32>,
    on_ground: bool,
) -> Option<f32> {
    if ball_cam {
        let dir = Vec3::new(to_ball.x, 0.0, to_ball.z);
        return (dir.length_squared() > 1e-6).then(|| yaw_of(dir));
    }

    if !on_ground && car_up.y < CAR_UPRIGHT_MIN_UP_Y {
        return None;
    }

    let right = car_up.cross(car_forward);
    // The yaw implied by an axis is the direction that axis points when the
    // car is upright: forward itself, and `right` rotated a quarter turn.
    let candidates = [
        Vec3::new(car_forward.x, 0.0, car_forward.z),
        Vec3::new(-right.z, 0.0, right.x),
    ];
    candidates
        .into_iter()
        .filter(|dir| dir.length_squared() > CAR_HEADING_MIN_HORIZONTAL.powi(2))
        .map(yaw_of)
        // Of the valid candidates, take the one nearest the heading we already
        // had; that picks the un-reversed axis through a flip. Without a current
        // heading (startup), the first valid candidate wins.
        .min_by(|a, b| {
            let distance = |yaw: f32| current.map_or(0.0, |c| shortest_angle(yaw - c).abs());
            distance(*a).total_cmp(&distance(*b))
        })
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
    if *since_check < SETTINGS_RELOAD_CHECK_SECS {
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

/// Hot-reload `controls.toml` when it changes. The old bindings stay in place
/// until the new file parses, so a half-saved edit cannot leave the car without
/// controls.
fn reload_controls(
    time: Res<Time>,
    mut config: ResMut<ControlsConfig>,
    mut controls: ResMut<Controls>,
    mut gamepads: Query<&mut Gamepad>,
    mut since_check: Local<f32>,
) {
    *since_check += time.delta_secs();
    if *since_check < SETTINGS_RELOAD_CHECK_SECS {
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
    match read_controls(&config.path) {
        Ok(new_controls) => {
            info!("controls: reloaded {}", config.path.display());
            *controls = new_controls;
            // A newly configured trigger axis may still be at its synthetic
            // 0.0 start value; give it the trigger rest value before it is read.
            for mut pad in &mut gamepads {
                seed_trigger_axes(&mut pad, &controls);
            }
        }
        Err(e) => error!("controls: reload failed: {e}"),
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
    controls: Res<Controls>,
    views: Res<RenderViews>,
    ball_cam: Res<BallCam>,
    settings: Res<CameraSettings>,
    time: Res<Time>,
    mut query: Query<&mut Transform, With<ChaseCam>>,
    mut heading: Local<Option<f32>>,
    mut swivel: Local<f32>,
    mut behind_last: Local<bool>,
) {
    // Follow the interpolated pose, not the newest tick, or the car would move
    // smoothly while the camera tracked a stuttering target.
    let Some(car) = views.cars.first().copied() else {
        return;
    };
    let Some(ball) = views.ball else {
        return;
    };
    let ball_pos = ball.pos;
    let car_pos = car.pos;
    let dt = time.delta_secs();

    let pad = gamepads.iter().next();
    let behind = look_behind(&keys, pad, &controls);
    // Look-behind is a hard 180° flip in RL: snap the pose on both edges rather
    // than easing the boom through the car.
    let snap = behind != *behind_last;
    *behind_last = behind;
    let (mut look_x, look_y) = look_input(&keys, pad, &controls, settings.invert_look_y);
    if settings.invert_swivel {
        look_x = -look_x;
    }

    // The heading is horizontal in both modes: car cam trails the car's heading,
    // ball cam looks along the bearing to the ball. This is the whole difference
    // from the old car-frame boom - the camera never rolls with the car, and on a
    // wall the ball still steers the camera around the field, as in RL.
    let to_ball = ball_pos - car_pos;
    let target_yaw = base_yaw(
        ball_cam.0,
        car.forward,
        car.up,
        to_ball,
        *heading,
        car.is_on_ground,
    );

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

/// Collect frame times for the `--stats` overlay. `Time<Real>` is wall-clock
/// time, so this measures the interval the display actually sees, not just the
/// time spent inside the ECS schedules.
fn track_performance(
    time: Res<Time<Real>>,
    keys: Res<ButtonInput<KeyCode>>,
    window: Single<&Window, With<PrimaryWindow>>,
    mut perf: ResMut<PerfStats>,
) {
    if keys.just_pressed(KeyCode::F3) {
        perf.visible = !perf.visible;
    }
    if time.elapsed_secs() < PERF_WARMUP_SECS {
        return;
    }

    let dt_ms = time.delta_secs() * 1000.0;
    if dt_ms <= 0.0 || !dt_ms.is_finite() {
        return;
    }

    perf.frames.push_back(dt_ms);
    perf.window_ms += dt_ms;
    while perf.window_ms > PERF_WINDOW_SECS * 1000.0 {
        match perf.frames.pop_front() {
            Some(old) => perf.window_ms -= old,
            None => {
                perf.window_ms = 0.0;
                break;
            }
        }
    }

    perf.since_refresh += dt_ms / 1000.0;
    if perf.since_refresh < PERF_REFRESH_SECS || perf.frames.is_empty() {
        return;
    }
    perf.since_refresh = 0.0;

    let count = perf.frames.len();
    let avg_ms = perf.frames.iter().sum::<f32>() / count as f32;
    let max_ms = perf.frames.iter().copied().fold(0.0, f32::max);

    // 1% low: the mean of the slowest 1% of frames (at least one), which is
    // steadier than reading the single worst frame.
    let mut sorted: Vec<f32> = perf.frames.iter().copied().collect();
    sorted.sort_by(f32::total_cmp);
    let slowest = (count / 100).max(1);
    let low_ms = sorted[count - slowest..].iter().sum::<f32>() / slowest as f32;

    perf.avg_ms = avg_ms;
    perf.fps = 1000.0 / avg_ms.max(0.001);
    perf.max_ms = max_ms;
    perf.low_fps = 1000.0 / low_ms.max(0.001);
    perf.width = window.physical_width();
    perf.height = window.physical_height();
}

#[allow(clippy::too_many_arguments)]
fn update_hud(
    sim: Res<Sim>,
    ball_cam: Res<BallCam>,
    playback: Res<Playback>,
    perf: Res<PerfStats>,
    hud: Single<Entity, With<HudText>>,
    blue_score: Single<Entity, With<BlueScoreText>>,
    orange_score: Single<Entity, With<OrangeScoreText>>,
    mut writer: TextUiWriter,
) {
    let entity = *hud;
    let car = sim.0.car();
    let ball = sim.0.ball();
    let score = sim.0.score();
    *writer.text(*blue_score, 0) = score[0].to_string();
    *writer.text(*orange_score, 0) = score[1].to_string();

    // Only shout about playback when it is not plain 1x running, so the normal
    // HUD stays uncluttered.
    let mut first_line = format!(
        "speed {:>5.1} m/s   tick {:>6}   cam {}",
        car.speed,
        sim.0.tick(),
        if ball_cam.0 { "ball" } else { "car" }
    );
    if playback.paused {
        first_line.push_str("   PAUSED");
    }
    if playback.speed() != PLAYBACK_DEFAULT_SPEED {
        first_line.push_str(&format!("   {:.1}x", playback.speed()));
    }
    first_line.push('\n');
    *writer.text(entity, 1) = first_line;
    *writer.text(entity, 2) = format!(
        "ball {:>5.1} m/s   height {:>5.2} m   spin {:>5.1} rad/s\n",
        ball.speed,
        ball.pos.y,
        ball.spin.length()
    );
    *writer.text(entity, 3) = if sim.0.num_cars() > 1 {
        let bot = sim.0.car_at(1);
        format!("bot  {:>5.1} m/s   boost {:>3.0}\n", bot.speed, bot.boost)
    } else {
        String::new()
    };
    *writer.text(entity, 4) = if !perf.visible {
        String::new()
    } else if perf.fps > 0.0 {
        format!(
            "fps {:>5.0}   frame {:>5.1} ms   max {:>5.1} ms   1% low {:>5.0} fps   {}x{}\n",
            perf.fps, perf.avg_ms, perf.max_ms, perf.low_fps, perf.width, perf.height
        )
    } else {
        "stats: warming up\n".to_string()
    };
}

/// Fill the bottom-right bar and its number from the car's boost (0..100).
fn update_boost_meter(
    sim: Res<Sim>,
    mut fill: Single<&mut Node, With<BoostMeterFill>>,
    mut text: Single<&mut Text, With<BoostMeterText>>,
) {
    let boost = sim.0.car().boost.clamp(0.0, 100.0);
    fill.width = percent(boost);
    text.0 = format!("{boost:.0}");
}

fn quit_on_escape(
    keys: Res<ButtonInput<KeyCode>>,
    gamepads: Query<&Gamepad>,
    controls: Res<Controls>,
    mut exit: MessageWriter<AppExit>,
) {
    let pad_quit = gamepads
        .iter()
        .any(|pad| controls.gamepad.quit.just_pressed(pad));
    if controls.keyboard.quit.just_pressed(&keys) || pad_quit {
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
    fn quality_presets_and_msaa_override_parse() {
        assert_eq!(parse_quality("LOW"), QualityPreset::Low);
        assert_eq!(parse_quality("medium"), QualityPreset::Medium);
        assert_eq!(parse_quality("high"), QualityPreset::High);
        assert_eq!(parse_msaa("off"), Msaa::Off);
        assert_eq!(parse_msaa("0"), Msaa::Off);
        assert_eq!(parse_msaa("2"), Msaa::Sample2);
        assert_eq!(parse_msaa("4"), Msaa::Sample4);
        assert_eq!(parse_msaa("8"), Msaa::Sample8);

        let low = RenderQuality::new(QualityPreset::Low, None);
        assert_eq!(low.msaa, Msaa::Off);
        assert!(!low.shadows);
        assert!(!low.bloom);

        let high = RenderQuality::new(QualityPreset::High, None);
        assert_eq!(high.msaa, Msaa::Sample4);
        assert!(high.shadows);
        assert!(high.bloom);

        // The override changes MSAA only; the preset still owns shadows/bloom.
        let overridden = RenderQuality::new(QualityPreset::High, Some(Msaa::Off));
        assert_eq!(overridden.msaa, Msaa::Off);
        assert!(overridden.shadows);
        assert!(overridden.bloom);
    }

    #[test]
    fn resolution_argument_parses_both_separators() {
        assert_eq!(parse_resolution("1920x1080"), (1920, 1080));
        assert_eq!(parse_resolution("2560X1440"), (2560, 1440));
        assert_eq!(parse_resolution(" 3840x2160 "), (3840, 2160));
    }

    #[test]
    fn playback_speeds_are_ascending_and_include_one_x() {
        assert!(PLAYBACK_SPEEDS.windows(2).all(|pair| pair[0] < pair[1]));
        assert_eq!(
            PLAYBACK_SPEEDS[Playback::default_speed_index()],
            PLAYBACK_DEFAULT_SPEED
        );
    }

    #[test]
    fn playback_speed_walks_below_one_x_in_small_steps() {
        // Regression: a fixed 0.5x step stopped at 0.5x after one press of `-`.
        let mut playback = Playback::default();
        assert_eq!(playback.speed(), 1.0);

        let mut seen = Vec::new();
        for _ in 0..PLAYBACK_SPEEDS.len() {
            playback.slower();
            seen.push(playback.speed());
        }

        assert_eq!(&seen[..9], &[0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]);
        assert_eq!(playback.speed(), 0.1);

        // The bottom rung is sticky.
        playback.slower();
        assert_eq!(playback.speed(), 0.1);
    }

    #[test]
    fn playback_speed_walks_up_from_one_x() {
        // Regression: `+` used to jump straight from 1x to 1.5x.
        let mut playback = Playback::default();
        playback.faster();
        assert_eq!(playback.speed(), 1.1);

        let mut seen = Vec::new();
        for _ in 0..PLAYBACK_SPEEDS.len() {
            playback.faster();
            seen.push(playback.speed());
        }

        assert_eq!(&seen[..4], &[1.2, 1.3, 1.4, 1.5]);
        assert_eq!(playback.speed(), 10.0);

        // The top rung is sticky.
        playback.faster();
        assert_eq!(playback.speed(), 10.0);
    }

    fn pose_car(pos: Vec3, forward: Vec3, up: Vec3, speed: f32) -> CarView {
        CarView {
            pos,
            forward,
            up,
            is_on_ground: true,
            is_demoed: false,
            speed,
            forward_speed: speed,
            boost: 0.0,
        }
    }

    #[test]
    fn car_interpolation_blends_position_and_attitude() {
        // Tick A faces +Z; tick B has moved +2 on X and yawed 90 degrees.
        let previous = pose_car(Vec3::ZERO, Vec3::Z, Vec3::Y, 0.0);
        let current = pose_car(Vec3::new(2.0, 0.0, 0.0), Vec3::X, Vec3::Y, 4.0);

        let start = sample_car(&previous, &current, 0.0);
        assert!((start.pos - previous.pos).length() < 1e-6);
        assert!((start.forward - previous.forward).length() < 1e-6);

        let end = sample_car(&previous, &current, 1.0);
        assert!((end.pos - current.pos).length() < 1e-6);
        assert!((end.forward - current.forward).length() < 1e-6);

        let half = sample_car(&previous, &current, 0.5);
        assert!((half.pos - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-6);
        assert!((half.speed - 2.0).abs() < 1e-6);
        // Slerp, not lerp: half way is a 45 degree yaw, so the nose still points
        // between the two ticks instead of collapsing toward the origin.
        let expected = Quat::from_rotation_y(PI / 4.0) * Vec3::Z;
        assert!(
            (half.forward - expected).length() < 1e-5,
            "half-way forward {:?}, expected {expected:?}",
            half.forward
        );
        assert!((half.up - Vec3::Y).length() < 1e-6);
    }

    #[test]
    fn demolished_cars_are_hidden() {
        assert_eq!(car_visibility(true), Visibility::Hidden);
        assert_eq!(car_visibility(false), Visibility::Inherited);
    }

    #[test]
    fn demo_state_follows_the_newest_tick() {
        let previous = pose_car(Vec3::ZERO, Vec3::Z, Vec3::Y, 0.0);
        let mut current = pose_car(Vec3::ZERO, Vec3::Z, Vec3::Y, 0.0);
        assert!(!sample_car(&previous, &current, 0.5).is_demoed);

        current.is_demoed = true;
        // Hidden as soon as the newest tick says so, even while the position is
        // still being interpolated from the previous tick.
        assert!(sample_car(&previous, &current, 0.0).is_demoed);
        assert!(sample_car(&previous, &current, 0.5).is_demoed);

        current.is_demoed = false;
        assert!(!sample_car(&previous, &current, 1.0).is_demoed);
    }

    #[test]
    fn wheels_roll_with_ground_speed_and_free_spin_in_the_air() {
        let input = SimInput::new(1.0, 0.0, false, false, false);

        // On the ground the signed forward speed rolls them, including reverse.
        let forward = pose_car(Vec3::ZERO, Vec3::Z, Vec3::Y, 4.0);
        assert_eq!(wheel_roll_speed(forward, input), 4.0);
        let reversing = pose_car(Vec3::ZERO, Vec3::Z, Vec3::Y, -4.0);
        assert_eq!(wheel_roll_speed(reversing, input), -4.0);

        // In the air the throttle drives them, even with no motion.
        let mut airborne = pose_car(Vec3::ZERO, Vec3::Z, Vec3::Y, 0.0);
        airborne.is_on_ground = false;
        assert_eq!(wheel_roll_speed(airborne, input), AIR_WHEEL_SPIN_SPEED);
        let reverse = SimInput::new(-1.0, 0.0, false, false, false);
        assert_eq!(wheel_roll_speed(airborne, reverse), -AIR_WHEEL_SPIN_SPEED);
        assert_eq!(wheel_roll_speed(airborne, SimInput::default()), 0.0);
    }

    #[test]
    fn trail_strength_follows_ball_speed() {
        assert_eq!(trail_target(0.0), 0.0);
        assert_eq!(trail_target(BALL_TRAIL_MIN_SPEED), 0.0);
        assert_eq!(trail_target(BALL_TRAIL_FULL_SPEED), 1.0);
        assert_eq!(trail_target(BALL_TRAIL_FULL_SPEED * 2.0), 1.0);
        let middle = (BALL_TRAIL_MIN_SPEED + BALL_TRAIL_FULL_SPEED) * 0.5;
        assert!((trail_target(middle) - 0.5).abs() < 1e-6);
    }

    fn trail_ball(speed: f32, direction: Vec3) -> BallView {
        BallView {
            pos: Vec3::ZERO,
            vel: direction * speed,
            speed,
            spin: Vec3::ZERO,
        }
    }

    #[test]
    fn trail_streaks_are_camera_facing_quads_that_fade_to_the_tail() {
        let ball = trail_ball(40.0, Vec3::X);
        let streaks = trail_streaks();
        assert_eq!(streaks.len(), BALL_TRAIL_STREAKS);

        let (positions, colors, indices) = trail_mesh_data(&ball, Vec3::Z, 1.0, &streaks);
        assert_eq!(positions.len(), BALL_TRAIL_STREAKS * 4);
        assert_eq!(colors.len(), BALL_TRAIL_STREAKS * 4);
        assert_eq!(indices.len(), BALL_TRAIL_STREAKS * 6);

        for index in 0..BALL_TRAIL_STREAKS {
            let base = index * 4;
            // The head is opaque and the tail is fully transparent, so the
            // streak fades out instead of ending in a hard edge.
            assert!(colors[base][3] > 0.0);
            assert_eq!(colors[base + 2][3], 0.0);
            // The streak runs backwards along -X.
            assert!(positions[base + 2][0] < positions[base][0]);
            // Each quad has a nonzero width.
            assert!(positions[base] != positions[base + 1]);
        }

        // Streaks are spread around the velocity axis, not stacked on one line.
        assert!(positions
            .iter()
            .any(|point| point[1].abs() > 0.1 || point[2].abs() > 0.1));
    }

    #[test]
    fn trail_needs_a_velocity_direction() {
        let ball = trail_ball(0.0, Vec3::ZERO);
        let (positions, colors, indices) = trail_mesh_data(&ball, Vec3::Z, 1.0, &trail_streaks());
        assert!(positions.is_empty());
        assert!(colors.is_empty());
        assert!(indices.is_empty());
    }

    #[test]
    fn trail_streak_pattern_is_stable_and_varied() {
        let first = trail_streaks();
        let second = trail_streaks();
        assert_eq!(first.len(), second.len());
        for (a, b) in first.iter().zip(&second) {
            assert_eq!(a.angle, b.angle);
            assert_eq!(a.radius, b.radius);
            assert_eq!(a.length_scale, b.length_scale);
            assert_eq!(a.alpha_scale, b.alpha_scale);
        }
        assert!(first
            .iter()
            .all(|streak| (0.4..=BALL_TRAIL_SPREAD).contains(&streak.radius)));
        assert!(first.iter().any(|streak| streak.length_scale < 0.6));
        assert!(first.iter().any(|streak| streak.length_scale > 0.9));
    }

    #[test]
    fn ball_interpolation_blends_position_velocity_and_speed() {
        let previous = BallView {
            pos: Vec3::ZERO,
            vel: Vec3::ZERO,
            speed: 0.0,
            spin: Vec3::Y,
        };
        let current = BallView {
            pos: Vec3::new(0.0, 4.0, 0.0),
            vel: Vec3::new(0.0, 8.0, 0.0),
            speed: 8.0,
            spin: Vec3::Y,
        };

        let half = sample_ball(&previous, &current, 0.5);
        assert!((half.pos - Vec3::new(0.0, 2.0, 0.0)).length() < 1e-6);
        assert!((half.vel - Vec3::new(0.0, 4.0, 0.0)).length() < 1e-6);
        assert!((half.speed - 4.0).abs() < 1e-6);

        assert!((sample_ball(&previous, &current, 0.0).pos - previous.pos).length() < 1e-6);
        assert!((sample_ball(&previous, &current, 1.0).pos - current.pos).length() < 1e-6);
    }

    #[test]
    fn playback_reset_returns_to_one_x_without_resuming() {
        let mut playback = Playback {
            paused: true,
            ..Default::default()
        };
        playback.faster();
        playback.reset_speed();
        assert_eq!(playback.speed(), 1.0);
        assert!(playback.paused, "`=` must not resume a paused simulation");

        playback.slower();
        playback.reset_speed();
        assert_eq!(playback.speed(), 1.0);
    }

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
        let heading =
            dir_from_yaw(base_yaw(false, Vec3::Z, Vec3::Y, Vec3::ZERO, None, true).unwrap());
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
    fn the_mowed_squares_fill_the_pitch_exactly() {
        // `pitch_texture` tiles the checkerboard in world space, so the tile
        // size has to divide both pitch dimensions or the pattern gets a cut-off
        // row at one edge.
        let across = 2.0 * ARENA_HALF_X / PITCH_TILE;
        let along = 2.0 * ARENA_HALF_Z / PITCH_TILE;
        assert!(
            (across - across.round()).abs() < 1e-4,
            "{across} squares across"
        );
        assert!(
            (along - along.round()).abs() < 1e-4,
            "{along} squares along"
        );
        assert_eq!(across.round() as u32, 8);
        assert_eq!(along.round() as u32, 10);
    }

    #[test]
    fn board_profile_follows_the_wall_fillet() {
        // The boards sit on the collision surface, so every team-band level has
        // to lie on the 2.5 m quarter circle. A board placed at the wall plane
        // instead would float over the ramp.
        let r = BOARD_FILLET_RADIUS;
        for level in board_levels().into_iter().take(BOARD_BAND_STEPS + 1) {
            let on_circle = (r - level.inset).powi(2) + (r - level.y).powi(2);
            assert!(
                (on_circle - r * r).abs() < 1e-4,
                "level at y={} inset={} is {on_circle} from the circle",
                level.y,
                level.inset
            );
            assert!(level.inset >= 0.0, "board pokes through the wall");
        }
    }

    #[test]
    fn boards_skip_goal_mouths_and_stay_inside_the_walls() {
        let inset = board_levels()[0].inset;
        for section in perimeter_sections() {
            for point in section.sample(inset) {
                let (x, z) = (point.pos.x, point.pos.y);
                assert!(x.abs() <= ARENA_HALF_X + 1e-3, "board outside +X wall");
                assert!(z.abs() <= ARENA_HALF_Z + 1e-3, "board outside +Z wall");
                // Any sample at the end wall must sit beside the goal opening.
                if z.abs() > ARENA_HALF_Z - 2.0 {
                    assert!(
                        x.abs() >= GOAL_HALF_WIDTH - 1e-3,
                        "board crosses the goal mouth at x={x}"
                    );
                }
            }
        }
    }

    #[test]
    fn ball_cam_steers_by_the_horizontal_bearing_to_the_ball() {
        let to_ball = Vec3::new(3.0, 7.0, 4.0);
        let heading = dir_from_yaw(base_yaw(true, Vec3::Z, Vec3::Y, to_ball, None, true).unwrap());
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
        // Nose straight up a wall: the nose has no horizontal direction, so the
        // heading comes from the right axis instead (and the boom stays
        // world-up) rather than collapsing onto the wall plane.
        let yaw = base_yaw(false, Vec3::Y, Vec3::NEG_Z, Vec3::ZERO, None, true).unwrap();
        assert!(yaw.abs() < 1e-5, "nose-up yaw {yaw}");
        let camera = chase_position(Vec3::new(0.0, 5.0, 0.0), dir_from_yaw(yaw), 2.7, 1.0);
        assert!(
            (camera.y - 6.0).abs() < 1e-5,
            "camera should stay world-up: {camera:?}"
        );

        // Nosed up at an angle: the same heading, still level.
        let yaw = base_yaw(
            false,
            Vec3::new(0.0, 1.0, 0.2),
            Vec3::NEG_Z,
            Vec3::ZERO,
            None,
            true,
        )
        .unwrap();
        assert!(yaw.abs() < 1e-5, "nosed-up yaw {yaw}");
    }

    #[test]
    fn a_flip_does_not_swivel_the_car_camera() {
        // Front/back, side and diagonal flips all rotate the car a full turn
        // about a horizontal axis. Their pitch/roll must not move the camera:
        // while the car is tilted in the air the heading is withheld, and it is
        // picked up again once the car is upright (at the same yaw, since the
        // turn is a full revolution).
        let start_yaw = 0.3_f32;
        let axes = [
            Vec3::X,                              // front/back flip
            Vec3::Z,                              // side flip / air roll
            Vec3::new(1.0, 0.0, 1.0).normalize(), // diagonal flip
        ];
        for axis in axes {
            let mut heading = start_yaw;
            for step in 0..=360 {
                let rotation = Quat::from_rotation_y(start_yaw)
                    * Quat::from_axis_angle(axis, (step as f32).to_radians());
                let forward = rotation * Vec3::Z;
                let up = rotation * Vec3::Y;
                let target = base_yaw(false, forward, up, Vec3::ZERO, Some(heading), false);
                if up.y < CAR_UPRIGHT_MIN_UP_Y {
                    assert!(
                        target.is_none(),
                        "tilted airborne car must hold at {step}° about {axis:?}"
                    );
                    continue;
                }
                let yaw = target.expect("upright car has a heading");
                assert!(
                    shortest_angle(yaw - heading).abs() < 0.15,
                    "yaw jumped from {heading} to {yaw} at {step}° about {axis:?}"
                );
                heading = yaw;
                assert!(
                    shortest_angle(heading - start_yaw).abs() < 0.15,
                    "heading drifted to {heading} at {step}° about {axis:?}"
                );
            }
            assert!(
                shortest_angle(heading - start_yaw).abs() < 0.1,
                "flip about {axis:?} ended at yaw {heading}, expected {start_yaw}"
            );
        }
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
        let controls = Controls::default();
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyL);
        let (x, _) = look_input(&keys, None, &controls, true);
        assert!(x > 0.5, "L should swivel right, got {x}");

        // Pushing up looks down, so the pitch input comes out negative.
        let mut keys = ButtonInput::<KeyCode>::default();
        keys.press(KeyCode::KeyI);
        let (_, y) = look_input(&keys, None, &controls, true);
        assert!(y < -0.5, "I should look down (inverted), got {y}");

        // The same key is not inverted when the setting is off.
        let (_, y) = look_input(&keys, None, &controls, false);
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
            read_input(&keys, None, &Controls::default())
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
    fn remapped_bindings_drive_the_car() {
        // Both keyboard and pad bindings come from the same file, and a remap
        // replaces the default rather than adding to it.
        let controls: Controls = toml::from_str(
            "[keyboard]\n\
             throttle_forward = \"F\"\n\
             throttle_backward = \"V\"\n\
             steer_left = \"G\"\n\
             steer_right = \"H\"\n\
             boost = \"B\"\n\
             \n\
             [gamepad]\n\
             jump = \"North\"\n\
             boost = \"West\"\n",
        )
        .unwrap();

        let press = |code| {
            let mut keys = ButtonInput::<KeyCode>::default();
            keys.press(code);
            read_input(&keys, None, &controls)
        };
        assert!(press(KeyCode::KeyF).throttle > 0.5);
        assert!(press(KeyCode::KeyW).throttle < 0.5, "W was unbound");
        assert!(press(KeyCode::KeyB).boost);

        // The pad side follows the same file: North jumps and West boosts.
        let mut pad = Gamepad::default();
        pad.digital_mut().press(GamepadButton::North);
        pad.digital_mut().press(GamepadButton::West);
        let input = read_input(&ButtonInput::default(), Some(&pad), &controls);
        assert!(input.jump, "{input:?}");
        assert!(input.boost, "{input:?}");
    }

    #[test]
    fn bumper_aliases_boost_and_drift() {
        // RB/LB are aliases for the bumper buttons (`RightTrigger`/`LeftTrigger`
        // in gilrs' naming), not for the analogue triggers.
        let controls: Controls =
            toml::from_str("[gamepad]\nboost = \"RB\"\ndrift = \"LB\"\n").unwrap();
        let mut pad = Gamepad::default();
        pad.digital_mut().press(GamepadButton::RightTrigger);
        pad.digital_mut().press(GamepadButton::LeftTrigger);

        let input = read_input(&ButtonInput::default(), Some(&pad), &controls);
        assert!(input.boost, "RB should boost: {input:?}");
        assert!(input.handbrake, "LB should be the handbrake: {input:?}");
    }

    #[test]
    fn an_unseen_trigger_axis_does_not_reverse_the_car() {
        // Regression: Bevy starts every pad axis at 0.0, which is *half* a
        // trigger. After using one trigger, the other would read half-pressed
        // and the car would creep backwards. Seeding both to -1.0 (rest) makes
        // releasing the forward trigger mean zero throttle, not reverse.
        let controls = Controls::default();
        let mut pad = Gamepad::default();
        seed_trigger_axes(&mut pad, &controls);

        pad.analog_mut().set(GamepadAxis::RightZ, 1.0);
        let pressed = read_input(&ButtonInput::default(), Some(&pad), &controls);
        assert!(pressed.throttle > 0.5, "{pressed:?}");

        pad.analog_mut().set(GamepadAxis::RightZ, -1.0);
        let released = read_input(&ButtonInput::default(), Some(&pad), &controls);
        assert_eq!(released.throttle, 0.0, "released: {released:?}");
    }

    #[test]
    fn dedicated_roll_buttons_work_alongside_drift_stick() {
        let controls: Controls =
            toml::from_str("[gamepad]\nroll_left = \"North\"\nroll_right = \"East\"\n").unwrap();

        let mut pad = Gamepad::default();
        pad.digital_mut().press(GamepadButton::North);
        let input = read_input(&ButtonInput::default(), Some(&pad), &controls);
        assert!(input.roll > 0.5, "X should roll left: {input:?}");

        let mut pad = Gamepad::default();
        pad.digital_mut().press(GamepadButton::East);
        let input = read_input(&ButtonInput::default(), Some(&pad), &controls);
        assert!(input.roll < -0.5, "B should roll right: {input:?}");

        // The RL-style drift + stick roll is still there when no dedicated
        // roll button is configured.
        let mut pad = Gamepad::default();
        pad.digital_mut().press(GamepadButton::North);
        pad.analog_mut().set(GamepadAxis::LeftStickX, -1.0);
        let input = read_input(&ButtonInput::default(), Some(&pad), &Controls::default());
        assert!(
            input.roll > 0.5,
            "drift + stick left should roll left: {input:?}"
        );
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
