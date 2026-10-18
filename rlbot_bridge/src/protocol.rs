//! RLBot v5 wire types and conversions.
//!
//! The bridge talks the bot-facing half of RLBot: a TCP stream of
//! 2-byte-big-endian-length-prefixed FlatBuffers. `CoreMessage` is sent from us
//! to the bots; `InterfaceMessage` is sent from the bots to us.
//!
//! These types are deliberately independent of Bevy and RocketSim. The caller
//! fills a [`GameSnapshot`] in RLBot's frame and units (uu, Z-up) each tick and
//! receives [`InputState`] values in RocketSim's control signs.

use std::net::SocketAddr;

use rlbot_flat::flat;
use rlbot_flat::planus::{Builder, ReadAsRoot};

/// Default RLBot TCP port.
pub const DEFAULT_PORT: u16 = 23234;

/// One player in the match.
#[derive(Clone, Debug)]
pub struct PlayerSpec {
    pub player_id: i32,
    pub name: String,
    /// Blue is 0, Orange is 1.
    pub team: u8,
    /// `Some` for a bot that may connect and control this car, `None` for the
    /// human player controlled by the viewer.
    pub agent_id: Option<String>,
}

/// A boost pad's static properties, in RLBot units (uu).
#[derive(Clone, Copy, Debug)]
pub struct PadSpec {
    pub pos: [f32; 3],
    pub is_big: bool,
}

/// A goal's static properties, in RLBot units (uu).
#[derive(Clone, Copy, Debug)]
pub struct GoalSpec {
    pub team: u8,
    pub pos: [f32; 3],
    pub direction: [f32; 3],
    pub width: f32,
    pub height: f32,
}

/// Static field geometry sent once during the handshake.
#[derive(Clone, Debug, Default)]
pub struct FieldSpec {
    /// Must be ordered by y, then x, to match RLBot's boost-pad indexing.
    pub pads: Vec<PadSpec>,
    pub goals: Vec<GoalSpec>,
}

/// Static server configuration.
#[derive(Clone, Debug)]
pub struct ServerConfig {
    pub bind_addr: SocketAddr,
    pub players: Vec<PlayerSpec>,
    pub field: FieldSpec,
}

/// A rigid body in RLBot's frame and units.
#[derive(Clone, Copy, Debug, Default)]
pub struct BodyState {
    /// Position in uu, Z-up.
    pub pos: [f32; 3],
    /// RLBot `Rotator` angles in radians: pitch, yaw, roll.
    pub rot: [f32; 3],
    /// Velocity in uu/s.
    pub vel: [f32; 3],
    /// Angular velocity in rad/s.
    pub ang_vel: [f32; 3],
}

/// A car's state for one tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct CarSnapshot {
    pub physics: BodyState,
    /// 0..100.
    pub boost: f32,
    pub is_supersonic: bool,
    pub is_on_ground: bool,
    pub is_jumping: bool,
    pub is_flipping: bool,
    pub has_jumped: bool,
    pub has_double_jumped: bool,
    pub has_flipped: bool,
    /// Used for the conservative `dodge_timeout` calculation.
    pub air_time_since_jump: f32,
    /// Seconds since the current flip started.
    pub flip_time: f32,
    pub is_demoed: bool,
    pub demo_respawn_timer: f32,
    /// Hitbox dimensions in uu: length, width, height.
    pub hitbox: [f32; 3],
    /// Hitbox centre offset in car-local coordinates, in uu.
    pub hitbox_offset: [f32; 3],
    /// The most recent input applied to this car, in RocketSim signs.
    pub last_input: InputState,
    /// Dodge direction for the latest dodge; `(0, 0)` for a stall.
    pub dodge_dir: [f32; 2],
}

/// The ball's state for one tick.
#[derive(Clone, Copy, Debug, Default)]
pub struct BallSnapshot {
    pub physics: BodyState,
    /// Ball diameter in uu.
    pub diameter: f32,
}

/// One boost pad's dynamic state.
#[derive(Clone, Copy, Debug, Default)]
pub struct PadState {
    pub is_active: bool,
    /// Seconds remaining until active; 0 while active.
    pub timer: f32,
}

/// A ball-prediction slice.
#[derive(Clone, Copy, Debug, Default)]
pub struct PredictionSlice {
    pub game_seconds: f32,
    pub physics: BodyState,
}

/// Match phase, mirroring RLBot's `MatchPhase`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Phase {
    #[default]
    Inactive,
    Countdown,
    Kickoff,
    Active,
    GoalScored,
    Replay,
    Paused,
    Ended,
}

/// The complete per-tick state the bridge serializes.
#[derive(Clone, Debug, Default)]
pub struct GameSnapshot {
    pub frame: u32,
    pub seconds_elapsed: f32,
    pub game_time_remaining: f32,
    pub is_overtime: bool,
    pub is_unlimited_time: bool,
    pub phase: Phase,
    /// Team scores, indexed `[blue, orange]`.
    pub scores: [u32; 2],
    /// In `ServerConfig::players` order.
    pub cars: Vec<CarSnapshot>,
    /// In `FieldSpec::pads` order.
    pub pads: Vec<PadState>,
    pub ball: BallSnapshot,
    /// Empty means "do not send a prediction this tick".
    pub prediction: Vec<PredictionSlice>,
}

/// Controller state in **RocketSim/viewer signs**.
///
/// The bridge passes controls through unchanged in both directions. RLBot's
/// documentation gives `steer`, `yaw` and `roll` the opposite sign to
/// RocketSim, but the bot is also *looking at* a world in RocketSim's frame:
/// positions, rotations and angular velocities go over the wire as-is. A bot
/// trained on RLBot-wire data therefore expects the same reflected frame it is
/// reading. Negating only the controls would break that loop. This matches
/// `rlbot-rocketsim`, the reference RocketSim↔RLBot converter.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct InputState {
    pub throttle: f32,
    pub steer: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub jump: bool,
    pub boost: bool,
    pub handbrake: bool,
}

/// Convert an RLBot controller state to RocketSim/viewer signs.
pub(crate) fn input_from_flat(c: &flat::ControllerState) -> InputState {
    InputState {
        throttle: c.throttle,
        steer: c.steer,
        // Both use +1 = nose up.
        pitch: c.pitch,
        yaw: c.yaw,
        roll: c.roll,
        jump: c.jump,
        boost: c.boost,
        handbrake: c.handbrake,
    }
}

/// Convert RocketSim/viewer signs back to the RLBot wire convention.
pub(crate) fn input_to_flat(i: InputState) -> flat::ControllerState {
    flat::ControllerState {
        throttle: i.throttle,
        steer: i.steer,
        pitch: i.pitch,
        yaw: i.yaw,
        roll: i.roll,
        jump: i.jump,
        boost: i.boost,
        handbrake: i.handbrake,
        use_item: false,
    }
}

fn phase_to_flat(phase: Phase) -> flat::MatchPhase {
    match phase {
        Phase::Inactive => flat::MatchPhase::Inactive,
        Phase::Countdown => flat::MatchPhase::Countdown,
        Phase::Kickoff => flat::MatchPhase::Kickoff,
        Phase::Active => flat::MatchPhase::Active,
        Phase::GoalScored => flat::MatchPhase::GoalScored,
        Phase::Replay => flat::MatchPhase::Replay,
        Phase::Paused => flat::MatchPhase::Paused,
        Phase::Ended => flat::MatchPhase::Ended,
    }
}

fn air_state(car: &CarSnapshot) -> flat::AirState {
    if car.is_jumping {
        flat::AirState::Jumping
    } else if car.is_flipping {
        flat::AirState::Dodging
    } else if car.is_on_ground {
        flat::AirState::OnGround
    } else {
        flat::AirState::InAir
    }
}

/// Conservative `dodge_timeout`: assumes no initial-jump hold extension.
fn dodge_timeout(car: &CarSnapshot) -> f32 {
    const DOUBLE_JUMP_MAX_DELAY: f32 = 1.25;
    let remaining = DOUBLE_JUMP_MAX_DELAY - car.air_time_since_jump;
    if car.is_on_ground
        || car.is_jumping
        || !car.has_jumped
        || car.has_double_jumped
        || car.has_flipped
        || remaining <= 0.0
    {
        -1.0
    } else {
        remaining
    }
}

fn vector3(v: [f32; 3]) -> flat::Vector3 {
    flat::Vector3 {
        x: v[0],
        y: v[1],
        z: v[2],
    }
}

fn physics(p: BodyState) -> flat::Physics {
    flat::Physics {
        location: vector3(p.pos),
        rotation: flat::Rotator {
            pitch: p.rot[0],
            yaw: p.rot[1],
            roll: p.rot[2],
        },
        velocity: vector3(p.vel),
        angular_velocity: vector3(p.ang_vel),
    }
}

fn player_info(player: &PlayerSpec, car: &CarSnapshot) -> flat::PlayerInfo {
    flat::PlayerInfo {
        physics: physics(car.physics),
        score_info: flat::ScoreInfo::default(),
        hitbox: Box::new(flat::BoxShape {
            length: car.hitbox[0],
            width: car.hitbox[1],
            height: car.hitbox[2],
        }),
        hitbox_offset: vector3(car.hitbox_offset),
        latest_touch: None,
        air_state: air_state(car),
        dodge_timeout: dodge_timeout(car),
        demolished_timeout: if car.is_demoed {
            car.demo_respawn_timer
        } else {
            -1.0
        },
        is_supersonic: car.is_supersonic,
        is_bot: player.agent_id.is_some(),
        name: player.name.clone(),
        team: u32::from(player.team),
        boost: car.boost,
        player_id: player.player_id,
        accolades: Vec::new(),
        last_input: input_to_flat(car.last_input),
        has_jumped: car.has_jumped,
        has_double_jumped: car.has_double_jumped,
        has_dodged: car.has_flipped,
        dodge_elapsed: if car.is_on_ground { 0.0 } else { car.flip_time },
        dodge_dir: flat::Vector2 {
            x: car.dodge_dir[0],
            y: car.dodge_dir[1],
        },
        rumble_item: None,
        time_until_next_item: 0.0,
        max_time_until_next_item: 0.0,
    }
}

pub(crate) fn build_match_configuration(config: &ServerConfig) -> flat::MatchConfiguration {
    flat::MatchConfiguration {
        launcher: flat::Launcher::NoLaunch,
        launcher_arg: String::new(),
        auto_start_agents: false,
        wait_for_agents: true,
        game_map_upk: "Stadium_P".to_string(),
        player_configurations: config
            .players
            .iter()
            .map(|p| flat::PlayerConfiguration {
                variety: match (&p.agent_id, p.team) {
                    (Some(agent_id), _) => {
                        flat::PlayerClass::CustomBot(Box::new(flat::CustomBot {
                            name: p.name.clone(),
                            root_dir: String::new(),
                            run_command: String::new(),
                            loadout: None,
                            agent_id: agent_id.clone(),
                            hivemind: false,
                            environment: None,
                        }))
                    }
                    (None, _) => flat::PlayerClass::Human(Box::new(flat::Human {})),
                },
                team: u32::from(p.team),
                player_id: p.player_id,
            })
            .collect(),
        script_configurations: Vec::new(),
        game_mode: flat::GameMode::Soccar,
        skip_replays: true,
        instant_start: false,
        mutators: None,
        existing_match_behavior: flat::ExistingMatchBehavior::Restart,
        enable_rendering: flat::DebugRendering::AlwaysOff,
        enable_state_setting: false,
        auto_save_replay: false,
        freeplay: false,
        performance_monitor: flat::PerformanceMonitor::NeverShow,
    }
}

pub(crate) fn build_field_info(field: &FieldSpec) -> flat::FieldInfo {
    flat::FieldInfo {
        boost_pads: field
            .pads
            .iter()
            .map(|p| flat::BoostPad {
                location: vector3(p.pos),
                is_full_boost: p.is_big,
            })
            .collect(),
        goals: field
            .goals
            .iter()
            .map(|g| flat::GoalInfo {
                team_num: i32::from(g.team),
                location: vector3(g.pos),
                direction: vector3(g.direction),
                width: g.width,
                height: g.height,
            })
            .collect(),
        tiles: Vec::new(),
    }
}

pub(crate) fn build_game_packet(
    config: &ServerConfig,
    snapshot: &GameSnapshot,
) -> flat::GamePacket {
    debug_assert_eq!(config.players.len(), snapshot.cars.len());
    let players = config
        .players
        .iter()
        .zip(snapshot.cars.iter())
        .map(|(player, car)| player_info(player, car))
        .collect();

    flat::GamePacket {
        players,
        boost_pads: snapshot
            .pads
            .iter()
            .map(|p| flat::BoostPadState {
                is_active: p.is_active,
                timer: p.timer,
            })
            .collect(),
        balls: vec![flat::BallInfo {
            physics: physics(snapshot.ball.physics),
            shape: flat::CollisionShape::SphereShape(Box::new(flat::SphereShape {
                diameter: snapshot.ball.diameter,
            })),
            charge_level: -1,
            target_speed: 0.0,
        }],
        match_info: Box::new(flat::MatchInfo {
            seconds_elapsed: snapshot.seconds_elapsed,
            game_time_remaining: snapshot.game_time_remaining,
            is_overtime: snapshot.is_overtime,
            is_unlimited_time: snapshot.is_unlimited_time,
            match_phase: phase_to_flat(snapshot.phase),
            world_gravity_z: -650.0,
            game_speed: 1.0,
            last_spectated: 0,
            frame_num: snapshot.frame,
        }),
        teams: vec![
            flat::TeamInfo {
                team_index: 0,
                score: snapshot.scores[0],
            },
            flat::TeamInfo {
                team_index: 1,
                score: snapshot.scores[1],
            },
        ],
        tiles: Vec::new(),
    }
}

pub(crate) fn build_ball_prediction(snapshot: &GameSnapshot) -> flat::BallPrediction {
    flat::BallPrediction {
        slices: snapshot
            .prediction
            .iter()
            .map(|slice| flat::PredictionSlice {
                game_seconds: slice.game_seconds,
                physics: physics(slice.physics),
            })
            .collect(),
    }
}

pub(crate) fn encode_core(message: flat::CoreMessage) -> Vec<u8> {
    let mut builder = Builder::with_capacity(1024);
    let payload = builder.finish(flat::CorePacket { message }, None);
    frame(payload)
}

#[cfg(test)]
pub(crate) fn encode_interface(message: flat::InterfaceMessage) -> Vec<u8> {
    let mut builder = Builder::with_capacity(1024);
    let payload = builder.finish(flat::InterfacePacket { message }, None);
    frame(payload)
}

/// Decode the length-stripped payload of an `InterfacePacket`.
pub(crate) fn decode_interface(payload: &[u8]) -> Result<flat::InterfaceMessage, String> {
    let packet = flat::InterfacePacketRef::read_as_root(payload)
        .map_err(|e| format!("invalid InterfacePacket: {e}"))?;
    let packet: flat::InterfacePacket = packet
        .try_into()
        .map_err(|e| format!("could not unpack InterfacePacket: {e}"))?;
    Ok(packet.message)
}

fn frame(payload: &[u8]) -> Vec<u8> {
    let len = u16::try_from(payload.len()).expect("RLBot packet exceeds u16 length");
    let mut framed = Vec::with_capacity(2 + payload.len());
    framed.extend_from_slice(&len.to_be_bytes());
    framed.extend_from_slice(payload);
    framed
}
