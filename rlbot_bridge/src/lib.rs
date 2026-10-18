//! A bot-facing RLBot v5 bridge for a custom simulation.
//!
//! RLBot v5 has two halves: bots connect to RLBotServer over TCP and
//! FlatBuffers, while RLBotServer drives Rocket League through a closed-source
//! Bridge.dll. This crate implements the bot-facing half, so bots can connect
//! to any simulation that can produce [`GameSnapshot`]s and consume
//! [`InputState`]s.
//!
//! ```text
//! RLBot bot  <-- TCP/FlatBuffers -->  rlbot_bridge  <-- GameSnapshot -->  simulation
//! ```
//!
//! The caller owns the simulation. Each tick it builds a snapshot, calls
//! [`BridgeServer::broadcast`], and drains [`BridgeEvent`]s for controller
//! input.

mod protocol;
mod server;

pub use protocol::{
    BallSnapshot, BodyState, CarSnapshot, FieldSpec, GameSnapshot, GoalSpec, InputState, PadSpec,
    PadState, Phase, PlayerSpec, PredictionSlice, ServerConfig, DEFAULT_PORT,
};
pub use server::{BridgeEvent, BridgeServer};
