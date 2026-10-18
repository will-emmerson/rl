//! The bot-facing RLBot v5 server.
//!
//! Each bot gets one thread. The thread reads length-prefixed FlatBuffers from
//! its socket, sends the static handshake packets, and forwards controller
//! states as events. Game packets flow the other way through a small
//! latest-only channel per client, so a slow bot cannot block the simulation.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, Sender, SyncSender};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use rlbot_flat::flat;

use crate::protocol::{
    self, build_field_info, build_game_packet, build_match_configuration, GameSnapshot, InputState,
    ServerConfig,
};

/// Things a connected bot did.
#[derive(Clone, Debug)]
pub enum BridgeEvent {
    Connected {
        client_id: u64,
    },
    Handshake {
        client_id: u64,
        agent_id: String,
        team: u8,
        player_ids: Vec<i32>,
    },
    Ready {
        client_id: u64,
    },
    Input {
        client_id: u64,
        player_id: i32,
        /// Already converted to RocketSim/viewer signs.
        input: InputState,
    },
    StopRequested {
        client_id: u64,
        shutdown_server: bool,
    },
    Disconnected {
        client_id: u64,
    },
}

struct Outbound {
    game: Arc<Vec<u8>>,
    prediction: Option<Arc<Vec<u8>>>,
}

struct ClientHandle {
    id: u64,
    ready: Arc<AtomicBool>,
    wants_prediction: Arc<AtomicBool>,
    tx: SyncSender<Arc<Outbound>>,
}

struct Shared {
    config: Arc<ServerConfig>,
    clients: Mutex<Vec<ClientHandle>>,
    next_id: AtomicU64,
    events: Sender<BridgeEvent>,
}

/// A running RLBot v5 bridge.
pub struct BridgeServer {
    shared: Arc<Shared>,
    events: Receiver<BridgeEvent>,
    stop: Arc<AtomicBool>,
    accept_thread: Option<JoinHandle<()>>,
    local_addr: SocketAddr,
}

impl BridgeServer {
    /// Bind and start accepting connections.
    ///
    /// Use port 0 in `config.bind_addr` to let the OS choose; read the actual
    /// address back with [`local_addr`](Self::local_addr).
    pub fn start(config: ServerConfig) -> io::Result<Self> {
        let listener = TcpListener::bind(config.bind_addr)?;
        listener.set_nonblocking(true)?;
        let local_addr = listener.local_addr()?;

        let stop = Arc::new(AtomicBool::new(false));
        let (events, event_rx) = mpsc::channel();
        let shared = Arc::new(Shared {
            config: Arc::new(config),
            clients: Mutex::new(Vec::new()),
            next_id: AtomicU64::new(1),
            events,
        });

        let accept_shared = Arc::clone(&shared);
        let accept_stop = Arc::clone(&stop);
        let accept_thread = thread::Builder::new()
            .name("rlbot-accept".into())
            .spawn(move || accept_loop(listener, accept_shared, accept_stop))?;

        Ok(Self {
            shared,
            events: event_rx,
            stop,
            accept_thread: Some(accept_thread),
            local_addr,
        })
    }

    pub fn local_addr(&self) -> SocketAddr {
        self.local_addr
    }

    /// Non-blocking drain of bot events.
    pub fn try_recv(&self) -> Option<BridgeEvent> {
        self.events.try_recv().ok()
    }

    /// Serialize one tick and send it to every ready bot.
    ///
    /// Packets are built once per tick, and each client gets a bounded
    /// latest-only queue: a stalled bot never stalls the sim or the other bots.
    pub fn broadcast(&self, snapshot: &GameSnapshot) {
        let game = Arc::new(protocol::encode_core(flat::CoreMessage::GamePacket(
            Box::new(build_game_packet(&self.shared.config, snapshot)),
        )));
        let prediction = if self.any_wants_prediction() && !snapshot.prediction.is_empty() {
            Some(Arc::new(protocol::encode_core(
                flat::CoreMessage::BallPrediction(Box::new(protocol::build_ball_prediction(
                    snapshot,
                ))),
            )))
        } else {
            None
        };

        let clients = self.shared.clients.lock().expect("rlbot clients mutex");
        for client in clients.iter() {
            if !client.ready.load(Ordering::Relaxed) {
                continue;
            }
            let wants_prediction = client.wants_prediction.load(Ordering::Relaxed);
            let outbound = Arc::new(Outbound {
                game: Arc::clone(&game),
                prediction: if wants_prediction {
                    prediction.clone()
                } else {
                    None
                },
            });
            // Full queue means the bot has not finished the previous tick; drop
            // this one rather than growing an unbounded backlog.
            let _ = client.tx.try_send(outbound);
        }
    }

    /// True while at least one bot has completed the handshake.
    pub fn has_ready_client(&self) -> bool {
        self.shared
            .clients
            .lock()
            .expect("rlbot clients mutex")
            .iter()
            .any(|client| client.ready.load(Ordering::Relaxed))
    }

    /// True while at least one ready bot has asked for ball prediction.
    pub fn any_wants_prediction(&self) -> bool {
        self.shared
            .clients
            .lock()
            .expect("rlbot clients mutex")
            .iter()
            .any(|client| {
                client.ready.load(Ordering::Relaxed)
                    && client.wants_prediction.load(Ordering::Relaxed)
            })
    }

    pub fn client_count(&self) -> usize {
        self.shared
            .clients
            .lock()
            .expect("rlbot clients mutex")
            .len()
    }

    pub fn ready_count(&self) -> usize {
        self.shared
            .clients
            .lock()
            .expect("rlbot clients mutex")
            .iter()
            .filter(|client| client.ready.load(Ordering::Relaxed))
            .count()
    }

    /// Stop accepting new bots and ask existing sessions to exit.
    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.accept_thread.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for BridgeServer {
    fn drop(&mut self) {
        self.stop();
    }
}

fn accept_loop(listener: TcpListener, shared: Arc<Shared>, stop: Arc<AtomicBool>) {
    while !stop.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, _peer)) => {
                let id = shared.next_id.fetch_add(1, Ordering::Relaxed);
                let session_shared = Arc::clone(&shared);
                let session_stop = Arc::clone(&stop);
                if let Err(e) = thread::Builder::new()
                    .name(format!("rlbot-client-{id}"))
                    .spawn(move || client_thread(stream, session_shared, id, session_stop))
                {
                    eprintln!("rlbot: could not spawn client thread: {e}");
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(5));
            }
            Err(e) => {
                eprintln!("rlbot: accept failed: {e}");
                break;
            }
        }
    }
}

struct Session {
    id: u64,
    stream: TcpStream,
    shared: Arc<Shared>,
    stop: Arc<AtomicBool>,
    out_rx: Receiver<Arc<Outbound>>,
    ready: Arc<AtomicBool>,
    wants_prediction: Arc<AtomicBool>,
    pending: Vec<u8>,
    handshaken: bool,
    owned_players: Vec<usize>,
    team: u8,
}

fn client_thread(stream: TcpStream, shared: Arc<Shared>, id: u64, stop: Arc<AtomicBool>) {
    let _ = stream.set_nodelay(true);
    // The read timeout doubles as the outbound-packet polling interval: one
    // thread owns both directions so writes cannot interleave, and 1 ms keeps
    // the 120 Hz game packets prompt even when the bot is not sending input.
    let _ = stream.set_read_timeout(Some(Duration::from_millis(1)));
    let (out_tx, out_rx) = mpsc::sync_channel(2);
    let ready = Arc::new(AtomicBool::new(false));
    let wants_prediction = Arc::new(AtomicBool::new(false));

    shared
        .clients
        .lock()
        .expect("rlbot clients mutex")
        .push(ClientHandle {
            id,
            ready: Arc::clone(&ready),
            wants_prediction: Arc::clone(&wants_prediction),
            tx: out_tx,
        });
    let _ = shared.events.send(BridgeEvent::Connected { client_id: id });

    let mut session = Session {
        id,
        stream,
        shared: Arc::clone(&shared),
        stop,
        out_rx,
        ready,
        wants_prediction,
        pending: Vec::new(),
        handshaken: false,
        owned_players: Vec::new(),
        team: 0,
    };
    session.run();

    shared
        .clients
        .lock()
        .expect("rlbot clients mutex")
        .retain(|client| client.id != id);
    let _ = shared
        .events
        .send(BridgeEvent::Disconnected { client_id: id });
}

impl Session {
    fn run(&mut self) {
        while !self.stop.load(Ordering::Relaxed) {
            while let Ok(outbound) = self.out_rx.try_recv() {
                if !self.ready.load(Ordering::Relaxed) {
                    continue;
                }
                if let Some(prediction) = &outbound.prediction {
                    if self.stream.write_all(prediction).is_err() {
                        return;
                    }
                }
                if self.stream.write_all(&outbound.game).is_err() {
                    return;
                }
            }

            match read_packet(&mut self.stream, &mut self.pending) {
                ReadOutcome::Packet(payload) => match protocol::decode_interface(&payload) {
                    Ok(message) => {
                        if !self.handle(message) {
                            return;
                        }
                    }
                    Err(e) => {
                        eprintln!("rlbot: client {}: {e}", self.id);
                        return;
                    }
                },
                ReadOutcome::Eof => return,
                ReadOutcome::Timeout => {}
                ReadOutcome::Error(e) => {
                    eprintln!("rlbot: client {} read failed: {e}", self.id);
                    return;
                }
            }
        }
    }

    fn handle(&mut self, message: flat::InterfaceMessage) -> bool {
        match message {
            flat::InterfaceMessage::ConnectionSettings(settings) => {
                let settings = *settings;
                let agent_id = settings.agent_id;
                let matches: Vec<(usize, i32, u8)> = self
                    .shared
                    .config
                    .players
                    .iter()
                    .enumerate()
                    .filter(|(_, player)| player.agent_id.as_deref() == Some(agent_id.as_str()))
                    .map(|(index, player)| (index, player.player_id, player.team))
                    .collect();

                self.owned_players = matches.iter().map(|(index, _, _)| *index).collect();
                self.team = matches.first().map(|(_, _, team)| *team).unwrap_or(0);
                self.wants_prediction
                    .store(settings.wants_ball_predictions, Ordering::Relaxed);
                self.handshaken = true;

                let config = protocol::encode_core(flat::CoreMessage::MatchConfiguration(
                    Box::new(build_match_configuration(&self.shared.config)),
                ));
                if self.stream.write_all(&config).is_err() {
                    return false;
                }
                let field = protocol::encode_core(flat::CoreMessage::FieldInfo(Box::new(
                    build_field_info(&self.shared.config.field),
                )));
                if self.stream.write_all(&field).is_err() {
                    return false;
                }
                let controllable = flat::ControllableTeamInfo {
                    team: u32::from(self.team),
                    controllables: matches
                        .iter()
                        .map(|(index, player_id, _)| flat::ControllableInfo {
                            index: *index as u32,
                            identifier: *player_id,
                        })
                        .collect(),
                };
                let controllable = protocol::encode_core(flat::CoreMessage::ControllableTeamInfo(
                    Box::new(controllable),
                ));
                if self.stream.write_all(&controllable).is_err() {
                    return false;
                }

                if matches.is_empty() {
                    eprintln!(
                        "rlbot: client {} connected as {agent_id:?}, but no player has that agent id",
                        self.id
                    );
                }
                let _ = self.shared.events.send(BridgeEvent::Handshake {
                    client_id: self.id,
                    agent_id,
                    team: self.team,
                    player_ids: matches.iter().map(|(_, player_id, _)| *player_id).collect(),
                });
                true
            }
            flat::InterfaceMessage::InitComplete(_) => {
                if self.handshaken {
                    self.ready.store(true, Ordering::Relaxed);
                    let _ = self
                        .shared
                        .events
                        .send(BridgeEvent::Ready { client_id: self.id });
                }
                true
            }
            flat::InterfaceMessage::PlayerInput(input) => {
                let input = *input;
                let index = input.player_index as usize;
                if self.owned_players.contains(&index) {
                    if let Some(player) = self.shared.config.players.get(index) {
                        let _ = self.shared.events.send(BridgeEvent::Input {
                            client_id: self.id,
                            player_id: player.player_id,
                            input: protocol::input_from_flat(&input.controller_state),
                        });
                    }
                }
                true
            }
            flat::InterfaceMessage::PingRequest(request) => {
                let response = protocol::encode_core(flat::CoreMessage::PingResponse(Box::new(
                    flat::PingResponse {
                        cookie: request.cookie,
                    },
                )));
                self.stream.write_all(&response).is_ok()
            }
            flat::InterfaceMessage::StopCommand(command) => {
                let _ = self.shared.events.send(BridgeEvent::StopRequested {
                    client_id: self.id,
                    shutdown_server: command.shutdown_server,
                });
                false
            }
            flat::InterfaceMessage::DisconnectSignal(_) => false,
            // Rendering, match comms, state setting, loadouts and scripts are
            // deliberately out of scope for this bridge. Ignoring them leaves
            // the bot's control loop intact.
            _ => true,
        }
    }
}

enum ReadOutcome {
    Packet(Vec<u8>),
    Eof,
    Timeout,
    Error(io::Error),
}

/// Read one RLBot packet (2-byte big-endian length, then payload).
///
/// `pending` retains bytes from a partial read so a read timeout cannot lose
/// the start of the next packet.
fn read_packet(stream: &mut TcpStream, pending: &mut Vec<u8>) -> ReadOutcome {
    loop {
        if pending.len() >= 2 {
            let len = usize::from(u16::from_be_bytes([pending[0], pending[1]]));
            if pending.len() >= 2 + len {
                let payload = pending[2..2 + len].to_vec();
                pending.drain(..2 + len);
                return ReadOutcome::Packet(payload);
            }
        }

        let mut buf = [0u8; 4096];
        match stream.read(&mut buf) {
            Ok(0) => return ReadOutcome::Eof,
            Ok(n) => pending.extend_from_slice(&buf[..n]),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                return ReadOutcome::Timeout;
            }
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return ReadOutcome::Error(e),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::io::Read;
    use std::net::TcpStream;
    use std::time::{Duration, Instant};

    use rlbot_flat::planus::ReadAsRoot;

    use super::*;
    use crate::protocol::{
        BallSnapshot, CarSnapshot, FieldSpec, GameSnapshot, GoalSpec, PadSpec, PadState, Phase,
        PlayerSpec,
    };

    fn config() -> ServerConfig {
        ServerConfig {
            bind_addr: "127.0.0.1:0".parse().unwrap(),
            players: vec![
                PlayerSpec {
                    player_id: 0,
                    name: "You".into(),
                    team: 0,
                    agent_id: None,
                },
                PlayerSpec {
                    player_id: 1,
                    name: "Bot".into(),
                    team: 1,
                    agent_id: Some("test/bot".into()),
                },
            ],
            field: FieldSpec {
                pads: vec![PadSpec {
                    pos: [0.0, 0.0, 0.0],
                    is_big: true,
                }],
                goals: vec![GoalSpec {
                    team: 0,
                    pos: [0.0, -5120.0, 321.3875],
                    direction: [0.0, 1.0, 0.0],
                    width: 1785.51,
                    height: 642.775,
                }],
            },
        }
    }

    fn snapshot() -> GameSnapshot {
        GameSnapshot {
            frame: 7,
            seconds_elapsed: 0.1,
            game_time_remaining: 299.9,
            phase: Phase::Active,
            scores: [1, 2],
            cars: vec![CarSnapshot::default(), CarSnapshot::default()],
            pads: vec![PadState {
                is_active: true,
                timer: 0.0,
            }],
            ball: BallSnapshot {
                diameter: 182.5,
                ..Default::default()
            },
            ..Default::default()
        }
    }

    fn write_interface(stream: &mut TcpStream, message: flat::InterfaceMessage) {
        stream
            .write_all(&protocol::encode_interface(message))
            .unwrap();
    }

    fn read_core(stream: &mut TcpStream) -> flat::CoreMessage {
        let mut len = [0u8; 2];
        stream.read_exact(&mut len).unwrap();
        let mut payload = vec![0u8; usize::from(u16::from_be_bytes(len))];
        stream.read_exact(&mut payload).unwrap();
        let packet = flat::CorePacketRef::read_as_root(&payload).unwrap();
        let packet: flat::CorePacket = packet.try_into().unwrap();
        packet.message
    }

    fn wait_for<F>(server: &BridgeServer, mut predicate: F) -> bool
    where
        F: FnMut(&BridgeEvent) -> bool,
    {
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            while let Some(event) = server.try_recv() {
                if predicate(&event) {
                    return true;
                }
            }
            thread::sleep(Duration::from_millis(2));
        }
        false
    }

    #[test]
    fn handshake_packet_and_controller_passthrough() {
        let server = BridgeServer::start(config()).unwrap();
        let mut client = TcpStream::connect(server.local_addr()).unwrap();
        client
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();

        write_interface(
            &mut client,
            flat::InterfaceMessage::ConnectionSettings(Box::new(flat::ConnectionSettings {
                agent_id: "test/bot".into(),
                wants_ball_predictions: false,
                wants_comms: false,
                close_between_matches: true,
            })),
        );

        let mut saw_config = false;
        let mut saw_field = false;
        let mut saw_controllable = false;
        for _ in 0..3 {
            match read_core(&mut client) {
                flat::CoreMessage::MatchConfiguration(config) => {
                    assert_eq!(config.player_configurations.len(), 2);
                    saw_config = true;
                }
                flat::CoreMessage::FieldInfo(field) => {
                    assert_eq!(field.boost_pads.len(), 1);
                    saw_field = true;
                }
                flat::CoreMessage::ControllableTeamInfo(info) => {
                    assert_eq!(info.team, 1);
                    assert_eq!(info.controllables.len(), 1);
                    assert_eq!(info.controllables[0].index, 1);
                    assert_eq!(info.controllables[0].identifier, 1);
                    saw_controllable = true;
                }
                _ => {}
            }
        }
        assert!(saw_config && saw_field && saw_controllable);

        assert!(wait_for(&server, |event| matches!(
            event,
            BridgeEvent::Handshake { agent_id, .. } if agent_id == "test/bot"
        )));

        write_interface(
            &mut client,
            flat::InterfaceMessage::InitComplete(Box::new(flat::InitComplete {})),
        );
        assert!(wait_for(&server, |event| matches!(
            event,
            BridgeEvent::Ready { .. }
        )));

        server.broadcast(&snapshot());
        let packet = loop {
            if let flat::CoreMessage::GamePacket(packet) = read_core(&mut client) {
                break packet;
            }
        };
        assert_eq!(packet.players.len(), 2);
        assert_eq!(packet.teams[0].score, 1);
        assert_eq!(packet.teams[1].score, 2);
        assert_eq!(packet.boost_pads.len(), 1);
        assert_eq!(packet.balls.len(), 1);
        assert_eq!(packet.match_info.frame_num, 7);

        write_interface(
            &mut client,
            flat::InterfaceMessage::PlayerInput(Box::new(flat::PlayerInput {
                player_index: 1,
                controller_state: flat::ControllerState {
                    steer: 1.0,
                    yaw: 1.0,
                    roll: 1.0,
                    pitch: 1.0,
                    ..Default::default()
                },
            })),
        );
        let input = loop {
            if let Some(BridgeEvent::Input {
                player_id, input, ..
            }) = server.try_recv()
            {
                break (player_id, input);
            }
            thread::sleep(Duration::from_millis(2));
        };
        assert_eq!(input.0, 1);
        // Everything the bot sees is in RocketSim's frame, so its controls are
        // applied in that same frame.
        assert_eq!(input.1.steer, 1.0);
        assert_eq!(input.1.yaw, 1.0);
        assert_eq!(input.1.roll, 1.0);
        assert_eq!(input.1.pitch, 1.0);
    }
}
