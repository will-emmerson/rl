//! Viewer-side glue for the RLBot bridge.
//!
//! The bridge owns the TCP server and serializes snapshots; this module owns
//! the Bevy resource, turns bot controller states into [`sim::Input`]s, and
//! keeps the latest input per player.

use std::collections::BTreeMap;
use std::net::SocketAddr;
use std::path::{Path, PathBuf};
use std::process::{Child, Command};
use std::sync::Mutex;

use bevy::prelude::*;
use rlbot_bridge::{
    BridgeEvent, BridgeServer, FieldSpec, GameSnapshot, InputState, PlayerSpec, ServerConfig,
};
use serde::Deserialize;

use crate::sim;

/// Command-line options for `--rlbot`.
#[derive(Clone, Debug)]
pub struct RlbotOptions {
    pub port: u16,
    pub agent_id: String,
    pub bot_name: String,
    /// A bot process to start once the server is listening.
    pub launch: Option<LaunchOptions>,
}

/// How to start a bot process, assembled by `--rlbot-command` or
/// `--rlbot-config`.
#[derive(Clone, Debug)]
pub struct LaunchOptions {
    /// A shell command. The bridge appends `RLBOT_SERVER_IP`,
    /// `RLBOT_SERVER_PORT` and `RLBOT_AGENT_ID` to its environment.
    pub command: String,
    /// Working directory; botpack bots use relative paths for models and
    /// meshes, so this is normally the directory holding the bot's toml.
    pub working_dir: Option<PathBuf>,
    /// Extra environment variables from the bot config's
    /// `[settings.environment]`.
    pub environment: Vec<(String, String)>,
}

/// The fields of an RLBot `bot.toml` that the viewer needs to start one.
/// RLBot v5 botpack files look like this:
///
/// ```toml
/// [settings]
/// name = "Element"
/// agent_id = "Rangler/Element"
/// run_command_linux = "./Element"
/// ```
#[derive(Debug)]
pub struct BotConfig {
    pub agent_id: String,
    pub name: String,
    pub launch: LaunchOptions,
}

/// Resolve `--rlbot-config` to a bot toml.
///
/// `spec` may be the path to a bot config or a directory holding one, or a bot
/// name. Names are matched case-insensitively against a config's directory
/// name, file stem (`beast.bot.toml` -> `beast`), `settings.name` and
/// `settings.agent_id`, anywhere under the RLBot botpack root. That root is
/// `$XDG_DATA_HOME/RLBot5/bots` (or `~/.local/share/RLBot5/bots`) on Linux and
/// `%LOCALAPPDATA%\RLBot5\bots` on Windows, which is where the GUI installs the
/// official botpack.
pub fn resolve_bot_config(spec: &str) -> PathBuf {
    let path = Path::new(spec);
    if path.is_file() {
        return path.to_path_buf();
    }

    let mut found = Vec::new();
    if path.is_dir() {
        collect_bot_configs(path, 3, &mut found);
    } else {
        let roots = botpack_roots();
        let configs = bot_configs_in(&roots);
        found = configs
            .iter()
            .filter(|config| bot_config_matches(config, spec))
            .cloned()
            .collect();
        if found.is_empty() {
            panic!("{}", no_bot_message(spec, &roots, &configs));
        }
    }

    found.sort();
    found.dedup();
    match found.len() {
        0 => panic!("--rlbot-config: no bot.toml found in {}", path.display()),
        1 => found.pop().expect("one match"),
        _ => panic!(
            "--rlbot-config: {spec:?} matches several bots; pass one of these paths:\n  {}",
            found
                .iter()
                .map(|config| config.display().to_string())
                .collect::<Vec<_>>()
                .join("\n  ")
        ),
    }
}

fn botpack_roots() -> Vec<PathBuf> {
    let base = if cfg!(windows) {
        std::env::var_os("LOCALAPPDATA").map(PathBuf::from)
    } else {
        std::env::var_os("XDG_DATA_HOME")
            .map(PathBuf::from)
            .or_else(|| {
                std::env::var_os("HOME").map(|home| PathBuf::from(home).join(".local/share"))
            })
    };

    base.map(|base| base.join("RLBot5").join("bots"))
        .filter(|root| root.is_dir())
        .into_iter()
        .collect()
}

fn bot_configs_in(roots: &[PathBuf]) -> Vec<PathBuf> {
    let mut found = Vec::new();
    for root in roots {
        collect_bot_configs(root, 4, &mut found);
    }
    found.sort();
    found.dedup();
    found
}

#[cfg(test)]
fn find_bot_configs(roots: &[PathBuf], spec: &str) -> Vec<PathBuf> {
    let mut found = bot_configs_in(roots);
    found.retain(|config| bot_config_matches(config, spec));
    found
}

/// A human-readable `Name (agent/id)` label for the not-found error.
fn bot_label(config: &Path) -> String {
    let Ok(text) = std::fs::read_to_string(config) else {
        return config.display().to_string();
    };
    let Ok(toml): Result<BotToml, _> = toml::from_str(&text) else {
        return config.display().to_string();
    };
    let name = toml.settings.name.filter(|name| !name.trim().is_empty());
    let agent_id = toml.settings.agent_id.filter(|id| !id.trim().is_empty());

    match (name, agent_id) {
        (Some(name), Some(agent_id)) => format!("{name} ({agent_id})"),
        (Some(name), None) => name,
        (None, Some(agent_id)) => agent_id,
        (None, None) => config.display().to_string(),
    }
}

fn no_bot_message(spec: &str, roots: &[PathBuf], configs: &[PathBuf]) -> String {
    let looked = if roots.is_empty() {
        "no RLBot botpack directory was found".to_string()
    } else {
        format!(
            "looked in {}",
            roots
                .iter()
                .map(|root| root.display().to_string())
                .collect::<Vec<_>>()
                .join(", ")
        )
    };

    let mut labels: Vec<String> = configs.iter().map(|config| bot_label(config)).collect();
    labels.sort();
    labels.dedup();

    if labels.is_empty() {
        format!(
            "--rlbot-config: no bot named {spec:?} ({looked}) and no bots were found there; pass the path to a bot.toml instead"
        )
    } else {
        format!(
            "--rlbot-config: no bot named {spec:?} ({looked}).\nInstalled bots:\n  {}\nPass the path to a bot.toml instead if the bot lives elsewhere",
            labels.join("\n  ")
        )
    }
}

fn collect_bot_configs(dir: &Path, depth: usize, out: &mut Vec<PathBuf>) {
    if depth == 0 {
        return;
    }
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            collect_bot_configs(&path, depth - 1, out);
        } else if is_bot_config(&path) {
            out.push(path);
        }
    }
}

fn is_bot_config(path: &Path) -> bool {
    path.file_name()
        .and_then(|name| name.to_str())
        .is_some_and(|name| name == "bot.toml" || name.ends_with(".bot.toml"))
}

fn bot_config_matches(config: &Path, spec: &str) -> bool {
    let spec = spec.trim();
    if spec.is_empty() {
        return false;
    }

    let file = config
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or_default();
    let file_stem = if file == "bot.toml" {
        None
    } else {
        file.strip_suffix(".bot.toml")
            .or_else(|| file.strip_suffix(".toml"))
    };
    let dir_name = config
        .parent()
        .and_then(|parent| parent.file_name())
        .and_then(|name| name.to_str());

    if file_stem.is_some_and(|stem| stem.eq_ignore_ascii_case(spec))
        || dir_name.is_some_and(|name| name.eq_ignore_ascii_case(spec))
    {
        return true;
    }

    let Ok(text) = std::fs::read_to_string(config) else {
        return false;
    };
    let Ok(toml): Result<BotToml, _> = toml::from_str(&text) else {
        return false;
    };
    toml.settings
        .name
        .as_deref()
        .is_some_and(|name| name.eq_ignore_ascii_case(spec))
        || toml
            .settings
            .agent_id
            .as_deref()
            .is_some_and(|id| id.eq_ignore_ascii_case(spec))
}

/// Load a bot's toml and turn it into launch options for this platform.
///
/// `root_dir`, when set, is relative to the toml; otherwise the toml's own
/// directory is the working directory, matching what RLBotServer does. If the
/// botpack's shared `torch-archive` is found above the config, its `lib`
/// directory is added to the loader path, because Bob-built ML bots link
/// against it instead of shipping torch themselves.
pub fn load_bot_config(path: &Path) -> BotConfig {
    let text = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("--rlbot-config: could not read {}: {e}", path.display()));
    bot_config_from_str(&text, path)
}

/// The subset of an RLBot `bot.toml` the viewer reads. Unknown keys (the
/// `[details]` section, loadout and logo paths, ...) are ignored.
#[derive(Debug, Default, Deserialize)]
struct BotToml {
    #[serde(default)]
    settings: BotSettings,
}

#[derive(Debug, Default, Deserialize)]
struct BotSettings {
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    agent_id: Option<String>,
    #[serde(default)]
    run_command: Option<String>,
    #[serde(default)]
    run_command_linux: Option<String>,
    #[serde(default)]
    root_dir: Option<String>,
    #[serde(default)]
    environment: BTreeMap<String, String>,
}

impl BotSettings {
    /// Pick the command for this platform. On Linux fall back to
    /// `run_command`, which is what source bots (e.g. `python bot.py`) often
    /// only have; the botpack always ships `run_command_linux`.
    fn command(&self, windows: bool) -> Option<String> {
        fn clean(command: Option<&String>) -> Option<String> {
            command
                .filter(|command| !command.trim().is_empty())
                .cloned()
        }

        if windows {
            clean(self.run_command.as_ref())
        } else {
            clean(self.run_command_linux.as_ref()).or_else(|| clean(self.run_command.as_ref()))
        }
    }
}

fn bot_config_from_str(text: &str, path: &Path) -> BotConfig {
    let config: BotToml = toml::from_str(text)
        .unwrap_or_else(|e| panic!("--rlbot-config: bad toml in {}: {e}", path.display()));
    let settings = config.settings;

    // Borrow the whole struct before moving any of its fields out.
    let command = settings.command(cfg!(windows)).unwrap_or_else(|| {
        let field = if cfg!(windows) {
            "run_command"
        } else {
            "run_command_linux (or run_command)"
        };
        panic!(
            "--rlbot-config: {} has no {field} for this platform",
            path.display()
        )
    });
    let agent_id = settings
        .agent_id
        .filter(|id| !id.trim().is_empty())
        .unwrap_or_else(|| {
            panic!(
                "--rlbot-config: {} has no settings.agent_id",
                path.display()
            )
        });

    let parent = path.parent().unwrap_or_else(|| Path::new("."));
    let working_dir = settings
        .root_dir
        .filter(|dir| !dir.trim().is_empty())
        .map_or_else(|| parent.to_path_buf(), |dir| parent.join(dir));

    let mut environment: Vec<(String, String)> = settings.environment.into_iter().collect();
    if let Some(lib) = torch_lib_dir(path) {
        let var = if cfg!(windows) {
            "PATH"
        } else {
            "LD_LIBRARY_PATH"
        };
        if !environment.iter().any(|(name, _)| name == var) {
            environment.push((var.to_string(), lib.to_string_lossy().into_owned()));
        }
    }

    let name = settings
        .name
        .filter(|name| !name.trim().is_empty())
        .unwrap_or_else(|| agent_id.clone());

    BotConfig {
        agent_id,
        name,
        launch: LaunchOptions {
            command,
            working_dir: Some(working_dir),
            environment,
        },
    }
}

/// Locate the botpack's shared torch libraries, if present. The RLBot GUI adds
/// these to the loader path too; do the same for manually launched bots.
fn torch_lib_dir(config: &Path) -> Option<PathBuf> {
    let mut dir = config.parent()?;
    for _ in 0..5 {
        let candidate = dir.join("torch-archive").join("torch").join("lib");
        if candidate.is_dir() {
            return Some(candidate);
        }
        dir = dir.parent()?;
    }
    None
}

/// Bevy resource holding the optional bridge server.
///
/// `BridgeServer` contains an `mpsc::Receiver`, which is `Send` but not `Sync`;
/// a `Mutex` makes the resource satisfy Bevy's `Send + Sync` requirement.
#[derive(Resource)]
pub struct Rlbot {
    server: Option<Mutex<BridgeServer>>,
    /// Latest controller state by player id. Index 0 is the human (unused by
    /// the bridge), index 1 is the bot.
    latest: Vec<InputState>,
    /// The bot process started by `--rlbot-command` / `--rlbot-config`. Its
    /// handles live in a `Mutex` so the resource stays `Sync`.
    child: Mutex<Option<Child>>,
    pub bot_player_id: i32,
}

impl Rlbot {
    /// Start the server, or return a disabled resource if binding fails.
    pub fn start(sim: &sim::Sim, options: RlbotOptions) -> Self {
        let agent_id = options.agent_id.clone();
        let players = vec![
            PlayerSpec {
                player_id: 0,
                name: "You".to_string(),
                team: 0,
                agent_id: None,
            },
            PlayerSpec {
                player_id: 1,
                name: options.bot_name.clone(),
                team: 1,
                agent_id: Some(agent_id.clone()),
            },
        ];
        let field: FieldSpec = sim.rlbot_field();
        let config = ServerConfig {
            bind_addr: SocketAddr::from(([127, 0, 0, 1], options.port)),
            players,
            field,
        };

        let server = match BridgeServer::start(config) {
            Ok(server) => {
                // `main` starts the bridge before Bevy's log plugin exists, so
                // this one goes to stderr rather than through `info!`.
                eprintln!(
                    "rlbot: listening on {} for one bot (agent id {:?})",
                    server.local_addr(),
                    agent_id
                );
                Some(Mutex::new(server))
            }
            Err(e) => {
                eprintln!("rlbot: could not listen on port {}: {e}", options.port);
                None
            }
        };

        // Only start the bot once the socket is ours; otherwise it would
        // connect to whatever else is on that port (normally RLBotServer).
        let child = if server.is_some() {
            spawn_bot(&options)
        } else {
            None
        };

        Self {
            server,
            latest: vec![InputState::default(); 2],
            child: Mutex::new(child),
            bot_player_id: 1,
        }
    }

    /// A resource with no server, used when `--rlbot` is not passed.
    pub fn disabled() -> Self {
        Self {
            server: None,
            latest: vec![InputState::default(); 2],
            child: Mutex::new(None),
            bot_player_id: 1,
        }
    }

    /// Drain bridge events. Bot inputs are kept; connection changes are logged.
    pub fn poll(&mut self) {
        let Some(server) = &self.server else {
            return;
        };
        let server = server.lock().expect("rlbot server mutex");
        while let Some(event) = server.try_recv() {
            match event {
                BridgeEvent::Connected { client_id } => {
                    info!("rlbot: client {client_id} connected");
                }
                BridgeEvent::Handshake {
                    agent_id,
                    team,
                    player_ids,
                    ..
                } => {
                    info!("rlbot: {agent_id:?} controls team {team}, player ids {player_ids:?}");
                }
                BridgeEvent::Ready { client_id } => {
                    info!("rlbot: client {client_id} ready; sending game packets");
                }
                BridgeEvent::Input {
                    player_id, input, ..
                } => {
                    if let Some(slot) = self.latest.get_mut(player_id as usize) {
                        *slot = input;
                    }
                }
                BridgeEvent::StopRequested {
                    client_id,
                    shutdown_server,
                } => {
                    info!("rlbot: client {client_id} requested stop (shutdown={shutdown_server})");
                }
                BridgeEvent::Disconnected { client_id } => {
                    info!("rlbot: client {client_id} disconnected");
                }
            }
        }
    }

    /// The latest input for `player_id`, if one has arrived.
    pub fn latest_input(&self, player_id: i32) -> Option<InputState> {
        self.latest.get(player_id as usize).copied()
    }

    /// Number of bots that have completed the handshake and are receiving ticks.
    pub fn ready_count(&self) -> usize {
        self.server.as_ref().map_or(0, |server| {
            server.lock().expect("rlbot server mutex").ready_count()
        })
    }

    /// Whether any ready bot has requested ball prediction.
    pub fn any_wants_prediction(&self) -> bool {
        self.server.as_ref().is_some_and(|server| {
            server
                .lock()
                .expect("rlbot server mutex")
                .any_wants_prediction()
        })
    }

    /// Send this tick's snapshot to every ready bot.
    pub fn broadcast(&self, snapshot: &GameSnapshot) {
        if let Some(server) = &self.server {
            if let Ok(server) = server.lock() {
                server.broadcast(snapshot);
            }
        }
    }
}

/// Start the bot process described by `options.launch`, if any.
///
/// It is run through the platform shell so bot configs can use ordinary
/// command strings (`./Element`, `uv run bot.py`, ...). The RLBot
/// environment variables are set here rather than in the bot config, exactly
/// as RLBotServer would set them.
fn spawn_bot(options: &RlbotOptions) -> Option<Child> {
    let launch = options.launch.as_ref()?;

    let mut command = if cfg!(windows) {
        let mut command = Command::new("cmd");
        command.args(["/C", &launch.command]);
        command
    } else {
        let mut command = Command::new("sh");
        command.args(["-c", &launch.command]);
        command
    };

    if let Some(dir) = &launch.working_dir {
        if !dir.as_os_str().is_empty() {
            command.current_dir(dir);
        }
    }

    command
        .env("RLBOT_SERVER_IP", "127.0.0.1")
        .env("RLBOT_SERVER_PORT", options.port.to_string())
        .env("RLBOT_AGENT_ID", &options.agent_id);
    for (name, value) in &launch.environment {
        command.env(name, value);
    }

    match command.spawn() {
        Ok(child) => {
            // Before Bevy's log plugin exists, so use stderr (same as above).
            match &launch.working_dir {
                Some(dir) => eprintln!("rlbot: launched {:?} in {}", launch.command, dir.display()),
                None => eprintln!("rlbot: launched {:?}", launch.command),
            }
            Some(child)
        }
        Err(e) => {
            eprintln!("rlbot: could not launch {:?}: {e}", launch.command);
            None
        }
    }
}

impl Drop for Rlbot {
    fn drop(&mut self) {
        if let Ok(mut child) = self.child.lock() {
            if let Some(mut child) = child.take() {
                let _ = child.kill();
                let _ = child.wait();
            }
        }
    }
}

/// Convert a bridge controller state to the viewer's input type.
///
/// The bridge has already flipped `steer`/`yaw`/`roll` from RLBot's right-hand
/// sign convention to RocketSim's left-hand one, so this is a straight copy.
pub fn to_sim_input(input: InputState) -> sim::Input {
    sim::Input::with_air(
        f64::from(input.throttle),
        f64::from(input.steer),
        f64::from(input.pitch),
        f64::from(input.yaw),
        f64::from(input.roll),
        input.boost,
        input.handbrake,
        input.jump,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    // A trimmed-down botpack config: both commands are the same, as they are
    // for most bots, so the test is platform-independent.
    const BOT_TOML: &str = r#"
[settings]
name = "Element"
agent_id = "Rangler/Element"
run_command = "uv run bot.py"
run_command_linux = "uv run bot.py"

[details]
language = "rlgym"
"#;

    #[test]
    fn bot_config_reads_the_botpack_fields() {
        let config = bot_config_from_str(BOT_TOML, Path::new("/bots/Element/Element.bot.toml"));

        assert_eq!(config.agent_id, "Rangler/Element");
        assert_eq!(config.name, "Element");
        assert_eq!(config.launch.command, "uv run bot.py");
        assert_eq!(
            config.launch.working_dir.as_deref(),
            Some(Path::new("/bots/Element"))
        );
    }

    #[test]
    fn bot_config_resolves_root_dir_and_environment() {
        let toml = r#"
[settings]
agent_id = "someone/bot"
run_command = "bot"
root_dir = "sub"

[settings.environment]
FOO = "bar"
"#;
        let config = bot_config_from_str(toml, Path::new("/bots/Thing/bot.toml"));

        // No name in the config: fall back to the agent id.
        assert_eq!(config.name, "someone/bot");
        assert_eq!(
            config.launch.working_dir.as_deref(),
            Some(Path::new("/bots/Thing/sub"))
        );
        assert_eq!(
            config.launch.environment,
            vec![("FOO".into(), "bar".into())]
        );
    }

    #[test]
    fn bot_command_prefers_the_platform_field() {
        let mut settings = BotSettings {
            run_command: Some("windows".into()),
            run_command_linux: Some("linux".into()),
            ..BotSettings::default()
        };

        assert_eq!(settings.command(true).as_deref(), Some("windows"));
        assert_eq!(settings.command(false).as_deref(), Some("linux"));

        // Source bots often only have `run_command`; Linux still falls back to it.
        settings.run_command_linux = None;
        assert_eq!(settings.command(false).as_deref(), Some("windows"));
    }

    #[test]
    fn torch_archive_is_found_above_the_bot_config() {
        let root = std::env::temp_dir().join(format!("rl_view_torch_{}", std::process::id()));
        let lib = root.join("torch-archive/torch/lib");
        std::fs::create_dir_all(&lib).unwrap();

        let config = root.join("Botpack/Element/Element.bot.toml");
        assert_eq!(torch_lib_dir(&config), Some(lib.clone()));

        std::fs::remove_dir_all(&root).unwrap();
    }

    fn temp_tree(tag: &str) -> PathBuf {
        let root = std::env::temp_dir().join(format!("rl_view_bots_{}_{tag}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(&root).unwrap();
        root
    }

    fn write_bot(root: &Path, dir: &str, file: &str, name: &str, agent_id: &str) -> PathBuf {
        let bot_dir = root.join(dir);
        std::fs::create_dir_all(&bot_dir).unwrap();
        let config = bot_dir.join(file);
        std::fs::write(
            &config,
            format!(
                "[settings]\nname = {name:?}\nagent_id = {agent_id:?}\nrun_command_linux = \"bot\"\n"
            ),
        )
        .unwrap();
        config
    }

    #[test]
    fn bot_names_resolve_under_the_botpack_root() {
        let root = temp_tree("names");
        let nexto = write_bot(&root, "bob_build/Nexto", "bot.toml", "Nexto", "rlgym/nexto");
        let element = write_bot(
            &root,
            "bob_build/Element",
            "bot.toml",
            "Element",
            "Rangler/Element",
        );
        let beast = write_bot(
            &root,
            "bob_build/Beast",
            "beast.bot.toml",
            "Beast from the East",
            "eastvillage/beast/v2.3.0",
        );

        // Directory name, display name and agent id all work, case-insensitively.
        for spec in ["Nexto", "nexto", "NEXTO", "rlgym/nexto", "RLGYM/NEXTO"] {
            assert_eq!(
                find_bot_configs(std::slice::from_ref(&root), spec),
                vec![nexto.clone()],
                "{spec}"
            );
        }
        assert_eq!(
            find_bot_configs(std::slice::from_ref(&root), "Element"),
            vec![element.clone()]
        );
        assert_eq!(
            find_bot_configs(std::slice::from_ref(&root), "beast"),
            vec![beast.clone()],
            "file stem"
        );
        assert_eq!(
            find_bot_configs(std::slice::from_ref(&root), "Beast from the East"),
            vec![beast.clone()]
        );
        assert!(find_bot_configs(std::slice::from_ref(&root), "No Such Bot").is_empty());

        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn bot_config_accepts_a_directory() {
        let root = temp_tree("dir");
        let nexto = write_bot(&root, "bob_build/Nexto", "bot.toml", "Nexto", "rlgym/nexto");

        let resolved = resolve_bot_config(root.join("bob_build/Nexto").to_str().unwrap());
        assert_eq!(resolved, nexto);

        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn duplicate_bot_names_are_ambiguous() {
        let root = temp_tree("dupes");
        write_bot(&root, "pack_a/Nexto", "bot.toml", "Nexto", "a/nexto");
        write_bot(&root, "pack_b/Nexto", "bot.toml", "Nexto", "b/nexto");

        assert_eq!(
            find_bot_configs(std::slice::from_ref(&root), "Nexto").len(),
            2
        );

        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn no_match_lists_the_installed_bots() {
        let root = temp_tree("message");
        write_bot(
            &root,
            "pack/Element",
            "bot.toml",
            "Element",
            "Rangler/Element",
        );
        write_bot(&root, "pack/Nexto", "bot.toml", "Nexto", "rlgym/nexto");

        let configs = bot_configs_in(std::slice::from_ref(&root));
        let message = no_bot_message("elementv5", std::slice::from_ref(&root), &configs);

        assert!(message.contains("\"elementv5\""), "{message}");
        assert!(message.contains("Element (Rangler/Element)"), "{message}");
        assert!(message.contains("Nexto (rlgym/nexto)"), "{message}");

        std::fs::remove_dir_all(&root).unwrap();
    }
}
