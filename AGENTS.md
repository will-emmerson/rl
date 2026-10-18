# AGENTS.md

Notes for coding agents working in this repo. Keep it short; update it when a
rule changes.

## What this is

A Rocket League-like game: a Bevy 3D viewer (`rl_view`) that runs
**[RocketSim](https://github.com/ZealanL/RocketSim)** (the pure-Rust v3) for
physics, and draws RocketSim's own arena collision mesh.

- `view/` — `rl_view`. `view/src/sim.rs` is the **only** place that knows about
  RocketSim: it owns the `Arena`, converts RocketSim's frame and units into the
  viewer's, exposes the car/ball/pads/score the scene reads, and builds the
  collision mesh. `view/src/main.rs` is the scene, input wiring, camera and HUD;
  `view/src/controls.rs` parses the keyboard/gamepad bindings;
  `view/src/rlbot.rs` is the viewer-side bridge resource (it converts bot inputs
  into `sim::Input`, calls `sim::rlbot_snapshot`, and can launch the bot process
  from `--rlbot-config` (bot name or toml) or `--rlbot-command`).
- `rlbot_bridge/` — the bot-facing half of RLBot v5 as a standalone crate: TCP
  framing, FlatBuffers, handshake, per-client threads, packet construction. It
  knows nothing about Bevy or RocketSim; the viewer hands it `GameSnapshot`s.
- `tools/rlbot_test_bot.py` — a tiny RLBot v5 Python bot for exercising the
  bridge end to end.
- `tools/rlacd.py` — a Linux/Proton port of RLArenaCollisionDumper: dumps RL's
  arena collision meshes, which RocketSim needs at runtime (see below). Upstream
  is Windows-only.
- `docs/PHYSICS.md` — a write-up of the hand-rolled `rl` simulation that used to
  be this repo's physics. **`rl` is retired**; the doc is kept as history.

The workspace is virtual; the crates are `view/` and `rlbot_bridge/`.

## RocketSim is the reference

**[RocketSim](https://github.com/ZealanL/RocketSim) is the reference for Rocket
League physics and dimensions** — use it (not memory or community lore) for both
the numbers and the model. The viewer does not approximate the game: it _runs_
RocketSim, and builds every constant it needs from RocketSim too.

Units are Unreal units (centimetres, `÷100` for metres). RocketSim's frame is
right-handed, Z-up, `+X` forward, with the long axis on `Y`; the viewer's is
Bevy's right-handed Y-up frame with the long axis on `Z`. `view/src/sim.rs`
bridges them with a _rotation_, `view = (-x, z, y)` — not a mirror, so spin (a
pseudovector) needs no sign flip. All of that lives in `sim.rs`; nothing else
should know RocketSim's frame or units.

## Build / run / validate

```sh
cargo run -p rl_view                   # play (dev profile is already playable)
cargo run -p rl_view -- --fullscreen --stats  # native-res fullscreen + frame timing
cargo run -p rl_view -- --resolution 1920x1080 --stats --no-vsync  # windowed, uncapped
cargo run -p rl_view -- --rlbot        # same, plus an RLBot v5 bot slot
cargo run -p rl_view -- --rlbot --rlbot-config bot.toml  # ...and launch the bot
cargo test --workspace                 # the check that matters
cargo clippy --workspace --all-targets # kept clean
```

- `--fullscreen` is borderless fullscreen on the primary monitor at its native
  resolution. `--resolution WxH` sets a windowed physical-pixel size (ignored
  by fullscreen), and `--no-vsync` uncaps the frame rate. `--stats` adds a
  frame-time/FPS/max/1%-low line to the HUD; **F3** toggles it at runtime.
- `--quality low|medium|high` selects the render-cost preset (`high` is the
  normal look; `low` is MSAA-off, no shadows, no bloom). `--msaa off|2|4|8`
  overrides just the MSAA level. At 1080p, 4x MSAA + shadows + bloom on an
  integrated GPU is what causes frame-time spikes; the presets are the lever.

- The dev profile is tuned on purpose (own crates `opt-level = 1`, deps `3`).
  **Don't default to `--release`**: it uses a separate target dir and rebuilds
  Bevy, which takes minutes.
- Linux needs **`libudev-dev`** (gamepad via `bevy_gilrs`). Audio/ALSA is
  deliberately excluded; Bevy features are listed explicitly in `view/Cargo.toml`.
- The viewer needs RL's dumped arena meshes in `RL/collision_meshes/soccar/` at
  runtime or it exits with a message. Dump them with `tools/rlacd.py`
  (Linux/Proton) or RLArenaCollisionDumper (Windows).
- `camera.toml` at the repo root holds Rocket League camera settings (distance,
  height, angle, FOV, stiffness, swivel/transition speed). It is optional —
  missing means built-in RL defaults — and hot-reloads while the viewer runs.
  `--camera-config PATH` overrides it.
- `controls.toml` at the repo root holds the keyboard/gamepad bindings. It is
  optional (built-in defaults mirror RL's pad layout), hot-reloads, and is
  overridden by `--controls-config PATH`. Bindings live in `view/src/controls.rs`;
  input _signs_ still live in `read_input` (see the input-signs trap below).
- Project tasks are defined in `.zed/tasks.json`.

**To see your work, render a PNG** — both forms exit on their own:

```sh
cargo run -p rl_view -- --screenshot out.png [TICK]          # chase cam, drives
cargo run -p rl_view -- --screenshot out.png --cam X,Y,Z --look X,Y,Z  # free cam
```

- `--screenshot PATH [TICK]` drives the self-driving script to `TICK` (default
  **250**) and shoots the chase cam — a _dynamic_ pose. With no `PATH`,
  `screenshot.png` is used.
- `--cam X,Y,Z` / `--look X,Y,Z` pin the camera at that eye point looking at that
  target and **skip the sim**: instant, for eyeballing
  geometry/materials/lighting from a chosen angle — but it only ever shows the
  kickoff scene (no dynamic pose). Either flag alone works (the other falls back
  to an overview default), and `TICK` is then ignored.
- `--gamepad-debug` logs every raw pad event; `--camera-config` and
  `--controls-config` choose the two settings files.

## Bevy is pinned to 0.19.1

Its API differs from most docs and training data — check the crate source rather
than memory. Known 0.19 details:

- `MessageWriter` / `MessageReader` for events (not `EventWriter`/`EventReader`).
- `DirectionalLight { shadow_maps_enabled, .. }` (not `shadows_enabled`).
- HUD text: `Text::new` + `children![TextSpan::new(..)]`; `px(12)` for UI sizes.
- `GlobalAmbientLight` for ambient light.

Upgrading Bevy is a deliberate migration, not a version-string bump.

## Workflow

- **Do not commit or push until the user has tested the change**, unless they
  explicitly ask. Running `cargo test` + `clippy` locally first is still
  expected.
- **Bound long-running commands** (`timeout_ms`). Bevy's `--quit-after` counts
  _frames_, not seconds. Never leave a server or watcher running unbounded.

## Traps already paid for

- **Chase camera:** RL parity means a world-upright, yaw-only boom, not a boom
  built in the car's frame. The camera sits at `car - heading * distance +
Y * height`, where `heading` is the car's horizontal forward in car cam or the
  horizontal bearing to the ball in ball cam. `CameraSettings::angle` pitches the
  _view_, not the boom, which is what leaves the car low in the frame. The arena
  is a see-through cage, so the camera is _not_ clipped to its walls — it may sit
  outside the shell and look back in — but it is kept above the pitch
  (`CAMERA_FLOOR_MARGIN`). Do not build the boom from the car's own `up`: that
  rolls the camera with the car on a wall or in the air and is exactly the
  behaviour this model replaced. On the ground (including a wall) the heading is
  the car's _yaw_, derived from two axes: the nose and the car's right. Each
  implies a yaw (the horizontal projection of the nose, or of the right rotated
  a quarter turn), and the code follows whichever candidate is closest to the
  heading it already had. That keeps a nose-up car on a wall stable and avoids
  the 180° swing a nose-only projection gives when it reverses through vertical.
  While **airborne and tilted** (`CarView::is_on_ground` false and `up.y` below
  ~0.95) the heading is deliberately _held_ instead: a dodge rotates the whole
  car about a horizontal axis, so both axes swing even though its yaw does not.
  Following them is what made the camera swivel during front/back and diagonal
  flips; RL's car cam does not. Tracking resumes as the car levels out or lands.
  Don't aim at a weighted average
  of car/ball positions (it ends up facing the floor). Settings live in
  `camera.toml` and hot-reload; see the comments in `view/src/main.rs`.
- **Wall/ceiling trim is flat, visual-only geometry.** `arena_grid_mesh` lays
  one-sided quads 1 cm inside the shell: wall uprights and rails, a ceiling
  grid, and a grid above each goal. Keep them flat (no thickness) so they never
  poke into the car when it drives on a wall. They are culled from outside like
  the shell, preserving the see-through cage.
- **Boost pads are two different props, as in RL.** A small pad is a hexagonal
  metal plate with a dark puck, a glowing ring/core and four translucent flame
  panels. Only the glow is collected: the ring, core and panels vanish on
  pickup while the hexagonal marker stays on the pitch. A big pad is a fiery orb
  (an icosphere with low-frequency displacement and a molten-to-charred
  vertex-colour gradient, so the spin shows), a ring and a light beam over a
  dark cross platform that stays put. Flames and beams are `unlit` +
  `AlphaMode::Add`; Bevy replaces `base_color` with the mesh's vertex colour
  (alpha included) there, so their gradient and brightness live in
  `with_vertical_gradient`, not in `emissive`. Motion lives in
  `animate_boost_pads`; visibility still lives in `sync_visuals` via
  `Vis::BoostPad(index)` on a parent.
- **The pitch is untracked geometry, not the collision floor.** `pitch_mesh` in
  `main.rs` is vertex-coloured quads 1 cm above the collision floor: 10.24 m
  mowed squares, thin white marking quads, a 48-segment centre ring and
  team-tinted goal boxes, all procedurally built (no texture). The arena
  collision mesh itself carries only region vertex colours (green floor, slate
  walls/roof, team-tinted ends). Don't put markings or a repeating grid back on
  the collision mesh.
- **The arena mesh is two sources, not one.** `sim::arena_collision_mesh` is
  RocketSim's dumped `.cmf` triangles _plus_ four flat quads. The dumped meshes
  carry only the _curved_ parts (corners, wall-to-floor/wall-to-ceiling fillets,
  goal ends); RocketSim makes the flat floor, ceiling and long side walls from
  static planes, so those must be added or the pitch/ceiling/side walls are holes
  you see the background through. The mesh is wound inward and drawn with
  `cull_mode: Some(Face::Back)`, so it is visible from inside and culled from
  outside — that is what keeps the see-through cage.
- **Seed RocketSim.** `ArenaConfig::with_rng_seed(0)` in `Sim::new`. Unseeded,
  RocketSim draws from the RNG (per tick), so a run — and even the self-driving
  capture — varies launch to launch.
- **Input signs are measured, not reasoned.** `sim.rs::controls_from` passes the
  viewer's `Input` straight into RocketSim's `CarControls` with **no** sign
  changes, and that is correct. `examples/axis_probe.rs` measures RocketSim
  directly — run it: positive `steer`, `yaw` and `roll` all move/rotate the car to
  its **left**, and positive `pitch` is nose-up. That is why the viewer keeps
  `yaw = steer` (same sign as steering, so a dodge leans the way the stick points)
  and builds `pitch` as `-throttle` (stick-up = frontflip). The frame bridge is a
  rotation, so signs survive it — negating a component in `controls_from` only
  mirrors the axis and has bitten before. If an axis ever looks wrong, re-run the
  probe and change the sign where the input is built (`read_input` in `main.rs`),
  not here.
- **RLBot controls pass straight through, like the state does.** RLBot's
  `ControllerState` documents positive `steer`/`yaw`/`roll` as "right", but the
  bridge sends the bot a **RocketSim** world: positions, rotations and angular
  velocities go over the wire unchanged (the same conversion as
  `VirxEC/rlbot-rocketsim`). A bot trained on RLGym/RocketSim reads that frame,
  so its controls must be applied in it too — `protocol::input_from_flat` and
  `input_to_flat` are **straight copies**. Negating just the controls mirrors
  them relative to everything the bot sees; Nexto drove in circles until this
  was fixed. Do not move a negation into `controls_from` either: that would
  break the viewer's own keyboard/pad path.
- **RLBot v5 framing is `u16 big-endian length || FlatBuffer`**, with the packet
  type inside `InterfacePacket`/`CorePacket`; there is no separate type byte.
  Older RLBotSim drafts used a type prefix, so don't copy their framing. The
  bridge uses the MIT-licensed `rlbot_flat` crate for the generated types and
  `planus` builder.
- **`cargo test` runs from the crate root (`view/`).** The viewer normally runs
  from the workspace root, where `./collision_meshes` resolves; `sim::init_rocketsim`
  falls back to `../collision_meshes` so tests keep working without duplicating
  the meshes.
- **Pad trigger axes start at `0.0`, not at rest.** Bevy pre-fills every axis with
  `0.0` until the pad sends its first event, but `LeftZ`/`RightZ` rest at `-1.0`,
  so `0.0` reads as half-pressed. `controls::seed_trigger_axes` sets configured
  trigger axes to `-1.0` when a pad connects or bindings reload; without it, using
  and releasing one trigger makes the untouched one look half-pressed and the car
  creeps backwards.
- **Screenshots need a render warm-up.** The first ~second of 3D frames is the
  bare clear colour (pipeline compilation + window bring-up); a frame grabbed
  before that is blank. The driven `--screenshot N` clears it by running to tick N
  (~125 frames), and the free-cam path gates on `FREE_SHOT_MIN_FRAMES` +
  `FREE_SHOT_MIN_SECS`. Don't "speed up" a capture by grabbing an early frame.
  The sim isn't what's slow — the frames are.
