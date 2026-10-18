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
  collision mesh. `view/src/main.rs` is the scene, input, camera and HUD.
- `tools/rlacd.py` — a Linux/Proton port of RLArenaCollisionDumper: dumps RL's
  arena collision meshes, which RocketSim needs at runtime (see below). Upstream
  is Windows-only.
- `docs/PHYSICS.md` — a write-up of the hand-rolled `rl` simulation that used to
  be this repo's physics. **`rl` is retired**; the doc is kept as history.

The workspace is virtual; `view/` is the only crate.

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
cargo test --workspace                 # the check that matters
cargo clippy --workspace --all-targets # kept clean
```

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
- `--gamepad-debug` logs every raw pad event — the viewer's only other flag.

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
  behaviour this model replaced. A vertical car heading (nose straight up a wall)
  has no horizontal direction; keep the last yaw rather than clamping the boom to
  the wall plane (that collapses it onto the car). Don't aim at a weighted average
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
- **Screenshots need a render warm-up.** The first ~second of 3D frames is the
  bare clear colour (pipeline compilation + window bring-up); a frame grabbed
  before that is blank. The driven `--screenshot N` clears it by running to tick N
  (~125 frames), and the free-cam path gates on `FREE_SHOT_MIN_FRAMES` +
  `FREE_SHOT_MIN_SECS`. Don't "speed up" a capture by grabbing an early frame.
  The sim isn't what's slow — the frames are.
