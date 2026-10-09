# AGENTS.md

Notes for coding agents working in this repo. Keep it short; update it when a
rule changes.

## What this is

A Rocket League-style prototype. The active code is the Rust implementation
under `rust/`:

- `rust/src/` — `rl_sim`: a hand-rolled, dependency-free, deterministic physics
  simulation (arena + ball + car). No game engine.
- `rust/view/` — `rl_view`: a Bevy 3D viewer that reads `rl_sim` and draws it.
- `rust/tests/` — physics behaviour and determinism tests.

An earlier Godot 4 prototype was removed once the Rust implementation became the
one we are building. `rust/docs/PHYSICS.md` still cites it as the motivation for
hand-rolling the physics.

Start with `rust/docs/PHYSICS.md` (why hand-rolled, and its honest caveats) and
the root `README.md` (how to run it).

## Determinism is the point — don't break it

`rl_sim` must produce a bit-identical state stream for a given input stream, so
client-side prediction and replays can work. Inside the simulation step:

- **No transcendentals.** `sin`/`cos`/`tan`/`powf`/`exp`/`ln` call platform libm
  and are not portable across machines. Rotation goes through `math::rotate_y`.
  Otherwise only `+ - * /` and `sqrt`.
- **No `mul_add`/FMA** — contraction is a real source of cross-target drift.
- **No RNG, threads, wall-clock, or unordered-map iteration.**
- **`DT` is a compile-time constant** (120 Hz). Never introduce a variable `dt`.
- **New state must be hashed.** Add fields to `StateHasher` in `Car::hash_into` /
  `Ball::hash_into`, and make sure `Clone` copies them, or divergence goes
  undetected.
- **`-0.0` is canonicalised** in the hash — keep it that way.
- `rl_sim` has **zero dependencies** by design. New crates go at most into
  `rust/view/`.

The viewer must never feed back into the simulation: it reads `World` and writes
transforms, nothing more.

## Build / run / validate

```sh
cd rust
cargo run -p rl_view                   # play (dev profile is already playable)
cargo test --workspace                 # the check that matters
cargo clippy --workspace --all-targets # kept clean
```

- The dev profile is tuned on purpose (own crates `opt-level = 1`, deps `3`).
  **Don't default to `--release`**: it uses a separate target dir and rebuilds
  Bevy, which takes minutes.
- Linux needs **`libudev-dev`** (gamepad via `bevy_gilrs`). Audio/ALSA is
  deliberately excluded; Bevy features are listed explicitly in
  `rust/view/Cargo.toml`.
- Project tasks are defined in `.zed/tasks.json`.

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

- **Chase camera:** don't clamp it to the wall plane (it collapses onto the car),
  and don't aim at a weighted average of car/ball positions (it ends up facing
  the floor). See the comments in `rust/view/src/main.rs`.
- **Car turning:** never derive yaw rate from a lateral-acceleration budget — it
  rises as `1/v` and pivots the car on the spot. The rate is bounded directly.
