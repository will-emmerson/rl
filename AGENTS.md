# AGENTS.md

Notes for coding agents working in this repo. Keep it short; update it when a
rule changes.

## What this is

A Rocket League-style prototype. The active code is a hand-rolled,
dependency-free simulation with a Bevy viewer:

- `src/` — `rl`: the deterministic simulation (arena + ball + car + boost pads). No game
  engine, no dependencies.
- `view/` — `rl_view`: a Bevy 3D viewer that reads `rl` and draws it.
- `tests/` — physics behaviour and determinism tests.

An earlier Godot 4 prototype was removed once the Rust implementation became the
one we are building. `docs/PHYSICS.md` still cites it as the motivation for
hand-rolling the physics.

Start with `docs/PHYSICS.md` (why hand-rolled, and its honest caveats) and
`README.md` (how to run it).

## Determinism is the point — don't break it

`rl` must produce a bit-identical state stream for a given input stream, so
client-side prediction and replays can work. Inside the simulation step:

- **No transcendentals.** `sin`/`cos`/`tan`/`powf`/`exp`/`ln` call platform libm
  and are not portable across machines. Rotation goes through `math::rotate_y`.
  Otherwise only `+ - * /` and `sqrt`.
- **No `mul_add`/FMA** — contraction is a real source of cross-target drift.
- **No RNG, threads, wall-clock, or unordered-map iteration.**
- **`DT` is a compile-time constant** (120 Hz). Never introduce a variable `dt`.
- **New state must be hashed.** Add fields to `StateHasher` in `Car::hash_into` /
  `Ball::hash_into`, and make sure `Clone` copies them, or divergence goes
  undetected. (The car-ball extra-impulse cooldown is hashed this way.)
- **`-0.0` is canonicalised** in the hash — keep it that way.
- `rl` has **zero dependencies** by design. New crates go at most into
  `view/`.

The viewer must never feed back into the simulation: it reads `World` and writes
transforms, nothing more. (The `sin`/`cos` ban is about the simulation step — the
viewer is free to use trigonometry.)

## Physics constants come from RocketSim

Gameplay constants (arena extents, ball radius/mass/friction, boost accel,
hitbox size, car-ball restitution, and the car-ball _extra impulse_) are taken
from **[RocketSim](https://github.com/ZealanL/RocketSim)**, not memory or
community lore. Unreal units are centimetres, so the sim's SI values are the
RocketSim number / 100. When you add or change one, cite the RocketSim name in
its doc comment, as `arena.rs` / `ball.rs` / `car.rs` already do.

The car's _handling_ is now largely ported from RocketSim: steering uses RL's
`STEER_ANGLE_FROM_SPEED_CURVE` through a bicycle model
(`omega = v * tan(steer) / wheelbase`), the analog powerslide, the jump
(`JUMP_IMMEDIATE_FORCE` + held `JUMP_ACCEL`), the mid-air air-control torque
(`CAR_AIR_CONTROL_TORQUE`), and flips/dodges (`_UpdateDoubleJumpOrFlip`). What is
_not_ reverse-engineered is the tyre grip (`CAR_GRIP_KEEP` / `CAR_DRIFT_KEEP`) and
the engine/brake forces (`CAR_ENGINE_ACCEL`, `CAR_BRAKE_DECEL`); suspension, tyre
slip and auto-roll are not modelled at all (a flipped car is snapped upright on
landing, so wall driving does not work yet).

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
  deliberately excluded; Bevy features are listed explicitly in
  `view/Cargo.toml`.
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
  the floor). See the comments in `view/src/main.rs`.
- **Car turning:** never derive yaw rate from a lateral-acceleration budget — it
  rises as `1/v` and pivots the car on the spot. The rate now comes from RL's
  steer-angle curve as `omega = v * tan(steer) / wheelbase`, which is
  proportional to `v`, so a standstill cannot pivot (`math::tan_small` keeps it
  libm-free).
- **Air control / flips: the world frame is left-handed.** The car's body frame
  is `(forward, right, up)` with `right = forward x up`, which is left-handed,
  whereas Rocket League's is right-handed. So RL's torque cross-products cannot
  be copied verbatim — the `apply_air_control` / `start_flip` axes were derived
  so the _behaviour_ matches (stick up = frontflip, stick left = yaw left, roll
  right = roll right). The magnitudes and timings are still RL's constants.
- **Attitude:** the car carries `forward` and `up` (orthonormal), integrated from
  `ang_vel` and re-orthonormalised every step. If you touch that, keep it
  orthonormal and hash every new field, or divergence hides.
