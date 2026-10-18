# INFO

- https://github.com/ZealanL/RocketSim
- https://github.com/VirxEC/rocketsim-rs
- https://wiki.rlbot.org/v5/

# QUESTIONS

# TODO

- ignore todo
- add the project goals:
  - work on low hardware / low poly
  - linux/mac compatibility without wine, etc
  - better bots (using rlbot)
  - load straight into freeplay without loads of button clicks
  - better training:
    - include custom training packs
    - use AI to help players work on their weak points
  - eventually, an online mode with zero smurfs (somehow)
- simplify/remove the collision mesh?:
  - why can't it be described with equations?
- sound (engine sound, boost sound)
- settings:
  - resolution
  - bots
  - unlimited boost
  - cpu graph
  - graphics detail
  - as a yaml file? or sqlite?
  - controller, keyboard shortcuts
  - camera
  - in game using a UI would be good
- rlbot integration
- make a release build to see how small/fast it can be
- show cpu/ram usage, graph?
- test
- make sure there is parity:
  - https://wiki.rlbot.org/v4/botmaking/useful-game-values/

# DOING

- make it look more like RL but keep low-poly:
  - show agent some images?

# DONE

- camera parity
- viewer screenshots: `--cam x,y,z` / `--look x,y,z` pin the camera and skip the
  simulation, so a static shot from a chosen angle (`Mode::Capture { free_cam }`)
  is instant instead of driving the sim to a tick. The driven `--screenshot PATH
[TICK]` stays for dynamic poses. The wait can't reach zero: the first fully
  drawn frame is ~a second in (pipeline/window warm-up), so a free shot still
  waits a short, bounded warm-up.
- curved arena + wall/ceiling driving: the arena is a signed-distance field (a
  box with chamfered vertical corners and filleted floor/wall and wall/ceiling
  joins), and the car drives in the frame of whatever surface it is on, so it
  can drive up the walls, across the ceiling and fall back off. Radii are
  gameplay guesses, not RocketSim figures. The viewer draws the collision
  surface itself, with a 2 m / 10 m grid so the corners can be eyeballed, and
  builds the chase-camera boom in the car's own frame so ball cam keeps tracking
  the ball on a wall.
- boost pads: the 34 Soccar pads (6 big / 28 small) at RocketSim's positions,
  with RL's amounts and cooldowns; the car takes one by driving over it.
- ported flips and air control from RocketSim (`_UpdateAirTorque` /
  `_UpdateDoubleJumpOrFlip`): a full 3D car attitude in the air, pitch/yaw/roll
  air-control torque with damping, and dodges (initial impulse + fixed-time flip
  torque). Because the sim's frame is left-handed the torque axes had to be
  re-derived rather than copied.
- ported the car handling from RocketSim: steering via
  `STEER_ANGLE_FROM_SPEED_CURVE` through a bicycle model (turn radius now grows
  from ~1.4 m to ~13 m with speed), the analog powerslide, and the jump model
  (`JUMP_IMMEDIATE_FORCE` + held `JUMP_ACCEL`, so a tap and a hold differ).
- parity pass: arena/ball/car constants from RocketSim (`RLConst.h`), including
  the real Soccar arena (81.92 x 102.40 x 20.48 m), ball radius/mass/friction,
  the Octane hitbox, boost accel ground/air, speed+spin caps, and the car-ball
  _extra impulse_ (`Ball::_OnHit`).
- stop cargo test using all cpu
- can I speed up compilation, some setting?
- get controller working properly with my normal mapping
- stop rust compile hanging the laptop
- stop zed doing cargo check
  - `.zed/settings.json`: rust-analyzer `checkOnSave: false` (no auto `cargo
check` on every save) and `cargo.targetDir: true` (rust-analyzer uses its own
    `rust/target/rust-analyzer`, so it never holds the lock a build waits on).
    Use the `RL: cargo check` task for on-demand diagnostics.
- get controller working
- initial project setup with basic car and ball in arena
