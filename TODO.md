# INFO

- https://github.com/ZealanL/RocketSim
- https://github.com/VirxEC/rocketsim-rs
- https://wiki.rlbot.org/v5/

# QUESTIONS

- how to do physics tests properly, make parity with actual rocket league a test suite?
  - started: the constants now come from RocketSim, but a real parity harness
    (record inputs + state hashes from a reference build, assert in CI) is still
    the missing piece.

# TODO

- free-cam shots show only the kickoff scene; add pose flags (`--car` / `--ball`)
  so a specific dynamic setup can be shot instantly without simulating to it
- aim for just being able to bounce a ball against a wall
- I need to break things down into really simple things:
  - get the car moving properly just on flat ground
  - get the ball moving properly
- port the rest of the car handling from RocketSim:
  - suspension and tyre slip, so the turn radii are more than a bicycle-model
    approximation, and so wall driving is a wheel model rather than "glue the
    car to the surface"
- start comparing our physics with the real game by looking at replay files:
  - I should be able to produce some replay files doing really simpler things like:
    - driving forward, reversing, to make sure acceleration is correct
    - turning
    - jumping
    - etc

# DOING

# DONE

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
