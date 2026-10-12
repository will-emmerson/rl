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

- aim for just being able to bounce a ball against a wall
- I need to break things down into really simple things:
  - get the car moving properly just on flat ground
  - get the ball moving properly
- port the rest of the car handling from RocketSim:
  - dodges/flips and air control (`CAR_AIR_CONTROL_TORQUE`)
  - suspension and tyre slip, so the turn radii are more than a bicycle-model
    approximation
- start comparing our physics with the real game by looking at replay files:
  - I should be able to produce some replay files doing really simpler things like:
    - driving forward, reversing, to make sure acceleration is correct
    - turning
    - jumping
    - etc

# DOING

# DONE

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
