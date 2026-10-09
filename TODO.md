# QUESTIONS

- how to do physics tests properly, make parity with actual rocket league a test suite?

# TODO

- can I speed up compilation, some setting?
- get controller working properly with my normal mapping
- aim for just being able to bounce a ball against a wall
- I need to break things down into really simple things:
  - get the car moving properly just on flat ground
  - get the ball moving properly
- start comparing our physics with the real game by looking at replay files:
  - I should be able to produce some replay files doing really simpler things like:
    - driving forward, reversing, to make sure acceleration is correct
    - turning
    - jumping
    - etc

# DOING

# DONE

- stop rust compile hanging the laptop
- stop zed doing cargo check
  - `.zed/settings.json`: rust-analyzer `checkOnSave: false` (no auto `cargo
check` on every save) and `cargo.targetDir: true` (rust-analyzer uses its own
    `rust/target/rust-analyzer`, so it never holds the lock a build waits on).
    Use the `RL: cargo check` task for on-demand diagnostics.
- get controller working
- initial project setup with basic car and ball in arena
