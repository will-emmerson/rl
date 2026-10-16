> **Current status.** `rl` has been retired and removed from the repo. The
> `rl_view` viewer runs on [RocketSim](https://github.com/ZealanL/RocketSim) v3
> (pure Rust) and draws RocketSim's own collision mesh. This document is kept as
> a write-up of the hand-rolled simulation that used to be here, and of the
> reasoning behind it; read it as a description of _`rl`_, not of how the game
> behaves today.

# Why hand-roll the physics?

Short version: for _this_ game a physics engine is mostly overhead, and the one
thing you actually want from it — determinism — is the one thing general
engines are worst at. But some of the reasoning in the original pitch doesn't
hold up, so this document separates what's true from what isn't.

## What the engine is doing for you, and when it isn't

A general 3D engine earns its keep on: broadphase, hierarchical contact
generation, constraint solvers, stacking stability, joints, sleeping, CCD.

This game has **one sphere, one box, and a rounded arena**. There is no stacking,
no joints, no chains. Broadphase is a no-op. So ~95% of what the engine provides
goes unused, and what remains is maybe 300 lines of arithmetic you can read in
one sitting.

The cost of using one shows up when the game's behaviour is _specific_ and you
have to fight the engine to express it. That is exactly what happened building
an earlier Godot prototype (since removed from the repo):

- `VehicleBody3D` rolled the car over in corners, so I had to add a fake
  anti-roll torque — essentially a hand-written physics hack layered on top of
  the engine's physics.
- `VehicleBody3D.brake` is clamped as an _impulse_ per wheel per step, not a
  force as the docs claim, which silently backflipped the car.
- The engine's suspension was underdamped once I raised stiffness, because the
  damping ratio works out to `damping / sqrt(stiffness)`.
- The ball got crushed between the car and a wall, and the engine squirted it
  clean through a 4 m thick wall — a solver blow-up.

Three of those four are _symptoms of using a general solver for a specific
problem_. In a hand-rolled model they simply don't exist: there is no
suspension to mistune, no solver to blow up, and nothing to roll over because
the car isn't a rigid body at all.

And the fourth — the ball getting crushed and ejected — is **a mechanic in
Rocket League, not a bug**. It's a _pinch_. Hand-rolling lets you decide what a
pinch does. With an engine, you're deciding how to stop it doing something
unintended.

## The specific claims

### "Sharper parts of the car mean harder shots"

**No, and this isn't how RL works.** RL's car-to-ball response is impulse-based
against a _box_ hitbox. There is no hardness or sharpness term. Every car is a
box; the difference between an Octane and a Dominus is the box's dimensions,
not its edge profile.

What actually changes a shot:

- **The contact point relative to the hitbox.** Hit with the corner and the
  normal points somewhere different, so the ball leaves in a different
  direction and picks up spin. That reads as "juicier", but it's direction, not
  force.
- **The car's velocity at the contact point**, including its rotation. This is
  why a flick or a well-timed dodge hit is harder — the contact point is
  moving faster than the car's centre of mass.
- **The pinch.** Ball squeezed between car and wall/floor. Both constraints
  fire in the same tick and the velocities add. This is where genuinely absurd
  ball speeds come from.

The implementation here does all three: the contact point carries the car's
linear _and_ angular velocity, friction at the contact converts sliding into
spin, and pinches fall out of sequential resolution rather than being clamped
away.

### "Ball on wall is just angle in = angle out"

**Only in the special case**, and this is a good example of where the intuition
is half right. For a sphere hitting a flat wall with **no spin and no
friction**, the reflection is exactly specular. `tests/physics.rs`
`head_on_floor_bounce_is_specular` asserts vx and vz are unchanged to 1e-12.

But with friction, some of the tangential sliding converts into rotation, so:

1. the ball leaves **slower** than it arrived,
2. the outgoing angle is **not** the mirror of the incoming one, and
3. the ball is now **spinning**, which the Magnus term turns into a curve in
   flight.

`angled_bounce_trades_sliding_for_spin` asserts all three. This is why
wall-play in RL isn't a pool table.

### "Ball on car is a tangent reflection"

**No** — and this is the important one. A wall is static, so its velocity is
zero and the reflection is geometry. A car is _moving_, so the response depends
on the **relative velocity at the contact point**:

```
j = -(1 + e) * dot(v_ball_contact - v_car_contact, n) / (1/m_ball + 1/m_car)
```

The car's contribution includes its angular velocity, so where on the hitbox
you make contact genuinely matters. This is why in RL you position your car and
angle it — the same contact point, approached from a different car velocity,
produces a different shot.

The mass ratio is 180 : 30 (a car is six times the ball, matching RocketSim's
`BALL_MASS_BT = CAR_MASS_BT / 6`), so the inelastic collision alone gives the
ball about 86% of the relative velocity. That on its own would feel weedy next
to Rocket League, which is why RL adds a _separate_ "extra impulse" on top of
the collision — covered below.

## Determinism

This is the strongest argument for the approach, and the reason it's worth
doing from the start rather than retrofitting.

The simulation is _defined_ at 120 Hz. `DT` is a compile-time constant, every
per-tick constant is expressed for that rate, there is no RNG, no parallelism,
no hash-map iteration, and no dependence on wall-clock time. `World` is
`Clone`, ~200 bytes, and `state_hash()` gives a 64-bit fingerprint.

Practices that make it hold:

- **No transcendentals in the step.** `f64::sin`/`cos` call platform libm and
  are _not_ guaranteed identical across platforms. The car's heading is carried
  as a unit vector; rotation uses a Taylor `sin`/`cos` written out in
  `math.rs` (pure arithmetic, accurate to a few parts in 1e-8 over the range
  used). Sub-stepping keeps it in range for large inputs.
- **`sqrt` only** otherwise — correctly rounded by IEEE-754, so portable.
- **Fixed iteration order** everywhere, including contact resolution.
- **Canonicalised `-0.0`** in the state hash so a sign flip on zero can't
  register as a false divergence.
- **Avoid `mul_add`/FMA** — contraction is a legitimate source of cross-target
  divergence.

`tests/determinism.rs` covers: two identical runs, eight runs across threads,
sensitivity of the hash to a single 1e-9 input change, and branching a warmed-up
world (the rollback-netcode primitive).

### The honest caveats

Determinism has tiers, and it matters which one you mean:

- **Same binary, same machine, same run count** → easy, and it holds today.
- **Same binary, same platform, different machine** → holds unless the build
  differs in target features (`-C target-cpu=native` would break it).
- **Bit-identical across operating systems, CPUs and compilers** → this is the
  hard bar, and _no floating-point code gets it for free_. The remaining risks
  are: compiler version changing codegen, auto-vectorisation differences, and
  any remaining libm call. If you need it (e.g. cross-platform replays that
  must re-simulate identically), the robust answer is **fixed-point
  arithmetic** for the state — that's what lockstep RTS engines do — or ship a
  canonical test-vector for each target and verify at load.

So: today this is in tier 2. The design is arranged so climbing to tier 3 is a
matter of swapping the numeric representation, not restructuring.

## Performance

Measured on this machine, release build, whole fixed step (car + ball +
contacts):

```
2,000,000 ticks in 0.4934 s  ->  4.1M ticks/s  (~34,000x real time at 120 Hz)
```

So one world costs roughly **0.24 µs per tick**. To be honest about what that
does and doesn't mean:

- A live match needs 120 ticks/s. We have ~4,000,000/s. The headroom is
  comically large, and it is **not** where the wins are — a real game's frame
  cost will be rendering and network, not this.
- Where it _does_ pay off: headless bulk simulation. Thousands of parallel
  self-play matches for bot training, or fuzzing 10 million ticks in CI in
  seconds. The `long_run_stays_finite_and_bounded` test runs 12,000 ticks with
  randomised input in ~20 ms.
- With rollback netcode, re-simulating 8 frames of correction costs ~2 µs.
  That's the actual justification for the design.
- Rust specifics that matter here: no GC means no frame-time spikes; `Clone`
  snapshots are cheap and predictable; tests run headless with no window, no
  assets and no engine binary.

## Model summary

| Piece    | Model                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Arena    | Rounded box as a signed-distance field (chamfered vertical corners, filleted wall/floor and wall/ceiling joins), 81.92 x 102.40 x 20.48 m (RocketSim `ARENA_EXTENT_X/Y`, `ARENA_HEIGHT`); not RL's collision mesh                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| Ball     | Sphere, `r = 0.9125`, `m = 30`. Gravity, Bullet-style linear drag, Magnus from spin, Coulomb friction at contacts that converts sliding into spin. Speed and spin are hard-capped at 60 m/s and 6 rad/s                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| Car      | Bespoke driving model, _not_ a rigid body. Box hitbox 1.205 x 0.867 x 0.387 (real Octane, per RocketSim), `m = 180`. Steering from RL's `STEER_ANGLE_FROM_SPEED_CURVE` through a bicycle model (`omega = v * tan(steer) / wheelbase`); analog powerslide; RL's throttle/brake/coast split; boost meter; jump = immediate impulse + held force, then a double jump _or_ a flip; full 3D air attitude with air control; throttle gated by RL's `DRIVE_SPEED_TORQUE_FACTOR_CURVE` (throttle alone tops out near 14.1 m/s, boost reaches the 23 m/s hard clamp), angular speed to 5.5 rad/s. Lateral grip comes from RL's `LAT_FRICTION_CURVE`, `NON_STICKY_FRICTION_FACTOR_CURVE` and the handbrake curve (off-throttle grip falls on vertical surfaces), mapped to a per-tick velocity retention. The driving model runs in the current _surface's_ frame, so the car drives on the floor, up the walls and across the ceiling |
| Boost    | 34 pads (6 big, 28 small) at RL's positions; big gives 100 boost / 10 s cooldown, small 12 / 4 s. Taken by driving over a pad near the floor                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| Goal     | A full-size Soccar goal at each end: a 17.8551 x 6.42775 m mouth, carved into the arena SDF. The net is a semicircular scoop (radius ~2.70 m) swept across the width, so a car can drive in low and ride the curve out the top. The ball scores when its centre passes RocketSim's `SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y` plus the ball radius (fully over the line), then resets to the centre spot                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| Car-ball | Sphere vs OBB closest-point, inelastic impulse (`e = 0`) with relative velocity at contact (car rotation included), plus Rocket League's extra hit impulse, equal-and-opposite reaction                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

Constants are community-sourced Rocket League figures converted to SI, now
sourced from RocketSim rather than memory. They are still _approximations_ of a
closed-source game: the steering curve and jump are ported from RL, but the grip
_range_ (`CAR_GRIP_KEEP` / `CAR_DRIFT_KEEP`) and the brake force
(`CAR_BRAKE_DECEL`) are tuned for feel, not reverse-engineered (the curves that
pick a value inside that range — `LAT_FRICTION_CURVE`,
`NON_STICKY_FRICTION_FACTOR_CURVE`, the handbrake curve — are RL's). The engine
force's magnitude is tuned too, but its speed gate is RL's real
`DRIVE_SPEED_TORQUE_FACTOR_CURVE`. Matching RL exactly would need a parity harness
against recorded ticks, which is a separate project.

## Parity with Rocket League

The dimensions and material constants are no longer community guesswork: they
are lifted from **[RocketSim](https://github.com/ZealanL/RocketSim)**, a
standalone reimplementation of Rocket League's gameplay physics that is accurate
enough to train SSL-level bots. Unreal units are centimetres, so every constant
above is the RocketSim value divided by 100. The provenance is recorded in the
doc comments next to each one (`arena.rs`, `ball.rs`, `car.rs`).

What that changed:

- **Arena.** Was 80 x 50 x 12 m; the real Soccar pitch is 81.92 x 102.40 x
  20.48 m. The old field was less than half its proper length. Its sharp box
  corners have since been rounded into an SDF so the car can drive up the walls
  and onto the ceiling.
- **Ball.** Radius 0.93 -> 0.9125 (the _collision_ radius), restitution 0.62 ->
  0.6, friction 0.45 -> 0.35, mass 1.5 -> 30 (a sixth of the car). Drag switched
  from an ad-hoc quadratic term to Rocket League's `linearDamping = 0.03`,
  applied as a per-tick velocity multiplier so no `pow` is needed. Speed and
  spin are now hard-capped, exactly as RL does at the end of its tick.
- **Car.** Boost acceleration split into ground/air (`2975/3` and `3175/3`
  uu/s^2), boost drain `100/3` per second, and the real Octane hitbox, which is
  slightly larger than the widely-shared figure. Total speed is clamped to
  `CAR_MAX_SPEED = 2300 uu/s` every tick. Engine force is gated by RL's
  `DRIVE_SPEED_TORQUE_FACTOR_CURVE` (1.0 at rest, 0 at 1410 uu/s), so throttle
  alone saturates near 14.1 m/s and boost is what reaches the clamp. Lateral grip
  uses RL's `LAT_FRICTION_CURVE` (which falls as the contact slides more
  sideways), `NON_STICKY_FRICTION_FACTOR_CURVE` (off-throttle, grip scales with
  how vertical the surface is) and `HANDBRAKE_LAT_FRICTION_FACTOR_CURVE`, mapped
  onto a per-tick retention between `CAR_GRIP_KEEP` and `CAR_DRIFT_KEEP`. The
  throttle/brake split is RL's `_UpdateWheels`: a throttle against the direction
  of travel brakes (and cuts the engine) before reversing, a released throttle
  coasts (`COASTING_BRAKE_FACTOR`, a full stop below `STOPPING_FORWARD_VEL`), and
  boost counts as full throttle. Steering
  now uses RL's `STEER_ANGLE_FROM_SPEED_CURVE` (max front-wheel angle vs speed)
  through a bicycle model with the Octane wheelbase: `omega = v * tan(steer) / 0.85`. The
  turn radius grows from ~1.4 m at a crawl to ~13 m at top speed. Powerslide is
  the analog 0..1 value RL uses, ramping in and out, and it widens the usable
  steer angle via `POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE`.
- **Jump, flips and air control.** Ported from `_UpdateJump`,
  `_UpdateAirTorque` and `_UpdateDoubleJumpOrFlip`: an immediate jump impulse
  (`JUMP_IMMEDIATE_FORCE = 875/3 uu/s`) plus a held force (`JUMP_ACCEL`) for a
  0.025-0.2 s window (a tap peaks ~0.8 m, a hold ~2.5 m); then a double jump _or_
  a flip within `DOUBLEJUMP_MAX_DELAY = 1.25 s`. Air control is RL's pitch/yaw/roll
  torque (`CAR_AIR_CONTROL_TORQUE`) with its damping, and a flip applies the
  dodge impulse (`FLIP_INITIAL_VEL_SCALE`) plus the fixed-time flip torque. The
  dodge is flattened to the ground plane as RL does (`To2D`), so a flip from a
  pitched car still moves horizontally.
- **Boost pads.** The 34 Soccar pads sit at RL's `LOCS_BIG_SOCCAR` /
  `LOCS_SMALL_SOCCAR` positions: the 6 big pads give the full 100 boost with a
  10 s cooldown (`COOLDOWN_BIG`) and the 28 small give 12 with 4 s. Boost is
  taken when the car drives over a pad near the floor; the pickup is a simple
  horizontal radius (`CYL_RAD_BIG` / `CYL_RAD_SMALL`) rather than RL's
  cylinder/box triggers.
- **Goals.** Both end walls now carry a full-size goal mouth cut into the arena
  SDF, with the net recess behind it (blue at -Z, orange at +Z). The mouth
  (RLBot `goal center-to-post` / `goal height`), depth (`goal depth`) and score
  threshold (RocketSim `SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y`) are the real
  figures. A ball whose centre crosses the line scores and resets to the centre
  spot.
- **Car-ball.** Restitution dropped to `0.0` and friction raised to `2.0`, and
  the collision is now followed by Rocket League's **extra impulse**
  (`Ball::_OnHit`): a velocity kick along the car->ball direction, scaled by the
  closing speed through the real factor curve, with the vertical squashed and
  part of the car-forward component removed. It is applied to the ball only and
  at most every other tick. This is the mechanic that makes a hit "pop" — the
  inelastic collision on its own is weaker than RL.

What is still approximate, deliberately:

- **The arena is a rounded box.** RL collides against a mesh with chamfered
  corners and a domed ceiling. The extents match and the joins are now filleted
  (which is what lets the car drive onto the ceiling), but the ceiling is flat
  rather than domed, and the rounding radii are gameplay guesses, not RL figures.
- **The Magnus force is ours, not RL's.** RocketSim/Bullet gives a spinning
  sphere no aerodynamic force; RL balls do curve, so this is kept as a small
  deliberate extra. Rocket League also caps ball spin at 6 rad/s, which bounds
  the effect.
- **The car is bespoke, and the vehicle is a bicycle model.** RL drives a
  `btRaycastVehicle` with four wheels, suspension and tyre slip. Steering uses
  RL's real steer curve and wheelbase, but a two-wheel approximation of a
  four-wheel vehicle that ignores slip, so the radii are a good first cut rather
  than exactly RL's. The grip and brake magnitudes are still tuned, and the map
  from RL's friction value to a velocity retention is ours (RL resolves grip with
  per-wheel impulses, not a retention factor), and coasting/braking is a chassis
  deceleration scaled by that traction rather than friction-limited wheel torque;
  the engine force is gated by RL's
  real `DRIVE_SPEED_TORQUE_FACTOR_CURVE`, but its magnitude is tuned too.
- **Attitude is free in the air.** Our car has no suspension or auto-roll, so a
  car that lands badly (inverted, or sideways off a wall) is set back to a
  yaw-only attitude rather than settling. Wall and ceiling driving does work, but
  through the point-contact, glue-to-the-surface model below, not RL's wheels.
- **The world frame is left-handed.** The body frame here is `(forward, right,
up)` with right = `forward x up`, which is left-handed, whereas RL's is
  right-handed. The air-control and flip torque _axes_ were therefore derived to
  match RL's behaviour rather than transliterated from its cross products; the
  magnitudes and timings are RL's.

Matching RL _exactly_ still needs a parity harness against recorded ticks; see
the end of this document.

## Jumping

Small surface area, but the choices are deliberate:

- **Edge-triggered.** `Input.jump` is a held level, like `boost`. The car keeps
  the previous tick's value and acts only on the press, so holding the button
  does not re-launch it the instant it lands
  (`holding_jump_does_not_re_jump_on_landing`).
- **Shared gravity.** Airborne, the car falls under the same `GRAVITY` as the
  ball.
- **Press and hold are different jumps.** RL applies an immediate impulse
  (`JUMP_IMMEDIATE_FORCE`) and then a held force (`JUMP_ACCEL`), for a minimum of
  0.025 s and a maximum of 0.2 s (`JUMP_MIN_TIME` / `JUMP_MAX_TIME`). A tap
  peaks ~0.8 m above rest; a full hold ~2.5 m
  (`holding_jump_goes_higher_than_tapping_it`).
- **One double jump or one flip.** A single extra action while airborne, within
  1.25 s of the first jump finishing: with the stick centred it is a double jump,
  off-centre (past `dodgeDeadzone`) it is a flip. Then nothing until the car
  touches down again.
- **Air control.** Once airborne the body attitude is free 3D, and pitch/yaw/roll
  drive RL's angular-acceleration torque with damping, so a jump can be aimed or
  turned into a shot. Boost and the (small) air throttle still work.

## Driving on walls and the ceiling

A signature mechanic, and the one that most obviously needs more than a box and a
ground plane:

- **The arena is an SDF, and its joins are rounded.** `Arena::inside_distance`
  is the `max` of a rounded pillar (the walls, vertical corners chamfered) and a
  filleted floor and ceiling. `Arena::normal_at` is the (inward) gradient, so the
  normal turns continuously from up to sideways to down as the car crosses a
  join: no face to wedge against, and no discontinuity to bump over.
- **A grounded car is glued to its surface.** `Car::step_grounded` runs the whole
  driving model in the surface's own frame: `up` is the surface normal, the steer
  axis is that normal (so turning works on a wall just as on the floor), and
  gravity only counts along the slope - none on the floor or ceiling, a full pull
  down a vertical wall. `Car::align_up_to` rotates the car _and its velocity_
  onto the new normal at each join, so speed carries from the floor up a wall and
  over onto the ceiling rather than stopping dead.
- **Landing is orientation-aware.** Arriving wheels-down for a surface lands on
  it; arriving sideways or inverted bounces (`CAR_SURFACE_RESTITUTION`), and on
  the floor it snaps the car back upright so it can settle.
- **Overhanging surfaces cannot hold the car.** A surface can only push, never
  pull, so on the ceiling (and the upper part of the rim fillet) gravity would
  peel the car off. It stays only while it is moving faster than
  `CAR_STICK_SPEED` (a floor or a wall has no such requirement), so coasting on
  the roof sheds speed and drops the car back down.
- **What is simplified.** The contact is a point at the car's centre held
  `CAR_HALF_HEIGHT` off the surface, not four wheels, so there is no suspension
  or tyre model pushing the car into a wall. The floor-to-wall transition leans
  on the small `FLOOR_FILLET` at the base rather than a wheel catching the wall,
  and the wheels do not visually rotate.

## Not implemented yet

Car-car collisions, demolitions, the kickoff countdown/replay, multiple cars,
and the actual network layer. (Goals themselves are in: a ball crossing the line
scores and returns to the centre spot - see `World::check_goal`.)

The useful next step for the determinism story is a _parity harness_: record
input streams plus state hashes from a reference build, then assert them in CI.
That turns "it's deterministic" from an argument into a regression test.
