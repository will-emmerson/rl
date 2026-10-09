# Why hand-roll the physics?

Short version: for _this_ game a physics engine is mostly overhead, and the one
thing you actually want from it — determinism — is the one thing general
engines are worst at. But some of the reasoning in the original pitch doesn't
hold up, so this document separates what's true from what isn't.

## What the engine is doing for you, and when it isn't

A general 3D engine earns its keep on: broadphase, hierarchical contact
generation, constraint solvers, stacking stability, joints, sleeping, CCD.

This game has **one sphere, one box, and six planes**. There is no stacking, no
joints, no chains. Broadphase is a no-op. So ~95% of what the engine provides
goes unused, and what remains is maybe 300 lines of arithmetic you can read in
one sitting.

The cost of using one shows up when the game's behaviour is _specific_ and you
have to fight the engine to express it. That is exactly what happened building
the Godot version:

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

The mass ratio here is 180 : 1.5, so the ball gains ~97% of the relative
velocity and the car loses ~1%. That's the RL feel, and it's emergent from
putting honest masses into the impulse equation — no fudge factor.

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

| Piece    | Model                                                                                                                                                                                                           |
| -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Arena    | 6 static planes, sequential resolution, 80 x 50 x 12 m                                                                                                                                                          |
| Ball     | Sphere, `r = 0.93`, `m = 1.5`. Gravity, quadratic drag, Magnus from spin, Coulomb friction at contacts that converts sliding into spin                                                                          |
| Car      | Bespoke driving model, _not_ a rigid body. Box hitbox 1.18 x 0.84 x 0.36, `m = 180`. Yaw rate bounded directly, so the turn radius (`v / yaw_rate`) grows with speed; lateral-grip decay for drift; boost meter |
| Car-ball | Sphere vs OBB closest-point, impulse with relative velocity at contact (car rotation included), equal-and-opposite reaction                                                                                     |

Constants are community-sourced Rocket League figures (gravity ~650 uu/s²,
Octane hitbox, 2200/2300 uu/s speed caps) converted to SI. They are
_approximations_, and the handling numbers (`CAR_MAX_YAW_RATE`,
`CAR_STEER_ENGAGE_SPEED`, grip decay) are tuned for feel, not reverse-engineered
from the game. Matching RL exactly would
need a parity harness against recorded ticks, which is a separate project.

## Not implemented yet

Jumping and dodging (fixed-time impulse flips), wall/ceiling driving (a
signature mechanic, and a real change to the car model), boost pads,
car-car collisions, demolitions, goals and kickoff, multiple cars, and the
actual network layer.

The useful next step for the determinism story is a _parity harness_: record
input streams plus state hashes from a reference build, then assert them in CI.
That turns "it's deterministic" from an argument into a regression test.
