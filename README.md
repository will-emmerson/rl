# Rocket Arena

A tiny Rocket League style playground built with **Godot 4**.

Drive a car around an enclosed arena and knock a ball about. That's it — no
boost, no jumping, no goals or scoreboard yet. It's meant as a starting point.

## Requirements

- [Godot 4.3](https://godotengine.org/download) or newer (the standard editor
  build, no need for the .NET/Mono version).

## Running it

1. Open Godot.
2. In the project manager, choose **Import**, point it at this folder's
   `project.godot`, and open it.
3. Press **F5** (or the play button).

Godot will import the few assets on first open, which takes a second.

## Controls

Keyboard and gamepad (Xbox-style layout) both work, and can be mixed.

| Action             | Keyboard                 | Gamepad              |
| ------------------ | ------------------------ | -------------------- |
| Accelerate         | `W` / `Up`               | `RT` (analog) or `A` |
| Brake / reverse    | `S` / `Down`             | `LT` (analog) or `B` |
| Steer              | `A` `D` / `Left` `Right` | Left stick (analog)  |
| Reset car & ball   | `R`                      | `X`                  |
| Swap camera target | `C` (car ⇄ ball)         | `Y`                  |

Throttle and steering are analog on a pad, so the triggers give you partial
throttle and the stick gives proportional steering. Button indices are Godot's
standard SDL mapping, so a PlayStation pad works too (cross/circle/square/
triangle land on the same indices).

## What's in here

```
project.godot          Project config + the input map
scenes/
  main.tscn            Entry scene: environment, light, arena, car, ball, camera
  arena.tscn           Floor, four walls and a glass ceiling (thick, so fast shots bounce)
  car.tscn             VehicleBody3D chassis with four VehicleWheel3D wheels
  ball.tscn            RigidBody3D ball
scripts/
  car.gd               Driving: engine, brakes, steering, upright assist
  ball.gd              Speed clamp + "keep the ball in play" safety net
  chase_camera.gd      Smoothed third-person camera with wall collision
  game.gd              Wires things together; handles reset + camera toggle
```

The car is built on Godot's built-in `VehicleBody3D`, so suspension, grip and
ground contact come from the engine. `car.gd` only decides how much engine
force / braking / steering to ask for each frame.

## Numbers currently dialled in

Measured in-engine (180 kg car, 4 driven wheels):

- 0 → 20 m/s in ~1.8 s (~11 m/s²)
- Braking 20 m/s → stop in ~16 m, ~1.6 s
- Cornering radius ~10 m, ~12 m/s² of lateral grip
- Top speed capped at 32 m/s
- Ball flies off at 20–35 m/s on a solid hit

## Tuning notes

Two Godot quirks are worth knowing before you change values, because they cost
me some debugging:

1. **`VehicleBody3D.engine_force` is a real force in Newtons**, applied to
   _every_ wheel with `use_as_traction`. So acceleration is roughly
   `max_engine_force * driven_wheels / mass`.

2. **`VehicleBody3D.brake` is not a force** — despite the class reference
   calling it one. Godot clamps it as an _impulse_ per wheel per physics step,
   so the useful value depends on the car's mass and the physics tick rate.
   `car.gd` therefore exposes `brake_deceleration` in m/s² and converts it via
   `_deceleration_to_brake()`. Don't set `brake` directly.

Also, `VehicleBody3D` has no anti-roll bar, and the rollover threshold is
`track_width / 2 / centre_of_gravity_height`. The car's origin is its centre of
gravity, so the wheels are deliberately mounted _above_ the origin to keep it
low — that is why the collision box sits high in `car.tscn`. On top of that,
`car.gd` fades steering out with speed and applies a small upright torque
(`upright_strength`); set that to `0` if you want to test the raw physics.

## Ideas for next steps

- Goals and scoring (two `Area3D` boxes at each end, emit a signal on
  `body_entered`).
- A HUD showing speed and score.
- Boost, jump and air control.
- Kickoff countdown and a post-goal reset.
- Better car and arena art.
