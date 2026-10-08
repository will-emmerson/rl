class_name Car
extends VehicleBody3D

## Arcade-flavoured Rocket League style car.
##
## It is built on Godot's built-in [VehicleBody3D] wheels, so suspension, grip
## and ground handling are handled for us. This script only decides how much
## engine force, braking and steering to ask for each physics frame, plus a
## small assist that keeps the car on its wheels.
##
## Note: a [VehicleBody3D] drives towards its local +Z (Vector3.MODEL_FRONT).
##
## Tuning notes:
## - engine_force is a real force in Newtons, applied to *every* wheel that has
##   use_as_traction, so acceleration is roughly
##   (max_engine_force * traction_wheels) / mass.
## - brake is NOT a force in Godot: it is clamped as an impulse per wheel per
##   physics step. That is why this script converts a deceleration in m/s^2
##   into a brake value for you.

@export_group("Engine")
## Force in Newtons applied to each driven wheel.
@export var max_engine_force := 600.0
## Fraction of the engine force used when reversing.
@export var reverse_force_scale := 0.6
## Forward speed (m/s) at which the engine stops pushing.
@export var max_speed := 32.0

@export_group("Brakes")
## Deceleration in m/s^2 while holding the reverse key at speed.
@export var brake_deceleration := 11.0
## Gentle deceleration in m/s^2 while coasting with no throttle.
@export var coast_deceleration := 1.5

@export_group("Steering")
## Maximum wheel angle, in radians.
@export var max_steer_angle := deg_to_rad(30.0)
## How fast the wheels turn towards the target angle (radians / second).
@export var steer_speed := 5.0
## Speed (m/s) at which only half of the steering angle is available.
## Steering has to fall away with speed or the car rolls over in a corner.
@export var steer_falloff_speed := 10.0

@export_group("Stability")
## Strength of the arcade "stay upright" torque. Set to 0 to disable it.
@export var upright_strength := 1600.0
## Damping for the roll/pitch wobble of the assist, so it cannot oscillate.
@export var upright_damping := 350.0
## How much of the assist still applies while airborne.
@export var airborne_assist_scale := 0.4

# Where the car was placed in the scene, used by reset().
var _spawn_transform: Transform3D
var _wheels: Array[VehicleWheel3D] = []

func _ready() -> void:
	_spawn_transform = global_transform
	for child in get_children():
		if child is VehicleWheel3D:
			_wheels.append(child)

func _physics_process(delta: float) -> void:
	_update_steering(delta)
	_update_drive(delta)
	_apply_upright_assist()

func _update_steering(delta: float) -> void:
	# +1 steers left, -1 steers right. Positive steering = positive yaw in Godot.
	var steer_input := Input.get_axis("steer_right", "steer_left")
	var speed := absf(forward_speed())

	# Steering authority falls off with the square of speed (1 / (1 + (v/v0)^2)).
	# Without this the car generates enough cornering force to roll itself over.
	var speed_scale := 1.0 / (1.0 + pow(speed / steer_falloff_speed, 2.0))
	var target_angle := steer_input * max_steer_angle * speed_scale
	steering = move_toward(steering, target_angle, steer_speed * delta)

func _update_drive(delta: float) -> void:
	var throttle := Input.get_axis("brake", "accelerate")
	var speed := forward_speed()
	var brake_value := _deceleration_to_brake(brake_deceleration, delta)
	var coast_value := _deceleration_to_brake(coast_deceleration, delta)

	if throttle > 0.0:
		# Hold at the top speed rather than accelerating forever.
		engine_force = throttle * max_engine_force if speed < max_speed else 0.0
		brake = 0.0
	elif throttle < 0.0:
		if speed > 0.5:
			# Still rolling forwards: the reverse key acts as a brake first.
			engine_force = 0.0
			brake = brake_value
		elif speed > -max_speed * 0.5:
			engine_force = throttle * max_engine_force * reverse_force_scale
			brake = 0.0
		else:
			engine_force = 0.0
			brake = 0.0
	else:
		engine_force = 0.0
		brake = coast_value

## Convert a wanted deceleration (m/s^2) into a VehicleBody3D.brake value.
##
## Godot clamps brake as an impulse (N*s) per wheel per physics step, so the
## usable value depends on the vehicle's mass and the physics tick rate.
func _deceleration_to_brake(deceleration: float, delta: float) -> float:
	return deceleration * mass * delta / maxf(float(_wheels.size()), 1.0)

## True if any wheel is touching a surface.
func is_grounded() -> bool:
	for wheel in _wheels:
		if wheel.is_in_contact():
			return true
	return false

## A gentle torque that rolls the car back onto its wheels.
##
## Godot's VehicleBody3D has no anti-roll bar, so a hard landing or a wall at
## speed can leave it stuck on its side. This is the arcade equivalent.
func _apply_upright_assist() -> void:
	if upright_strength <= 0.0:
		return
	var strength := upright_strength
	var damping := upright_damping
	if not is_grounded():
		strength *= airborne_assist_scale
		damping *= airborne_assist_scale
	var up := global_transform.basis.y
	# Axis (and magnitude) that rotates the car's up back towards world up.
	var correction := up.cross(Vector3.UP) * strength
	# Only damp roll/pitch, never yaw, so steering is left alone.
	var wobble := angular_velocity - Vector3.UP * angular_velocity.dot(Vector3.UP)
	apply_torque(correction - wobble * damping)

## Signed speed along the car's forward axis, in metres per second.
func forward_speed() -> float:
	return global_transform.basis.z.dot(linear_velocity)

## Teleport the car back to where it started and kill all momentum.
func reset() -> void:
	global_transform = _spawn_transform
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	engine_force = 0.0
	brake = 0.0
	steering = 0.0
