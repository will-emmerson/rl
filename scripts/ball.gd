class_name Ball
extends RigidBody3D

## The match ball.
##
## The arena shell does the real bouncing, but the ball is light enough that
## when the car pins it against a wall the solver can squeeze it clean through.
## This script is a safety net for that case: it clamps runaway speeds and, if
## the ball does end up outside the play area, drops it back in and bounces it.

## Inner half-size of the play volume (the arena is 80 x 12 x 50).
@export var arena_half_extents := Vector3(40.0, 12.0, 25.0)
## Hard cap on speed in m/s; keeps a bad contact from launching the ball.
@export var max_speed := 70.0
## How far past the wall surface the ball may drift before the net reacts.
@export var tunnel_allowance := 0.15
## Bounciness used when the safety net reflects the ball.
@export var net_restitution := 0.6

var _radius := 0.8

func _ready() -> void:
	var shape := ($CollisionShape3D as CollisionShape3D).shape
	if shape is SphereShape3D:
		_radius = shape.radius

func _integrate_forces(state: PhysicsDirectBodyState3D) -> void:
	var velocity := state.linear_velocity
	if velocity.length_squared() > max_speed * max_speed:
		velocity = velocity.normalized() * max_speed
		state.linear_velocity = velocity

	var contact := arena_half_extents - Vector3.ONE * _radius
	var limit := contact + Vector3.ONE * tunnel_allowance
	var p := state.transform.origin
	var escaped := absf(p.x) > limit.x \
			or absf(p.z) > limit.z \
			or p.y < _radius - tunnel_allowance \
			or p.y > limit.y
	if not escaped:
		return

	var restored := Vector3(
		clampf(p.x, -contact.x, contact.x),
		clampf(p.y, _radius, contact.y),
		clampf(p.z, -contact.z, contact.z))
	state.transform.origin = restored
	# Bounce off whichever faces were crossed.
	state.linear_velocity = Vector3(
		-velocity.x * net_restitution if p.x != restored.x else velocity.x,
		-velocity.y * net_restitution if p.y != restored.y else velocity.y,
		-velocity.z * net_restitution if p.z != restored.z else velocity.z)
