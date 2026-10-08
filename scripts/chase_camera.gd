class_name ChaseCamera
extends Camera3D

## A smooth third-person camera that trails a target node.
##
## It also pulls itself in when a wall would come between it and the target,
## so the arena geometry does not block the view.

## Node to follow. It can be swapped at runtime (e.g. to look at the ball).
@export var target: Node3D
## Camera position, expressed in the target's local space (-Z is "behind").
@export var offset := Vector3(0.0, 3.0, -8.5)
## The point the camera aims at, relative to the target.
@export var look_offset := Vector3(0.0, 1.0, 0.0)
## Higher is snappier: roughly "1 / seconds to catch up".
@export var follow_sharpness := 6.0
@export var look_sharpness := 10.0
## Enable to stop the camera from clipping through walls.
@export var wall_collision := true
## How far in front of a wall the camera is kept when it collides.
@export var collision_margin := 0.5

# Bodies the wall ray should ignore (the car and the ball), set by Game.
var _excluded_rids: Array[RID] = []
var _look_point := Vector3.ZERO
var _initialized := false

func _ready() -> void:
	_snap()

func _physics_process(delta: float) -> void:
	if not is_instance_valid(target):
		return
	if not _initialized:
		_snap()
		return

	var desired := _desired_position()
	var follow_weight := 1.0 - exp(-follow_sharpness * delta)
	global_position = global_position.lerp(desired, follow_weight)

	var focus := target.global_position + look_offset
	var look_weight := 1.0 - exp(-look_sharpness * delta)
	_look_point = _look_point.lerp(focus, look_weight)
	look_at(_look_point, Vector3.UP)

## Register a body that the wall raycast should ignore.
func add_exclusion(body: CollisionObject3D) -> void:
	if body and not _excluded_rids.has(body.get_rid()):
		_excluded_rids.append(body.get_rid())

## Jump straight to the ideal position, without smoothing.
func _snap() -> void:
	if not is_instance_valid(target):
		return
	global_position = target.global_transform * offset
	_look_point = target.global_position + look_offset
	look_at(_look_point, Vector3.UP)
	_initialized = true

## Where the camera would like to sit, pulled in if a wall is in the way.
func _desired_position() -> Vector3:
	var desired := target.global_transform * offset
	if not wall_collision:
		return desired

	var from := target.global_position + look_offset
	var space := get_world_3d().direct_space_state
	var query := PhysicsRayQueryParameters3D.create(from, desired)
	query.exclude = _excluded_rids
	var hit := space.intersect_ray(query)
	if hit.is_empty():
		return desired
	return hit["position"] + (from - desired).normalized() * collision_margin
