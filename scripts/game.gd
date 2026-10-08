extends Node3D

## Top level scene: wires the camera to the car and handles whole-round resets.

@onready var car: Car = $Car
@onready var ball: RigidBody3D = $Ball
@onready var chase_camera: ChaseCamera = $ChaseCamera

var _ball_spawn: Transform3D

func _ready() -> void:
	_ball_spawn = ball.global_transform
	chase_camera.target = car
	chase_camera.add_exclusion(car)
	chase_camera.add_exclusion(ball)

func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("reset"):
		_reset_round()
	elif event.is_action_pressed("camera_toggle"):
		chase_camera.target = ball if chase_camera.target == car else car

func _reset_round() -> void:
	car.reset()
	ball.global_transform = _ball_spawn
	ball.linear_velocity = Vector3.ZERO
	ball.angular_velocity = Vector3.ZERO
