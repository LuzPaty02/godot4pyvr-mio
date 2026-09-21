extends Node

@export var screen_hand_z := -1.8
@export var screen_hand_scale := 1.25

@onready var station: Node3D = get_node("../Station")
@onready var left_arm: MeshInstance3D = get_node("../ScreenViewport/ViewportWorld/LeftArm")
@onready var right_arm: MeshInstance3D = get_node("../ScreenViewport/ViewportWorld/RightArm")
@onready var left_marker: MeshInstance3D = get_node("../ScreenViewport/ViewportWorld/LeftMarker")
@onready var right_marker: MeshInstance3D = get_node("../ScreenViewport/ViewportWorld/RightMarker")

var _last_yaw := {
	&"left": 0.0,
	&"right": 0.0,
}
var _continuous_yaw := {
	&"left": 0.0,
	&"right": 0.0,
}

func update_hand(controller: XRController3D, side: StringName) -> Transform3D:
	var local_transform := station.global_transform.affine_inverse() * controller.global_transform
	var current_yaw := local_transform.basis.get_euler().y
	var yaw_delta := wrapf(current_yaw - _last_yaw[side], -PI, PI)
	_continuous_yaw[side] += yaw_delta
	_last_yaw[side] = current_yaw
	var hand_position := Vector3(
		clamp(local_transform.origin.x * screen_hand_scale, -1.25, 1.25),
		clamp((local_transform.origin.y - 1.2) * screen_hand_scale, -0.85, 0.85),
		screen_hand_z
	)
	var shoulder_x := -0.35 if side == &"left" else 0.35
	var shoulder_position := Vector3(shoulder_x, 0.65, screen_hand_z)
	var arm_direction := hand_position - shoulder_position
	var arm_length := maxf(arm_direction.length(), 0.05)
	var arm := left_arm if side == &"left" else right_arm
	var marker := left_marker if side == &"left" else right_marker

	arm.position = shoulder_position + arm_direction * 0.5
	arm.quaternion = Quaternion(Vector3.UP, arm_direction.normalized())
	(arm.mesh as CylinderMesh).height = arm_length
	marker.position = hand_position
	return local_transform

func get_continuous_yaw(side: StringName) -> float:
	return _continuous_yaw[side]
