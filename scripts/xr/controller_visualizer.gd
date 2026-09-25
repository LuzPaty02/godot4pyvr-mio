extends Node

@export var screen_hand_z := -1.8 #fixed depth in the viewport
@export var screen_hand_scale := 1.25 #helps to amplify hand movement for better visibility in the viewport

@export_group("Arm geometry")
@export var segment_1_length := 0.42
@export var segment_2_unit_length := 0.08
@export var segment_2_segments_count := 9
@export var segment_3_length := 0.30
@export var segment_radius := 0.08


#references to the arm visualizer nodes, which are children of the controller visualizer node
@onready var station: Node3D = get_node("../Station")
@onready var left_arm: Node3D = get_node("../ScreenViewport/ViewportWorld/LeftArm")
@onready var right_arm: Node3D = get_node("../ScreenViewport/ViewportWorld/RightArm")
@onready var left_marker: MeshInstance3D = get_node("../ScreenViewport/ViewportWorld/LeftMarker")
@onready var right_marker: MeshInstance3D = get_node("../ScreenViewport/ViewportWorld/RightMarker")

#state variables for continuous yaw tracking used in the update_hand function
var _last_yaw := {
	&"left": 0.0,
	&"right": 0.0,
}
var _continuous_yaw := { 
	&"left": 0.0,
	&"right": 0.0,
}

func _update_arm_segments(side: StringName, anchor_position: Vector3, arm_direction: Vector3) -> void:
	var arm := left_arm if side == &"left" else right_arm
	if arm != null and arm.has_method("update_from_origin_and_direction"):
		arm.update_from_origin_and_direction(anchor_position, arm_direction)


#main 
func update_hand(controller: XRController3D, side: StringName) -> Transform3D:
	#converts controller's coordinates to coordinates relative to the station
	var local_transform := station.global_transform.affine_inverse() * controller.global_transform
	#calculates turn angle (yaw) of the controller, allows detection of continuous rotation beyond 360 degrees when the controller is rotated multiple times
	var current_yaw := local_transform.basis.get_euler().y
	var yaw_delta := wrapf(current_yaw - _last_yaw[side], -PI, PI) #tells how much the controller has rotated since the last frame, wrapped to the range [-PI, PI]
	_continuous_yaw[side] += yaw_delta
	_last_yaw[side] = current_yaw

	#
	var hand_position := Vector3(
		clamp(local_transform.origin.x * screen_hand_scale, -1.25, 1.25),
		clamp((local_transform.origin.y - 1.2) * screen_hand_scale, -0.85, 0.85),
		screen_hand_z
	)
	var shoulder_x := -0.35 if side == &"left" else 0.35
	var shoulder_position := Vector3(shoulder_x, 0.65, screen_hand_z)
	var arm_direction := shoulder_position - hand_position
	var arm := left_arm if side == &"left" else right_arm
	var marker := left_marker if side == &"left" else right_marker

	_update_arm_segments(side, hand_position, arm_direction)
	marker.position = hand_position
	return local_transform

func get_continuous_yaw(side: StringName) -> float:
	return _continuous_yaw[side]
