extends Node

@export var screen_hand_z := -1.8 #fixed depth in the viewport
@export var screen_hand_scale := 1.25 #helps to amplify hand movement for better visibility in the viewport

# References to the station and markers in the floating-screen viewport.
@onready var station: Node3D = get_node("../Station")
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

# Main controller pose conversion for HUD/marker feedback.
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
	var marker := left_marker if side == &"left" else right_marker

	marker.position = hand_position
	return local_transform

func get_continuous_yaw(side: StringName) -> float:
	return _continuous_yaw[side]
