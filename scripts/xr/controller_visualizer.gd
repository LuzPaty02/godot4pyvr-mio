extends Node

@export var screen_hand_z := -1.8
@export var screen_hand_scale := 1.25

@export_group("Arm geometry")
@export var segment_1_length := 0.42
@export var segment_2_unit_length := 0.08
@export var segment_2_segments_count := 9
@export var segment_3_length := 0.30
@export var segment_radius := 0.08

@onready var station: Node3D = get_node("../Station")
@onready var left_arm: Node3D = get_node("../ScreenViewport/ViewportWorld/LeftArm")
@onready var right_arm: Node3D = get_node("../ScreenViewport/ViewportWorld/RightArm")
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

func _get_segment_mesh(arm: Node3D, segment_name: StringName, color: Color) -> MeshInstance3D:
	var segment: MeshInstance3D = arm.get_node_or_null(NodePath(String(segment_name)))
	if segment == null:
		segment = MeshInstance3D.new()
		segment.name = String(segment_name)
		arm.add_child(segment)

	var mesh := CylinderMesh.new()
	mesh.top_radius = segment_radius
	mesh.bottom_radius = segment_radius
	mesh.height = 0.1
	segment.mesh = mesh
	segment.visible = true

	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_color = color
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	segment.material_override = material
	return segment

func _update_arm_segments(side: StringName, anchor_position: Vector3, arm_direction: Vector3) -> void:
	var arm := left_arm if side == &"left" else right_arm
	var direction := arm_direction.normalized()
	if direction.length_squared() < 0.0001:
		direction = Vector3.UP

	var segment_2_total_length := segment_2_unit_length * segment_2_segments_count
	var segment_1 := _get_segment_mesh(arm, &"Segment1", Color(0.15, 0.75, 1.0, 1.0))
	var segment_2 := _get_segment_mesh(arm, &"Segment2", Color(0.3, 1.0, 0.55, 1.0))
	var segment_3 := _get_segment_mesh(arm, &"Segment3", Color(1.0, 0.6, 0.25, 1.0))

	arm.position = anchor_position
	arm.quaternion = Quaternion(Vector3.UP, direction)

	var segment_1_mesh := segment_1.mesh as CylinderMesh
	segment_1_mesh.height = segment_1_length
	segment_1.position = Vector3(0.0, segment_1_length * 0.5, 0.0)
	segment_1.rotation = Vector3.ZERO

	var segment_2_mesh := segment_2.mesh as CylinderMesh
	segment_2_mesh.height = segment_2_total_length
	segment_2.position = Vector3(0.0, segment_1_length + segment_2_total_length * 0.5, 0.0)
	segment_2.rotation = Vector3.ZERO

	var segment_3_mesh := segment_3.mesh as CylinderMesh
	segment_3_mesh.height = segment_3_length
	segment_3.position = Vector3(0.0, segment_1_length + segment_2_total_length + segment_3_length * 0.5, 0.0)
	segment_3.rotation = Vector3.ZERO

	segment_1_mesh.height = segment_1_length
	segment_2_mesh.height = segment_2_total_length
	segment_3_mesh.height = segment_3_length

	segment_1.position = Vector3(0.0, segment_1_length * 0.5, 0.0)
	segment_2.position = Vector3(0.0, segment_1_length + segment_2_total_length * 0.5, 0.0)
	segment_3.position = Vector3(0.0, segment_1_length + segment_2_total_length + segment_3_length * 0.5, 0.0)

	segment_1.rotation = Vector3.ZERO
	segment_2.rotation = Vector3.ZERO
	segment_3.rotation = Vector3.ZERO

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
	var arm_direction := shoulder_position - hand_position
	var arm := left_arm if side == &"left" else right_arm
	var marker := left_marker if side == &"left" else right_marker

	_update_arm_segments(side, hand_position, arm_direction)
	marker.position = hand_position
	return local_transform

func get_continuous_yaw(side: StringName) -> float:
	return _continuous_yaw[side]
