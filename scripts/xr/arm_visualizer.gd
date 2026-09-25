extends Node3D

@export_group("Arm geometry")
@export var segment_1_length := 0.42
@export var segment_2_unit_length := 0.08
@export var segment_2_segments_count := 9
@export var segment_3_length := 0.30
@export var segment_radius := 0.08

var _segment_names := [&"Segment1", &"Segment2", &"Segment3"]
var _colors := [
	Color(0.15, 0.75, 1.0, 1.0),
	Color(0.3, 1.0, 0.55, 1.0),
	Color(1.0, 0.6, 0.25, 1.0),
]

func update_from_origin_and_direction(origin: Vector3, direction: Vector3) -> void:
	var arm_direction := direction.normalized()
	if arm_direction.length_squared() < 0.0001:
		arm_direction = Vector3.UP

	var segment_2_total_length := segment_2_unit_length * segment_2_segments_count
	var segment_1 := _ensure_segment(_segment_names[0], _colors[0])
	var segment_2 := _ensure_segment(_segment_names[1], _colors[1])
	var segment_3 := _ensure_segment(_segment_names[2], _colors[2])

	_set_segment_length(segment_1, segment_1_length)
	_set_segment_length(segment_2, segment_2_total_length)
	_set_segment_length(segment_3, segment_3_length)

	position = origin
	quaternion = Quaternion(Vector3.UP, arm_direction)
	segment_1.position = Vector3(0.0, segment_1_length * 0.5, 0.0)
	segment_2.position = Vector3(0.0, segment_1_length + segment_2_total_length * 0.5, 0.0)
	segment_3.position = Vector3(0.0, segment_1_length + segment_2_total_length + segment_3_length * 0.5, 0.0)
	segment_1.rotation = Vector3.ZERO
	segment_2.rotation = Vector3.ZERO
	segment_3.rotation = Vector3.ZERO

func _ensure_segment(segment_name: StringName, color: Color) -> MeshInstance3D:
	var segment: MeshInstance3D = get_node_or_null(NodePath(String(segment_name)))
	if segment == null:
		segment = MeshInstance3D.new()
		segment.name = String(segment_name)
		add_child(segment)

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

func _set_segment_length(segment: MeshInstance3D, length: float) -> void:
	var mesh := segment.mesh as CylinderMesh
	if mesh == null:
		mesh = CylinderMesh.new()
		mesh.top_radius = segment_radius
		mesh.bottom_radius = segment_radius
		segment.mesh = mesh
	mesh.height = length
	return
