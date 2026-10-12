extends Node3D

# The model and viewport camera should share the same origin; keep the model in
# camera space and avoid any extra per-arm screen offsets.
@export var tube_radius := 0.02
@export_range(8, 24, 1) var radial_segments := 16
@export var model_to_scene_scale := 0.02
# Temporary debug offset for the viewport/test rig. The real system uses the
# endoscope tip pose from the model, not the model origin itself.
@export var model_origin := Vector3(0.0, -0.1, -1.2)
@export var arm_color := Color(0.2, 0.9, 1.0, 1.0)

var _tube_instance: MeshInstance3D

func _ready() -> void:
	_tube_instance = MeshInstance3D.new()
	_tube_instance.name = "KinematicTube"
	add_child(_tube_instance)

	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_color = arm_color
	material.metallic = 0.1
	material.roughness = 0.4
	_tube_instance.material_override = material
	_tube_instance.visible = false

func update_polyline(model_points: PackedVector3Array) -> void:
	if model_points.size() < 2:
		_tube_instance.visible = false
		return

	var points := PackedVector3Array()
	points.resize(model_points.size())
	for index in range(model_points.size()):
		var point := model_points[index]
		# Model coordinates are millimetres in x-right/y-down/z-forward;
		# convert to Godot metres, y-up, and -z-forward.
		var scaled := Vector3(point.x, -point.y, -point.z) * model_to_scene_scale
		points[index] = model_origin + scaled

	_tube_instance.mesh = _build_tube(points)
	_tube_instance.visible = _tube_instance.mesh != null

func _build_tube(points: PackedVector3Array) -> ArrayMesh:
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)

	for index in range(points.size() - 1):
		var start := points[index]
		var finish := points[index + 1]
		var segment := finish - start
		if segment.length_squared() < 1e-12:
			continue

		var tangent := segment.normalized()
		var reference_axis := Vector3.UP
		if absf(tangent.dot(reference_axis)) > 0.95:
			reference_axis = Vector3.RIGHT
		var axis_x := reference_axis.cross(tangent).normalized()
		var axis_y := tangent.cross(axis_x).normalized()

		for radial_index in range(radial_segments):
			var next_radial_index := (radial_index + 1) % radial_segments
			var angle_a := TAU * float(radial_index) / float(radial_segments)
			var angle_b := TAU * float(next_radial_index) / float(radial_segments)
			var normal_a := (axis_x * cos(angle_a) + axis_y * sin(angle_a)).normalized()
			var normal_b := (axis_x * cos(angle_b) + axis_y * sin(angle_b)).normalized()

			var a := start + normal_a * tube_radius
			var b := finish + normal_a * tube_radius
			var c := finish + normal_b * tube_radius
			var d := start + normal_b * tube_radius

			# Reverse winding so the tube faces point outwards.
			_add_vertex(surface, a, normal_a)
			_add_vertex(surface, c, normal_b)
			_add_vertex(surface, b, normal_a)
			_add_vertex(surface, a, normal_a)
			_add_vertex(surface, d, normal_b)
			_add_vertex(surface, c, normal_b)

	return surface.commit()

func _add_vertex(surface: SurfaceTool, vertex_position: Vector3, normal: Vector3) -> void:
	surface.set_normal(normal)
	surface.add_vertex(vertex_position)
