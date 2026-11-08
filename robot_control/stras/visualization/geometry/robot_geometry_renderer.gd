extends RefCounted
## Maintains the section meshes in the world, simulation screen, and preview.

const TubeMeshBuilder = preload("res://robot_control/stras/visualization/geometry/tube_mesh_builder.gd")

var robot_to_godot: Basis = Basis.IDENTITY
var tube_radius_mm: float = 2.0
var radial_segments: int = 12

var _owner: Node3D
var _simulation_root: Node3D
var _preview_root: Node3D
var _meshes: Dictionary = {}
var _simulation_meshes: Dictionary = {}
var _preview_meshes: Dictionary = {}
var _diameters_mm: Dictionary = {}
var _section_colors: Dictionary = {}


func configure(owner: Node3D, simulation_root: Node3D, preview_root: Node3D,
		options: Dictionary) -> void:
	_owner = owner
	_simulation_root = simulation_root
	_preview_root = preview_root
	robot_to_godot = options.robot_to_godot
	tube_radius_mm = options.tube_radius_mm
	radial_segments = options.radial_segments
	_add_part("right", options.right_section_colors,
		options.right_section_diameters_mm)
	_add_part("left", options.left_section_colors,
		options.left_section_diameters_mm)


func update_frame(message: Dictionary) -> void:
	for part in ["right", "left"]:
		var sections = message.get(part + "_sections")
		if sections is Array:
			_set_sections(part, sections)
		else:
			_set_flat_geometry(part, message.get(part))


func _set_sections(part: String, sections: Array) -> void:
	var instances: Array = _meshes[part]
	var diameters: PackedFloat32Array = _diameters_mm[part]
	var first_valid_section := -1
	var last_valid_section := -1
	for i in sections.size():
		if sections[i] is Array and sections[i].size() >= 2:
			if first_valid_section < 0:
				first_valid_section = i
			last_valid_section = i
	while instances.size() < sections.size():
		var section_index := instances.size()
		var color := _section_color(part, section_index)
		instances.append(_create_mesh_instance(part, color, _owner))
		_simulation_meshes[part].append(
			_create_mesh_instance(part, color, _simulation_root))
		if _preview_root != null:
			_preview_meshes[part].append(
				_create_mesh_instance(part, color, _preview_root))
	for i in instances.size():
		if i >= sections.size() or not sections[i] is Array or sections[i].size() < 2:
			_set_part_mesh(part, i, null)
			continue
		var radius := tube_radius_mm
		if i < diameters.size():
			radius = diameters[i] * 0.5
		var points := TubeMeshBuilder.to_points(sections[i], robot_to_godot)
		var mesh := TubeMeshBuilder.build_tube(points, radius, radial_segments,
			i == first_valid_section, i == last_valid_section)
		_set_part_mesh(part, i, mesh)


func _set_flat_geometry(part: String, raw) -> void:
	var instances: Array = _meshes[part]
	for i in instances.size():
		if i == 0 and raw is Array and raw.size() >= 2:
			var points := TubeMeshBuilder.to_points(raw, robot_to_godot)
			_set_part_mesh(part, i, TubeMeshBuilder.build_tube(
				points, tube_radius_mm, radial_segments, true, true))
		else:
			_set_part_mesh(part, i, null)


func _set_part_mesh(part: String, index: int, mesh: ArrayMesh) -> void:
	(_meshes[part][index] as MeshInstance3D).mesh = mesh
	(_simulation_meshes[part][index] as MeshInstance3D).mesh = mesh
	if index < _preview_meshes[part].size():
		(_preview_meshes[part][index] as MeshInstance3D).mesh = mesh


func _add_part(part_name: String, colors: PackedColorArray,
		diameters_mm: PackedFloat32Array) -> void:
	_meshes[part_name] = []
	_simulation_meshes[part_name] = []
	_preview_meshes[part_name] = []
	_diameters_mm[part_name] = diameters_mm
	_section_colors[part_name] = colors
	var color := _section_color(part_name, 0)
	_meshes[part_name].append(_create_mesh_instance(part_name, color, _owner))
	_simulation_meshes[part_name].append(
		_create_mesh_instance(part_name, color, _simulation_root))
	if _preview_root != null:
		_preview_meshes[part_name].append(
			_create_mesh_instance(part_name, color, _preview_root))


func _section_color(part_name: String, section_index: int) -> Color:
	var colors: PackedColorArray = _section_colors[part_name]
	if section_index < colors.size():
		return colors[section_index]
	return Color.BLACK


func _create_mesh_instance(part_name: String, color: Color,
		parent: Node) -> MeshInstance3D:
	var instance := MeshInstance3D.new()
	instance.name = part_name + "_Mesh"
	if parent == _owner:
		instance.visible = false
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.metallic_specular = 0.4
	material.roughness = 0.4
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	instance.material_override = material
	parent.add_child(instance)
	return instance