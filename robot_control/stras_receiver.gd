extends Node3D
## Receives STRAS geometry from robot_control.fk_runner
## and draws each arm / the endoscope as a tube.
##
## Setup (Godot 4.x): attach this script to an empty Node3D and press play.
## It creates its own camera and light unless you turn that off below.

@export var port: int = 9100

## Robot model units -> metres (0.001 if the model works in millimetres).
@export var unit_scale: float = 0.001
## Fixed robot-frame -> Godot-frame rotation (Godot is Y-up, right-handed).
## Identity by default; edit once you see how the robot appears.
## Example, robot Z-up -> Godot Y-up:
##   Basis(Vector3(1, 0, 0), Vector3(0, 0, -1), Vector3(0, 1, 0))
@export var robot_to_godot: Basis = Basis.IDENTITY

@export var tube_radius: float = 0.002
@export var radial_segments: int = 12
@export var right_color: Color = Color(0.90, 0.30, 0.30)
@export var left_color: Color = Color(0.30, 0.50, 0.90)
@export var endoscope_color: Color = Color(0.85, 0.85, 0.85)
## Section diameters from the STRAS model, in millimetres (VTK's old units).
@export var right_section_diameters_mm: PackedFloat32Array = PackedFloat32Array([3.5, 3.5, 3.5])
@export var left_section_diameters_mm: PackedFloat32Array = PackedFloat32Array([3.5, 3.5, 3.5])
@export var endoscope_section_diameters_mm: PackedFloat32Array = PackedFloat32Array([3.5, 3.5, 3.5])

@export var add_default_camera: bool = true
@export var camera_distance: float = 0.3
## Show a calibrated endoscope-camera view without replacing the XR headset view.
@export var show_endoscope_preview: bool = true
@export var endoscope_preview_size: Vector2i = Vector2i(320, 240)

const PROTOCOL_VERSION := 1
const PARTS := ["right", "left", "endoscope"]

@export var last_frame: int = -1
var _logged_first_frame := false
var _udp := PacketPeerUDP.new()
var _meshes: Dictionary = {}
var _diameters_mm: Dictionary = {}
var _colors: Dictionary = {}
var _preview_meshes: Dictionary = {}
var _preview_viewport: SubViewport
var _preview_root: Node3D
var _endoscope_camera: Camera3D


func _ready() -> void:
	var err := _udp.bind(port)
	if err != OK:
		push_error("Could not bind UDP port %d (error %d)" % [port, err])
	else:
		print("STRAS receiver listening on UDP port %d" % port)
	if show_endoscope_preview:
		_setup_endoscope_preview()
	_add_part("right", right_color, right_section_diameters_mm)
	_add_part("left", left_color, left_section_diameters_mm)
	_add_part("endoscope", endoscope_color, endoscope_section_diameters_mm)
	if add_default_camera:
		_add_camera_and_light()


func _process(_delta: float) -> void:
	# Drain the socket and keep only the newest frame, so a slow frame
	# never builds up a backlog of stale poses.
	var latest = null
	while _udp.get_available_packet_count() > 0:
		var parsed = JSON.parse_string(_udp.get_packet().get_string_from_utf8())
		if parsed is Dictionary:
			latest = parsed
	if latest != null:
		_apply(latest)


func _apply(msg: Dictionary) -> void:
	if int(msg.get("v", 0)) != PROTOCOL_VERSION:
		push_warning("Unsupported message version: %s" % str(msg.get("v")))
		return
	last_frame = int(msg.get("frame", -1))
	if not _logged_first_frame:
		print("STRAS receiver got geometry frame %d" % last_frame)
		_logged_first_frame = true
	for part in PARTS:
		var sections = msg.get(part + "_sections")
		if sections is Array:
			_set_sections(part, sections)
		else:
			_set_flat_geometry(part, msg.get(part))
	var camera_data = msg.get("camera")
	if camera_data is Dictionary:
		_update_endoscope_camera(camera_data)


func _set_sections(part: String, sections: Array) -> void:
	var instances: Array = _meshes[part]
	var diameters: PackedFloat32Array = _diameters_mm[part]
	while instances.size() < sections.size():
		instances.append(_create_mesh_instance(part, _colors[part]))
		if _preview_root != null:
			_preview_meshes[part].append(
				_create_mesh_instance(part, _colors[part], _preview_root))
	for i in instances.size():
		if i >= sections.size() or not sections[i] is Array or sections[i].size() < 2:
			_set_part_mesh(part, i, null)
			continue
		var radius := tube_radius
		if i < diameters.size():
			radius = diameters[i] * unit_scale * 0.5
		_set_part_mesh(part, i, _build_tube(_to_points(sections[i]), radius))


func _set_flat_geometry(part: String, raw) -> void:
	var instances: Array = _meshes[part]
	for i in instances.size():
		if i == 0 and raw is Array and raw.size() >= 2:
			_set_part_mesh(part, i, _build_tube(_to_points(raw), tube_radius))
		else:
			_set_part_mesh(part, i, null)


func _set_part_mesh(part: String, index: int, mesh: ArrayMesh) -> void:
	(_meshes[part][index] as MeshInstance3D).mesh = mesh
	if index < _preview_meshes[part].size():
		(_preview_meshes[part][index] as MeshInstance3D).mesh = mesh


func _to_points(raw: Array) -> PackedVector3Array:
	var out := PackedVector3Array()
	for p in raw:
		out.append(robot_to_godot * (Vector3(p[0], p[1], p[2]) * unit_scale))
	return out


func _build_tube(points: PackedVector3Array, radius: float) -> ArrayMesh:
	var n := points.size()
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)

	# One ring of vertices per point, using parallel transport so the ring
	# does not twist along the backbone.
	var rings: Array = []
	var prev_normal := Vector3.UP
	for i in n:
		var tangent: Vector3
		if i == 0:
			tangent = points[1] - points[0]
		elif i == n - 1:
			tangent = points[i] - points[i - 1]
		else:
			tangent = points[i + 1] - points[i - 1]
		if tangent.length() < 1e-9:
			tangent = Vector3.FORWARD
		tangent = tangent.normalized()

		var normal := prev_normal - tangent * prev_normal.dot(tangent)
		if normal.length() < 1e-6:
			normal = tangent.cross(Vector3.RIGHT)
			if normal.length() < 1e-6:
				normal = tangent.cross(Vector3.UP)
		normal = normal.normalized()
		prev_normal = normal
		var binormal := tangent.cross(normal)

		var ring: Array = []
		for j in radial_segments:
			var a := TAU * float(j) / float(radial_segments)
			ring.append(points[i] + (normal * cos(a) + binormal * sin(a)) * radius)
		rings.append(ring)

	for i in n - 1:
		for j in radial_segments:
			var j2 := (j + 1) % radial_segments
			var a: Vector3 = rings[i][j]
			var b: Vector3 = rings[i][j2]
			var c: Vector3 = rings[i + 1][j]
			var d: Vector3 = rings[i + 1][j2]
			st.add_vertex(a)
			st.add_vertex(c)
			st.add_vertex(b)
			st.add_vertex(b)
			st.add_vertex(c)
			st.add_vertex(d)
	st.generate_normals()
	return st.commit()


func _add_part(part_name: String, color: Color, diameters_mm: PackedFloat32Array) -> void:
	_meshes[part_name] = []
	_preview_meshes[part_name] = []
	_diameters_mm[part_name] = diameters_mm
	_colors[part_name] = color
	_meshes[part_name].append(_create_mesh_instance(part_name, color))
	if _preview_root != null:
		_preview_meshes[part_name].append(
			_create_mesh_instance(part_name, color, _preview_root))


func _create_mesh_instance(part_name: String, color: Color,
		parent: Node = null) -> MeshInstance3D:
	var instance := MeshInstance3D.new()
	instance.name = part_name + "_Mesh"
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.cull_mode = BaseMaterial3D.CULL_DISABLED  # winding-order safe
	instance.material_override = material
	if parent == null:
		add_child(instance)
	else:
		parent.add_child(instance)
	return instance


func _setup_endoscope_preview() -> void:
	_preview_viewport = SubViewport.new()
	_preview_viewport.size = Vector2i(640, 480)
	_preview_viewport.own_world_3d = true
	_preview_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_preview_root = Node3D.new()
	_preview_viewport.add_child(_preview_root)

	_endoscope_camera = Camera3D.new()
	_endoscope_camera.current = true
	_preview_root.add_child(_endoscope_camera)
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-35.0, 25.0, 0.0)
	_preview_root.add_child(light)

	var layer := CanvasLayer.new()
	add_child(layer)
	var container := SubViewportContainer.new()
	container.stretch = true
	container.set_anchors_preset(Control.PRESET_BOTTOM_RIGHT)
	container.position = Vector2(-endoscope_preview_size.x - 20,
		-endoscope_preview_size.y - 20)
	container.size = Vector2(endoscope_preview_size)
	container.add_child(_preview_viewport)
	layer.add_child(container)


func _update_endoscope_camera(data: Dictionary) -> void:
	if _endoscope_camera == null:
		return
	var pose = data.get("pose")
	if not pose is Array or pose.size() != 4:
		return
	var intrinsics = data.get("intrinsics")
	if not intrinsics is Dictionary:
		return
	if pose[0].size() != 4 or pose[1].size() != 4 or pose[2].size() != 4:
		return

	# Model/OpenCV camera coordinates use +Z forward and +Y down; Godot cameras
	# use -Z forward and +Y up. Te is the endoscope/camera pose in model space.
	var model_basis := Basis(
		Vector3(float(pose[0][0]), float(pose[1][0]), float(pose[2][0])),
		Vector3(float(pose[0][1]), float(pose[1][1]), float(pose[2][1])),
		Vector3(float(pose[0][2]), float(pose[1][2]), float(pose[2][2])))
	var optical_to_godot := Basis(Vector3.RIGHT, Vector3.DOWN, Vector3.FORWARD)
	var camera_basis := robot_to_godot * model_basis * optical_to_godot
	var camera_origin := robot_to_godot * Vector3(
		float(pose[0][3]), float(pose[1][3]), float(pose[2][3])) * unit_scale
	_endoscope_camera.transform = Transform3D(camera_basis, camera_origin)

	var width := int(intrinsics.get("width", 0))
	var height := int(intrinsics.get("height", 0))
	var fx := float(intrinsics.get("fx", 0.0))
	var fy := float(intrinsics.get("fy", 0.0))
	if width <= 0 or height <= 0 or fx <= 0.0 or fy <= 0.0:
		return
	var near := 0.01
	var frustum_height := 2.0 * near * float(height) / fy
	var cx := float(intrinsics.get("cx", width / 2.0))
	var cy := float(intrinsics.get("cy", height / 2.0))
	var frustum_offset := Vector2(
		(cx - width / 2.0) * 2.0 * near / fx,
		(height / 2.0 - cy) * 2.0 * near / fy)
	_endoscope_camera.set_frustum(frustum_height, frustum_offset, near, 10.0)


func _add_camera_and_light() -> void:
	var cam := Camera3D.new()
	add_child(cam)
	cam.look_at_from_position(Vector3(0.0, 0.05, camera_distance), Vector3.ZERO)
	var light := DirectionalLight3D.new()
	add_child(light)
	light.rotation_degrees = Vector3(-45.0, 30.0, 0.0)
