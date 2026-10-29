extends Node3D
## Receives STRAS geometry from ease_stras.godot_visualization.GodotVisualizer
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

@export var add_default_camera: bool = true
@export var camera_distance: float = 0.3

const PROTOCOL_VERSION := 1
const PARTS := ["right", "left", "endoscope"]

var last_frame: int = -1
var _udp := PacketPeerUDP.new()
var _meshes: Dictionary = {}


func _ready() -> void:
	var err := _udp.bind(port)
	if err != OK:
		push_error("Could not bind UDP port %d (error %d)" % [port, err])
	_add_part("right", right_color)
	_add_part("left", left_color)
	_add_part("endoscope", endoscope_color)
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
	for part in PARTS:
		var raw = msg.get(part)
		var instance: MeshInstance3D = _meshes[part]
		if raw == null or raw.size() < 2:
			instance.mesh = null
			continue
		instance.mesh = _build_tube(_to_points(raw), tube_radius)


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


func _add_part(part_name: String, color: Color) -> void:
	var instance := MeshInstance3D.new()
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.cull_mode = BaseMaterial3D.CULL_DISABLED  # winding-order safe
	instance.material_override = material
	add_child(instance)
	_meshes[part_name] = instance


func _add_camera_and_light() -> void:
	var cam := Camera3D.new()
	add_child(cam)
	cam.look_at_from_position(Vector3(0.0, 0.05, camera_distance), Vector3.ZERO)
	var light := DirectionalLight3D.new()
	add_child(light)
	light.rotation_degrees = Vector3(-45.0, 30.0, 0.0)