extends Node3D
## Coordinates the STRAS geometry stream, renderer, and XR camera display.

const GeometryUdpReceiver = preload("res://robot_control/stras/visualization/network/geometry_udp_receiver.gd")
const RobotGeometryRenderer = preload("res://robot_control/stras/visualization/geometry/robot_geometry_renderer.gd")
const StrasCameraDisplay = preload("res://robot_control/stras/visualization/display/stras_camera_display.gd")
const NetworkConstants = preload("res://robot_control/network_constants.gd")

@export var port: int = NetworkConstants.GEOMETRY_PORT

## Fixed robot-frame -> Godot-frame rotation (Godot is Y-up, right-handed).
## Example, robot Z-up -> Godot Y-up:
##   Basis(Vector3(1, 0, 0), Vector3(0, 0, -1), Vector3(0, 1, 0))
@export var robot_to_godot: Basis = Basis.IDENTITY

## Fallback radius for flat geometry, expressed in the model's millimetres.
@export var tube_radius_mm: float = 2.0
@export_range(3, 32) var radial_segments: int = 12
## One color per model section, from proximal shaft to distal tip.
@export var right_section_colors: PackedColorArray = PackedColorArray([
	Color(0.025, 0.025, 0.03), Color(0.025, 0.025, 0.03), Color(0.78, 0.79, 0.82),
])
@export var left_section_colors: PackedColorArray = PackedColorArray([
	Color(0.025, 0.025, 0.03), Color(0.025, 0.025, 0.03), Color(0.78, 0.79, 0.82),
])
## Section diameters from the STRAS model, in millimetres (VTK's old units).
@export var right_section_diameters_mm: PackedFloat32Array = PackedFloat32Array([3.5, 3.5, 3.5])
@export var left_section_diameters_mm: PackedFloat32Array = PackedFloat32Array([3.5, 3.5, 3.5])

@export var add_default_camera: bool = false
@export var camera_distance: float = 0.3
@export var simulation_screen_distance: float = 1.4
@export var simulation_screen_size: Vector2 = Vector2(1.0, 0.53)
@export var simulation_screen_vertical_offset: float = 0.10
## Values above 1 zoom out from the calibrated endoscope view.
@export_range(1.0, 3.0, 0.1) var simulation_view_scale: float = 1.5
@export var show_endoscope_preview: bool = false
@export var endoscope_preview_size: Vector2i = Vector2i(320, 240)

@export var last_frame: int = -1

var _logged_first_frame := false
var _udp_receiver
var _renderer
var _display


func _ready() -> void:
	_display = StrasCameraDisplay.new()
	_display.simulation_screen_distance = simulation_screen_distance
	_display.simulation_screen_size = simulation_screen_size
	_display.simulation_screen_vertical_offset = simulation_screen_vertical_offset
	_display.simulation_view_scale = simulation_view_scale
	_display.show_endoscope_preview = show_endoscope_preview
	_display.endoscope_preview_size = endoscope_preview_size
	_display.add_default_camera = add_default_camera
	_display.camera_distance = camera_distance
	_display.setup(self)

	_renderer = RobotGeometryRenderer.new()
	_renderer.configure(self, _display.simulation_root, _display.preview_root, {
		"robot_to_godot": robot_to_godot,
		"tube_radius_mm": tube_radius_mm,
		"radial_segments": radial_segments,
		"right_section_colors": right_section_colors,
		"left_section_colors": left_section_colors,
		"right_section_diameters_mm": right_section_diameters_mm,
		"left_section_diameters_mm": left_section_diameters_mm,
	})

	_udp_receiver = GeometryUdpReceiver.new()
	_udp_receiver.port = port
	_udp_receiver.frame_received.connect(_on_geometry_frame_received)
	add_child(_udp_receiver)


func _on_geometry_frame_received(message: Dictionary) -> void:
	if int(message.get("v", 0)) != NetworkConstants.GEOMETRY_PROTOCOL_VERSION:
		push_warning("Unsupported message version: %s" % str(message.get("v")))
		return
	last_frame = int(message.get("frame", -1))
	if not _logged_first_frame:
		print("STRAS receiver got geometry frame %d" % last_frame)
		_logged_first_frame = true
	_renderer.update_frame(message)
	var hud_data = message.get("hud")
	if hud_data is Dictionary:
		_display.update_teleop_hud(hud_data)
	var camera_data = message.get("camera")
	if camera_data is Dictionary:
		_display.update_endoscope_camera(camera_data, robot_to_godot)