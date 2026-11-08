extends RefCounted
## Creates and updates the floating simulation display and optional preview.

const StrasTeleopHud = preload("res://robot_control/stras/visualization/display/stras_teleop_hud.gd")

var simulation_screen_distance: float = 1.4
var simulation_screen_size: Vector2 = Vector2(1.0, 0.53)
var simulation_screen_vertical_offset: float = 0.10
var simulation_view_scale: float = 1.5
var show_endoscope_preview: bool = false
var endoscope_preview_size: Vector2i = Vector2i(320, 240)
var add_default_camera: bool = false
var camera_distance: float = 0.3

var simulation_root: Node3D
var preview_root: Node3D

var _owner: Node3D
var _simulation_viewport: SubViewport
var _simulation_camera: Camera3D
var _simulation_screen: MeshInstance3D
var _teleop_hud: Control
var _preview_viewport: SubViewport
var _endoscope_camera: Camera3D


func setup(owner: Node3D) -> void:
	_owner = owner
	_setup_simulation_display()
	if show_endoscope_preview:
		_setup_endoscope_preview()
	if add_default_camera:
		_add_default_camera()


func update_endoscope_camera(data: Dictionary, robot_to_godot: Basis) -> void:
	var pose = data.get("pose")
	if not pose is Array or pose.size() != 4:
		return
	for row in pose:
		if not row is Array or row.size() != 4:
			return

	# OpenCV uses +Z forward and +Y down; Godot cameras use -Z forward and +Y up.
	var model_basis := Basis(
		Vector3(float(pose[0][0]), float(pose[1][0]), float(pose[2][0])),
		Vector3(float(pose[0][1]), float(pose[1][1]), float(pose[2][1])),
		Vector3(float(pose[0][2]), float(pose[1][2]), float(pose[2][2])))
	var optical_to_godot := Basis(Vector3.RIGHT, Vector3.DOWN, Vector3.FORWARD)
	var camera_basis := robot_to_godot * model_basis * optical_to_godot
	var camera_origin := robot_to_godot * Vector3(
		float(pose[0][3]), float(pose[1][3]), float(pose[2][3]))
	var camera_transform := Transform3D(camera_basis, camera_origin)
	_simulation_camera.transform = camera_transform
	if _endoscope_camera != null:
		_endoscope_camera.transform = camera_transform

	var intrinsics = data.get("intrinsics")
	if not intrinsics is Dictionary:
		return
	var width := int(intrinsics.get("width", 0))
	var height := int(intrinsics.get("height", 0))
	var fx := float(intrinsics.get("fx", 0.0))
	var fy := float(intrinsics.get("fy", 0.0))
	if width <= 0 or height <= 0 or fx <= 0.0 or fy <= 0.0:
		return
	_simulation_viewport.size = Vector2i(width, height)
	if _simulation_screen != null and _simulation_screen.mesh is QuadMesh:
		var quad := _simulation_screen.mesh as QuadMesh
		quad.size = Vector2(simulation_screen_size.y * float(width) / height,
			simulation_screen_size.y)
	# Model coordinates are millimetres. Keep the near plane very close because
	# the endoscope camera sits only about 0.5 mm from the arm tips at neutral.
	var near := 0.01
	var frustum_height := 2.0 * near * float(height) / fy * simulation_view_scale
	var cx := float(intrinsics.get("cx", width / 2.0))
	var cy := float(intrinsics.get("cy", height / 2.0))
	var frustum_offset := Vector2(
		(cx - width / 2.0) * 2.0 * near / fx * simulation_view_scale,
		(height / 2.0 - cy) * 2.0 * near / fy * simulation_view_scale)
	var far := 1000.0
	_simulation_camera.set_frustum(frustum_height, frustum_offset, near, far)
	if _endoscope_camera != null:
		_endoscope_camera.set_frustum(frustum_height, frustum_offset, near, far)


func update_teleop_hud(values: Dictionary) -> void:
	if _teleop_hud != null:
		_teleop_hud.set_values(values)


func _setup_simulation_display() -> void:
	_simulation_viewport = SubViewport.new()
	_simulation_viewport.name = "SimulationViewport"
	_simulation_viewport.size = Vector2i(1024, 543)
	_simulation_viewport.own_world_3d = true
	_simulation_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_owner.add_child(_simulation_viewport)

	simulation_root = Node3D.new()
	_simulation_viewport.add_child(simulation_root)
	_simulation_camera = Camera3D.new()
	simulation_root.add_child(_simulation_camera)
	_simulation_camera.current = true
	var environment := Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Color(0.025, 0.035, 0.055)
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = Color(0.72, 0.76, 0.84)
	environment.ambient_light_energy = 0.85
	_simulation_camera.environment = environment
	_add_camera_headlight(_simulation_camera)
	_setup_teleop_hud()

	var xr_camera := _owner.get_tree().root.find_child("XRCamera3D", true, false) as XRCamera3D
	if xr_camera == null:
		push_error("Could not find XRCamera3D for the floating simulation screen.")
		return
	_simulation_screen = MeshInstance3D.new()
	_simulation_screen.name = "FloatingSimulationScreen"
	var quad := QuadMesh.new()
	quad.size = simulation_screen_size
	_simulation_screen.mesh = quad
	_simulation_screen.position = Vector3(
		0.0, simulation_screen_vertical_offset, -simulation_screen_distance)
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_texture = _simulation_viewport.get_texture()
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	_simulation_screen.material_override = material
	xr_camera.add_child(_simulation_screen)


func _setup_teleop_hud() -> void:
	var hud_layer := CanvasLayer.new()
	hud_layer.name = "TeleoperationHudLayer"
	_simulation_viewport.add_child(hud_layer)
	_teleop_hud = Control.new()
	_teleop_hud.name = "TeleoperationHud"
	_teleop_hud.set_script(StrasTeleopHud)
	_teleop_hud.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	hud_layer.add_child(_teleop_hud)


func _setup_endoscope_preview() -> void:
	_preview_viewport = SubViewport.new()
	_preview_viewport.size = Vector2i(640, 480)
	_preview_viewport.own_world_3d = true
	_preview_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	preview_root = Node3D.new()
	_preview_viewport.add_child(preview_root)

	_endoscope_camera = Camera3D.new()
	_endoscope_camera.current = true
	preview_root.add_child(_endoscope_camera)
	_add_camera_headlight(_endoscope_camera)

	var layer := CanvasLayer.new()
	_owner.add_child(layer)
	var container := SubViewportContainer.new()
	container.stretch = true
	container.set_anchors_preset(Control.PRESET_BOTTOM_RIGHT)
	container.position = Vector2(-endoscope_preview_size.x - 20,
		-endoscope_preview_size.y - 20)
	container.size = Vector2(endoscope_preview_size)
	container.add_child(_preview_viewport)
	layer.add_child(container)


func _add_default_camera() -> void:
	var camera := Camera3D.new()
	_owner.add_child(camera)
	camera.look_at_from_position(Vector3(0.0, 0.05, camera_distance), Vector3.ZERO)
	_add_camera_headlight(camera)


func _add_camera_headlight(camera: Camera3D) -> void:
	var light := OmniLight3D.new()
	light.position = Vector3.ZERO
	light.omni_range = 250.0
	light.light_energy = 1.2
	camera.add_child(light)