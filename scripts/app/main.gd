extends Node3D

@onready var floating_screen: MeshInstance3D = $FloatingScreen
@onready var screen_viewport: SubViewport = $ScreenViewport
@onready var left_controller: XRController3D = $XROrigin3D/lefthand
@onready var right_controller: XRController3D = $XROrigin3D/righthand
@onready var telemetry_hud: Control = $ScreenViewport/TelemetryHUD
@onready var controller_visualizer: Node = $ControllerVisualizer
@onready var right_arm: Node3D = $ScreenViewport/ViewportWorld/RightArm
@onready var left_arm: Node3D = $ScreenViewport/ViewportWorld/LeftArm
@onready var stras_udp_client: Node = $PythonXRManager

func _ready() -> void:
	get_viewport().use_xr = true
	get_viewport().transparent_bg = true
	RenderingServer.set_default_clear_color(Color(0, 0, 0, 0))

	var screen_material := floating_screen.material_override as StandardMaterial3D
	screen_material.albedo_texture = screen_viewport.get_texture()
	screen_material.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR

	var xr_interface = XRServer.find_interface("OpenXR")
	if xr_interface:
		if xr_interface.set_environment_blend_mode(XRInterface.XR_ENV_BLEND_MODE_ALPHA_BLEND):
			print("Native: OpenXR alpha blend passthrough enabled.")
		else:
			print("Native: passthrough blend mode rejected by runtime.")
	
	$XROrigin3D/lefthand/leftsphere.visible = false
	$XROrigin3D/righthand/rightsphere.visible = false
	$ScreenViewport/ViewportWorld/LeftMarker.visible = false
	$ScreenViewport/ViewportWorld/RightMarker.visible = false
	stras_udp_client.shapes_received.connect(_on_shapes_received)

func _process(_delta: float) -> void:
	_update_controller(left_controller, &"left")
	_update_controller(right_controller, &"right")

func _update_controller(controller: XRController3D, side: StringName) -> void:
	var pose: Transform3D = controller_visualizer.update_hand(controller, side)
	var controller_rotation := pose.basis.get_euler()
	controller_rotation.y = controller_visualizer.get_continuous_yaw(side)
	telemetry_hud.set_controller_state(side, pose.origin, controller_rotation)

func _on_shapes_received(_sequence: int, right_points: PackedVector3Array, left_points: PackedVector3Array) -> void:
	right_arm.update_polyline(right_points)
	left_arm.update_polyline(left_points)
