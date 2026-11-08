extends Node3D

const NetworkConstants = preload("res://robot_control/network_constants.gd")
const StrasVisualizationConstants = preload(
	"res://robot_control/stras/visualization/visualization_constants.gd")

var _input_socket := PacketPeerUDP.new()
var _input_sequence := 0
var _robot_body_anchor_set := false


func _ready() -> void:
	process_mode = Node.PROCESS_MODE_ALWAYS
	get_viewport().use_xr = true
	get_viewport().transparent_bg = true
	RenderingServer.set_default_clear_color(Color(0, 0, 0, 0))
	print("Native: Composition layer defaults set to alpha zero clear.")
	var socket_error := _input_socket.set_dest_address(
		NetworkConstants.LOOPBACK_HOST, NetworkConstants.INPUT_PORT)
	if socket_error != OK:
		push_error("Could not set VR input UDP destination (%s:%d): %s" % [
			NetworkConstants.LOOPBACK_HOST, NetworkConstants.INPUT_PORT,
			error_string(socket_error)])
	else:
		print("Native: Sending VR input snapshots to %s:%d." % [
			NetworkConstants.LOOPBACK_HOST, NetworkConstants.INPUT_PORT])
	$XRRig/XROrigin3D/LeftHand.show_when_tracked = false
	$XRRig/XROrigin3D/RightHand.show_when_tracked = false

	var xr_interface = XRServer.find_interface("OpenXR")
	if xr_interface:
		if xr_interface.set_environment_blend_mode(XRInterface.XR_ENV_BLEND_MODE_ALPHA_BLEND):
			print("Native: OpenXR Alpha Blend Passthrough enabled.")
		else:
			print("Native: Passthrough blend mode rejected by runtime.")

func _process(_delta: float) -> void:
	var left_hand: XRController3D = $XRRig/XROrigin3D/LeftHand
	var right_hand: XRController3D = $XRRig/XROrigin3D/RightHand
	var xr_camera: XRCamera3D = $XRRig/XROrigin3D/XRCamera3D
	_anchor_robot_to_body(xr_camera)
	var payload := {
		"v": NetworkConstants.INPUT_PROTOCOL_VERSION,
		"seq": _input_sequence,
		"left": _controller_payload(left_hand),
		"right": _controller_payload(right_hand),
		"camera": {
			"position": _vector3_payload(xr_camera.global_position),
			"yaw": xr_camera.global_transform.basis.get_euler().y,
		},
		"dt": maxf(0.0, _delta),
	}
	_input_sequence += 1
	var send_error := _input_socket.put_packet(JSON.stringify(payload).to_utf8_buffer())
	if send_error != OK:
		push_warning("Could not send VR input UDP packet: %s" % error_string(send_error))


func _anchor_robot_to_body(xr_camera: XRCamera3D) -> void:
	var head_distance := xr_camera.global_position.length()
	var head_is_near_origin := (
		head_distance < StrasVisualizationConstants.ROBOT_ANCHOR_MIN_HEAD_DISTANCE_METRES
	)
	if _robot_body_anchor_set or head_is_near_origin:
		return
	var robot_geometry: Node3D = $World/RobotGeometry
	var body_position := (
		xr_camera.global_position
		+ Vector3.DOWN * StrasVisualizationConstants.ROBOT_BELOW_HEAD_METRES
	)
	var robot_yaw := xr_camera.global_transform.basis.get_euler().y
	# Preserve model X (so right/left stay on the correct sides), preserve Y,
	# and flip model +Z to Godot's camera-forward -Z before applying headset yaw.
	var model_to_forward := Basis(Vector3.RIGHT, Vector3.UP, Vector3.FORWARD)
	var body_basis := Basis(Vector3.UP, robot_yaw) * model_to_forward
	# Anchor once at the tracked head pose as a torso proxy; do not alter camera calibration.
	robot_geometry.global_transform = Transform3D(body_basis, body_position)
	_robot_body_anchor_set = true
	print("Native: Robot geometry anchored below the initial XR head pose.")


func _exit_tree() -> void:
	_input_socket.close()


func _controller_payload(controller: XRController3D) -> Dictionary:
	if not controller.get_is_active():
		return {
			"connected": false,
			"position": {"x": 0.0, "y": 0.0, "z": 0.0},
			"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
			"trigger": 0.0,
			"grip": 0.0,
			"primary": {"x": 0.0, "y": 0.0},
			"secondary": {"x": 0.0, "y": 0.0},
			"buttons": _empty_buttons(),
		}
	var orientation := controller.global_transform.basis.get_rotation_quaternion()
	var primary := controller.get_vector2("primary")
	var secondary := controller.get_vector2("secondary")
	return {
		"connected": true,
		"position": _vector3_payload(controller.global_position),
		"orientation": {
			"x": orientation.x, "y": orientation.y,
			"z": orientation.z, "w": orientation.w,
		},
		"trigger": clampf(controller.get_float("trigger"), 0.0, 1.0),
		"grip": clampf(controller.get_float("grip"), 0.0, 1.0),
		"primary": {"x": clampf(primary.x, -1.0, 1.0), "y": clampf(primary.y, -1.0, 1.0)},
		"secondary": {"x": clampf(secondary.x, -1.0, 1.0), "y": clampf(secondary.y, -1.0, 1.0)},
		"buttons": {
			"trigger_click": controller.is_button_pressed("trigger_click"),
			"trigger_touch": controller.is_button_pressed("trigger_touch"),
			"primary_click": controller.is_button_pressed("primary_click"),
			"primary_touch": controller.is_button_pressed("primary_touch"),
			"secondary_click": controller.is_button_pressed("secondary_click"),
			"secondary_touch": controller.is_button_pressed("secondary_touch"),
		},
	}


func _empty_buttons() -> Dictionary:
	return {
		"trigger_click": false,
		"trigger_touch": false,
		"primary_click": false,
		"primary_touch": false,
		"secondary_click": false,
		"secondary_touch": false,
	}


func _vector3_payload(value: Vector3) -> Dictionary:
	return {"x": value.x, "y": value.y, "z": value.z}
