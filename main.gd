extends Node3D

const INPUT_HOST := "127.0.0.1"
const INPUT_PORT := 9000
const PROTOCOL_VERSION := 1

var _input_socket := PacketPeerUDP.new()
var _input_sequence := 0


func _ready() -> void:
	process_mode = Node.PROCESS_MODE_ALWAYS
	get_viewport().use_xr = true
	get_viewport().transparent_bg = true
	RenderingServer.set_default_clear_color(Color(0, 0, 0, 0))
	print("Native: Composition layer defaults set to alpha zero clear.")
	var socket_error := _input_socket.set_dest_address(INPUT_HOST, INPUT_PORT)
	if socket_error != OK:
		push_error("Could not set VR input UDP destination (%s:%d): %s" % [
			INPUT_HOST, INPUT_PORT, error_string(socket_error)])
	else:
		print("Native: Sending VR input snapshots to %s:%d." % [INPUT_HOST, INPUT_PORT])
	_setup_hand_marker($XRRig/XROrigin3D/LeftHand, Color.GREEN)
	_setup_hand_marker($XRRig/XROrigin3D/RightHand, Color.BLUE)

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
	var payload := {
		"v": PROTOCOL_VERSION,
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


func _exit_tree() -> void:
	_input_socket.close()


func _setup_hand_marker(controller: XRController3D, color: Color) -> void:
	var mesh_instance := MeshInstance3D.new()
	mesh_instance.name = "TrackedHandMarker"
	var sphere := SphereMesh.new()
	sphere.radius = 0.025
	sphere.height = 0.05
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	mesh_instance.mesh = sphere
	mesh_instance.material_override = material
	controller.show_when_tracked = true
	controller.add_child(mesh_instance)


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
