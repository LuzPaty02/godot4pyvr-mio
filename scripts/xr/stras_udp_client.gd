extends Node3D

signal shapes_received(sequence: int, right_points: PackedVector3Array, left_points: PackedVector3Array)

@export var server_host := "127.0.0.1"
@export var server_port := 4242
@export var local_port := 4243
@export var request_rate_hz := 60.0
@export var send_test_configuration := false
@export_range(0.0, 5.0, 0.01) var bend_gain := 1.0
@export_range(0.0, 5.0, 0.01) var twist_gain := 1.0
@export_range(0.0, 5.0, 0.1) var deadzone_deg := 0.5
@export_range(0.05, 0.99, 0.01) var low_pass_alpha := 0.25
@export var left_hand_signs := Vector3(-1.0, 1.0, 1.0)
@export var right_hand_signs := Vector3(1.0, 1.0, 1.0)
@export var test_configuration := PackedFloat32Array([
	0.02, 0.1, 25.0,
	0.02, -0.1, 25.0,
	0.001, 0.001, 0.1, 10.0,
])

var _origin: XROrigin3D
var _station: Node3D
var _left_controller: XRController3D
var _right_controller: XRController3D
var _controller_zero_pose: Dictionary[int, Transform3D] = {}
var _controller_filtered_q: Dictionary[int, PackedFloat32Array] = {}

const MAGIC := "STRS"
const POINT_COUNT := 32
const REQUEST_SIZE := 4 + 4 + 10 * 4
const RESPONSE_SIZE := 4 + 4 + 1 + 2 * POINT_COUNT * 3 * 4

var _udp := PacketPeerUDP.new()
var _sequence := 0
var _last_received_sequence := -1
var _send_timer := 0.0
var _has_reported_response := false

func _ready() -> void:
	_origin = get_parent().get_node("XROrigin3D") as XROrigin3D
	_station = get_parent().get_node("Station") as Node3D
	_left_controller = _origin.get_node("lefthand") as XRController3D
	_right_controller = _origin.get_node("righthand") as XRController3D

	if is_instance_valid(_left_controller):
		_left_controller.button_pressed.connect(_on_controller_button_pressed)
	if is_instance_valid(_right_controller):
		_right_controller.button_pressed.connect(_on_controller_button_pressed)
	get_tree().create_timer(2.0).timeout.connect(calibrate)

	var bind_error := _udp.bind(local_port, "127.0.0.1")
	if bind_error != OK:
		push_error("STRAS UDP client could not bind local port %d (error %d)." % [local_port, bind_error])
		set_process(false)
		return

	var destination_error := _udp.set_dest_address(server_host, server_port)
	if destination_error != OK:
		push_error("STRAS UDP client could not set server %s:%d (error %d)." % [server_host, server_port, destination_error])
		set_process(false)
		return

	print("STRAS UDP client ready; server %s:%d, local port %d." % [server_host, server_port, local_port])

func _process(delta: float) -> void:
	_receive_responses()

	_send_timer -= delta
	if _send_timer > 0.0:
		return

	_send_timer = 1.0 / maxf(request_rate_hz, 1.0)
	var configuration := _build_controller_configuration() if not send_test_configuration else test_configuration
	send_configuration(configuration)

func _build_controller_configuration() -> PackedFloat32Array:
	var right_q := _controller_to_q(_right_controller)
	var left_q := _controller_to_q(_left_controller)
	var config := PackedFloat32Array()
	config.resize(10)
	# STRAS config order is [BR, RR, TR, BL, RL, TL, EBV, EBH, ER, ET]
	# The left/right mirroring is handled by the arm geometry, not by changing the
	# joint math between hands.
	config[0] = right_q[0]
	config[1] = right_q[1]
	config[2] = right_q[2]
	config[3] = left_q[0]
	config[4] = left_q[1]
	config[5] = left_q[2]
	config[6] = 0.0
	config[7] = 0.0
	config[8] = 0.0
	config[9] = 0.0
	return config

func _controller_to_q(controller: XRController3D) -> PackedFloat32Array:
	var q := PackedFloat32Array([0.0, 0.0, 0.0])
	if not is_instance_valid(controller) or not is_instance_valid(_station) or not is_instance_valid(_origin):
		return q

	var key := controller.get_instance_id()
	if not _controller_zero_pose.has(key):
		calibrate()
	if not _controller_zero_pose.has(key):
		return q

	var zero_pose: Transform3D = _controller_zero_pose[key]
	var current_pose := _station.global_transform.affine_inverse() * controller.global_transform
	var relative_pose := zero_pose.affine_inverse() * current_pose
	var sign_vec := _controller_signs(controller)
	var local_offset := relative_pose.origin * sign_vec
	var bend_raw := -_relative_rotation_about_local_x(relative_pose.basis) * sign_vec.x
	var twist_raw := _relative_rotation_about_local_y(relative_pose.basis) * sign_vec.y
	var rail_mm := -local_offset.z * 1000.0   # controller forward is -Z
	var deadzone := deg_to_rad(deadzone_deg)
	if abs(bend_raw) <= deadzone:
		bend_raw = 0.0
	else:
		bend_raw -= signf(bend_raw) * deadzone
	if abs(twist_raw) <= deadzone:
		twist_raw = 0.0
	else:
		twist_raw -= signf(twist_raw) * deadzone

	var bend_limit := PI / (2.0 * 18.5)
	var handle_limit := deg_to_rad(44.0)
	var bend_target := clampf(bend_raw * bend_gain, -handle_limit, handle_limit) / handle_limit * bend_limit
	var twist_handle_limit := deg_to_rad(163.0)
	var twist_joint_limit := deg_to_rad(160.0)
	var twist_target := clampf(twist_raw * twist_gain, -twist_handle_limit, twist_handle_limit) / twist_handle_limit * twist_joint_limit
	var reach_target := clampf(remap(clampf(rail_mm, 0.0, 105.0), 0.0, 105.0, 2.0, 72.0), 2.0, 72.0)

	var filtered: PackedFloat32Array = _controller_filtered_q.get(key, PackedFloat32Array([0.0, 0.0, 0.0]))
	filtered[0] = lerpf(filtered[0], bend_target, low_pass_alpha)
	filtered[1] = lerpf(filtered[1], twist_target, low_pass_alpha)
	filtered[2] = lerpf(filtered[2], reach_target, low_pass_alpha)
	_controller_filtered_q[key] = filtered

	q[0] = filtered[0]
	q[1] = filtered[1]
	q[2] = filtered[2]
	return q

func _controller_signs(controller: XRController3D) -> Vector3:
	if controller == _left_controller:
		return left_hand_signs
	if controller == _right_controller:
		return right_hand_signs
	return Vector3(1.0, 1.0, 1.0)

func calibrate() -> void:
	if is_instance_valid(_left_controller):
		_controller_zero_pose[_left_controller.get_instance_id()] = _station.global_transform.affine_inverse() * _left_controller.global_transform
	if is_instance_valid(_right_controller):
		_controller_zero_pose[_right_controller.get_instance_id()] = _station.global_transform.affine_inverse() * _right_controller.global_transform

func _on_controller_button_pressed(button_name: StringName) -> void:
	if String(button_name).to_lower() == "grip_click":
		calibrate()

func _relative_rotation_about_local_x(basis: Basis) -> float:
	return atan2(basis.z.y, basis.y.y)

func _relative_rotation_about_local_y(basis: Basis) -> float:
	return atan2(basis.x.z, basis.x.x)

func send_configuration(configuration: PackedFloat32Array) -> bool:
	if configuration.size() != 10:
		push_error("STRAS configuration must contain exactly 10 float values; got %d." % configuration.size())
		return false

	var packet := PackedByteArray()
	packet.resize(REQUEST_SIZE)
	packet.encode_u32(0, int(MAGIC.to_ascii_buffer()[0]) | (int(MAGIC.to_ascii_buffer()[1]) << 8) | (int(MAGIC.to_ascii_buffer()[2]) << 16) | (int(MAGIC.to_ascii_buffer()[3]) << 24))
	packet.encode_u32(4, _sequence)
	for index in range(10):
		packet.encode_float(8 + index * 4, configuration[index])

	var error := _udp.put_packet(packet)
	if error != OK:
		push_error("Could not send STRAS UDP request (error %d)." % error)
		return false

	_sequence += 1
	return true

func _receive_responses() -> void:
	while _udp.get_available_packet_count() > 0:
		var packet := _udp.get_packet()
		if packet.size() != RESPONSE_SIZE:
			continue
		if packet.slice(0, 4).get_string_from_ascii() != MAGIC:
			continue

		var response_sequence := packet.decode_u32(4)
		var status := packet[8]
		if response_sequence <= _last_received_sequence:
			continue
		_last_received_sequence = response_sequence
		if status != 0:
			push_warning("STRAS model rejected request %d." % response_sequence)
			continue

		var right_points := _decode_points(packet, 9)
		var left_points := _decode_points(packet, 9 + POINT_COUNT * 3 * 4)
		shapes_received.emit(response_sequence, right_points, left_points)
		if not _has_reported_response:
			print("Received STRAS model shapes: %d points per arm." % POINT_COUNT)
			_has_reported_response = true

func _decode_points(packet: PackedByteArray, offset: int) -> PackedVector3Array:
	var points := PackedVector3Array()
	points.resize(POINT_COUNT)
	for index in range(POINT_COUNT):
		var point_offset := offset + index * 3 * 4
		points[index] = Vector3(
			packet.decode_float(point_offset),
			packet.decode_float(point_offset + 4),
			packet.decode_float(point_offset + 8)
		)
	return points
