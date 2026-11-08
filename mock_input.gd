extends Node3D

const NetworkConstants = preload("res://robot_control/network_constants.gd")
const ACTIVATION_DELAY_SECONDS := 1.0

var _input_socket := PacketPeerUDP.new()
var _sequence := 0
var _elapsed_seconds: float = 0.0


func _ready() -> void:
	get_viewport().use_xr = false
	var socket_error := _input_socket.set_dest_address(
		NetworkConstants.LOOPBACK_HOST, NetworkConstants.INPUT_PORT)
	if socket_error != OK:
		push_error("Could not set mock input UDP destination: %s" % error_string(socket_error))
	else:
		print("Mock input sender: sending to %s:%d" % [
			NetworkConstants.LOOPBACK_HOST, NetworkConstants.INPUT_PORT])


func _process(delta: float) -> void:
	_elapsed_seconds += delta
	var active := _elapsed_seconds >= ACTIVATION_DELAY_SECONDS
	var hand := {
		"connected": true,
		"position": {"x": 0.0, "y": 0.0, "z": 0.0},
		"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
		"trigger": 0.0,
		"grip": 0.0,
		"primary": {"x": 0.5, "y": 0.5},
		"secondary": {"x": 0.5, "y": 0.5},
		"buttons": {},
	}
	var right_hand := hand.duplicate(true)
	right_hand["grip"] = 1.0 if active else 0.0
	right_hand["primary"] = {"x": 0.5, "y": 1.0 if active else 0.5}
	var payload := {
		"v": NetworkConstants.INPUT_PROTOCOL_VERSION,
		"seq": _sequence,
		"left": hand,
		"right": right_hand,
		"camera": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "yaw": 0.0},
		"dt": maxf(0.0, delta),
	}
	_sequence += 1
	var send_error := _input_socket.put_packet(JSON.stringify(payload).to_utf8_buffer())
	if send_error != OK:
		push_warning("Could not send mock input UDP packet: %s" % error_string(send_error))


func _exit_tree() -> void:
	_input_socket.close()