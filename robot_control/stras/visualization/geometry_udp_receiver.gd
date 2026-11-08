extends Node
## Owns the geometry UDP socket and emits only the newest parsed frame each tick.

const NetworkConstants = preload("res://robot_control/network_constants.gd")

signal frame_received(message: Dictionary)

@export var port: int = NetworkConstants.GEOMETRY_PORT

var _udp := PacketPeerUDP.new()
var _is_bound := false


func _ready() -> void:
	var err := _udp.bind(port)
	if err != OK:
		push_error("Could not bind UDP port %d (error %d)" % [port, err])
		return
	_is_bound = true
	print("STRAS receiver listening on UDP port %d" % port)


func _process(_delta: float) -> void:
	if not _is_bound:
		return
	# Drain stale datagrams; consumers only need the latest pose.
	var latest: Dictionary
	var has_latest := false
	while _udp.get_available_packet_count() > 0:
		var parsed = JSON.parse_string(_udp.get_packet().get_string_from_utf8())
		if parsed is Dictionary:
			latest = parsed
			has_latest = true
	if has_latest:
		frame_received.emit(latest)


func _exit_tree() -> void:
	if _is_bound:
		_udp.close()
		_is_bound = false