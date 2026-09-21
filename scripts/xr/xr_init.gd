extends Node3D

func _enter_tree():
	# Force the engine's primary viewport to flag XR active at the earliest microsecond
	get_viewport().use_xr = true
	print("Native: Main viewport successfully marked for XR.")
