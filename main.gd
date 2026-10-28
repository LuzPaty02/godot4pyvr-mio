extends Node3D

func _ready() -> void:
	# 1. Initialize Viewport and Alpha Pass-through Clear
	get_viewport().use_xr = true
	get_viewport().transparent_bg = true
	RenderingServer.set_default_clear_color(Color(0, 0, 0, 0))
	print("Native: Composition layer defaults set to alpha zero clear.")

	# 2. Safely fetch and cast the OpenXR Interface using internal Godot Enums
	var xr_interface = XRServer.find_interface("OpenXR")
	if xr_interface:
		# Use the specific built-in constant mapping (which evaluates to 2)
		if xr_interface.set_environment_blend_mode(XRInterface.XR_ENV_BLEND_MODE_ALPHA_BLEND):
			print("Native: OpenXR Alpha Blend Passthrough enabled.")
		else:
			print("Native: Passthrough blend mode rejected by runtime.")
			
	# 3. Hand off references safely to the child Python node
	var python_manager = $XRSystems
	var left_hand = $XRRig/XROrigin3D/LeftHand
	var right_hand = $XRRig/XROrigin3D/RightHand
	
	if python_manager and python_manager.has_method("initialize_spheres"):
		python_manager.initialize_spheres(left_hand, right_hand)
		print("Native: Target hand pointers successfully transferred to Python layer.")

	if python_manager and python_manager.has_method("read_controller_inputs"):
		var controller_inputs = python_manager.read_controller_inputs(left_hand, right_hand)
		print("Native: Controller inputs normalized to [0, 1]: ", controller_inputs)
