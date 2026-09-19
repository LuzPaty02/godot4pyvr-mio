extends Node3D

func _ready() -> void:
	# 1. Initialize Viewport and Alpha Pass-through Clear
	get_viewport().use_xr = true
	get_viewport().transparent_bg = true
	RenderingServer.set_default_clear_color(Color(0, 0, 0, 0))
	print("Native: Composition layer defaults set to alpha zero clear.")

	# 2. Configure the OpenXR Interface safely
	var xr_interface = XRServer.find_interface("OpenXR")
	if xr_interface:
		if xr_interface.set_environment_blend_mode(XRInterface.XR_ENV_BLEND_MODE_ALPHA_BLEND):
			print("Native: OpenXR Alpha Blend Passthrough enabled.")
		else:
			print("Native: Passthrough blend mode rejected by runtime.")
			
	# 3. Ensure your new spheres are visible and tracking active spatial slots
	var left_sphere = $XROrigin3D/lefthand/LeftSphere
	var right_sphere = $XROrigin3D/righthand/RightSphere
	
	if left_sphere and right_sphere:
		left_sphere.visible = true
		right_sphere.visible = true
		print("Native: Tracking indicators ready.")
