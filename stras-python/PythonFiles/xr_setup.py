from py4godot.enums.enums import *
from py4godot.core import *
from py4godot.classes.Node3D import Node3D
from py4godot.classes.MeshInstance3D import MeshInstance3D
from py4godot.classes.SphereMesh import SphereMesh
from py4godot.classes.StandardMaterial3D import StandardMaterial3D

class XRSetup(Node3D):
	
	def _ready(self):
		# Initialize an internal setup lock flag
		self.initialized = False

	def _process(self, delta):
		# Frame 1: Execute tracking setup once the scene nodes are fully online
		if not self.initialized:
			print("Python: Checking scene hierarchy for tracking targets...")
			try:
				# Use plain string paths to fetch the nodes directly without cross-language passing
				left_hand = self.get_node("/root/main/XROrigin3D/lefthand")
				right_hand = self.get_node("/root/main/XROrigin3D/righthand")
				
				if left_hand and right_hand:
					print("Python: Located nodes natively. Attaching sphere indicators...")
					self.attach_colored_sphere(left_hand, Color(0, 1, 0, 1))   # Green
					self.attach_colored_sphere(right_hand, Color(0, 0, 1, 1))  # Blue
					self.initialized = True # Lock setup so it only runs once
				else:
					print("Python: Nodes not found yet, retrying next frame...")
			except Exception as e:
				print(f"Python: Error fetching nodes: {e}")
				self.initialized = True # Prevent crash loops on failure

	def attach_colored_sphere(self, parent_node, color_value):
		mesh_instance = MeshInstance3D.new()
		sphere_mesh = SphereMesh.new()
		sphere_mesh.set_radius(0.05)
		sphere_mesh.set_height(0.1)
		
		material = StandardMaterial3D.new()
		material.set_albedo(color_value)
		material.set_shading_mode(0) # Unlit
		
		sphere_mesh.surface_set_material(0, material)
		mesh_instance.set_mesh(sphere_mesh)
		
		parent_node.add_child(mesh_instance)
