from py4godot.classes.MeshInstance3D import MeshInstance3D
from py4godot.classes.SphereMesh import SphereMesh
from py4godot.classes.StandardMaterial3D import StandardMaterial3D


class HandVisualFactory:
    @staticmethod
    def create_sphere(parent_node, color_value, radius=0.05, height=0.1):
        if parent_node is None:
            return None

        mesh_instance = MeshInstance3D.new()
        sphere_mesh = SphereMesh.new()
        sphere_mesh.set_radius(radius)
        sphere_mesh.set_height(height)

        material = StandardMaterial3D.new()
        material.set_albedo(color_value)
        material.set_shading_mode(0)

        sphere_mesh.surface_set_material(0, material)
        mesh_instance.set_mesh(sphere_mesh)
        parent_node.add_child(mesh_instance)
        return mesh_instance
