extends RefCounted
## Converts model-space centerlines into capped, circular tube meshes.


static func to_points(raw: Array, robot_to_godot: Basis) -> PackedVector3Array:
	var out := PackedVector3Array()
	for point in raw:
		out.append(robot_to_godot * Vector3(point[0], point[1], point[2]))
	return out


static func build_tube(points: PackedVector3Array, radius: float,
			radial_segments: int, cap_start: bool = false,
			cap_end: bool = false) -> ArrayMesh:
	var point_count := points.size()
	if point_count < 2 or radius <= 0.0 or radial_segments < 3:
		return null
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)

	# Build a circular cross-section at each point along the tube centerline.
	var rings: Array[PackedVector3Array] = []
	for i in point_count:
		var tangent: Vector3
		if i == 0:
			tangent = points[1] - points[0]
		elif i == point_count - 1:
			tangent = points[i] - points[i - 1]
		else:
			tangent = points[i + 1] - points[i - 1]
		if tangent.length() < 1e-9:
			tangent = Vector3.FORWARD
		tangent = tangent.normalized()

		var reference_axis := Vector3.UP
		if abs(tangent.dot(reference_axis)) > 0.95:
			reference_axis = Vector3.RIGHT
		var axis_a := tangent.cross(reference_axis).normalized()
		var axis_b := tangent.cross(axis_a).normalized()

		var ring := PackedVector3Array()
		for j in radial_segments:
			var angle := TAU * float(j) / float(radial_segments)
			ring.append(points[i] + radius * (axis_a * cos(angle) + axis_b * sin(angle)))
		rings.append(ring)

	for i in point_count - 1:
		for j in radial_segments:
			var next_j := (j + 1) % radial_segments
			var a: Vector3 = rings[i][j]
			var b: Vector3 = rings[i][next_j]
			var c: Vector3 = rings[i + 1][j]
			var d: Vector3 = rings[i + 1][next_j]
			surface.add_vertex(a)
			surface.add_vertex(b)
			surface.add_vertex(d)
			surface.add_vertex(a)
			surface.add_vertex(d)
			surface.add_vertex(c)
	if cap_start:
		for j in radial_segments:
			var next_j := (j + 1) % radial_segments
			surface.add_vertex(points[0])
			surface.add_vertex(rings[0][next_j])
			surface.add_vertex(rings[0][j])
	if cap_end:
		for j in radial_segments:
			var next_j := (j + 1) % radial_segments
			surface.add_vertex(points[point_count - 1])
			surface.add_vertex(rings[point_count - 1][j])
			surface.add_vertex(rings[point_count - 1][next_j])
	surface.index()
	surface.generate_normals()
	return surface.commit()