extends Control

var left_position := Vector3.ZERO
var right_position := Vector3.ZERO
var left_rotation := Vector3.ZERO
var right_rotation := Vector3.ZERO

const TRACK_COLOR := Color(0.18, 0.28, 0.38, 1.0)
const GUIDE_COLOR := Color(0.35, 0.48, 0.58, 1.0)
const LEFT_COLOR := Color(0.08, 0.65, 0.95, 1.0)
const RIGHT_COLOR := Color(0.95, 0.35, 0.25, 1.0)
const TEXT_COLOR := Color(0.72, 0.84, 0.92, 1.0)
const KNOB_RADIUS := 8.0

@export_category("Telemetry Ranges")
@export_range(0.01, 10.0, 0.01, "suffix:m") var horizontal_range_m := 1.25
@export_range(0.01, 10.0, 0.01, "suffix:m") var depth_range_m := 2.5
@export_range(0.01, 6.283185, 0.01, "suffix:rad") var rotation_range_rad := PI

func set_controller_state(side: StringName, controller_position: Vector3, controller_rotation: Vector3) -> void:
	if side == &"left":
		left_position = controller_position
		left_rotation = controller_rotation
	else:
		right_position = controller_position
		right_rotation = controller_rotation
	queue_redraw()

func _draw() -> void:
	var half_width := size.x * 0.5
	_draw_controller(&"LEFT", left_position, left_rotation, LEFT_COLOR, 24.0, half_width - 36.0)
	_draw_controller(&"RIGHT", right_position, right_rotation, RIGHT_COLOR, half_width + 36.0, size.x - 24.0)

func _draw_controller(
	label: String,
	controller_position: Vector3,
	controller_rotation: Vector3,
	color: Color,
	left_edge: float,
	right_edge: float
) -> void:
	var center_x := (left_edge + right_edge) * 0.5
	var horizontal_left := left_edge + 22.0
	var horizontal_right := right_edge - 22.0
	var top_y := 38.0
	var vertical_x := left_edge + 8.0 if label == "LEFT" else right_edge - 8.0
	var vertical_top := 112.0
	var vertical_bottom := size.y - 106.0
	var arc_center := Vector2(center_x, size.y - 38.0)
	var arc_radius := minf((right_edge - left_edge) * 0.34, 48.0)
	var horizontal_value := _normalize_range(controller_position.x, horizontal_range_m)
	var depth_value := _normalize_range(controller_position.z, depth_range_m)
	var rotation_value := _normalize_range(controller_rotation.y, rotation_range_rad)

	draw_string(ThemeDB.fallback_font, Vector2(horizontal_left, 20.0), "%s | 3-DOF" % label, HORIZONTAL_ALIGNMENT_LEFT, -1.0, 15, color)
	_draw_horizontal_slider(
		Vector2(horizontal_left, top_y),
		Vector2(horizontal_right, top_y),
		horizontal_value,
		color,
		"X %.2f m" % controller_position.x
	)
	_draw_vertical_slider(
		Vector2(vertical_x, vertical_top),
		Vector2(vertical_x, vertical_bottom),
		depth_value,
		color,
		"Z %.2f m" % controller_position.z,
		label == "RIGHT"
	)
	_draw_rotation_slider(
		arc_center,
		arc_radius,
		rotation_value,
		color,
		"YAW %.0f deg" % rad_to_deg(controller_rotation.y)
	)

func _normalize_range(value: float, limit: float) -> float:
	return clampf((value + limit) / (limit * 2.0), 0.0, 1.0)

func _draw_horizontal_slider(start: Vector2, end: Vector2, value: float, color: Color, label: String) -> void:
	draw_line(start, end, TRACK_COLOR, 7.0, true)
	draw_line(start, end, GUIDE_COLOR, 1.0, true)
	var knob := start.lerp(end, value)
	draw_circle(knob, KNOB_RADIUS + 3.0, Color(0.02, 0.06, 0.1, 0.9))
	draw_circle(knob, KNOB_RADIUS, color)
	draw_string(ThemeDB.fallback_font, Vector2(start.x, start.y + 25.0), label, HORIZONTAL_ALIGNMENT_LEFT, end.x - start.x, 11, TEXT_COLOR)

func _draw_vertical_slider(
	start: Vector2,
	end: Vector2,
	value: float,
	color: Color,
	label: String,
	place_label_left: bool
) -> void:
	draw_line(start, end, TRACK_COLOR, 7.0, true)
	draw_line(start, end, GUIDE_COLOR, 1.0, true)
	var knob := start.lerp(end, value)
	draw_circle(knob, KNOB_RADIUS + 3.0, Color(0.02, 0.06, 0.1, 0.9))
	draw_circle(knob, KNOB_RADIUS, color)
	var label_position := Vector2(start.x + 12.0, (start.y + end.y) * 0.5)
	var label_width := 100.0
	if place_label_left:
		label_position.x = start.x - label_width - 12.0
	draw_string(ThemeDB.fallback_font, label_position, label, HORIZONTAL_ALIGNMENT_LEFT, label_width, 11, TEXT_COLOR)

func _draw_rotation_slider(center: Vector2, radius: float, value: float, color: Color, label: String) -> void:
	var start_angle := PI
	var end_angle := TAU
	draw_arc(center, radius, start_angle, end_angle, 32, TRACK_COLOR, 8.0, true)
	draw_arc(center, radius, start_angle, lerpf(start_angle, end_angle, value), 24, color, 4.0, true)
	var knob_angle := lerpf(start_angle, end_angle, value)
	var knob := center + Vector2(cos(knob_angle), sin(knob_angle)) * radius
	draw_circle(knob, KNOB_RADIUS + 3.0, Color(0.02, 0.06, 0.1, 0.9))
	draw_circle(knob, KNOB_RADIUS, color)
	draw_string(ThemeDB.fallback_font, Vector2(center.x - 32.0, center.y + 17.0), label, HORIZONTAL_ALIGNMENT_LEFT, 64.0, 11, TEXT_COLOR)
