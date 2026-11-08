extends Control
## Lightweight 2D teleoperation gauges composited over the simulation viewport.

const TRACK_COLOR := Color(0.48, 0.52, 0.57, 0.88)
const ACTIVE_COLOR := Color(0.16, 0.88, 0.40, 1.0)
const LIMIT_COLOR := Color(1.0, 0.20, 0.18, 1.0)
const TICK_COLOR := Color(0.92, 0.94, 0.97, 0.95)
const LABEL_COLOR := Color(0.96, 0.97, 1.0, 1.0)
const JOINTS := ["BL", "RL", "TL", "BR", "RR", "TR"]

var _values := {
	"BL": 0.5, "RL": 0.5, "TL": 0.5,
	"BR": 0.5, "RR": 0.5, "TR": 0.5,
}


func set_values(data: Dictionary) -> void:
	for label in JOINTS:
		if not data.has(label):
			continue
		var raw = data[label]
		if typeof(raw) != TYPE_INT and typeof(raw) != TYPE_FLOAT:
			continue
		var value := float(raw)
		if is_nan(value) or is_inf(value):
			continue
		_values[label] = clampf(value, 0.0, 1.0)
	queue_redraw()


func _draw() -> void:
	if size.x <= 0.0 or size.y <= 0.0:
		return
	var width := size.x
	var height := size.y
	var font_size := maxf(14.0, width * 0.021)
	var track_width := maxf(3.0, width * 0.008)
	var marker_radius := maxf(5.0, width * 0.009)

	_draw_horizontal_slider("BL", Vector2(width * 0.14, height * 0.105),
		Vector2(width * 0.43, height * 0.105), _values["BL"], false,
		font_size, track_width, marker_radius)
	_draw_horizontal_slider("BR", Vector2(width * 0.86, height * 0.105),
		Vector2(width * 0.57, height * 0.105), _values["BR"], true,
		font_size, track_width, marker_radius)

	_draw_vertical_slider("TL", Vector2(width * 0.065, height * 0.20),
		Vector2(width * 0.065, height * 0.69), _values["TL"], true,
		font_size, track_width, marker_radius)
	_draw_vertical_slider("TR", Vector2(width * 0.935, height * 0.20),
		Vector2(width * 0.935, height * 0.69), _values["TR"], false,
		font_size, track_width, marker_radius)

	_draw_arc_slider("RL", Vector2(width * 0.285, height * 0.985),
		height * 0.105, _values["RL"], true, font_size, track_width, marker_radius)
	_draw_arc_slider("RR", Vector2(width * 0.715, height * 0.985),
		height * 0.105, _values["RR"], false, font_size, track_width, marker_radius)


func _draw_horizontal_slider(label: String, start: Vector2, finish: Vector2,
		value: float, inverted: bool, font_size: float, track_width: float,
		marker_radius: float) -> void:
	var t := 1.0 - value if inverted else value
	var marker := start.lerp(finish, t)
	var neutral := start.lerp(finish, 0.5)
	var color := _marker_color(value)
	draw_line(start, finish, TRACK_COLOR, track_width, true)
	draw_line(start, marker, color, track_width, true)
	draw_line(neutral + Vector2(0.0, -font_size * 0.42),
		neutral + Vector2(0.0, font_size * 0.42), TICK_COLOR, maxf(2.0, track_width * 0.35), true)
	draw_circle(marker, marker_radius, color)
	_draw_label(label, Vector2((start.x + finish.x) * 0.5, start.y - font_size * 0.72),
		font_size)


func _draw_vertical_slider(label: String, top: Vector2, bottom: Vector2,
		value: float, on_left: bool, font_size: float, track_width: float,
		marker_radius: float) -> void:
	var marker := bottom.lerp(top, value)
	var neutral := bottom.lerp(top, 0.5)
	var color := _marker_color(value)
	draw_line(top, bottom, TRACK_COLOR, track_width, true)
	draw_line(bottom, marker, color, track_width, true)
	draw_line(neutral + Vector2(-font_size * 0.42, 0.0),
		neutral + Vector2(font_size * 0.42, 0.0), TICK_COLOR, maxf(2.0, track_width * 0.35), true)
	draw_circle(marker, marker_radius, color)
	var label_x := top.x + font_size * 0.95 if on_left else top.x - font_size * 0.95
	_draw_label(label, Vector2(label_x, top.y - font_size * 0.8), font_size)


func _draw_arc_slider(label: String, center: Vector2, radius: float, value: float,
		inverted: bool, font_size: float, track_width: float,
		marker_radius: float) -> void:
	var start_angle := TAU if inverted else PI
	var end_angle := PI if inverted else TAU
	var marker_angle := lerpf(start_angle, end_angle, value)
	var neutral_angle := lerpf(start_angle, end_angle, 0.5)
	var color := _marker_color(value)
	var track_points := _arc_points(center, radius, start_angle, end_angle, 40)
	draw_polyline(track_points, TRACK_COLOR, track_width, true)
	var filled_segments: int = maxi(1, ceili(40.0 * absf(value)))
	var fill_points := _arc_points(center, radius, start_angle, marker_angle, filled_segments)
	draw_polyline(fill_points, color, track_width, true)
	var tick_direction := Vector2(cos(neutral_angle), sin(neutral_angle))
	draw_line(center + tick_direction * (radius - font_size * 0.30),
		center + tick_direction * (radius + font_size * 0.30), TICK_COLOR,
		maxf(2.0, track_width * 0.35), true)
	var marker_position := center + Vector2(cos(marker_angle), sin(marker_angle)) * radius
	draw_circle(marker_position, marker_radius, color)
	_draw_label(label, Vector2(center.x, center.y - radius - font_size * 0.48), font_size)


func _arc_points(center: Vector2, radius: float, start_angle: float,
		end_angle: float, segments: int) -> PackedVector2Array:
	var points := PackedVector2Array()
	for index in range(segments + 1):
		var angle := lerpf(start_angle, end_angle, float(index) / float(segments))
		points.append(center + Vector2(cos(angle), sin(angle)) * radius)
	return points


func _draw_label(text: String, center: Vector2, font_size: float) -> void:
	var font := ThemeDB.fallback_font
	var text_width := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, int(font_size)).x
	var baseline := center.y + font_size * 0.34
	draw_string(font, Vector2(center.x - text_width * 0.5 + 1.0, baseline + 1.0),
		text, HORIZONTAL_ALIGNMENT_LEFT, -1, int(font_size), Color(0.02, 0.025, 0.04, 0.9))
	draw_string(font, Vector2(center.x - text_width * 0.5, baseline), text,
		HORIZONTAL_ALIGNMENT_LEFT, -1, int(font_size), LABEL_COLOR)


func _marker_color(value: float) -> Color:
	return LIMIT_COLOR if value < 0.02 or value > 0.98 else ACTIVE_COLOR
