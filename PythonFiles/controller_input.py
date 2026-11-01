import math


class ControllerInputReader:
    """Read controller controls plus raw Godot global pose snapshots.

    ``position`` is supplied from Godot's ``global_position`` in metres;
    ``orientation`` is its global rotation quaternion in xyzw order. These pose
    fields are retained as XR input data; the FK runner consumes normalized
    joint positions and does not convert controller poses into robot targets.
    """

    @staticmethod
    def normalize(value, minimum=0.0, maximum=1.0):
        if value is None or isinstance(value, bool):
            return 0.0
        if maximum <= minimum:
            return 0.0
        value = float(value)
        if not math.isfinite(value):
            return 0.0
        clamped = max(minimum, min(maximum, value))
        return (clamped - minimum) / (maximum - minimum)

    @staticmethod
    def normalize_axis(value):
        """Clamp a signed stick axis while preserving its [-1, 1] range."""
        if value is None or isinstance(value, bool):
            return 0.0
        value = float(value)
        if not math.isfinite(value):
            return 0.0
        return max(-1.0, min(1.0, value))

    @staticmethod
    def _pose_vector(values, fields, label):
        if not isinstance(values, dict):
            raise ValueError(f"connected controller is missing {label}")
        result = []
        for field in fields:
            value = values.get(field)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))):
                raise ValueError(f"controller {label}.{field} must be a finite number")
            result.append(float(value))
        return result

    @classmethod
    def read_controller(cls, controller_node, connected=True, position=None,
                        orientation=None):
        if controller_node is None or not connected:
            return {
                "connected": False,
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "trigger": 0.0,
                "grip": 0.0,
                "primary": {"x": 0.0, "y": 0.0},
                "secondary": {"x": 0.0, "y": 0.0},
                "buttons": {
                    "trigger_click": False,
                    "trigger_touch": False,
                    "primary_click": False,
                    "primary_touch": False,
                    "secondary_click": False,
                    "secondary_touch": False,
                },
            }
        position_values = cls._pose_vector(position, ("x", "y", "z"), "position")
        orientation_values = cls._pose_vector(
            orientation, ("x", "y", "z", "w"), "orientation")
        quaternion_norm = math.sqrt(sum(value * value for value in orientation_values))
        if quaternion_norm < 1e-12:
            raise ValueError("controller orientation quaternion must be non-zero")
        orientation_values = [value / quaternion_norm for value in orientation_values]
        state = {
            "connected": True,
            "position": dict(zip(("x", "y", "z"), position_values)),
            "orientation": dict(zip(("x", "y", "z", "w"), orientation_values)),
            "trigger": 0.0,
            "grip": 0.0,
            "primary": {"x": 0.0, "y": 0.0},
            "secondary": {"x": 0.0, "y": 0.0},
            "buttons": {
                "trigger_click": False,
                "trigger_touch": False,
                "primary_click": False,
                "primary_touch": False,
                "secondary_click": False,
                "secondary_touch": False,
            },
        }
        state["trigger"] = cls.normalize(controller_node.get_float("trigger"), 0.0, 1.0)
        state["grip"] = cls.normalize(controller_node.get_float("grip"), 0.0, 1.0)
        primary_vec = controller_node.get_vector2("primary")
        state["primary"] = {
            "x": cls.normalize_axis(primary_vec.x),
            "y": cls.normalize_axis(primary_vec.y),
        }
        secondary_vec = controller_node.get_vector2("secondary")
        state["secondary"] = {
            "x": cls.normalize_axis(secondary_vec.x),
            "y": cls.normalize_axis(secondary_vec.y),
        }
        for button_name in (
            "trigger_click", "trigger_touch", "primary_click", "primary_touch",
            "secondary_click", "secondary_touch",
        ):
            state["buttons"][button_name] = bool(
                controller_node.is_button_pressed(button_name))
        return state