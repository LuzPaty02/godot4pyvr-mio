import json
import math
import os


class ControllerInputReader:
    @staticmethod
    def normalize(value, minimum=0.0, maximum=1.0):
        if value is None:
            return 0.0
        if maximum <= minimum:
            return 0.0
        clamped = max(minimum, min(maximum, float(value)))
        return (clamped - minimum) / (maximum - minimum)

    @staticmethod
    def normalize_axis(value):
        return ControllerInputReader.normalize(value, -1.0, 1.0)

    @classmethod
    def read_controller(cls, controller_node, connected=True, position=None):
        if controller_node is None or not connected:
            return {
                "connected": False,
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
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
        state = {
            "connected": True,
            "position": {
                axis: float(position.get(axis, 0.0)) if position else 0.0
                for axis in ("x", "y", "z")
            },
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
        try:
            state["trigger"] = cls.normalize(controller_node.get_float("trigger"), 0.0, 1.0)
        except Exception:
            pass
        try:
            state["grip"] = cls.normalize(controller_node.get_float("grip"), 0.0, 1.0)
        except Exception:
            pass
        try:
            primary_vec = controller_node.get_vector2("primary")
            state["primary"] = {
                "x": cls.normalize_axis(primary_vec.x),
                "y": cls.normalize_axis(primary_vec.y),
            }
        except Exception:
            pass
        try:
            secondary_vec = controller_node.get_vector2("secondary")
            state["secondary"] = {
                "x": cls.normalize_axis(secondary_vec.x),
                "y": cls.normalize_axis(secondary_vec.y),
            }
        except Exception:
            pass
        for button_name in [
            "trigger_click",
            "trigger_touch",
            "primary_click",
            "primary_touch",
            "secondary_click",
            "secondary_touch",
        ]:
            try:
                state["buttons"][button_name] = bool(controller_node.is_button_pressed(button_name))
            except Exception:
                pass
        return state


class ControllerPoseNormalizer:
    DOF_AXES = {
        ("right", "y"): "ET",
        ("right", "x"): "ER",
        ("left", "y"): "EBV",
        ("left", "x"): "EBH",
    }

    def __init__(self, calibration_path):
        self._calibration_path = calibration_path
        self._ranges = self._load_ranges() if os.path.exists(calibration_path) else None
        self._observed = {
            f"{hand}.{axis}": [math.inf, -math.inf]
            for hand, axis in self.DOF_AXES
        }

    @property
    def ready(self):
        return self._ranges is not None

    def observe(self, inputs):
        if self.ready:
            return
        for (hand, axis), _dof in self.DOF_AXES.items():
            state = inputs.get(hand)
            if not isinstance(state, dict) or state.get("connected") is not True:
                continue
            position = state.get("position")
            value = position.get(axis) if isinstance(position, dict) else None
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                continue
            value = float(value)
            if not math.isfinite(value):
                continue
            bounds = self._observed[f"{hand}.{axis}"]
            bounds[0] = min(bounds[0], value)
            bounds[1] = max(bounds[1], value)

    def normalize(self, inputs):
        if not self.ready:
            return {}
        targets = {}
        for (hand, axis), dof in self.DOF_AXES.items():
            state = inputs.get(hand)
            if not isinstance(state, dict) or state.get("connected") is not True:
                continue
            position = state.get("position")
            value = position.get(axis) if isinstance(position, dict) else None
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))):
                continue
            minimum, maximum = self._ranges[f"{hand}.{axis}"]
            targets[dof] = max(
                0.0, min(1.0, (float(value) - minimum) / (maximum - minimum)))
        return targets

    def save(self):
        if self.ready:
            return False
        invalid = [
            key for key, (minimum, maximum) in self._observed.items()
            if not math.isfinite(minimum) or not math.isfinite(maximum)
            or maximum <= minimum
        ]
        if invalid:
            raise ValueError(
                "controller pose calibration needs movement on: "
                + ", ".join(invalid)
            )
        data = {
            "version": 1,
            "ranges": {
                key: {"min": bounds[0], "max": bounds[1]}
                for key, bounds in self._observed.items()
            },
        }
        temporary_path = self._calibration_path + ".tmp"
        with open(temporary_path, "w", encoding="utf-8") as calibration_file:
            json.dump(data, calibration_file, indent=2)
            calibration_file.write("\n")
        os.replace(temporary_path, self._calibration_path)
        self._ranges = {
            key: (bounds[0], bounds[1])
            for key, bounds in self._observed.items()
        }
        return True

    def _load_ranges(self):
        try:
            with open(self._calibration_path, encoding="utf-8") as calibration_file:
                data = json.load(calibration_file)
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(
                f"could not load controller pose calibration "
                f"{self._calibration_path}: {exc}"
            ) from exc
        if not isinstance(data, dict) or data.get("version") != 1:
            raise ValueError(
                f"unsupported controller pose calibration: {self._calibration_path}"
            )
        ranges = data.get("ranges")
        if not isinstance(ranges, dict):
            raise ValueError(
                f"missing ranges in controller pose calibration: "
                f"{self._calibration_path}"
            )
        loaded = {}
        for hand, axis in self.DOF_AXES:
            key = f"{hand}.{axis}"
            bounds = ranges.get(key)
            if not isinstance(bounds, dict):
                raise ValueError(f"missing {key} range in controller pose calibration")
            minimum, maximum = bounds.get("min"), bounds.get("max")
            if (isinstance(minimum, bool) or not isinstance(minimum, (int, float))
                    or isinstance(maximum, bool) or not isinstance(maximum, (int, float))
                    or not math.isfinite(float(minimum))
                    or not math.isfinite(float(maximum))
                    or maximum <= minimum):
                raise ValueError(f"invalid {key} range in controller pose calibration")
            loaded[key] = (float(minimum), float(maximum))
        return loaded