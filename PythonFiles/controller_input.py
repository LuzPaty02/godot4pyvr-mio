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
    def read_controller(cls, controller_node):
        if controller_node is None:
            return {
                "connected": False,
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
