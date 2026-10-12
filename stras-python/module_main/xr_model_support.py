"""Shared local-path resolution for the XR STRAS adapters."""

import os


_MODULE_ROOT = os.path.dirname(os.path.abspath(__file__))
_MODEL_ROOT = os.path.join(_MODULE_ROOT, "model_utilities")


def resolve_model_files(config_file=None):
    """Resolve calibration and robot config paths without machine-specific paths."""
    if config_file is None:
        config_file = os.path.join("model_utilities", "datafolders.txt")

    if os.path.isabs(config_file):
        config_path = config_file
    else:
        candidates = (
            os.path.join(os.getcwd(), config_file),
            os.path.join(_MODULE_ROOT, config_file),
        )
        config_path = next((path for path in candidates if os.path.isfile(path)), candidates[-1])

    settings = {}
    with open(config_path, encoding="utf-8") as config:
        for line in config:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            settings[key.strip()] = value.strip()

    def resolve(value, bases):
        if os.path.isabs(value):
            return value
        candidates = [os.path.join(base, value) for base in bases]
        return next((path for path in candidates if os.path.isfile(path)), candidates[0])

    return {
        "camera_calibration": resolve(settings["camera_calibration"], (_MODULE_ROOT,)),
        "robot_config_right": resolve(settings["robot_config_right"], (_MODEL_ROOT, _MODULE_ROOT)),
        "robot_config_left": resolve(settings["robot_config_left"], (_MODEL_ROOT, _MODULE_ROOT)),
        "robot_config_endoscope": resolve(settings["robot_config_endoscope"], (_MODEL_ROOT, _MODULE_ROOT)),
    }
