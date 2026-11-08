"""Construction helpers for the configured STRAS model and camera calibration."""
from __future__ import annotations

from pathlib import Path

from robot_control.stras.model.config import load_model_files
from robot_control.stras.model.kinematics import Kinematics


def build_kinematics() -> Kinematics:
    """Build the STRAS model from repository-relative robot configuration files."""
    from module_main.model_utilities.robot_model import robot_model, robot_twobending
    from module_main.model_utilities.stras_model import StrasModel

    model_dir = Path(__file__).resolve().parents[3] / "module_main" / "model_utilities"
    model = StrasModel(
        robot_model(str(model_dir / "robot_config_right.txt")),
        robot_model(str(model_dir / "robot_config_left.txt")),
        robot_twobending(str(model_dir / "robot_config_endoscope.txt")),
    )
    model.defineEndoscopeShapeOn()
    return Kinematics(model)


def build_camera_calibration(module_main: Path):
    """Load the optical calibration configured for the STRAS endoscope."""
    calibration_path = load_model_files(module_main).camera_calibration
    if calibration_path is None:
        return None

    from module_main.model_utilities.camera_model import camera_model

    return camera_model(str(calibration_path))