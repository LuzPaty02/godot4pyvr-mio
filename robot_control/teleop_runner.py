"""Map normalized STRAS joint positions to forward-kinematics geometry for Godot.

Input packets must contain ``joints`` as a mapping from every QP joint label
to a normalized position in [0, 1]. This intentionally does not infer joint
positions from controller poses or sticks: that mapping belongs at the input
source and must be specified separately.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import socket
import time
from typing import Mapping

import numpy as np

from PythonFiles.input_client import PROTOCOL_VERSION
from robot_control.input_server import InputMessage, InputServer
from robot_control.latest_slot import Mailbox
from robot_control.stras.constants import QP_ORDER
from robot_control.stras.kinematics import Kinematics, ShapeOutput
from robot_control.stras.limits import RobotLimits


def normalized_joints_to_q(values: Mapping, limits: RobotLimits) -> np.ndarray:
    """Convert a complete label->normalized-position mapping to STRAS q."""
    if not isinstance(values, Mapping):
        raise ValueError("joints must be an object mapping every joint label to [0, 1]")
    missing = set(QP_ORDER) - set(values)
    extra = set(values) - set(QP_ORDER)
    if missing or extra:
        raise ValueError(f"joints labels mismatch (missing={sorted(missing)}, extra={sorted(extra)})")

    normalized = []
    for joint in QP_ORDER:
        value = values[joint]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"normalized {joint} must be a finite number in [0, 1]")
        value = float(value)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError(f"normalized {joint} must be a finite number in [0, 1]")
        normalized.append(value)

    joint_limits = limits.virtual_cs
    lower = np.asarray(joint_limits.lower, dtype=np.float64)
    upper = np.asarray(joint_limits.upper, dtype=np.float64)
    return lower + np.asarray(normalized, dtype=np.float64) * (upper - lower)


def build_geometry_packet(frame: int, shape: ShapeOutput,
                          camera_calibration=None) -> dict:
    """Build the JSON-compatible frame expected by ``stras_receiver.gd``."""
    right_sections = [np.asarray(section).tolist() for section in shape.right_sections]
    left_sections = [np.asarray(section).tolist() for section in shape.left_sections]
    endoscope_sections = shape.endoscope_sections or ()
    endoscope = (np.concatenate(endoscope_sections, axis=0)
                 if endoscope_sections else np.empty((0, 3)))
    return {
        "v": 1,
        "frame": int(frame),
        "right": np.asarray(shape.right).tolist(),
        "left": np.asarray(shape.left).tolist(),
        "endoscope": endoscope.tolist(),
        "right_sections": right_sections,
        "left_sections": left_sections,
        "endoscope_sections": [np.asarray(section).tolist()
                               for section in endoscope_sections],
        "camera": {
            "pose": np.asarray(shape.tip_e).tolist(),
            "intrinsics": ({
                "fx": float(camera_calibration.f[0]),
                "fy": float(camera_calibration.f[1]),
                "cx": float(camera_calibration.c[0]),
                "cy": float(camera_calibration.c[1]),
                "width": int(camera_calibration.imgSize[0]),
                "height": int(camera_calibration.imgSize[1]),
            } if camera_calibration is not None else None),
        },
    }


class GodotGeometrySender:
    """Send latest-only STRAS geometry frames to the Godot UDP receiver."""

    def __init__(self, host: str = "127.0.0.1", port: int = 9100):
        self._address = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, frame: int, shape: ShapeOutput,
             camera_calibration=None) -> None:
        packet = build_geometry_packet(frame, shape, camera_calibration)
        self._socket.sendto(json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                            self._address)

    def close(self) -> None:
        self._socket.close()


def _build_kinematics() -> Kinematics:
    """Build the STRAS model from repository-relative robot configuration files."""
    from module_main.model_utilities.robot_model import robot_model, robot_twobending
    from module_main.model_utilities.stras_model import StrasModel

    model_dir = Path(__file__).resolve().parents[1] / "module_main" / "model_utilities"
    model = StrasModel(
        robot_model(str(model_dir / "robot_config_right.txt")),
        robot_model(str(model_dir / "robot_config_left.txt")),
        robot_twobending(str(model_dir / "robot_config_endoscope.txt")),
    )
    model.defineEndoscopeShapeOn()
    return Kinematics(model)


def _build_camera_calibration(module_main: Path):
    from module_main.model_utilities.camera_model import camera_model
    from robot_control.stras.config import load_model_files

    calibration_path = load_model_files(module_main).camera_calibration
    return camera_model(str(calibration_path)) if calibration_path else None


def _latest_message(mailbox: Mailbox[InputMessage], after_seq: int) -> InputMessage | None:
    message = mailbox.get()
    if message is None or message.seq <= after_seq:
        return None
    return message


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Compute STRAS forward kinematics and stream geometry to Godot.")
    parser.add_argument("--input-host", default="127.0.0.1")
    parser.add_argument("--input-port", type=int, default=9000)
    parser.add_argument("--godot-host", default="127.0.0.1")
    parser.add_argument("--godot-port", type=int, default=9100)
    args = parser.parse_args(argv)
    for name in ("input_port", "godot_port"):
        if not 1 <= getattr(args, name) <= 65535:
            parser.error(f"--{name.replace('_', '-')} must be between 1 and 65535")

    mailbox: Mailbox[InputMessage] = Mailbox()
    server = InputServer(mailbox, host=args.input_host, port=args.input_port)
    try:
        server.start()
    except OSError as exc:
        parser.exit(1, f"Could not start input receiver: {exc}\n")

    module_main = Path(__file__).resolve().parents[1] / "module_main"
    kinematics = _build_kinematics()
    camera_calibration = _build_camera_calibration(module_main)
    limits = RobotLimits.default(module_main)
    sender = GodotGeometrySender(args.godot_host, args.godot_port)
    print(
        f"Listening for normalized joint positions on {server.address}; "
        f"streaming FK geometry to {args.godot_host}:{args.godot_port} "
        f"(input protocol v{PROTOCOL_VERSION}).",
        flush=True,
    )

    last_seq = -1
    try:
        while True:
            message = _latest_message(mailbox, last_seq)
            if message is None:
                time.sleep(0.005)
                continue
            last_seq = message.seq
            try:
                q = normalized_joints_to_q(message.payload.get("joints"), limits)
                shape = kinematics.shape(q)
                sender.send(message.seq, shape, camera_calibration)
            except (TypeError, ValueError, OSError) as exc:
                print(f"Skipping input frame {message.seq}: {exc}", flush=True)
    except KeyboardInterrupt:
        print("\nStopping FK visualization runner.", flush=True)
    finally:
        server.stop()
        sender.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())