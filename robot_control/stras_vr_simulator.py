"""CLI orchestration for the simulation-only VR-to-STRAS geometry pipeline."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import time

from robot_control.stras.geometry_output.geometry_stream import GodotGeometrySender, build_geometry_packet
from robot_control.stras.input.transport_and_validation.input_protocol import PROTOCOL_VERSION
from robot_control.stras.input.transport_and_validation.input_server import InputMessage, InputServer
from robot_control.stras.input.transport_and_validation.latest_slot import Mailbox
from robot_control.network_constants import (
    DEFAULT_MAX_GEOMETRY_FPS,
    DEFAULT_STATS_INTERVAL_SECONDS,
    GEOMETRY_PORT,
    INPUT_PORT,
    LOOPBACK_HOST,
    RUNNER_IDLE_POLL_INTERVAL_SECONDS,
)
from robot_control.stras.simulation.virtual_follower import VirtualFollower
from robot_control.stras.input.mapping_and_limits.input_mapping import ControllerToConfiguration
from robot_control.stras.input.mapping_and_limits.joint_mapping import normalized_joints_to_q
from robot_control.stras.model.kinematics import Kinematics
from robot_control.stras.input.mapping_and_limits.limits import RobotLimits
from robot_control.stras.model.model_factory import build_camera_calibration, build_kinematics

# Keep the former private helpers available for callers that imported them.
_build_kinematics = build_kinematics
_build_camera_calibration = build_camera_calibration


class InputServerStartupError(RuntimeError):
    """Raised when the input UDP listener cannot bind its configured address."""


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compute STRAS forward kinematics and stream geometry to Godot.")
    parser.add_argument("--input-host", default=LOOPBACK_HOST)
    parser.add_argument("--input-port", type=int, default=INPUT_PORT)
    parser.add_argument("--godot-host", default=LOOPBACK_HOST)
    parser.add_argument("--godot-port", type=int, default=GEOMETRY_PORT)
    parser.add_argument(
        "--max-fps", type=float, default=DEFAULT_MAX_GEOMETRY_FPS,
        help="maximum geometry datagrams per second sent to Godot (default: 30)",
    )
    parser.add_argument(
        "--stats-interval", type=float, default=DEFAULT_STATS_INTERVAL_SECONDS,
        help="print cumulative input/FK counters every N seconds; 0 disables them (default)",
    )
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> float:
    for name in ("input_port", "godot_port"):
        if not 1 <= getattr(args, name) <= 65535:
            parser.error(f"--{name.replace('_', '-')} must be between 1 and 65535")
    if not math.isfinite(args.max_fps) or args.max_fps <= 0.0:
        parser.error("--max-fps must be a finite number greater than zero")
    if not math.isfinite(args.stats_interval) or args.stats_interval < 0.0:
        parser.error("--stats-interval must be a finite number greater than or equal to zero")
    return 1.0 / args.max_fps


def _latest_message(mailbox: Mailbox[InputMessage], after_seq: int) -> InputMessage | None:
    message = mailbox.get()
    if message is None or message.seq <= after_seq:
        return None
    return message


def _process_message(message: InputMessage, limits: RobotLimits,
                     input_mapping: ControllerToConfiguration,
                     follower: VirtualFollower, kinematics: Kinematics,
                     sender: GodotGeometrySender, camera_calibration) -> None:
    if "joints" in message.payload:
        # Preserve support for scripted/test inputs that supply normalized joints.
        q = normalized_joints_to_q(message.payload["joints"], limits)
    else:
        q = input_mapping.update(message.payload)
    follower.set_q(q)
    shape = kinematics.shape(follower.q_config_space)
    sender.send(message.seq, shape, camera_calibration, q=q, limits=limits)


def _run_loop(args: argparse.Namespace, mailbox: Mailbox[InputMessage],
              server: InputServer, sender: GodotGeometrySender,
              limits: RobotLimits, input_mapping: ControllerToConfiguration,
              follower: VirtualFollower, kinematics: Kinematics,
              camera_calibration, geometry_interval: float) -> None:
    print(
        f"Listening for local VR pose snapshots on {server.address}; "
        "updating the simulation-only VirtualFollower; "
        f"streaming FK geometry to {args.godot_host}:{args.godot_port} "
        f"(input protocol v{PROTOCOL_VERSION}; no hardware commands).",
        flush=True,
    )

    last_seq = -1
    processed_frames = 0
    sent_frames = 0
    last_geometry_frame = -1
    next_geometry_send = 0.0
    last_stats_report = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if (args.stats_interval > 0.0
                    and now - last_stats_report >= args.stats_interval):
                stats = server.stats
                print(
                    "Input UDP: "
                    f"received={stats.received} accepted={stats.accepted} "
                    f"malformed={stats.malformed} stale={stats.stale}; "
                    f"FK processed={processed_frames} geometry_sent={sent_frames} "
                    f"last_geometry_frame={last_geometry_frame}",
                    flush=True,
                )
                last_stats_report = now
            message = _latest_message(mailbox, last_seq)
            if message is None:
                time.sleep(RUNNER_IDLE_POLL_INTERVAL_SECONDS)
                continue
            last_seq = message.seq
            now = time.monotonic()
            if now < next_geometry_send:
                time.sleep(next_geometry_send - now)
                continue
            try:
                _process_message(
                    message, limits, input_mapping, follower,
                    kinematics, sender, camera_calibration)
                processed_frames += 1
                sent_frames += 1
                last_geometry_frame = message.seq
                next_geometry_send = time.monotonic() + geometry_interval
            except (TypeError, ValueError, OSError) as exc:
                print(f"Skipping input frame {message.seq}: {exc}", flush=True)
    except KeyboardInterrupt:
        print("\nStopping FK visualization runner.", flush=True)


def run(args: argparse.Namespace, geometry_interval: float) -> None:
    """Build runtime components, then run the input-to-geometry loop."""
    mailbox: Mailbox[InputMessage] = Mailbox()
    server = InputServer(mailbox, host=args.input_host, port=args.input_port)
    sender: GodotGeometrySender | None = None
    try:
        module_main = Path(__file__).resolve().parents[1] / "module_main"
        kinematics = build_kinematics()
        camera_calibration = build_camera_calibration(module_main)
        limits = RobotLimits.default()
        input_mapping = ControllerToConfiguration(limits.virtual_cs)
        follower = VirtualFollower(limits=limits)
        sender = GodotGeometrySender(args.godot_host, args.godot_port)
        try:
            server.start()
        except OSError as exc:
            raise InputServerStartupError(str(exc)) from exc
        _run_loop(
            args, mailbox, server, sender, limits, input_mapping, follower,
            kinematics, camera_calibration, geometry_interval)
    finally:
        server.stop()
        if sender is not None:
            sender.close()


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    geometry_interval = _validate_args(parser, args)
    try:
        run(args, geometry_interval)
    except InputServerStartupError as exc:
        parser.exit(1, f"Could not start input receiver: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())