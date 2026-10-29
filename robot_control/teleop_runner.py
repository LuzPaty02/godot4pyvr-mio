"""Receive VR input and route watchdog-gated commands to virtual STRAS.""" 
from __future__ import annotations 
import argparse 
import time 
from dataclasses import dataclass 
from typing import Optional 
from PythonFiles.input_client import PROTOCOL_VERSION 
from robot_control.input_server import InputMessage, InputServer 
from robot_control.latest_slot import Mailbox 
from robot_control.teleop_mapping import TeleopCommand, TeleopMapper 
from robot_control.stras.constants import AXIS_SOCKET_INDEX 
from robot_control.stras.follower import VirtualFollower 
from robot_control.watchdog import Reason, Verdict, Watchdog, WatchdogConfig 

@dataclass(frozen=True) 
class PipelineState: 
    message: Optional[InputMessage] 
    verdict: Verdict 
    command: TeleopCommand 

class InputMonitor: 
    """Apply watchdog gating and mapping to the latest controller frame.""" 
    def __init__( 
        self, 
        mailbox: Mailbox[InputMessage], 
        watchdog: Optional[Watchdog] = None, 
        mapper: Optional[TeleopMapper] = None, 
    ): 
        self._mailbox = mailbox 
        self._watchdog = watchdog or Watchdog() 
        # ControllerInputReader maps stick axes from [-1, 1] to [0, 1]. 
        self._mapper = mapper or TeleopMapper(axes_normalized=True) 
    def tick(self, now: float) -> PipelineState: 
        message = self._mailbox.get() 
        verdict = self._watchdog.check(message, now) 
        payload = message.payload if message is not None else {} 
        command = self._mapper.map(payload, verdict.enabled_hands) 
        return PipelineState(message, verdict, command) 

def _to_stras_axes( 
    command: TeleopCommand, follower: VirtualFollower 
) -> list[tuple[int, float]]: 
    """Scale normalized joint velocities to STRAS axis units.""" 
    limits = follower.limits.joint_velocity 
    max_speed = dict(zip(limits.labels, limits.upper)) 
    return [ 
        (AXIS_SOCKET_INDEX[joint], float(value) * abs(float(max_speed[joint]))) 
        for joint, value in command.velocity.items() 
    ] 

def _format_state(state: PipelineState, follower: VirtualFollower) -> str: 
    velocity = dict(follower.last_velocity) 
    position = dict(follower.last_position) 
    if state.verdict.reason is not Reason.OK: 
        return ( 
            f"SAFE STOP ({state.verdict.reason.value}); " 
            f"STRAS velocity={velocity}; position={position}" 
        ) 
    seq = state.message.seq if state.message is not None else "?" 
    return ( 
        f"seq={seq} enabled={sorted(state.verdict.enabled_hands)} " 
        f"STRAS velocity={velocity}; position={position}" 
    ) 

def main(argv: Optional[list[str]] = None) -> int: 
    parser = argparse.ArgumentParser( 
        description="Receive local Godot VR input and route it to virtual STRAS." 
    ) 
    parser.add_argument("--host", default="127.0.0.1", 
                        help="bind address (default: loopback only)") 
    parser.add_argument("--port", type=int, default=9000) 
    parser.add_argument("--timeout", type=float, default=0.2, 
                        help="input freshness timeout in seconds") 
    args = parser.parse_args(argv) 
    if not 1 <= args.port <= 65535: 
        parser.error("--port must be between 1 and 65535") 
    if args.timeout <= 0: 
        parser.error("--timeout must be greater than zero") 
    mailbox: Mailbox[InputMessage] = Mailbox() 
    monitor = InputMonitor( 
        mailbox, watchdog=Watchdog(WatchdogConfig(timeout_s=args.timeout))) 
    follower = VirtualFollower() 
    follower.start() 
    server = InputServer(mailbox, host=args.host, port=args.port) 
    try: 
        server.start() 
    except OSError as exc: 
        parser.exit(1, f"Could not start input receiver: {exc}\n") 
    print( 
        f"Listening on {server.address}; protocol v{PROTOCOL_VERSION}. " 
        "Virtual STRAS only: no hardware connection or actuation." 
    ) 
    last_signature = None 
    last_print = 0.0 
    last_update = time.monotonic() 
    try: 
        while True: 
            now = time.monotonic() 
            follower.advance(now - last_update) 
            last_update = now 
            state = monitor.tick(now) 
            follower.send_velocity(_to_stras_axes(state.command, follower)) 
            signature = ( 
                state.verdict.reason, 
                state.verdict.enabled_hands, 
                tuple(sorted(state.command.velocity.items())), 
                state.message.seq if state.message else None, 
            ) 
            if signature != last_signature and ( 
                state.verdict.reason is not Reason.OK 
                or now - last_print >= 0.1 
            ): 
                print(_format_state(state, follower), flush=True) 
                last_signature = signature 
                last_print = now 
            time.sleep(0.01) 
    except KeyboardInterrupt: 
        print("\nStopping local input receiver.") 
    finally: 
        server.stop() 
        follower.close() 
    return 0 

if __name__ == "__main__": 
    raise SystemExit(main())