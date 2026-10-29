"""Follower classes. 
``Follower``          shared behaviour: limits, models, optional visualiser 
``RealFollower``      talks to the robot through a ``StrasTransport`` 
``VirtualFollower``   no hardware; commands update simulation-visible state 
``make_follower``     picks the class from 'real' / 'virtual' / 'simulated' 
This replaces the ``if typeCondition == 'real' or typeCondition == 'REAL'`` 
check that opened almost every method of the original class. 
""" 
from __future__ import annotations 
import logging 
from abc import ABC, abstractmethod 
from typing import Iterable, List, Optional, Sequence, Tuple, Union 
import numpy as np 
from .config import PathLike 
from .constants import (AXIS_POSITION, AXIS_SOCKET_INDEX, CHANNEL_CLEARANCE_NORM, 
                        CHANNEL_GUARD, COMMAND_PORT, QP_ORDER, 
                        RANGE_TRACKING_PORT, Command) 
from .errors import StrasError, TransportError, TransportTimeout 
from .experiment import ExperimentState 
from .kinematics import Kinematics, ModelOutput 
from .limits import JointLimits, LimitCheck, RobotLimits 
from .protocol import build_cmd, build_goto, build_vel_idx, parse_header 
from .state import (AxisInfos, AxisRanges, RobotState, parse_axis_infos, 
                    parse_state) 
from .transport import StrasTransport 
from .visualizer_base import VisualizerLike 
logger = logging.getLogger(__name__) 
GoToTarget = Tuple[int, float, float]             # (axis index, position, max rel. vel) 
LabelTarget = Tuple[str, float, float]             # (axis label, position, max rel. vel) 

class Follower(ABC): 
    """Behaviour shared by the real and the virtual follower.""" 
    def __init__(self, limits: Optional[RobotLimits] = None, 
                     module_main: Optional[PathLike] = None, 
                     kinematics: Optional[Kinematics] = None, 
                     visualizer: Optional[VisualizerLike] = None): 
        self.limits = limits or RobotLimits.default(module_main) 
        self.experiment = ExperimentState() 
        self.kinematics = kinematics 
        # No default renderer: pass e.g. GodotVisualizer(kinematics). 
        self.visualizer: Optional[VisualizerLike] = visualizer 
        self.model_output: Optional[ModelOutput] = None 
        # latest state, normalised axis values (slave order) and the same state 
        # in configuration space order and range (was q2teleLeader_configSpace) 
        self.state: Optional[RobotState] = None 
        self.axis_normalized = np.array([], dtype=np.float64) 
        self.q_config_space = np.zeros(len(QP_ORDER)) 
        # error reporting (read by experiment loops) 
        self.timeout_error = False 
        self.send_error = False 
        self.last_error: Optional[str] = None 
    # --- hardware interface (implemented by subclasses) -------------------------- 
    @property 
    @abstractmethod 
    def range_limits(self) -> JointLimits: 
        """Limits used by ``check_q_in_range``.""" 
    @abstractmethod 
    def start(self) -> None: ... 
    @abstractmethod 
    def close(self) -> None: ... 
    @abstractmethod 
    def get_state(self) -> Optional[RobotState]: ... 
    @abstractmethod 
    def get_axis_info(self) -> Optional[AxisInfos]: ... 
    @abstractmethod 
    def send_velocity(self, axes_vals: Sequence[Tuple[int, float]], 
                        verbose: bool = False) -> bool: ... 
    @abstractmethod 
    def send_goto(self, targets: Sequence[GoToTarget], verbose: bool = False, 
                  per_axis: bool = False) -> bool: ... 
    # --- limits ------------------------------------------------------------------- 
    def check_q_in_range(self, q) -> LimitCheck: 
        return self.range_limits.check(q) 
    def activate_virtual_bounds(self) -> None: 
        self.experiment.flag_virtual_limits = True 
    def deactivate_virtual_bounds(self) -> None: 
        self.experiment.flag_virtual_limits = False 
    def redefine_bounds(self, q_min, q_max) -> None: 
        from dataclasses import replace 
        self.limits = replace(self.limits, q_bound=JointLimits( 
            self.limits.q_bound.labels, np.asarray(q_min), np.asarray(q_max))) 
    def filter_configuration_state(self, q) -> np.ndarray: 
        """Clip q to the virtual limits (rotations excluded). Returns a copy.""" 
        return self.limits.virtual_cs.clip(q) 
    # --- configuration space ----------------------------------------------------- 
    def _normalized_to_config_space(self, normalized) -> np.ndarray: 
        from module_main.useful import map_norm2range, order_follower2configspace 
        lim = self.limits.joint_velocity 
        return map_norm2range(order_follower2configspace(normalized), 
                              lim.lower, lim.upper) 
    def get_state_in_config_space(self) -> np.ndarray: 
        """Refresh the state and return q in configuration-space order/range.""" 
        self.get_state() 
        if self.axis_normalized.size: 
            self.q_config_space = self._normalized_to_config_space( 
                self.axis_normalized) 
        return self.q_config_space 
    # --- models & visualisation -------------------------------------------------- 
    def compute_models(self, q) -> ModelOutput: 
        if self.kinematics is None: 
            raise StrasError( 
                "no kinematics model configured; pass kinematics=Kinematics(model)") 
        self.model_output = self.kinematics.compute(q) 
        return self.model_output 
    def _require_visualizer(self) -> VisualizerLike: 
        if self.visualizer is None: 
            raise StrasError( 
                "no visualizer configured; pass visualizer=... to the follower " 
                "(for example GodotVisualizer)") 
        return self.visualizer 
    def visualize_frame(self, q, render: bool = False): 
        """Update the models for ``q`` and optionally send / render the frame. 
        With ``render=True`` the configured visualizer is called and its result 
        is returned (an image for image-producing renderers, None for 
        ``GodotVisualizer``).  Returns None without rendering when q is outside 
        the limits (``experiment.limit_flag_virtual`` is set in that case). 
        """ 
        visualizer = self._require_visualizer() if render else None 
        check = self.check_q_in_range(q) 
        if check.out_of_range: 
            logger.warning("System limits reached: %s", check.per_axis) 
            self.experiment.limit_flag_virtual = True 
            return None 
        output = self.compute_models(q) 
        if visualizer is not None: 
            return visualizer.render(q, output) 
        return None 
    def visualize_state(self): 
        """Render the most recent measured state through the visualizer.""" 
        self._require_visualizer() 
        if self.axis_normalized.size == 0: 
            raise StrasError("no state received yet; call get_state() first") 
        q = self._normalized_to_config_space(self.axis_normalized) 
        return self.visualize_frame(q, render=True) 

class RealFollower(Follower): 
    """Follower connected to the physical (or hardware-in-the-loop) system.""" 
    def __init__(self, transport: StrasTransport, 
                     limits: Optional[RobotLimits] = None, 
                     module_main: Optional[PathLike] = None, 
                     expand_ranges: Optional[bool] = None, 
                     enforce_channel_guard: bool = False, **kwargs): 
        """ 
        Args: 
            transport: connected-or-not ``StrasTransport``. 
            expand_ranges: widen axis ranges when a measurement exceeds them. 
                Defaults to True only on port 6667 (the original behaviour). 
            enforce_channel_guard: block BR / BL go-to commands while the 
                matching tool is still inside the channel.  The original code 
                intended this but never triggered it (see ``send_goto_by_label``), 
                so it is off by default to keep behaviour identical. 
        """ 
        super().__init__(limits=limits, module_main=module_main, **kwargs) 
        self.transport = transport 
        self.ranges = AxisRanges.empty() 
        self.axis_infos: Optional[AxisInfos] = None 
        self.expand_ranges = (transport.port == RANGE_TRACKING_PORT 
                              if expand_ranges is None else expand_ranges) 
        self.enforce_channel_guard = enforce_channel_guard 
    @property 
    def range_limits(self) -> JointLimits: 
        return self.limits.joint_velocity 
    # --- connection -------------------------------------------------------------- 
    def start(self, timeout: float = 1.0) -> None: 
        self.transport.set_timeout(timeout) 
        self.transport.connect() 
        logger.info("Connected to %s:%s", self.transport.host, self.transport.port) 
    def close(self) -> None: 
        self.transport.close() 
    # --- simple commands --------------------------------------------------------- 
    def _command(self, command: int, reply_mode: int = 1) -> bytes: 
        return self.transport.request(build_cmd(command, reply_mode)) 
    def on(self) -> bytes: 
        return self._command(Command.ON) 
    def off(self) -> bytes: 
        return self._command(Command.OFF) 
    def stop(self) -> bytes: 
        return self._command(Command.STOP) 
    def reset(self) -> bytes: 
        return self._command(Command.RESET) 
    # --- queries ----------------------------------------------------------------- 
    def get_axis_info(self) -> AxisInfos: 
        raw = self._command(Command.SEND_AXIS_INFOS) 
        infos = parse_axis_infos(raw) 
        self.axis_infos = infos 
        self.ranges = infos.ranges            # replaced, not appended to 
        return infos 
    def get_state(self) -> Optional[RobotState]: 
        try: 
            raw = self._command(Command.GET_STATE) 
        except TransportTimeout: 
            logger.warning("Timeout Error in Slave...") 
            self.timeout_error = True 
            return None 
        self.timeout_error = False 
        if not parse_header(raw).sync_ok: 
            logger.warning("Check Synchro! Reply discarded.") 
            return None 
        state = parse_state(raw) 
        self.axis_normalized = self.ranges.normalize(state.measured) 
        if self.expand_ranges: 
            self.ranges.expand(state.measured, upto=7) 
        self.state = state 
        return state 
    # --- motion commands --------------------------------------------------------- 
    def _command_port_ok(self) -> bool: 
        if self.transport.port != COMMAND_PORT: 
            logger.warning("USING SLAVE, CHANGE PORT TO %d PLEASE!", COMMAND_PORT) 
            return False 
        return True 
    def _send(self, msg: bytes) -> bool: 
        try: 
            self.transport.send(msg) 
        except TransportTimeout: 
            logger.error("Timeout Error") 
            self.timeout_error = True 
            return False 
        except TransportError as exc: 
            logger.error("COULD NOT SEND: %s", exc) 
            self.send_error = True 
            self.last_error = 'ErrorVelIDX'   # tag kept from the original 
            return False 
        return True 
    def send_velocity(self, axes_vals, verbose: bool = False) -> bool: 
        if not self._command_port_ok(): 
            return False 
        if verbose: 
            for axis, vel in axes_vals: 
                logger.info("Axes %s increasing by: %s", axis, vel) 
        return self._send(build_vel_idx(axes_vals)) 
    def send_goto(self, targets, verbose: bool = False, 
                  per_axis: bool = False) -> bool: 
        if not self._command_port_ok(): 
            return False 
        if verbose: 
            for axis, pos, vel in targets: 
                logger.info("Axis %s moving to %.2f at max rel. velocity %.2f", 
                            axis, pos, vel) 
        command = Command.GO_TO_PER_AXIS if per_axis else Command.IDX_GO_TO_VEL 
        return self._send(build_goto(targets, command)) 
    def send_goto_by_label(self, targets: Iterable[LabelTarget], 
                           verbose: bool = False) -> bool: 
        """Like ``send_goto`` but with axis labels ('BR', 'TL', ...).""" 
        out: List[GoToTarget] = [] 
        for label, pos, max_vel in targets: 
            if (self.enforce_channel_guard and label in CHANNEL_GUARD 
                    and not self._clear_of_channel(CHANNEL_GUARD[label])): 
                logger.warning("%s tool inside the channel; %s command skipped", 
                               CHANNEL_GUARD[label], label) 
                continue 
            out.append((AXIS_SOCKET_INDEX[label], pos, max_vel)) 
        if not out: 
            return False 
        return self.send_goto(out, verbose) 
    def _clear_of_channel(self, translation_label: str) -> bool: 
        # The original indexed with the *socket* index and put the ">" inside 
        # the brackets; the intent is "normalised translation > 0.45". 
        pos = AXIS_POSITION[translation_label] 
        return (self.axis_normalized.size > pos 
                and self.axis_normalized[pos] > CHANNEL_CLEARANCE_NORM) 

class VirtualFollower(Follower): 
    """No hardware: integrate velocity commands into simulated joint state.""" 
    def __init__(self, *args, **kwargs): 
        super().__init__(*args, **kwargs) 
        self.last_velocity = {joint: 0.0 for joint in QP_ORDER} 
        self.last_position = {joint: 0.0 for joint in QP_ORDER} 
    @property 
    def range_limits(self) -> JointLimits: 
        return self.limits.virtual_cs 
    def set_q(self, q) -> None: 
        """Set the simulated configuration (configuration-space order/range).""" 
        values = np.asarray(q, dtype=np.float64) 
        if values.shape != (len(QP_ORDER),): 
            raise ValueError(f"q must have shape ({len(QP_ORDER)},), got {values.shape}") 
        self.q_config_space = values.copy() 
        self.last_position = dict(zip(QP_ORDER, values.tolist())) 
    def start(self) -> None: 
        self.last_velocity = {joint: 0.0 for joint in QP_ORDER} 
    def close(self) -> None: 
        pass 
    def get_state(self) -> Optional[RobotState]: 
        logger.debug("Virtual follower: no updated state") 
        return None 
    def get_axis_info(self) -> Optional[AxisInfos]: 
        return None 
    def send_velocity(self, axes_vals, verbose: bool = False) -> bool: 
        label_by_socket_index = { 
            socket_index: label 
            for label, socket_index in AXIS_SOCKET_INDEX.items() 
        } 
        updated = {joint: 0.0 for joint in QP_ORDER} 
        for axis, velocity in axes_vals: 
            label = label_by_socket_index.get(int(axis)) 
            if label is None or label not in updated: 
                raise ValueError(f"unknown STRAS axis socket index: {axis!r}") 
            updated[label] = float(velocity) 
        self.last_velocity = updated 
        return True 
    def advance(self, delta_s: float) -> None: 
        """Integrate the current joint velocities and clamp to virtual limits.""" 
        delta_s = float(delta_s) 
        if not np.isfinite(delta_s) or delta_s < 0: 
            raise ValueError("delta_s must be a finite non-negative number") 
        limits = self.limits.virtual_cs 
        lower = dict(zip(limits.labels, limits.lower)) 
        upper = dict(zip(limits.labels, limits.upper)) 
        for joint in QP_ORDER: 
            position = self.last_position[joint] + self.last_velocity[joint] * delta_s 
            self.last_position[joint] = float( 
                np.clip(position, lower[joint], upper[joint])) 
        self.q_config_space = np.array( 
            [self.last_position[joint] for joint in QP_ORDER], dtype=np.float64) 
    def send_goto(self, targets, verbose: bool = False, 
                  per_axis: bool = False) -> bool: 
        return False 

def make_follower(kind: str, host: str = '100.100.100.100', 
                  port: Union[int, str] = 9999, **kwargs) -> Follower: 
    """Create a follower from 'real' / 'virtual' / 'simulated' (any case).""" 
    k = kind.strip().lower() 
    if k == 'real': 
        return RealFollower(StrasTransport(host, port), **kwargs) 
    if k in ('virtual', 'simulated'): 
        return VirtualFollower(**kwargs) 
    raise ValueError(f"unknown follower kind: {kind!r}")