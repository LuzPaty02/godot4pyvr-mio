"""XR-oriented adapter for the STRAS leader state.

This module keeps only the parts that are useful for XR-driven interaction and
preview logic:
- robot-axis state tracking
- normalization and axis reordering
- button/teleoperation activation toggles
- optional model/visualization setup

It intentionally omits the low-level STRAS socket protocol and raw command
serialization used by the hardware driver.
"""

import os
import sys
import importlib
from math import radians

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from xr_mappings import map_normalized_to_range as map_norm2range
from xr_mappings import order_ranges as orderRanges
from xr_mappings import follower_to_config_space as order_follower2configspace
from xr_model_support import resolve_model_files

# Optional model dependencies are loaded lazily so the XR layer can still be
# imported in environments that do not have the full robot visualization stack.
try:
    from model_utilities.camera_model import camera_model
except Exception:  # pragma: no cover - optional dependency
    camera_model = None

try:
    from model_utilities.robot_model import robot_model, robot_twobending
except Exception:  # pragma: no cover - optional dependency
    robot_model = None
    robot_twobending = None

try:
    from model_utilities.stras_model import StrasModel
except Exception:  # pragma: no cover - optional dependency
    StrasModel = None


qlims_down_joint_pos = {
    'BR': -np.pi/(2*18.5),
    'RR': radians(2),
    'TR': 2,
    'BL': -np.pi/(2*18.5),
    'RL': radians(2),
    'TL': 2,
    'EBH': -np.pi/(2*185),
    'EBV': -np.pi/(2*185),
    'ER': radians(2),
    'ET': 0,
}

qlims_up_joint_pos = {
    'BR': np.pi/(2*18.5),
    'RR': radians(248),
    'TR': 72,
    'BL': np.pi/(2*18.5),
    'RL': radians(248),
    'TL': 72,
    'EBH': np.pi/(2*185),
    'EBV': np.pi/(2*185),
    'ER': radians(248),
    'ET': 95,
}

qlims_down_slave_pos = {
    'BR': -6.5,
    'RR': -270,
    'TR': 2,
    'BL': -6.5,
    'RL': -270,
    'TL': 2,
    'EBH': -9.12,
    'EBV': -9.12,
    'ER': -90,
    'ET': 0.5,
}

qlims_up_slave_pos = {
    'BR': 6.5,
    'RR': 360,
    'TR': 73.5,
    'BL': 6.5,
    'RL': 360,
    'TL': 73.5,
    'EBH': 9.12,
    'EBV': 9.12,
    'ER': 50,
    'ET': 90,
}

qlims_down_master_pos = {
    'BR': -44,
    'RR': -163,
    'TR': 0,
    'BL': -44,
    'RL': -163,
    'TL': 0,
    'EBH': 0,
    'EBV': 0,
    'ER': 0,
    'ET': 0,
}

qlims_up_master_pos = {
    'BR': 44,
    'RR': 163,
    'TR': 105,
    'BL': 44,
    'RL': 163,
    'TL': 105,
    'EBH': 0,
    'EBV': 0,
    'ER': 0,
    'ET': 0,
}

slaveOrder = ['EBV', 'EBH', 'ER', 'ET', 'TR', 'RR', 'BR', 'TL', 'RL', 'BL']
masterOrder = ['TL', 'RL', 'BL', 'AL', 'TR', 'RR', 'BR', 'AR', 'EBV', 'EBH', 'ER', 'ET']
qpOrder = ['BR', 'RR', 'TR', 'BL', 'RL', 'TL', 'EBV', 'EBH', 'ER', 'ET']

masterIndexLabels = masterOrder.copy()
masterIndexSocket = range(len(masterIndexLabels))

lims_down_joint = np.array(list(qlims_down_joint_pos.values()), dtype=np.float32)
lims_up_joint = np.array(list(qlims_up_joint_pos.values()), dtype=np.float32)
lims_down_master = np.array(list(qlims_down_master_pos.values()), dtype=np.float32)
lims_up_master = np.array(list(qlims_up_master_pos.values()), dtype=np.float32)
lims_down_slave = np.array(list(qlims_down_slave_pos.values()), dtype=np.float32)
lims_up_slave = np.array(list(qlims_up_slave_pos.values()), dtype=np.float32)
configFile = 'datafolders.txt'


class XRLeaderAdapter:
    """Lightweight XR-friendly wrapper around STRAS state and robot-space maps."""

    def __init__(self, host=None, port=None, buttonActivated=False,
                 armsSwitched_=False, rightButtonOnly=True):
        self.host = host
        self.port = port
        self.armsSwitched = armsSwitched_
        self.buttonActivatedFlag = buttonActivated
        self.all_activatedByRightButton = rightButtonOnly

        self.axisMins = []
        self.axisMaxs = []
        self.axisEnable = []
        self.axisCalibrated = []
        self.axisUsed = []
        self.axisReference = []
        self.axisButtons = []

        self.axisCurrVal = np.array([], dtype=np.float32)
        self.axisCurrValNormalized = np.array([], dtype=np.float32)
        self.axisCurrValNormalized_backUp = np.array([], dtype=np.float32)
        self.axisCurrValVel = np.array([], dtype=np.float32)
        self.axisPastVal = np.array([], dtype=np.float32)

        self.buttonFlag = False
        self.counterButton = 0.0
        self.handInLeftHandle = False
        self.handInRightHandle = False

        self.robot_stras = None
        self.cam = None
        self.render = None
        self.initializeModelFlag = False
        self.initializeRenderFlag = False
        self.axisNames = masterOrder
        self.configFile = os.path.join('model_utilities', 'datafolders.txt')

    # ------------------------------------------------------------------
    # State setup and normalization
    # ------------------------------------------------------------------
    def set_axis_limits(self, mins, maxs):
        self.axisMins = list(mins)
        self.axisMaxs = list(maxs)

    def normalize_axis_value(self, value, axis_min, axis_max, inverted=False):
        if axis_max == axis_min:
            return 0.0
        mapped = (value - axis_min) / (axis_max - axis_min)
        if inverted:
            mapped = 1.0 - mapped
        return float(np.clip(mapped, 0.0, 1.0))

    def normalize_state(self, raw_axis_values):
        raw_axis_values = np.asarray(raw_axis_values, dtype=np.float32)
        normalized = []
        for i, value in enumerate(raw_axis_values):
            if i < len(self.axisMins) and i < len(self.axisMaxs):
                axis_min = float(self.axisMins[i])
                axis_max = float(self.axisMaxs[i])
                inverted = bool((i == 4) or (i == 0) or (i == 1) or (i == 5))
                normalized.append(self.normalize_axis_value(value, axis_min, axis_max, inverted=inverted))
            else:
                normalized.append(0.0)
        return np.asarray(normalized, dtype=np.float32)

    def update_state(self, raw_axis_values, raw_buttons=None):
        """Update current robot state from a source array.

        The input is expected to be in master/driver ordering and will be stored
        in `axisCurrVal` as-is, with `axisCurrValNormalized` computed from the
        known axis limits.
        """
        if raw_axis_values is None:
            return

        self.axisCurrVal = np.asarray(raw_axis_values, dtype=np.float32).copy()
        self.axisCurrValNormalized = self.normalize_state(self.axisCurrVal)
        self.axisCurrValNormalized_backUp = self.axisCurrValNormalized.copy()

        if raw_buttons is not None:
            self.axisButtons = list(raw_buttons)
        elif not self.axisButtons:
            self.axisButtons = [0] * max(4, len(self.axisCurrVal))

        if self.buttonActivatedFlag:
            self.update_button_state()

    def update_button_state(self):
        """Simplified button behavior used by the original teleoperation logic."""
        if not self.axisButtons:
            return

        if (not self.all_activatedByRightButton) and (len(self.axisButtons) > 1) and (self.axisButtons[0] == 1 or self.axisButtons[1] == 1):
            self.buttonFlag = not self.buttonFlag
        elif self.all_activatedByRightButton and len(self.axisButtons) > 1 and self.axisButtons[1] == 1:
            if self.counterButton == 0:
                self.counterButton = float(np.sqrt(0.0))  # placeholder to keep API consistent
            self.buttonFlag = not self.buttonFlag

        if len(self.axisButtons) > 2:
            self.handInLeftHandle = bool(self.axisButtons[2] == 1)
        if len(self.axisButtons) > 3:
            self.handInRightHandle = bool(self.axisButtons[3] == 1)

    def deactivateTeleoperation_button(self):
        self.buttonFlag = not self.buttonFlag

    # ------------------------------------------------------------------
    # Simulated/dev/testing modes
    # ------------------------------------------------------------------
    def simulatedInit(self, printOpt_=False):
        self.NumberAxis = 12
        self.axisMins = [0, -163, -44, -1,
                         0, -163, -44, -1,
                         -1, -1, -1, -1]
        self.axisMaxs = [97, 163, 44, 1,
                         97, 163, 44, 1,
                         1, 1, 1, 1]
        self.ActiveAxis = [False] * self.NumberAxis

        if printOpt_:
            for i_ in range(self.NumberAxis):
                print(masterIndexLabels[i_], '', masterIndexSocket[i_], '\t',
                      ('ACTIVE' if self.ActiveAxis[i_] > 0 else 'INACTIVE'))
                print('\tmin: ', self.axisMins[i_])
                print('\tmax: ', self.axisMaxs[i_])

    def simulatedState(self, printOpt_=False, q_in_=0, q_in_norm=0):
        if q_in_ == 0 and q_in_norm == 0:
            q_simMaster = np.array([45, 0, -10, -1,
                                    45, 0, -10, -1,
                                    0, 0, 0, 0], dtype=np.float32)
            q_simMasterNormalized = np.array([0.53, 0, 0.38, 0,
                                              0.53, 0, 0.38, 0,
                                              0, 0, 0, 0], dtype=np.float32)
        else:
            q_simMaster = np.asarray(q_in_, dtype=np.float32).copy()
            q_simMasterNormalized = np.asarray(q_in_norm, dtype=np.float32).copy()

        self.axisCurrVal = q_simMaster.copy()
        self.axisCurrValNormalized = q_simMasterNormalized.copy()
        self.axisCurrValNormalized_backUp = self.axisCurrValNormalized.copy()

        if printOpt_:
            print('Axis    \t AXmea  \tNorm')
            for i_ in range(len(self.axisCurrVal)):
                print(masterIndexLabels[i_], '\t',
                      masterIndexSocket[i_], '   \t',
                      np.round(self.axisCurrVal[i_], 4), '   \t',
                      np.round(self.axisCurrValNormalized[i_], 2))

    # ------------------------------------------------------------------
    # Robot model / visualization (optional)
    # ------------------------------------------------------------------
    def initializeModels(self, printOpt_=False):
        if robot_model is None or robot_twobending is None or StrasModel is None:
            raise ImportError('Robot model dependencies are not available in this environment.')

        paths = resolve_model_files(self.configFile)
        self.cam = camera_model(paths['camera_calibration']) if camera_model is not None else None
        robot_right = robot_model(configfile=paths['robot_config_right'])
        robot_left = robot_model(configfile=paths['robot_config_left'])
        robot_end = robot_twobending(configfile=paths['robot_config_endoscope'])

        self.robot_stras = StrasModel(robot_right, robot_left, robot_end)
        self.initializeModelFlag = True

    def computeModels(self, q2compute, printOpt_=False):
        if not self.initializeModelFlag:
            self.initializeModels(printOpt_)

        stras_output = self.robot_stras.computemodels(np.asarray(q2compute, dtype=np.float32).copy())
        self.Tr_e_follower = stras_output[0].copy()
        self.Jr_e_follower = stras_output[1].copy()
        self.Tl_e_follower = stras_output[2].copy()
        self.Jl_e_follower = stras_output[3].copy()
        self.Tr_0_follower = stras_output[4].copy()
        self.Jr_0_follower = stras_output[5].copy()
        self.Tl_0_follower = stras_output[6].copy()
        self.Jl_0_follower = stras_output[7].copy()
        self.Te_follower = stras_output[8].copy()
        self.Je_follower = stras_output[9].copy()
        self.zr_e = stras_output[10].copy()

    def visualizeState(self, printOpt_=False):
        """Convert current normalized state to visualization space."""
        q_to_visualize = map_norm2range(
            order_follower2configspace(self.axisCurrValNormalized),
            orderRanges(qlims_down_joint_pos, qpOrder),
            orderRanges(qlims_up_joint_pos, qpOrder),
        )
        return self.visualizeFrame(q_to_visualize, printOpt_)

    def initializeVisualization(self, printOpt_=False):
        if camera_model is None or robot_model is None or robot_twobending is None or StrasModel is None:
            raise ImportError('Visualization dependencies are not available in this environment.')

        paths = resolve_model_files(self.configFile)
        self.cam = camera_model(paths['camera_calibration'])
        robot_right = robot_model(configfile=paths['robot_config_right'])
        robot_left = robot_model(configfile=paths['robot_config_left'])
        robot_end = robot_twobending(configfile=paths['robot_config_endoscope'])

        self.robot_stras = StrasModel(robot_right, robot_left, robot_end)
        self.initializeRenderFlag = True

    def visualizeFrame(self, q2visualize, printOpt_=False):
        if not self.initializeRenderFlag:
            self.initializeVisualization(printOpt_)

        stras_output = self.robot_stras.computemodels(np.asarray(q2visualize, dtype=np.float32).copy())
        Tr_e_slave = stras_output[0]
        Tl_e_slave = stras_output[2]
        Tr_0_slave = stras_output[4]
        Tl_0_slave = stras_output[6]
        Te_slave = stras_output[8]

        shpRslve, shapeLslave, tipend_slave = self.robot_stras.computeShape(np.asarray(q2visualize, dtype=np.float32).copy())

        try:
            render_shape = importlib.import_module('model_utilities.vtk_render').render_shape
            self.render = render_shape(640, 480, self.robot_stras)
            self.render.setCamProperties(self.cam.getF(), self.cam.getC(), self.cam.K)
            img = self.render.render_stras_arms(shpRslve, shapeLslave, tipend_slave,
                                               Tr_e_slave, Tl_e_slave,
                                               Tr_0_slave, Tl_0_slave, Te_slave)
            return img
        except Exception:
            return {
                'q2visualize': np.asarray(q2visualize, dtype=np.float32),
                'Tr_e_slave': Tr_e_slave,
                'Tl_e_slave': Tl_e_slave,
            }


# ----------------------------------------------------------------------
# Convenience constructors
# ----------------------------------------------------------------------
def make_xr_adapter(*args, **kwargs):
    return XRLeaderAdapter(*args, **kwargs)


if __name__ == '__main__':
    xr = XRLeaderAdapter()
    xr.simulatedInit(True)
    xr.simulatedState(q_in_=np.array([10, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0], dtype=np.float32))
    print('Normalized state sample:', xr.axisCurrValNormalized)
