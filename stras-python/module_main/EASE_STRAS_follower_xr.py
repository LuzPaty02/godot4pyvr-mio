"""XR-oriented adapter for the STRAS follower state.

This module keeps only the parts useful for XR interaction and preview:
- state tracking for follower joints
- normalized state conversion and config-space mapping
- safety/limit checks
- optional model-based visualization

It intentionally omits the low-level STRAS socket communication and command
sending used by the hardware controller.
"""

import os
import sys
import importlib
from math import radians
from typing import Any

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from xr_mappings import map_normalized_to_range as map_norm2range
from xr_mappings import order_ranges as orderRanges
from xr_mappings import follower_to_config_space as order_follower2configspace
from xr_model_support import resolve_model_files

try:
    from model_utilities.camera_model import camera_model
except Exception:  # pragma: no cover
    camera_model = None

try:
    from model_utilities.robot_model import robot_model, robot_twobending
except Exception:  # pragma: no cover
    robot_model = None
    robot_twobending = None

try:
    from model_utilities.stras_model import StrasModel
except Exception:  # pragma: no cover
    StrasModel = None


# ----------------------------------------------------------------------
# Follower configuration limits
# ----------------------------------------------------------------------
qlims_down_joint_pos_virtual = {
    'BR': -np.pi/(2*18.5),
    'RR': np.radians(-160),
    'TR': -10,
    'BL': -np.pi/(2*18.5),
    'RL': np.radians(-160),
    'TL': -10,
    'EBH': -np.pi/(2*185),
    'EBV': -np.pi/(2*185),
    'ER': np.radians(-90),
    'ET': -100,
}

qlims_up_joint_pos_virtual = {
    'BR': np.pi/(2*18.5),
    'RR': np.radians(160),
    'TR': 72,
    'BL': np.pi/(2*18.5),
    'RL': np.radians(160),
    'TL': 72,
    'EBH': np.pi/(2*185),
    'EBV': np.pi/(2*185),
    'ER': np.radians(90),
    'ET': 95,
}

lowerCSlims = {
    'BR': -np.pi/(2*18.5),
    'RR': np.radians(-160),
    'TR': -200,
    'BL': -np.pi/(2*18.5),
    'RL': np.radians(-160),
    'TL': -200,
    'EBH': -np.pi/(2*185),
    'EBV': -np.pi/(2*185),
    'ER': np.radians(-90),
    'ET': -200,
}

upperCSlims = {
    'BR': np.pi/(2*18.5),
    'RR': np.radians(160),
    'TR': 200,
    'BL': np.pi/(2*18.5),
    'RL': np.radians(160),
    'TL': 72,
    'EBH': np.pi/(2*185),
    'EBV': np.pi/(2*185),
    'ER': np.radians(90),
    'ET': 200,
}

slaveIndexLabels = ['EBV', 'EBH', 'ER', 'ET', 'TR', 'RR', 'BR', 'TL', 'RL', 'BL', 'AL']
slaveIndexSocket = [1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]
qpOrder = ['BR', 'RR', 'TR', 'BL', 'RL', 'TL', 'EBV', 'EBH', 'ER', 'ET']

# Order used in follower state arrays
stateOrder = ['EBV', 'EBH', 'ER', 'ET', 'TR', 'RR', 'BR', 'TL', 'RL', 'BL', 'AL']

# These arrays are the follower equivalents of the leader mapping data
lims_down_joint_virtual = np.asarray(orderRanges(qlims_down_joint_pos_virtual, stateOrder[:10]), dtype=np.float32)
lims_up_joint_virtual = np.asarray(orderRanges(qlims_up_joint_pos_virtual, stateOrder[:10]), dtype=np.float32)
lims_down_joint_cs = np.asarray(list(orderRanges(qlims_down_joint_pos_virtual, qpOrder)), dtype=np.float32)
lims_up_joint_cs = np.asarray(list(orderRanges(qlims_up_joint_pos_virtual, qpOrder)), dtype=np.float32)


class XRFollowerAdapter:
    """Lightweight follower adapter suitable for XR interaction and preview."""

    def __init__(self, typeCondition='virtual', configFile='model_utilities/datafolders.txt'):
        self.typeCondition = typeCondition
        self.configFile = configFile
        self.q_overwritten = False
        self.axisCurrVal = np.array([], dtype=np.float32)
        self.axisCurrValNormalized = np.array([], dtype=np.float32)
        self.axisCurrValVel = np.array([], dtype=np.float32)
        self.axisPastVal = np.array([], dtype=np.float32)
        self.axisButtons = []
        self.axisEnable = []
        self.axisCalibrated = []
        self.axisUsed = []
        self.axisReference = []

        self.lowerCSlims = lowerCSlims
        self.upperCSlims = upperCSlims
        self.qlims_down_virtual = qlims_down_joint_pos_virtual
        self.qlims_up_virtual = qlims_up_joint_pos_virtual

        self.robot_stras: Any = None
        self.render: Any = None
        self.initializeModelFlag = False
        self.initializeRenderFlag = False
        self.limitFlagVirtual = False

    # ------------------------------------------------------------------
    # State update and normalization
    # ------------------------------------------------------------------
    def set_axis_limits(self, mins, maxs):
        self.lims_down_joint = np.asarray(mins, dtype=np.float32)
        self.lims_up_joint = np.asarray(maxs, dtype=np.float32)

    def normalize_axis_value(self, value, axis_min, axis_max, inverted=False):
        if axis_max == axis_min:
            return 0.0
        mapped = (value - axis_min) / (axis_max - axis_min)
        if inverted:
            mapped = 1.0 - mapped
        return float(np.clip(mapped, 0.0, 1.0))

    def normalize_state(self, raw_state):
        raw_state = np.asarray(raw_state, dtype=np.float32).reshape(-1)
        if raw_state.size < len(stateOrder) - 1:
            raise ValueError('Follower state must provide at least 10 joint values.')
        raw_state = raw_state[:len(stateOrder) - 1]
        normalized = []
        for i, value in enumerate(raw_state):
            axis_min = float(self.lims_down_joint[i])
            axis_max = float(self.lims_up_joint[i])
            inverted = bool(i in (0, 1, 2, 3, 4, 5, 6))
            normalized.append(self.normalize_axis_value(value, axis_min, axis_max, inverted=inverted))
        return np.asarray(normalized, dtype=np.float32)

    def update_state(self, raw_state):
        if raw_state is None:
            return
        self.axisCurrVal = np.asarray(raw_state, dtype=np.float32).reshape(-1)[:len(stateOrder) - 1].copy()
        if not hasattr(self, 'lims_down_joint') or not hasattr(self, 'lims_up_joint'):
            self.lims_down_joint = lims_down_joint_virtual.copy()
            self.lims_up_joint = lims_up_joint_virtual.copy()
        self.axisCurrValNormalized = self.normalize_state(self.axisCurrVal)
        self.axisCurrValVel = np.zeros_like(self.axisCurrVal)
        self.axisPastVal = self.axisCurrVal.copy()

    def passCurrentq(self, q_two_pass_):
        self.update_state(q_two_pass_)

    # ------------------------------------------------------------------
    # Config-space conversion and safety
    # ------------------------------------------------------------------
    def checkQinrange(self, q_):
        q_arr = np.asarray(q_, dtype=np.float32)
        bool_list = []
        for ii, value in enumerate(q_arr):
            min_val = self.lowerCSlims[list(self.lowerCSlims.keys())[ii]]
            max_val = self.upperCSlims[list(self.upperCSlims.keys())[ii]]
            bool_list.append(value < min_val or value > max_val)
        bool_flag = any(bool_list)
        return bool_flag, bool_list

    def filterConfigurationState(self, q_):
        q_arr = np.asarray(q_, dtype=np.float32).copy()
        for i in range(len(q_arr)):
            q_arr[i] = np.clip(q_arr[i], self.lowerCSlims[list(self.lowerCSlims.keys())[i]], self.upperCSlims[list(self.upperCSlims.keys())[i]])
        return q_arr

    def filterConfigurationState_bendingsOnly(self, q_):
        q_arr = np.asarray(q_, dtype=np.float32).copy()
        for i in range(len(q_arr)):
            if i in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9):
                q_arr[i] = np.clip(q_arr[i], self.lowerCSlims[list(self.lowerCSlims.keys())[i]], self.upperCSlims[list(self.upperCSlims.keys())[i]])
        return q_arr

    # ------------------------------------------------------------------
    # Optional rendering / model preview
    # ------------------------------------------------------------------
    def initializeModels(self, printOpt_=False):
        if robot_model is None or robot_twobending is None or StrasModel is None:
            raise ImportError('Robot model dependencies are not available in this environment.')

        paths = resolve_model_files(self.configFile)
        cam = camera_model(paths['camera_calibration']) if camera_model is not None else None
        robot_right = robot_model(configfile=paths['robot_config_right'])
        robot_left = robot_model(configfile=paths['robot_config_left'])
        robot_end = robot_twobending(configfile=paths['robot_config_endoscope'])

        self.robot_stras = StrasModel(robot_right, robot_left, robot_end)
        self.initializeModelFlag = True
        self.cam = cam

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
        if self.axisCurrValNormalized.size == 0:
            return None

        q_to_visualize = map_norm2range(
            order_follower2configspace(self.axisCurrValNormalized),
            lims_down_joint_cs,
            lims_up_joint_cs,
        )
        return self.visualizeFrame(q_to_visualize, imRet=True, printOpt_=printOpt_)

    def visualizeFrame(self, q2visualize, imRet=False, printOpt_=False):
        if not self.initializeModelFlag:
            self.initializeModels(printOpt_)

        bool_flag, bool_list = self.checkQinrange(q2visualize)
        if bool_flag:
            self.limitFlagVirtual = True
            print('System limits reached!!!', bool_list)
            return None

        stras_output = self.robot_stras.computemodels(np.asarray(q2visualize, dtype=np.float32).copy())
        self.Tr_e_follower = stras_output[0].copy()
        self.Tl_e_follower = stras_output[2].copy()
        self.Tr_0_follower = stras_output[4].copy()
        self.Tl_0_follower = stras_output[6].copy()
        self.Te_follower = stras_output[8].copy()

        if not imRet:
            return 0

        if self.cam is None:
            raise ImportError('Camera calibration dependencies are unavailable for image rendering.')

        try:
            render_shape = importlib.import_module('model_utilities.vtk_render').render_shape
            self.render = render_shape(self.cam.getImSize()[0], self.cam.getImSize()[1], self.robot_stras)
            self.render.setCamProperties(self.cam.getF(), self.cam.getC(), self.cam.K)
            shpRslve, shapeLslave, tipend_follower = self.robot_stras.computeShape(np.asarray(q2visualize, dtype=np.float32).copy())
            img_ = self.render.render_stras_arms(
                shpRslve, shapeLslave, tipend_follower,
                self.Tr_e_follower, self.Tl_e_follower,
                self.Tr_0_follower, self.Tl_0_follower, self.Te_follower,
            )
            return img_
        except Exception:
            return {
                'q2visualize': np.asarray(q2visualize, dtype=np.float32),
                'Tr_e_follower': self.Tr_e_follower,
                'Tl_e_follower': self.Tl_e_follower,
            }


# ----------------------------------------------------------------------
# Simple factory
# ----------------------------------------------------------------------
def make_xr_follower_adapter(*args, **kwargs):
    return XRFollowerAdapter(*args, **kwargs)


if __name__ == '__main__':
    follower = XRFollowerAdapter()
    follower.lims_down_joint = lims_down_joint_virtual.copy()
    follower.lims_up_joint = lims_up_joint_virtual.copy()
    follower.update_state(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32))
    print('Normalized state sample:', follower.axisCurrValNormalized)
