import os
import sys
sys.path.insert(0, sys.path[0]+'module_main/')
import socket
from csv import reader as csvreader
import numpy as np
from numpy import float32
from numpy import float64
from struct import unpack
from struct import pack
from copy import copy
# from module_main.useful import map_norm2slave_followerOrder, normalize_q_qp  # Here for compatibility, not used
from module_main.useful import orderRanges  # To order the dictionaries in order of configuration space, leader or follower
from module_main.useful import map_norm2range  # To interpolate from [0, 1] to [mins, maxs] the desired output space
from module_main.useful import order_follower2configspace  # to order from follower to configuration space
from module_main.useful import  checkInRange  # to check if the q is in the said ranges
# from model_utilities.vtk_render import render_shape  # Render the basic shapes
from module_main.model_utilities.vtk_render_clean import render_shape  # Render the basic shapes
from module_main.model_utilities.camera_model import camera_model  # Iimplementing a pinhole camera model
from module_main.model_utilities.robot_model import robot_model  # Class describing model single piecewise constant curvature robot
from module_main.model_utilities.robot_model import robot_twobending  # Class describing the model of the STRAS architecture
from module_main.model_utilities.stras_model import StrasModel
import module_main.model_utilities.transformations as tf
# from module_main.control.qpsolverSlack import qpSolver  # present when controlling the robot
import matplotlib.pyplot as plt
from matplotlib.pyplot import ioff as pltioff
from matplotlib import use as pltuse
import matplotlib.ticker as tckticker
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas  # for testing
from matplotlib.figure import Figure  # for testing
from PIL.Image import frombytes
# sys.path.insert(0, os.getcwd()+'/exp-esr14/module_main/')

messageTypeDic = {0: 'CMD',
                  1: 'ACK',
                  2: 'TRACE',
                  3: 'DEBUG',
                  4: 'STATE',
                  41: 'STATEEXTENDED',
                  42: 'AXISINFOS',
                  43: 'SUPERVISIORMESSAGE',
                  44: 'TELEOPINFOSTYPE'}
controllerModeOps = {0: 'INIT', 1: 'ON UNCAL', 2: 'DO CAL',
                     3: 'GO TO INIT POS', 4: 'POS CONTROL',
                     5: 'POS PLANNER(DEBUG)', 6: 'VEL PLANNER(DEBUG)',
                     61: 'EMBEDDEDTELEOP', 62: 'MASTEREXTERNALTELEOP',
                     63: 'SLAVEEXTERNALTELEOP',
                     11: 'ETHERCAT BUS INITIALIZATION', 12: 'BUS DISCONNECTED'}
commandByCMD = {0: 'NONE',
                1: 'CONTROLLER_CMD_ON',
                2: 'CONTROLLER_CMD_OFF',
                3: 'CONTROLLER_CMD_CALIBRATION',
                4: 'CONTROLLER_CMD_GO_TO',  # FOR 1 AXIS AT A TIME
                400: 'CONTROLLER_CMD_REACH_END_POINT',
                5: 'CONTROLLER_CMD_VEL',  # FOR VELOCITY CONTROL
                52: 'CONTROLLER_CMD_IDX_VELOCITY',
                6: 'CONTROLLER_CMD_GO_TO_VEL',  # Debug command
                620: 'CONTROLLER_CMD_IDX_GO_TO_VEL',  # USE THIS COMMAND!
                7: 'CONTROLLER_CMD_GET_STATE',
                8: 'CONTROLLER_CMD_STOP',
                9: 'CMD_DEBUG_FAULT',  # Debug command
                # 10: 'CMD_DEBUG_CALIBRATION_NOSUCCESS',  # Debug command
                # 11: 'CMD_DEBUG_NO_SENSOR_FEEDBACK',  # Debug command
                # 12: 'CMD_DEBUG_ACCELERATION_ERR',  # Debug command
                # 13: 'CMD_DEBUG_TRACKING_ERR',  # Debug command
                # 14: 'CMD_DEBUG_CONTROL_ERR',  # Debug command
                41: 'DEBUG_CMD_SET_POS',  # Debug command
                410: 'DEBUG_CMD_SET_POS_IDX',  # Debug command
                # 50: 'DEBUG_CMD_PUSH_BUTTON',  # Debug command
                # 51: 'DEBUG_CMD_RELEASE_BUTTON',  # Debug command
                # 70: 'DEBUG_CMD_SUPERVISOR_ERROR',  # Debug command
                71: 'CONTROLLER_CMD_SEND_AXIS_INFOS',
                72: 'CONTROLLER_CMD_SEND_STATE_EXTENDED',
                73: 'CONTROLLER_CMD_SUPERIOR_MESSAGE',
                # 79: 'DEBUG_CMD_SUPERVISOR_RESET',  # Debug command
                700: 'CONTROLLER_CMD_GET_CURTIME',
                100: 'CONTROLLER_CMD_RESET'}
qpOrder = ['BR', 'RR', 'TR',
           'BL', 'RL', 'TL',
           'EBV', 'EBH', 'ER', 'ET']
slaveIndexSocket = [1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]
slaveIndexLabels = ['EBV', 'EBH', 'ER', 'ET',
                    'TR', 'RR', 'BR',
                    'TL', 'RL', 'BL', 'AL']
dicSlaveIndex = dict(zip(slaveIndexLabels, slaveIndexSocket))

class EASEfollower:
    def __init__(self, typeCondition_, host='100.100.100.100', port='9999'):
        """EASEfollower class to provide connection, simulation and control of real or virtual EASE/STRAS system.

        Args:
            typeCondition_ (str): 'REAL', 'real' or 'simulated','SIMULATED'
            host (str, optional): IP address of the STRAS computer. Defaults to '100.100.100.100'.
            port (str, optional): Port asigned to the slave. Defaults to '9999'.
        """
        
        # Testing new max velocities
        bendArmVel = (((np.pi/(18.5*2))+(np.pi/(18.5*2))))*0.5
        bendEndVel = (((np.pi/(185*2))+(np.pi/(185*2))))*0.5
        maxTransArms, maxTransEndo = 5, 10
        radiansArm, radiansEndo =  10, 1
        self.lims_down_joint = np.array([  # MIN
                    bendArmVel, np.radians(radiansArm), maxTransArms,  # min right arm vel
                    bendArmVel, np.radians(radiansArm), maxTransArms,  # min right arm vel
                    bendEndVel, bendEndVel, np.radians(radiansEndo), maxTransEndo]) # min endo vel (neg max) - TESTING
        self.lims_up_joint = np.array([  # MAX
                    bendArmVel, np.radians(radiansArm), maxTransArms,  # max right arm vel
                    bendArmVel, np.radians(radiansArm), maxTransArms,  # max right arm vel
                    bendEndVel, bendEndVel, np.radians(radiansEndo), maxTransEndo]) # min endo vel (neg max) - TESTING
        
        self.q_overwriten = False
        
        qlims_down_joint_pos_virtual = {
            'BR': -np.pi/(2*18.5),  # deg/s
            'RR': np.radians(-160),  # Rotation in degrees/s
            'TR': -10,  # Translation in mm/s
            'BL': -np.pi/(2*18.5),  # deg/s
            'RL': np.radians(-160),  # Rotation in degrees/s
            'TL': -10,  # Translation in mm/s
            'EBH': -np.pi/(2*185),  # HorB-E
            'EBV': -np.pi/(2*185),  # VertB-E
            'ER': np.radians(-90),  # Rotation ENDO in degrees/s
            'ET': -100}  # Translation ENDO in mm/s
        qlims_up_joint_pos_virtual = {
            'BR': np.pi/(2*18.5),  # deg/s
            'RR': np.radians(160),  # Rotation in degrees/s
            'TR': 72,  # Translation in mm/s
            'BL': np.pi/(2*18.5),  # deg/s
            'RL': np.radians(160),  # Rotation in degrees/s
            'TL': 72,  # Translation in mm/s
            'EBH': np.pi/(2*185),  # HorB-E
            'EBV': np.pi/(2*185),  # VertB-E
            'ER': np.radians(90),  # Rotation ENDO in degrees/s
            'ET': 95}  # Translation ENDO in mm/s
        

        
        self.lowerCSlims = {
            'BR': -np.pi/(2*18.5),      # deg/s
            'RR': np.radians(-160),     # Rotation in degrees/s
            'TR': -200,                 # Translation in mm/s
            'BL': -np.pi/(2*18.5),      # deg/s
            'RL': np.radians(-160),     # Rotation in degrees/s
            'TL': -200,                 # Translation in mm/s
            'EBH': -np.pi/(2*185),      # HorB-E
            'EBV': -np.pi/(2*185),      # VertB-E
            'ER': np.radians(-90),      # Rotation ENDO in degrees/s
            'ET': -200}                 # Translation ENDO in mm/s
        self.upperCSlims = {
            'BR': np.pi/(2*18.5),       # deg/s
            'RR': np.radians(160),      # Rotation in degrees/s
            'TR': 200,                  # Translation in mm/s
            'BL': np.pi/(2*18.5),       # deg/s
            'RL': np.radians(160),      # Rotation in degrees/s
            'TL': 200,                  # Translation in mm/s
            'EBH': np.pi/(2*185),       # HorB-E
            'EBV': np.pi/(2*185),       # VertB-E
            'ER': np.radians(90),       # Rotation ENDO in degrees/s
            'ET': 200}                  # Translation ENDO in mm/s

        self.lims_down_joint_virtual = np.array(list(qlims_down_joint_pos_virtual.values()), dtype=float32)
        self.lims_up_joint_virtual = np.array(list(qlims_up_joint_pos_virtual.values()), dtype=float32)
        
        # orderRanges(qlims_down_joint_pos_virtual, qpOrder)
        # orderRanges(qlims_up_joint_pos_virtual, qpOrder)
        
        self.lims_down_joint_cs = np.array(list(orderRanges(qlims_down_joint_pos_virtual, qpOrder)), dtype=float32)
        self.lims_up_joint_cs = np.array(list(orderRanges(qlims_up_joint_pos_virtual, qpOrder)), dtype=float32)
        
        slaveIndexSocket = [1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]
        slaveIndexLabels = ['EBV', 'EBH', 'ER', 'ET',
                            'TR', 'RR', 'BR',
                            'TL', 'RL', 'BL', 'AL']
        dicSlaveIndex = dict(zip(slaveIndexLabels, slaveIndexSocket))
        
        self.configFile = 'module_main/model_utilities/datafolders.txt'
        self.typeCondition = typeCondition_
        
        # if typeCondition_=='real':
        qpDicRange = '/module_main/dicts/dofDictRange-qp.txt'
        dict_ = {}
        with open(os.getcwd()+qpDicRange) as f:
            for line in f:
                (key, val) = line.split("=")
                dict_[key] = val
        
        qpkeysMin = [s for s in list(dict_.keys()) if "min" in s]
        qpkeysMax = [s for s in list(dict_.keys()) if "max" in s]
        
        diQPMin = {}
        for name_ in qpkeysMin:
            exec("%s = %f" % ('qp'+name_,float(dict_[name_])))
            diQPMin[name_] = float(dict_[name_])
        diQPMax = {}
        for name_ in qpkeysMax:
            exec("%s = %f" % ('qp'+name_,float(dict_[name_])))
            diQPMax[name_] = float(dict_[name_])
        
        self.lims_down_joint_console = np.array(
            orderRanges(diQPMin, [s+'min' for s in qpOrder]), dtype=float32)
        self.lims_up_joint_console = np.array(
            orderRanges(diQPMax, [s+'max' for s in qpOrder]), dtype=float32)
        
        # Only if a real port is going to be used
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # Connect
            self.host = host
            self.port = port
            self.EASEfollower = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Error handlers
            self.errors = None
            self.errorPairing = False
            self.errorVelIDX = False
            self.errorSendMsg = False
            self.errorRecvMsg = False
            # Get state
            self.tick = 0
            self.controllerMode = 0
            self.TraceOn = 0
            self.CalibratedRobot = 0
            self.SecurityOnRobot = 0
            self.PowerOnRobot = 0
            self.DcOnRobot = 0
            # Variables from state
            self.axisEnable = []
            self.axisCalibrated = []
            self.axisUsed = []
            self.axisPastVal = []
            self.axisPastValNorm = []
            self.axisCurrVal = np.array([], dtype=float64)
            self.axisCurrValVel = np.array([])
            self.axisCurrValNormalized = np.array([], dtype=float64)
            self.axisReference = []
            self.axisTorque = []
            self.axisButtons = []
        
        # Axis infos
        self.NumberAxis = 0
        self.ActiveAxis = 0
        self.axisMins = []
        self.axisMaxs = []
        
        # For the coupling-decoupling visualization
        self.caseCoupled = []
        self.stateTriggets = []
        
        # HARDWARE maximum velocities
        # Philippe's comments
        # BR,BL : 33.31 mm/s
        # RR,RL : 201.13 deg/s
        # TR,TL : 54.28 mm/s
        # EB1, EB2 : 17.84 mm/s
        # ER : 10.76 deg/s
        # ET : 100.60 mm/s
        # LIMITS PER STEP
        # pre_h_ = np.array([  # MAX AND MIN
        #             0.01745, np.radians(1), 70/self.dt,
        #             0.0085, 0.0085, np.radians(1), 60/self.dt,
        #             0.01745, np.radians(1), 70/self.dt,
        #             0.0085, 0.0085, np.radians(1), 60/self.dt])
        self.safety_factor = 0.8
        self.q_dot_lim_seg = {0: 0.0085*2*2,  # Bending in curvature
                        1: np.radians(200),  # Rotation in degrees
                        2: 54,  # Translation in mm
                        3: 0.0085*2*2,  # Bending in curvature
                        4: np.radians(200),  # Rotation in degrees
                        5: 54,  # Translation in mm
                        6: 0.085*0.08,  # Horizontal bending ENDO in mm
                        7: -0.085*0.08,   # Vertical bending ENDO in mm
                        8: np.radians(30),  # Rotation ENDO in degrees
                        9: 100  # Translation ENDO in mm
                        }
        self.q_dot_lim_segFactor = dict(
            zip(self.q_dot_lim_seg.keys(),list(np.array(list(self.q_dot_lim_seg.values()))*1.1)))
        self.max_step_q = dict(
            zip(self.q_dot_lim_seg.keys(),list(np.array(list(self.q_dot_lim_seg.values()))*0.01)))
        self.q_bound_min = [
            (-np.pi/(18.5*2))*0.83,-163,24,
            (-np.pi/(18.5*2)),-163,24,
            (-np.pi/(185*2)), (-np.pi/(185*2)), -163, -500]
        self.q_bound_max = [
            (np.pi/(18.5*2)),163,65,
            (np.pi/(18.5*2)),163,65,
            (np.pi/(185*2)), (np.pi/(185*2)), 163, 500]
        
        # For internal usage (replace with reference)
        self.cycle = 0
        # For qp output
        self.preliminariesForCompute = False
        self.past_q = np.array([], dtype=float32)
        self.current_q_normalized = np.array([], dtype=float32)
        self.optStateType = 'vel'  # by defaul agent control - velocity based
        self.flagInitializeModel = False
        self.modelsComputedFlag = False
        # For reference
        self.axisNames = slaveIndexLabels
        self.axisIdxNames = slaveIndexSocket
        # For visualization purposes
        self.initializeRenderFlag = False
        self.initializeModelFlag = False
        # For debugging QP
        self.qp_value = 0
        self.Tr_e_follower = 0
        self.Jr_e_follower = 0
        self.Tl_e_follower = 0
        self.Jl_e_follower = 0
        self.Tr_0_follower = 0
        self.Jr_0_follower = 0
        self.Tl_0_follower = 0
        self.Jl_0_follower = 0
        self.Te_follower = 0
        self.Je_follower = 0
        self.state = 'S0'
        self.IR_e = 0
        self.I_e = 0
        self.MR_e = 0
        self.M_e = 0
        
        # Flags
        self.flag_virtualLimits = False
        self.limitFlagVirtual = False
        self.allDofCoupledFlag = True
        self.targetAchieved = False
        self.flagInTeleoperation = False
        # self.secondsInTarget = 0
        self.controlMode = 0
        self.timeOutError = False
        self.visualAid_coupling_flag = ''
        
        # For experiments
        self.lastQvisualized = [
            0.0085, np.radians(0), 60,  # Right 66.5
            0.0085, np.radians(0), 30,  # Left NOT VISIBLE
            0.000000001, 0.000000001, 0, 0]   # Endoscope
        
        # For adquiring in configuration space order and ranges the current q
        self.q2teleLeader_configSpace = [0]*len(qpOrder)

    # --------------------------------------------------------------------------
    # ------------------------ Miscellaneous communication commands
    # --------------------------------------------------------------------------
    def setTimeOut(self, tx):
        """Sets timeout to the initialized STRAS slave node

        Args:
            tx (float): Number of seconds to wait on the socket
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # ------------------------ No expected message reply
            self.EASEfollower.settimeout(float(tx))

    def connect(self, printOpt_=True):
        """Manually connects to the STRAS socket with the previously defined
        configuration

        Args:
            printOpt_ (bool, optional): To print outputs. Defaults to True.
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # ------------------------ No expected message reply
            if self.EASEfollower.connect_ex((self.host, self.port)) == 0:
                if printOpt_:
                    print('Succesful connection!')
                    print('hostname:\t', self.host)
                    print('port:\t\t', self.port)
            else:
                self.errorPairing = True

    def closeSocket(self):
        """To close the socket after usage
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # ------------------------ No expected message reply
            self.EASEfollower.close()

    def onCMD(self, printOpt_=True):
        """Turn on the CMD capapibility on the STRAS socket, by default turned
        OFF
        Args:
            printOpt_ (bool, optional): To print verbose. Defaults to True.

        Returns:
            bytes : Byte message to be unpacked
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            data = self.sendCMDmsg2STRAS(msgtype2send=1,
                                        replyMode=1,
                                        xtraV='',
                                        printOpt=printOpt_)
            self.cycle = self.cycle + 1
            return data

    def offCMD(self, printOpt_=True):
        """Turn Off the CMD capapibility on the STRAS socket, by default turned
        OFF

        Args:
            printOpt_ (bool, optional): To print verbose. Defaults to True.

        Returns:
            bytes : Byte message to be unpacked
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            data = self.sendCMDmsg2STRAS(msgtype2send=2,
                                        replyMode=1,
                                        xtraV='',
                                        printOpt=printOpt_)
            self.cycle = self.cycle + 1
            return data

    def stopCMD(self, printOpt_=True):
        """Stop the CMD capapibility on the STRAS socket, by default turned
        OFF. REDUNDANT

        Args:
            printOpt_ (bool, optional): To print verbose. Defaults to True.

        Returns:
            bytes : Byte message to be unpacked
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            data = self.sendCMDmsg2STRAS(msgtype2send=8,
                                        replyMode=1,
                                        xtraV='',
                                        printOpt=printOpt_)
            self.cycle = self.cycle + 1
            return data

    def resetCMD(self, printOpt_=True):
        """reset the CMD capapibility on the STRAS socket, by default turned
        OFF. REDUNDANT

        Args:
            printOpt_ (bool, optional): To print verbose. Defaults to True.

        Returns:
            bytes : Byte message to be unpacked
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            data = self.sendCMDmsg2STRAS(msgtype2send=1000,
                                        replyMode=1,
                                        xtraV='',
                                        printOpt=printOpt_)
            self.cycle = self.cycle + 1
            return data

    def passCurrentq(self, qTwoPass_):
        self.current_q_follower = qTwoPass_

    def updateScalingMaxVelFactor(self, factor_):
        """Update the scaling factor for maximum velocities on hardware

        Args:
            factor_ (float): Scaling factor to use w.r.t. the maximum velocities
        """
        # self.safety_factor = factor_
        self.q_dot_lim_segFactor = dict(
            zip(self.q_dot_lim_seg.keys(),list(np.array(list(self.q_dot_lim_seg.values()))*factor_)))

    def checkQinrange(self, q_):
        """Check if the next q to simulate in the virtual or real environment is feasible given the hardware constraints.

        Args:
            q (list): List of configuration variables in configuration space order (qr,ql,qe)

        Returns:
            list: boolFlag: True if the q is feasible, False otherwise.
                  listOfBools: List of the variables outside of range.
        """
        limsDown_ = self.lims_down_joint
        limsTop_ = self.lims_up_joint
        q2check = q_
        if self.typeCondition == 'real' or self.typeCondition=='REAL':
            if len(limsDown_) == len(q2check):
                boolResList, boolFlag_ = checkInRange(q2check, limsDown_, limsTop_)
                return [boolFlag_, boolResList]
            else:
                print('Check size of q2check')
        elif self.typeCondition == 'virtual' or self.typeCondition=='VIRTUAL':
            if len(limsDown_) == len(q2check):
                boolResList, boolFlag_ = checkInRange(q2check, self.lims_down_joint_cs, self.lims_up_joint_cs)
                return [boolFlag_, boolResList]
        else:
            print('Check size of q2check')

    def setCouplingInfo(self, allDofCoupled, decoupledList, active_flag, fsmState):
        """Pass information about the coupling of virtual follower and real leader.

        Args:
            allDofCoupled (bool): True if all DOF are coupled.
            decoupledList (list): List of DOF not coupled.

        """
        self.allDofCoupledFlag = allDofCoupled
        self.dofDeCoupled = decoupledList
        self.render_teleop_exp.defineCoupling_fromExpScript(
            allDofCoupled, decoupledList, active_flag, fsmState)
        
    def passTargetAchieved(self, secs_):
        """Inderectly pass boolean flag if target was achieved and the number of seconds ocurred since last target.

        Args:
            secs_ (float): Seconds passed since the last time target was achieved.
        """
        self.targetAchieved = True
        self.secsSinceTargetAchieved = secs_

    def passTargetNotAchieved(self):
        """Pass boolean flag of target not achieved and reset the counter.
        """
        self.targetAchieved = False
        self.secsSinceTargetAchieved = 0

    def passControlMode(self, controlMode_):
        self.controlMode = controlMode_

    # --------------------------------------------------------------------------
    # ------------------------ For experiment - TESTING
    # --------------------------------------------------------------------------

    def activateVirtualBounds(self):
        """To activate virtual limits on joint values. Calls filterConfigurationState eachtime after passCurrentQ
        """
        self.flag_virtualLimits = True

    def deactivateVirtualBounds(self):
        """To activate virtual limits on joint values. Calls filterConfigurationState eachtime after passCurrentQ
        """
        self.flag_virtualLimits = False

    def redefineBounds(self, q_min_, q_max_):
        """To define the bounds used for limiting the bounds.

        Args:
            q_min (list): Configuration variables minimum
            q_max (list): Configuration variables maximum
        """
        self.q_bound_min = q_min_
        self.q_bound_max = q_max_

    def filterConfigurationState(self, q_):
        """Filter the current Q state to bound within limits defined at initialization.

        Args:
            q_ (list): Configuration variable q.

        Returns:
            list: Updated q based on virtual limits.
        """
        # curr_q = self.current_q_follower.copy()
        curr_q = q_
        # new_q = self.current_q_follower.copy()
        new_q = q_
        if (len(self.lims_up_joint_cs)!=len(curr_q)) or (len(self.lims_down_joint_cs)!=len(curr_q)):
            print('UNMATCHED SIZE')
        else:
            for ix, _ in enumerate(q_):
                if (ix!=1) and (ix!=4) and (ix!=8):
                    if q_[ix] < self.lims_down_joint_cs[ix]:
                        new_q[ix] = self.lims_down_joint_cs[ix]
                    elif q_[ix] > self.lims_up_joint_cs[ix]:
                        new_q[ix] = self.lims_up_joint_cs[ix]
            # For updating the current q in the class
            # self.current_q_follower = new_q
        return new_q

    def defineVisualityLimits(self, maxVertical_, minVertical_, maxHorizontal_):
        """To set the min-max limits on the image space. Normalized values [0,1]

        Args:
            maxVertical (float): Maximum vertical value for the right arm
            minVertical (float): Minimum vertical value for the right arm
            maxHorizontal_ (float): Maximum horizontal value for the right arm

        Returns:
            _type_: _description_
        """
        self.maxVertical_ = maxVertical_
        self.minVertical_ = minVertical_
        self.maxHorizontal_ = maxHorizontal_

    # --------------------------------------------------------------------------
    # ------------------------ SENDING messages
    # --------------------------------------------------------------------------
    def sendCMDmsg2STRAS(self, msgtype2send, replyMode, xtraV, printOpt=False):
        """Low level send message to STRAS socket. Message locked to 33 bytes.

        Args:
            msgtype2send (int): Check the dictionary of message type.
            replyMode (int): If a answer is requested, 0 or 1.
            xtraV (str): If extended state or axis infos is requested.
            printOpt (bool, optional): Verbose. Defaults to False.

        Returns:
            bytes: If no error has been found, else returns 0
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # ------------------------ Send a CMD message to the STRAS platform
            # ------------------------ see commandByCMD dictionary
            mesType = 0
            if printOpt:
                print('Sending message of type:\t', messageTypeDic[mesType])
            # ------------------------ Build message
            m2stras_syn = bytearray([0x21, 0x2a, 0x2a, 0x21])  # Synchro 0:4
            m2stras_1 = bytearray([0x21, 0x0, 0x0, 0x0])  # MessageLength 4:8
            m2stras_2 = int(0).to_bytes(4, "little", signed=False)  # MessType 8:12
            m2stras_3 = bytearray([0x1, 0x0, 0x0, 0x0])  # MessageVersion 12:16
            m2stras_4 = int(0).to_bytes(4, "little",
                                        signed=False)  # MsgRfrnce16:20
            m2stras_5 = int(replyMode).to_bytes(4, "little",
                                                signed=False)  # MesReplyMode 20:24
            m2stras_6 = int(msgtype2send).to_bytes(8, "little",
                                                signed=False)  # Message 24:32
            if xtraV != '':
                m2stras_7 = int(xtraV).to_bytes(4, "little", signed=False)
                pre_msg2stras = m2stras_syn + m2stras_1 + m2stras_2 + m2stras_3 \
                    + m2stras_4 + m2stras_5 + m2stras_6 + m2stras_7
            else:
                pre_msg2stras = m2stras_syn + m2stras_1 + m2stras_2 + m2stras_3 \
                    + m2stras_4 + m2stras_5 + m2stras_6

            # ------------------------ Do checksum
            if self.cycle < 0:
                m2stras_checksum = bytearray([0x1])
            else:
                # print('Chcksm:\t', sum(pre_msg2stras), sum(pre_msg2stras) % 256)
                m2stras_checksum = int(sum(pre_msg2stras) % 256
                                    ).to_bytes(1, "big", signed=False)
            msg2stras = pre_msg2stras + m2stras_checksum  # Message + checksum
            try:
                self.EASEfollower.sendall(msg2stras)  # Send message
            except Exception:
                self.errorSendMsg = True
            try:
                data = self.EASEfollower.recvmsg(4096)
                return data
            except socket.timeout:
                return 'err:timeout'
                self.timeOutError = True
            except Exception:
                self.errorRecvMsg = True
                print('Couldnt receive data! - INCOMPATIBLE MESSAGE')
                return 0

    def send_VELIDX(self, axes_vals, printOpt_):
        """Adds constant velocitity to the STRAS slave DOF until set to 0 or
        reach limits.

        Args:
            axes_vals (list): List of list of the velocities to be sent to the
            STRAS slave socket [[axis_number0, velocity to send0],
            [axis_number1, velocity to send1], ...].
            printOpt_ (bool): Verbose on terminal
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            if self.port == 6666 or self.port == '6666':
                # ------------------------ Send a CMD message to the STRAS platform
                # ------------------------ see commandByCMD dictionary
                print('***SENDING VELOCITY***')
                num_axis = len(axes_vals)
                mesType = 0
                replyMode = 0
                mssLen = 28 + 4 + 1 + num_axis*8 + num_axis*2
                # 24 for the first 6 bytes (Synchro, Length, Type, Version,
                # Reference and Reply mode) + 1 Byte Checksum
                # + 4 (command) + 8*number of axis (double value)
                # + 2*number of axis(short) + 4 CMD_MESSAGE
                if printOpt_:
                    print('Sending message of type:\t', messageTypeDic[mesType],
                        'with len', mssLen)
                # ------------------------ Build message
                m2stras_syn = bytearray([0x21, 0x2a, 0x2a, 0x21])  # Synchro 0:4
                m2stras_1 = int(mssLen).to_bytes(4, "little",
                                                signed=False)  # MessageLength 4:8
                m2stras_2 = int(0).to_bytes(
                    4, "little", signed=False)  # MessType 8:12
                m2stras_3 = bytearray([0x1, 0x0, 0x0, 0x0])  # MessageVersion 12:16
                m2stras_4 = int(0).to_bytes(4, "little",
                                            signed=False)  # MsgRfrnce 16:20
                m2stras_5 = int(replyMode).to_bytes(
                    4, "little", signed=False)  # MesReplyMode 20:24
                m2stras_6 = int(52).to_bytes(4, "little",
                                            signed=False)  # MessageType 24:28
                m2stras_7 = int(num_axis+num_axis*8+1).to_bytes(
                    4, "little", signed=False)  # Lenght of bytes sent 28:32
                num_axis_byte = int(num_axis).to_bytes(1, "little",
                                                    signed=False)  # Idx 32
                pre_msg2stras = m2stras_syn + m2stras_1 + m2stras_2 + m2stras_3 \
                    + m2stras_4 + m2stras_5 + m2stras_6 + m2stras_7 + num_axis_byte
                for i in range(num_axis):
                    ax_ = int(axes_vals[i][0]).to_bytes(1,
                                                        "little",
                                                        signed=False)  # Index 33
                    vel_ = pack('<d', axes_vals[i][1])  # Value 34:42
                    pre_msg2stras = pre_msg2stras + ax_ + vel_
                    if printOpt_:
                        print('Axes ', axes_vals[i][0],
                            ' increasing by: ', axes_vals[i][1])
                # ------------------------ Do checksum
                if self.cycle < 0:
                    m2stras_checksum = bytearray([0x1])
                else:
                    m2stras_checksum = int(sum(pre_msg2stras) % 256
                                        ).to_bytes(1, "big", signed=False)
                msg2stras = pre_msg2stras + m2stras_checksum  # Message + checksum
                if printOpt_:
                    print('Len of MSG:\t'+str(int(len(msg2stras)))+'\n',
                        msg2stras)

                try:
                    self.EASEfollower.sendall(msg2stras)  # Send message
                    print('***\tVELOCITY SENT\t***')
                except self.EASEfollower.timeout:
                    print ('Timeout Error')
                except Exception:
                    self.errorSendMsg = True
                    self.errors = 'ErrorVelIDX'
                    print('!!!!!!\tCOULD NOT SENT\t!!!!!!')
                self.cycle = self.cycle + 1
                # try:
                #     data = self.EASEfollower.recvmsg(4096)
                #     return data
                # except socket.timeout:
                #     print('Timeout error!')
                # except Exception:
                #     self.errorRecvMsg = True
                #     print('Couldnt receive data! - UNKNOWN')
                #     return 0
            else:
                print('USING SLAVE, CHANGE PORT TO 6666 PLEASE!')

    def send_GoToPosPerAxis(self, listOfListByAxes_, printOpt_):
        """Send a comand to reach certain joint position. Low level controller handles
        separetly each degree of freedom, meaning only 1 DOF moves at a time.

        Args:
            listOfListByAxes_ (list): List of list of positions to be set by
            each DOF. [[axis_number0, axis_position0, axis_rel-speed0],
            [axis_number1, axis_position1, axis_rel-speed1]]
            printOpt_ (bool): Verbose.
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            if self.port == 6666 or self.port == '6666':
                # ------------------------ Send a CMD message to the STRAS platform
                # ------------------------ see commandByCMD dictionary
                # ------------------------ listOfListByAxes[axis, pos, rl_max_vel]
                if printOpt_:
                    print('***GO TO POS***')
                num_axis = len(listOfListByAxes_)
                mesType = 0
                replyMode = 0
                mssLen = 28 + 4 + 1 + 1*num_axis + 8*num_axis + 8*num_axis + 1
                # 28 'syncr','len','type','vrsion','refnce','rplymode','cmndCOMAND'
                # 4  'lenOfInfo'
                # 1  'numIndex'
                # 1  'index'
                # 8  'maxVel'
                # 8  'position'

                if printOpt_:
                    print('Sending message of type:\t', messageTypeDic[mesType],
                        'with len', mssLen)
                # ------------------------ Build message
                m2stras_syn = bytearray([0x21, 0x2a, 0x2a, 0x21])  # Synchro 0:4
                m2stras_1 = int(mssLen).to_bytes(
                    4, "little", signed=False)  # MessageLength 4:8
                m2stras_2 = int(0).to_bytes(
                    4, "little", signed=False)  # MessType 8:12
                m2stras_3 = bytearray([0x1, 0x0, 0x0, 0x0])  # MessageVersion 12:16
                m2stras_4 = int(0).to_bytes(
                    4, "little", signed=False)  # MsgRfrc 16:20
                m2stras_5 = int(replyMode).to_bytes(
                    4, "little", signed=False)  # MesReplyMode 20:24
                m2stras_6 = int(42).to_bytes(
                    4, "little", signed=False)  # MessageType 24:28
                m2stras_7 = int(num_axis+num_axis*8*2+1).to_bytes(
                    4, "little", signed=False)  # Len of CMD commands 28:32
                num_axis_byte = int(num_axis).to_bytes(1, "little",
                                                    signed=False)  # Idx 33
                pre_msg2stras = m2stras_syn + m2stras_1 + m2stras_2 + m2stras_3 \
                    + m2stras_4 + m2stras_5 + m2stras_6 + m2stras_7 + num_axis_byte
                for i in range(num_axis):
                    ax_ = int(listOfListByAxes_[i][0]).to_bytes(
                        1, "little", signed=False)  # Axs 1Byte
                    pos_ = pack('<d', listOfListByAxes_[i][1])  # Value 8Bytes
                    max_vel_ = pack('<d', listOfListByAxes_[i][2])  # Value 8Bytes
                    pre_msg2stras = pre_msg2stras + ax_ + pos_ + max_vel_
                    if printOpt_:
                        print('Axis ',
                            listOfListByAxes_[i][0],
                            ' moving to: ',
                            np.round(listOfListByAxes_[i][1], 2),
                            ' at a max relative velocity ',
                            np.round(listOfListByAxes_[i][2], 2))
                # ------------------------ Do checksum
                if self.cycle < 0:
                    m2stras_checksum = bytearray([0x1])
                else:
                    m2stras_checksum = int(sum(pre_msg2stras) % 256
                                        ).to_bytes(1, "big", signed=False)
                msg2stras = pre_msg2stras + m2stras_checksum  # Message + checksum
                if printOpt_:
                    print('Len of MSG:\t'+str(int(len(msg2stras)))+'\n',
                        msg2stras)

                try:
                    self.EASEfollower.sendall(msg2stras)  # Send message
                    if printOpt_:
                        print('***\tSENT GOTOIDX\t***')
                except self.EASEfollower.timeout:
                    print ('Timeout Error')
                    self.timeOutError = True
                except Exception:
                    self.errorSendMsg = True
                    self.errors = 'ErrorVelIDX'
                    print('!!!!!!\tCOULD NOT SENT\t!!!!!!')
                # try:
                #     data = self.EASEfollower.recvmsg(4096)
                #     return data
                # except socket.timeout:
                #     print('Timeout error!')
                # except Exception:
                #     self.errorRecvMsg = True
                #     print('Couldnt receive data! - UNKNOWN')
                #     return 0
            else:
                print('USING SLAVE, CHANGE PORT TO 6666 PLEASE!')

    def send_GoToPos(self, listOfListByAxes_, printOpt_):
        """Send a comand to reach certain position. Low level controller handles
        separetly each degree of freedom, meaning only 1 DOF moves at a time

        Args:
            listOfListByAxes_ (list): List of list of positions to be set by
            each DOF. [[axis_number0, axis_position0, axis_rel-speed0],
            [axis_number1, axis_position1, axis_rel-speed1]]
            printOpt_ (bool): Verbose
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            if self.port == 6666 or self.port == '6666':
                # ------------------------ Send a CMD to the STRAS platform
                # ------------------------ see commandByCMD dictionary
                # ------------------------ listOfListByAxes[axs, pos, rel_maxvel]
                if printOpt_:
                    print('***GO TO POS***')
                num_axis = len(listOfListByAxes_)
                mesType = 0
                replyMode = 0
                mssLen = 28 + 4 + 1 + 1*num_axis + 8*num_axis + 8*num_axis + 1
                # 28 'sync','len','type','vrsion','refnce','rplymode','cmdCOMAND'
                # 4  'lenOfInfo'
                # 1  'numIndex'
                # 1  'index'
                # 8  'maxVel'
                # 8  'position'

                if printOpt_:
                    print('Sending message of type:\t', messageTypeDic[mesType],
                        'with len', mssLen)
                # ------------------------ Build message
                m2stras_syn = bytearray([0x21, 0x2a, 0x2a, 0x21])  # Synchro 0:4
                m2stras_1 = int(mssLen).to_bytes(4, "little",
                                                signed=False)  # MsgLength 4:8
                m2stras_2 = int(0).to_bytes(
                    4, "little", signed=False)  # MessType 8:12
                m2stras_3 = bytearray([0x1, 0x0, 0x0, 0x0])  # MsgVersion 12:16
                m2stras_4 = int(0).to_bytes(
                    4, "little", signed=False)  # MsgRfrc 16:20
                m2stras_5 = int(replyMode).to_bytes(
                    4, "little", signed=False)  # MesReplyMode 20:24
                m2stras_6 = int(620).to_bytes(4, "little",
                                            signed=False)  # MessageType 24:28
                m2stras_7 = int(num_axis+num_axis*8*2+1).to_bytes(
                    4, "little", signed=False)  # Len of CMD commands 28:32
                num_axis_bt = int(num_axis).to_bytes(1, "little",
                                                    signed=False)  # Idx 33
                pre_msg2stras = m2stras_syn + m2stras_1 + m2stras_2 + m2stras_3 \
                    + m2stras_4 + m2stras_5 + m2stras_6 + m2stras_7 + num_axis_bt
                for i in range(num_axis):
                    ax_ = int(listOfListByAxes_[i][0]).to_bytes(
                        1, "little", signed=False)  # Axs 1Byte
                    pos_ = pack('<d', listOfListByAxes_[i][1])  # Value 8Bytes
                    max_vel_ = pack('<d', listOfListByAxes_[i][2])  # Value 8Btes
                    pre_msg2stras = pre_msg2stras + ax_ + pos_ + max_vel_
                    if printOpt_:
                        print('Axis ',
                            listOfListByAxes_[i][0],
                            ' moving to: ',
                            np.round(listOfListByAxes_[i][1], 2),
                            ' at a max relative velocity ',
                            np.round(listOfListByAxes_[i][2], 2))
                # ------------------------ Do checksum
                if self.cycle < 0:
                    m2stras_checksum = bytearray([0x1])
                else:
                    # print(
                    #     'Chcksm:',
                    #     sum(pre_msg2stras), sum(pre_msg2stras) % 256)
                    m2stras_checksum = int(sum(pre_msg2stras) % 256
                                        ).to_bytes(1, "big", signed=False)
                msg2stras = pre_msg2stras + m2stras_checksum  # Message + chcksum
                if printOpt_:
                    print('Len of MSG:\t'+str(int(len(msg2stras)))+'\n',
                        msg2stras)

                try:
                    self.EASEfollower.sendall(msg2stras)  # Send message
                    if printOpt_:
                        print('***\tSENT GOTOIDX\t***')
                except self.EASEfollower.timeout:
                    print ('Timeout Error')
                    self.timeOutError = True
                except Exception:
                    self.errorSendMsg = True
                    self.errors = 'ErrorVelIDX'
                    print('!!!!!!\tCOULD NOT SENT\t!!!!!!')
                # try:
                #     data = self.EASEfollower.recvmsg(4096)
                #     return data
                # except socket.timeout:
                #     print('Timeout error!')
                # except Exception:
                #     self.errorRecvMsg = True
                #     print('Couldnt receive data! - UNKNOWN')
                #     return 0
            else:
                print('USING SLAVE, CHANGE PORT TO 6666 PLEASE!')

    def send_GoToPos_byLabel(self, listOfListByAxes_lbl, printOpt_):
        """The same command as send_GoToPos but uses labels instead of axis
        index number.

        Args:
            listOfListByAxes_lbl (list): List of list containing the label of
            the DOF to control, position and maximum relative velocity.
            [[axis_label0, axis_position0, axis_rel-speed0],
            [axis_label1, axis_position1, axis_rel-speed1]]
            printOpt_ (bool): Verbose.
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            output_ = []
            for k_ in (listOfListByAxes_lbl):
                idx_ = dicSlaveIndex[k_[0]]
                pos_ = k_[1]
                max_rel_vel_ = k_[2]
                # For boundaries on the bending then inside the channel
                if idx_ == 'BR':
                    if self.axisCurrValNormalized[dicSlaveIndex['TR'] > 0.45]:
                        output_.append([idx_, pos_, max_rel_vel_])
                    else:
                        print('Right tool inside the channel....')
                        pass
                elif idx_ == 'BL':
                    if self.axisCurrValNormalized[dicSlaveIndex['TL'] > 0.45]:
                        output_.append([idx_, pos_, max_rel_vel_])
                    else:
                        print('Left tool inside the channel....')
                        pass
                elif idx_ != 'BR' and idx_ != 'BL':
                    # idxInSlave_ = [kk_ for kk_ in
                    #                range(len(slaveOrder))
                    #                if slaveOrder[kk_] == k_[0]][0]
                    # if ((self.axisCurrVal[idxInSlave_] <
                    #     self.axisMaxs[idxInSlave_]) and
                    #     (self.axisCurrVal[idxInSlave_] >
                    #      self.axisMins[idxInSlave_])):
                    output_.append([idx_, pos_, max_rel_vel_])
                    # else:
                    #     print('Requested position outside of boundaries:',
                    #           idx_, 'to', pos_)
                # if printOpt_:
                # print(
                #     'CMD2STRAS',
                #     k_[0],
                #     np.round(k_[1], 2),
                #     k_[2])
            self.send_GoToPos(output_, printOpt_)

    # --------------------------------------------------------------------------
    # ------------------------ RECEIVING/DECODING messages
    # --------------------------------------------------------------------------
    def decodeMsg(self, data, expectedReply, printOpt):
        """Decodes a byte message from a socket reply. Updates: axisCurrVal, axisCurrValNormalized, axisCurrValVel, axisPastVal, axisEnable, axisCalibrated, axisUsed, axisReference,

        Args:
            data (bytes): bytes received from the socket.
            expectedReply (int): If a reply is expected to decode a longer
            message.
            printOpt (bool): Verbose.
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # ------------------------ Decode a message dependant of the expected
            # ------------------------ reply from previos message sent
            # ------------------------ see commandByCMD dictionary
            if type(data) == 'int':
                print(data)
                print('^^ Check message received! ^^')
            else:
                r_STRAS = data[0]
                r_STRAS_syn = r_STRAS[:][:4].decode()  # Synchro
                r_STRAS_len = int.from_bytes(r_STRAS[4:8],
                                            "little", signed=False)  # Length
                r_STRAS_type = int.from_bytes(r_STRAS[8:12],
                                            "little", signed=False)  # Type
                # r_STRAS_mes_ver = int.from_bytes(r_STRAS[12:16],
                #                                  "little",
                #                                  signed=False)  # Version
                # r_STRAS_mes_ref = int.from_bytes(r_STRAS[16:20],
                #                                  "little",
                #                                  signed=False)  # Reference
                # r_STRAS_rep_mode = int.from_bytes(r_STRAS[20:24],
                #                                   "little",
                #                                   signed=False)  # RplyMode
                # ------------------------ If synchro is OFF breaks the loop
                if r_STRAS_syn == '!**!' and self.cycle == 0:
                    if printOpt:
                        print('Synchro OK ...')
                        print('INCOMING Message type',
                            messageTypeDic[r_STRAS_type],
                            'with len\t', r_STRAS_len-24-1, 'bytes')
                elif r_STRAS_syn != '!**!' and self.cycle == 0:
                    print('Check Synchro!')
                if expectedReply == 71:
                    EASEfollower.msgCMD_AXIS_infos(self, r_STRAS, printOpt)
                if expectedReply == 7:
                    EASEfollower.msgCMD_RECV_STATE(self, r_STRAS, printOpt)
                if expectedReply == 410:
                    EASEfollower.msgCMD_RECV_STATE(self, r_STRAS, printOpt)

    def msgCMD_AXIS_infos(self, r_STRAS, printOpt):
        """Decodes axis information message from the STRAS slave socket.

        Args:
            r_STRAS (bytes): A byte response from the socket, previously asked
            for axis infos.
            printOpt (bool): Verbose.
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # ------------------------ Decode AXIS infos message
            # ------------------------ Get axis infos
            if printOpt:
                print(commandByCMD[71], len(r_STRAS)-24-1)
                print('RAW LEN:\t', len(r_STRAS), 'bytes')
            # New main message unpack
            self.NumberAxis = int.from_bytes(
                r_STRAS[24:25], "little", signed=False)
            self.ActiveAxis = int.from_bytes(
                r_STRAS[25:26], "little", signed=False)
            if printOpt:
                print('NumberOfAxisRobot:\t', self.NumberAxis)
                print('NumberOfActiveAxisRobot:\t', self.ActiveAxis)
            init_axis_info = 26
            buffer_size = 17
            for i in range(0, self.NumberAxis):
                ix_1 = init_axis_info+(i*buffer_size)
                active_axis = int.from_bytes(r_STRAS[ix_1:ix_1+1],
                                            "little", signed=False)
                min_per_axis = unpack('<d', r_STRAS[ix_1+1:ix_1+9])
                max_per_axis = unpack('<d', r_STRAS[ix_1+9:ix_1+17])
                self.axisMins.append(min_per_axis[0])
                self.axisMaxs.append(max_per_axis[0])
                if printOpt:
                    print(slaveIndexLabels[i], '', slaveIndexSocket[i], '\t',
                        ('ACTIVE' if active_axis > 0 else 'INACTIVE'))
                    print('\tmin: ', min_per_axis)
                    print('\tmax: ', max_per_axis)

    def msgCMD_RECV_STATE(self, r_STRAS, printOpt, extendedState=False):
        """Receive a message from the STRAS robot, expected a state. Updates: axisCurrVal, axisCurrValNormalized, axisCurrValVel, axisPastVal, axisEnable, axisCalibrated, axisUsed, axisReference

        Args:
            r_STRAS (bytes): Byte response from requested a state.
            printOpt (bool): Verbose.
            extendedState (bool, optional): If an extended state is expected.
            Defaults to False.
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # ------------------------ Receive message depending on the expected
            # ------------------------ reply from previos message sent
            if printOpt:
                print(commandByCMD[7])
                # print('RAW LEN:\t', len(r_STRAS), 'bytes')
                print('MSG LEN:\t', len(r_STRAS)-24-1)
            self.tick = int.from_bytes(r_STRAS[24:32],
                                    "little",
                                    signed=False)  # Int64-8Bits
            self.controllerMode = int.from_bytes(r_STRAS[32:33], "little",
                                                signed=False)
            self.ContErr = int.from_bytes(r_STRAS[33:34], "little", signed=False)
            self.TraceOn = int.from_bytes(r_STRAS[34:35], "little", signed=False)
            self.CalibratedRobot = int.from_bytes(r_STRAS[35:36],
                                                "little", signed=False)
            self.SecurityOnRobot = int.from_bytes(r_STRAS[36:37], "little",
                                                signed=False)
            self.PowerOnRobot = int.from_bytes(r_STRAS[37:38], "little",
                                            signed=False)
            self.DcOnRobot = int.from_bytes(r_STRAS[38:39], "little",
                                            signed=False)
            self.NumberAxis = int.from_bytes(
                r_STRAS[39:40], "little", signed=False)
            self.ActiveAxis = int.from_bytes(
                r_STRAS[40:41], "little", signed=False)
            if printOpt:
                print('Controller Mode:\t', self.controllerMode)
                # print('Controller Mode:\t', r_STRAS_ContMode, '\t',
                #       controllerModeOps[r_STRAS_ContMode])
                print('NumberOfAxisRobot:\t', self.NumberAxis)
                print('NumberOfActiveAxisRobot:\t',
                    self.ActiveAxis)

            init_axis_info = 41
            buffer_size_axis = 27
            last_ix = init_axis_info
            if printOpt:
                print('Axis    \t AXmea  \t AXref  \t ME-REF  \tNorm')
            if self.cycle > 0:
                # self.axisPastVal = self.axisCurrVal.copy()
                # self.axisPastValNorm = self.axisCurrValNormalized.copy()
                # self.axisEnable = []
                # self.axisCalibrated = []
                # self.axisUsed = []
                self.axisCurrVal = np.array([], dtype=float32)
                # self.axisCurrValVel = np.array([], dtype=float32)
                self.axisCurrValNormalized = np.array([], dtype=float32)
                # self.axisReference = []
                # self.axisTorque = []
                # self.axisButtons = []
            for i in range(0, self.NumberAxis):
                ix_1 = last_ix+(i*buffer_size_axis)
                ax_enab = unpack('<?', r_STRAS[ix_1:ix_1+1])
                ax_cal = unpack('<?', r_STRAS[ix_1+1:ix_1+2])
                ax_used = unpack('<?', r_STRAS[ix_1+2:ix_1+3])
                ax_meas = unpack('<d', r_STRAS[ix_1+3:ix_1+3+8])
                ax_ref = unpack('<d', r_STRAS[ix_1+3+8:ix_1+3+8+8])
                # ax_torq = unpack('<d', r_STRAS[ix_1+3+8+8:ix_1+3+8+8+8])

                self.axisEnable.append(ax_enab[0])
                self.axisCalibrated.append(ax_cal[0])
                self.axisUsed.append(ax_used[0])
                # self.axisCurrVal.append(ax_meas[0])
                self.axisCurrVal = np.append(self.axisCurrVal, ax_meas[0])
                self.axisReference.append(ax_ref[0])
                # self.axisTorque.append(ax_torq[0])
                if (i == 4) or (i == 7):  # Invert: TR, TL
                    self.axisCurrValNormalized = np.append(
                        self.axisCurrValNormalized,
                        np.interp(ax_meas[0],
                                (self.axisMins[i],
                                self.axisMaxs[i]), (1, 0)))
                elif (i == 6):  # Invert: BR
                    self.axisCurrValNormalized = np.append(
                        self.axisCurrValNormalized,
                        np.interp(ax_meas[0],
                                (self.axisMins[i],
                                self.axisMaxs[i]), (1, 0)))
                else:
                    self.axisCurrValNormalized = np.append(
                        self.axisCurrValNormalized,
                        np.interp(ax_meas[0],
                                (self.axisMins[i],
                                self.axisMaxs[i]), (0, 1)))

                if i == self.NumberAxis-1:
                    last_ix = ix_1+3+8+8+8
                if printOpt:
                    print(slaveIndexLabels[i], '\t', slaveIndexSocket[i], '   \t',
                        np.round(ax_meas[0], 4),
                        '   \t', np.round(ax_ref[0], 2),
                        '   \t', np.round(ax_meas[0]-ax_ref[0], 2),
                        '   \t', np.round(self.axisCurrValNormalized[-1], 2))

            r_STRAS_NBut = int.from_bytes(r_STRAS[last_ix:last_ix+1],
                                        "little", signed=False)
            if printOpt:
                print('NumberOfButtons:\t', r_STRAS_NBut)
            last_ix = last_ix + 1
            for k in range(0, r_STRAS_NBut):
                ix_1 = last_ix+(k*1)
                buttonLvl = int(unpack('<?', r_STRAS[ix_1:ix_1+1])[0])
                self.axisButtons.append(buttonLvl)
                if k == r_STRAS_NBut-1:
                    last_ix = ix_1 + 1
            if self.cycle > 1 and len(self.axisButtons) > r_STRAS_NBut:
                self.axisButtons = self.axisButtons[-r_STRAS_NBut:]
            if printOpt and r_STRAS_NBut > 0:
                print('Buttons', self.axisButtons)
            r_STRAS_NumDDiom = int.from_bytes(r_STRAS[last_ix:last_ix+1],
                                            "little", signed=False)
            last_ix = last_ix + 1
            r_STRAS_DDiom = []
            for i in range(0, r_STRAS_NumDDiom):
                ix_1 = init_axis_info+(i*r_STRAS_NumDDiom)
                lvlDDio = int(unpack('<?', r_STRAS[ix_1:ix_1+1])[0])
                r_STRAS_DDiom.append(lvlDDio)
                if i == r_STRAS_NumDDiom - 1:
                    last_ix = ix_1+1
            # r_STRAS_Ngroups = int.from_bytes(r_STRAS[last_ix:last_ix+1],
            #                        git          "little", signed=False)  # Usable
            # print(self.axisCurrVal)
            last_ix = last_ix + 1
            # print('NumberOfGroups:\t', r_STRAS_Ngroups)

            # Keep only last records
            self.axisCurrVal = self.axisCurrVal[-12:]
            self.axisCurrValNormalized = self.axisCurrValNormalized[-12:]
            self.axisCurrValVel = self.axisCurrValVel[-12:]
            self.axisPastVal = self.axisPastVal[-12:]
            self.axisEnable = self.axisEnable[-12:]
            self.axisCalibrated = self.axisCalibrated[-12:]
            self.axisUsed = self.axisUsed[-12:]
            self.axisReference = self.axisReference[-12:]

            if extendedState:
                # r_STRAS_refTeleOp = unpack('<d', r_STRAS[last_ix:last_ix+8])
                last_ix = last_ix + 8
                # r_STRAS_ax_lck_dic = {0: 'UNLOCKED', 1: 'LOCKED', 2: 'ND'}
                # r_STRAS_ax_lck = int.from_bytes(r_STRAS[last_ix:last_ix+1],
                #                                 "little", signed=False)
            # print('Loop count:\t'+str(f+1))
            pass

    # --------------------------------------------------------------------------
    # ------------------------ EASY COMMANDS TO USE
    # --------------------------------------------------------------------------
    def getAxisInfo(self, printOpt_):
        """A second layer of the axis info sending/receiving of commands. Updates: axisCurrVal, axisCurrValNormalized, axisCurrValVel, axisPastVal, axisEnable, axisCalibrated, axisUsed, axisReference

        Args:
            printOpt_ (bool): Verbose.
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # ------------------------ To ease of use
            r_STRAS = self.sendCMDmsg2STRAS(msgtype2send=71,
                                            replyMode=1,
                                            xtraV='',
                                            printOpt=printOpt_)
            self.decodeMsg(r_STRAS, expectedReply=71, printOpt=printOpt_)
            self.cycle = self.cycle + 1
        elif self.typeCondition == 'virtual' or self.typeCondition == 'VIRTUAL':
            pass

    def getState(self, printOpt_):
        """A second layer of the axis info sending/receiving of commands. 
        Easier way to adquire a state. 
        Updates: axisCurrVal, axisCurrValNormalized, axisCurrValVel, axisPastVal, axisEnable, axisCalibrated, axisUsed, axisReference

        Args:
            printOpt_ (bool): Verbose.
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # ------------------------ To ease of use
            r_STRAS = self.sendCMDmsg2STRAS(msgtype2send=7,
                                            replyMode=1,
                                            xtraV='',
                                            printOpt=printOpt_)
            if r_STRAS == 'err:timeout':
                print('Timeout Error in Slave...\n')
                self.timeOutError = True
                pass
            elif type(r_STRAS) == tuple and len(r_STRAS) != 0:
                self.decodeMsg(r_STRAS, expectedReply=7, printOpt=printOpt_)
                self.timeOutError = False
                if self.port == 6667:
                    for i in range(len(self.axisCurrVal)):
                        if i in range(7):
                            # To update min and max JUST IN CASE - CALIBRATION
                            if self.axisCurrVal[i] > self.axisMaxs[i]:
                                self.axisMaxs[i] = self.axisCurrVal[i]
                            if self.axisCurrVal[i] < self.axisMins[i]:
                                self.axisMins[i] = self.axisCurrVal[i]
                    self.cycle = self.cycle + 1
            else:
                print('DATA RECEIVED NOT USEFUL!\n', r_STRAS)
        elif self.typeCondition == 'virtual' or self.typeCondition == 'VIRTUAL':
            print('Virtual socket! NO UPDATED STATE')
            pass

    def getStateInConfigSpace(self, printOpt_=False):
        """A second layer of the axis info sending/receiving of commands.
        Easier way to adquire a state.
        Updates: current_q, axisCurrVal, axisCurrValNormalized, axisCurrValVel, axisPastVal, axisEnable, axisCalibrated, axisUsed, axisReference

        Args:
            printOpt_ (bool): Verbose. For compatibility purposes.
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            # ------------------------ To ease of use
            self.getState(False)
            self.q2teleLeader_configSpace = map_norm2range(
                order_follower2configspace(self.axisCurrValNormalized),
                self.lims_down_joint,
                self.lims_up_joint)
        elif self.typeCondition == 'virtual' or self.typeCondition == 'VIRTUAL':
            pass

    def easyStart(self):
        """Easy command to start the socket.
        """
        if self.typeCondition == 'real' or self.typeCondition == 'REAL':
            self.setTimeOut(1.0)
            self.connect(True)
        elif self.typeCondition == 'virtual' or self.typeCondition == 'VIRTUAL':
            pass

    def passTriggerState(self, qTriggers_):
        """Pass the state of the triggers

        Args:
            qTriggers (list): Normalized information about the triggers.
        """
        self.stateTriggets = qTriggers_
    
    # --------------------------------------------------------------------------
    # ------------------------ For model computation
    # --------------------------------------------------------------------------
    def computeModels(self, q2compute, printOpt_=False):
        """To compute models, updates transformations and jacobians.

        Args:
            q2compute (list): q in configuration space order and ranges.
            printOpt_ (bool, optional): For verbose. Defaults to False.
        """
        if not(self.initializeModelFlag):
            self.initializeModels(self)
        strasOutput_follower = self.robot_stras.computemodels(q2compute.copy())
        self.Tr_e_follower = strasOutput_follower[0].copy()
        self.Jr_e_follower = strasOutput_follower[1].copy()
        self.Tl_e_follower = strasOutput_follower[2].copy()
        self.Jl_e_follower = strasOutput_follower[3].copy()
        self.Tr_0_follower = strasOutput_follower[4].copy()
        self.Jr_0_follower = strasOutput_follower[5].copy()
        self.Tl_0_follower = strasOutput_follower[6].copy()
        self.Jl_0_follower = strasOutput_follower[7].copy()
        self.Te_follower = strasOutput_follower[8].copy()
        self.Je_follower = strasOutput_follower[9].copy()
        self.Jzr_e = strasOutput_follower[10].copy()
        self.zr_e = strasOutput_follower[11].copy()

    def initializeModels(self,printOpt_=False):
        """To initialize models when only computing the models, DO NOT USE FOR VISUALIZATION.

        Args:
            printOpt_ (bool, optional): Verbose. Defaults to False.
        """
        d = {}
        with open(os.getcwd()+'/'+self.configFile) as f:
            for line in f:
                (key, val) = line.split("=")
                d[key] = val

        # Create camera and robot model
        cam = camera_model(
            os.getcwd()+'/module_main/'+d['camera_calibration'][:-1])
        robot_right = robot_model(
            configfile=os.getcwd()+'/module_main/model_utilities/'+d['robot_config_right'][:-1])
        robot_left = robot_model(
            configfile=os.getcwd()+'/module_main/model_utilities/'+d['robot_config_left'][:-1])
        robot_end = robot_twobending(
            configfile=os.getcwd()+'/module_main/model_utilities/'+d['robot_config_endoscope'][:-1])

        # Create the overall stras robot model
        # (to compute pose and jacobian of each robot)
        self.robot_stras = StrasModel(robot_right, robot_left, robot_end)

        # Create rendering to render robot shape
        w, h = cam.getImSize()
        self.render = render_shape(w, h, self.robot_stras)
        self.render.setCamProperties(cam.getF(), cam.getC(), cam.K)

        # Update the initialization flag
        self.initializeModelFlag = True

    # --------------------------------------------------------------------------
    # ------------------------ For visualization
    # --------------------------------------------------------------------------
    def visualizeState(self, printOpt_=False):
        """Function to visualize the current state, it changes from the state
        on master range and order to the control space.

        Args:
            printOpt_ (bool, optional): Verbose. Defaults to False.

        Returns:
            np.array: a numpy array with the rendered robot in the camera image
            plane
        """
        img = self.visualizeFrame(
            map_norm2range(
                order_follower2configspace(self.axisCurrValNormalized),
                self.lims_down_joint,
                self.lims_down_joint))
        return img

    def visualizeFrame(self, q2visualize, imRet=False, printOpt_=False):
        """Visualize the frame corresponding to q2visualize list-or-np.array in
        qp order and range.

        Args:
            q2visualize (list): List of DOF of the STRAS robot in qp form and
            in qp range.
            printOpt_ (bool, optional): Verbose. Defaults to False.

        Returns:
            np.array: a numpy array with the rendered robot in the camera image
            plane
        """
        boolFlag_, boolList_ = self.checkQinrange(q2visualize)
        if not(boolFlag_):
            if not(self.initializeRenderFlag):
                self.initializeVisualization(self)
            strasOutput_follower = self.robot_stras.computemodels(
                q2visualize.copy())
            self.Tr_e_follower = strasOutput_follower[0].copy()
            self.Jr_e_follower = strasOutput_follower[1].copy()
            self.Tl_e_follower = strasOutput_follower[2].copy()
            self.Jl_e_follower = strasOutput_follower[3].copy()
            self.Tr_0_follower = strasOutput_follower[4].copy()
            self.Jr_0_follower = strasOutput_follower[5].copy()
            self.Tl_0_follower = strasOutput_follower[6].copy()
            self.Jl_0_follower = strasOutput_follower[7].copy()
            self.Te_follower = strasOutput_follower[8].copy()
            self.Je_follower = strasOutput_follower[9].copy()
            self.Jzr_e = strasOutput_follower[10].copy()
            self.zr_e = strasOutput_follower[11].copy()
            if imRet:
                shpRslve, shapeLslave, tipend_follower = self.robot_stras.computeShape(
                    q2visualize.copy())
                img_ = self.render.render_stras_arms(
                shpRslve, shapeLslave, tipend_follower, self.Tr_e_follower,
                self.Tl_e_follower, self.Tr_0_follower, self.Tl_0_follower, self.Te_follower)
                return img_
            else:
                return 0
        else:
            print('System limits reached!!!', boolList_)
            self.limitFlagVirtual = True

    def initializeVisualization(self, printOpt_=False):
        """To initialize the visualization renders and else.

        Args:
            printOpt_ (bool, optional): Verbose. Defaults to False.
        """
        d = {}
        with open(os.getcwd()+'/'+self.configFile) as f:
            for line in f:
                (key, val) = line.split("=")
                d[key] = val

        # Create camera and robot model
        cam = camera_model(
            os.getcwd()+'/module_main/'+d['camera_calibration'][:-1])
        robot_right = robot_model(
            configfile=os.getcwd()+'/module_main/model_utilities/'+d['robot_config_right'][:-1])
        robot_left = robot_model(
            configfile=os.getcwd()+'/module_main/model_utilities/'+d['robot_config_left'][:-1])
        robot_end = robot_twobending(
            configfile=os.getcwd()+'/module_main/model_utilities/'+d['robot_config_endoscope'][:-1])


        # Create the overall stras robot model
        # (to compute pose and jacobian of each robot)
        self.robot_stras = StrasModel(robot_right, robot_left, robot_end)

        # Create rendering to render robot shape
        w, h = cam.getImSize()
        self.render = render_shape(w, h, self.robot_stras)
        self.render.setCamProperties(cam.getF(), cam.getC(), cam.K)

        # Update the initialization flag
        self.initializeRenderFlag = True


    # --------------------------------------------------------------------------
    # ------------------------ To control frame position
    # --------------------------------------------------------------------------
    def clean_matrices(self):
        """Clean variables
        """
        self.Tr_e_follower = 0
        self.Jr_e_follower = 0
        self.Tl_e_follower = 0
        self.Jl_e_follower = 0
        self.Tr_0_follower = 0
        self.Jr_0_follower = 0
        self.Tl_0_follower = 0
        self.Jl_0_follower = 0
        self.Te_follower = 0
        self.Je_follower = 0

    def clean_matrices_follower2(self):
        """Clean variables
        """
        self.Tr_e_follower2 = 0
        self.Jr_e_follower2 = 0
        self.Tl_e_follower2 = 0
        self.Jl_e_follower2 = 0
        self.Tr_0_follower2 = 0
        self.Jr_0_follower2 = 0
        self.Tl_0_follower2 = 0
        self.Jl_0_follower2 = 0
        self.Te_follower2 = 0
        self.Je_follower2 = 0