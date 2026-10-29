"""Static protocol and robot constants for the EASE/STRAS follower."""
from enum import IntEnum

class MessageType(IntEnum):
    CMD = 0
    ACK = 1
    TRACE = 2
    DEBUG = 3
    STATE = 4
    STATE_EXTENDED = 41
    AXIS_INFOS = 42
    SUPERVISOR_MESSAGE = 43
    TELEOP_INFOS = 44

class Command(IntEnum):
    NONE = 0
    ON = 1
    OFF = 2
    CALIBRATION = 3
    GO_TO = 4                          # one axis at a time
    REACH_END_POINT = 400
    VEL = 5
    IDX_VELOCITY = 52
    GO_TO_VEL = 6                      # debug
    IDX_GO_TO_VEL = 620                # use this one for position commands
    GET_STATE = 7
    STOP = 8
    DEBUG_FAULT = 9
    DEBUG_SET_POS = 41
    DEBUG_SET_POS_IDX = 410
    SEND_AXIS_INFOS = 71
    SEND_STATE_EXTENDED = 72
    SUPERIOR_MESSAGE = 73
    GET_CURTIME = 700
    RESET = 100
    # Not present in the original commandByCMD table, but used by
    # send_GoToPosPerAxis:
    GO_TO_PER_AXIS = 42



# --- Axes ---------------------------------------------------------------------
# Order of the 10 configuration variables used by the QP / kinematic models.
QP_ORDER = ('BR', 'RR', 'TR', 'BL', 'RL', 'TL', 'EBV', 'EBH', 'ER', 'ET')
# Order in which the slave reports its axes (and their socket indices).
SLAVE_AXIS_LABELS = ('EBV', 'EBH', 'ER', 'ET',
                     'TR', 'RR', 'BR',
                     'TL', 'RL', 'BL', 'AL')
SLAVE_AXIS_SOCKET = (1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12)
AXIS_SOCKET_INDEX = dict(zip(SLAVE_AXIS_LABELS, SLAVE_AXIS_SOCKET))
AXIS_POSITION = {label: i for i, label in enumerate(SLAVE_AXIS_LABELS)}
# Axes whose normalised value is inverted ([min, max] -> [1, 0]).
INVERTED_AXES = frozenset({'TR', 'TL', 'BR'})
# Rotation joints are not clipped by the virtual bounds filter.
ROTATION_JOINTS = ('RR', 'RL', 'ER')
# --- Ports / thresholds -------------------------------------------------------
COMMAND_PORT = 6666            # only this port accepts velocity / go-to commands
RANGE_TRACKING_PORT = 6667   # on this port the axis ranges auto-expand
CHANNEL_CLEARANCE_NORM = 0.45  # bending allowed once translation is past this
# bending axis -> translation axis that must be clear of the channel first
CHANNEL_GUARD = {'BR': 'TR', 'BL': 'TL'}



# --- Legacy lookup tables (kept for scripts that import them) ----------------
MESSAGE_TYPE_NAMES = {0: 'CMD', 1: 'ACK', 2: 'TRACE', 3: 'DEBUG', 4: 'STATE',
                      41: 'STATEEXTENDED', 42: 'AXISINFOS',
                      43: 'SUPERVISIORMESSAGE', 44: 'TELEOPINFOSTYPE'}
CONTROLLER_MODE_NAMES = {0: 'INIT', 1: 'ON UNCAL', 2: 'DO CAL',
                         3: 'GO TO INIT POS', 4: 'POS CONTROL',
                         5: 'POS PLANNER(DEBUG)', 6: 'VEL PLANNER(DEBUG)',
                         61: 'EMBEDDEDTELEOP', 62: 'MASTEREXTERNALTELEOP',
                         63: 'SLAVEEXTERNALTELEOP',
                         11: 'ETHERCAT BUS INITIALIZATION',
                         12: 'BUS DISCONNECTED'}
COMMAND_NAMES = {0: 'NONE', 1: 'CONTROLLER_CMD_ON', 2: 'CONTROLLER_CMD_OFF',
                 3: 'CONTROLLER_CMD_CALIBRATION', 4: 'CONTROLLER_CMD_GO_TO',
                 400: 'CONTROLLER_CMD_REACH_END_POINT',
                 5: 'CONTROLLER_CMD_VEL', 52: 'CONTROLLER_CMD_IDX_VELOCITY',
                 6: 'CONTROLLER_CMD_GO_TO_VEL',
                 620: 'CONTROLLER_CMD_IDX_GO_TO_VEL',
                 7: 'CONTROLLER_CMD_GET_STATE', 8: 'CONTROLLER_CMD_STOP',
                 9: 'CMD_DEBUG_FAULT', 41: 'DEBUG_CMD_SET_POS',
                 410: 'DEBUG_CMD_SET_POS_IDX',
                 71: 'CONTROLLER_CMD_SEND_AXIS_INFOS',
                 72: 'CONTROLLER_CMD_SEND_STATE_EXTENDED',
                 73: 'CONTROLLER_CMD_SUPERIOR_MESSAGE',
                 700: 'CONTROLLER_CMD_GET_CURTIME',
                 100: 'CONTROLLER_CMD_RESET'}