from sympy.geometry import Point
from sympy.geometry import Line
from sympy import Float
from math import tan, cos, sin, radians
from typing import Iterable 
import numpy as np
from math import isclose, sqrt
# from liegroups import SE3

followerOrder_local = [
    'EBV', 'EBH', 'ER', 'ET',
    'TR', 'RR', 'BR',
    'TL', 'RL', 'BL']
tolIsClosefollower_local = [
    0.5, 0.5, 2, 1,
    2, 2, 0.2,
    2, 2, 0.2]
tooCloseDict_local = dict(zip(followerOrder_local, tolIsClosefollower_local))
tooIsClosefollower_qp = [
    0.0005, 0.0005, 1, 0.2,
    0.0005, 1, 0.01,
    0.0005, 1, 0.01]
tooCloseDict_qp = dict(zip(followerOrder_local, tooIsClosefollower_qp))
maxRelativeDict = dict(zip(followerOrder_local, [
    0.1, 0.1, 0.1, 0.1,
    0.1, 0.3, 0.3,
    0.1, 0.3, 0.3]))
followerLabels_teleoperate_local = ['EBV', 'EBH', 'ET', 'ER']
followerOrder_teleoperate_local = [0.25, 0.25, 1, 1]
stepEndoTeleoperation = dict(
    zip(followerLabels_teleoperate_local, followerOrder_teleoperate_local))
qlims_down_follower_pos = {
    'BR': -6.5,  # deg/s
    'RR': -270,  # Rotation in degrees/s
    'TR': 2,  # Translation in mm/s
    'BL': -6.5,  # deg/s
    'RL': -270,  # Rotation in degrees/s
    'TL': 2,  # Translation in mm/s
    'EBH': -9.12,  # HorB-E
    'EBV': -9.12,  # VertB-E
    'ER': -90,  # Rotation ENDO in degrees/s
    'ET': 0.5}  # Translation ENDO in mm/s
qlims_top_follower_pos = {
    'BR': 6.5,  # deg/s
    'RR': 360,  # Rotation in degrees/s
    'TR': 73.5,  # Translation in mm/s
    'BL': 6.5,  # deg/s
    'RL': 360,  # Rotation in degrees/s
    'TL': 73.5,  # Translation in mm/s
    'EBH': 9.12,  # HorB-E
    'EBV': 9.12,  # VertB-E
    'ER': 50,  # Rotation ENDO in degrees/s
    'ET': 90}  # Translation ENDO in mm/s
qlims_down_joint_pos = {
    'BR': -np.pi/(2*18.5),  # deg/s
    'RR': radians(2),  # Rotation in degrees/s
    'TR': 2,  # Translation in mm/s
    'BL': -np.pi/(2*18.5),  # deg/s
    'RL': radians(2),  # Rotation in degrees/s
    'TL': 2,  # Translation in mm/s
    'EBH': -np.pi/(2*185),  # HorB-E
    'EBV': -np.pi/(2*185),  # VertB-E
    'ER': radians(2),  # Rotation ENDO in degrees/s
    'ET': 0}  # Translation ENDO in mm/s
qlims_top_joint_pos = {
    'BR': np.pi/(2*18.5),  # deg/s
    'RR': radians(180),  # Rotation in degrees/s
    'TR': 75,  # Translation in mm/s
    'BL': np.pi/(2*18.5),  # deg/s
    'RL': radians(180),  # Rotation in degrees/s
    'TL': 75,  # Translation in mm/s
    'EBH': np.pi/(2*185),  # HorBend-E
    'EBV': np.pi/(2*185),   # VertBnd_E
    'ER': radians(358),  # Rotation ENDO in degrees/s
    'ET': 100}  # Translation ENDO in mm/s
lims_down = np.array(list(qlims_down_joint_pos.values()))
lims_up = np.array(list(qlims_top_joint_pos.values()))
followerOrder = [
    'EBV', 'EBH', 'ER', 'ET',
    'TR', 'RR', 'BR',
    'TL', 'RL', 'BL']
leaderOrder = [
    'TL', 'RL', 'BL', 'AL',
    'TR', 'RR', 'BR', 'AR',
    'EBV', 'EBH', 'ET', 'ER']
qpOrder = [
    'BR', 'RR', 'TR',
    'BL', 'RL', 'TL',
    'EBV', 'EBH', 'ER', 'ET']

# ------------------------------------------------------------------------------
# ------------------------ RANDOM
# ------------------------------------------------------------------------------
# To count consecutive numbers in a list
def ranges(nums):
    nums = sorted(set(nums))
    gaps = [[s, e] for s, e in zip(nums, nums[1:]) if s+1 < e]
    edges = iter(nums[:1] + sum(gaps, []) + nums[-1:])
    return list(zip(edges, edges))

# Flatten nested list of list
def flatten(items):
    """Yield items from any nested iterable; see Reference."""
    for x in items:
        if isinstance(x, Iterable) and not isinstance(x, (str, bytes)):
            for sub_x in flatten(x):
                yield sub_x
        else:
            yield x

# Generate top arc coordinates for a parametric circle 
def topArc(x0, y0, r, init_angl, angleOfArc):
    """_summary_

    Args:
        x0 (_type_): _description_
        y0 (_type_): _description_
        r (_type_): _description_
        init_angl (_type_): _description_
        angleOfArc (_type_): _description_

    Returns:
        _type_: _description_
    """
    x = []
    y = []
    for i in range(angleOfArc):
        angle = init_angl+i
        x.append( x0 + r*cos(radians(angle)))
        y.append( y0 + r*sin(radians(angle)))
    return x, y

# To check if the list is within the min and max, return list of errors
def checkInRange(list_, mins_, maxs_):
    """Check if a list is within the mins and maxs ranges.

    Args:
        list_ (list): List of values
        mins_ (list): List of mins
        maxs_ (list): List of mins

    Returns:
        list: result, flag 
    """
    res = []
    flag_ = False
    for i in range(len(list_)):
        a = []
        if list_[i] < mins_[i]:
            a.append('min')
            flag_ = True
        if list_[i] > maxs_[i]:
            a.append('max')
            flag_ = True
        res.append(a)
    return res, flag_

def filterQendoLeader(qin_):
    """Filter the variables of the leader (endo) to binarize them. 

    Args:
        qin_ (list): Variables of the leader joysticks.

    Returns:
        list: Binarized array.
    """
    q_in_binary = []
    for i in range(len(qin_)):
        if qin_[i]>0.75:
            q_in_binary.append(1)
        elif qin_[i]<(-0.75):
            q_in_binary.append(-1)
        else:
            q_in_binary.append(0)
    return q_in_binary

def SwitchHandle(qin_):
    """Switch right joystick handle to left and viceversa. 

    Args:
        qin_ (list): list of q values

    Returns:
        list: updated list of q values
    """
    qout_ = [qin_[3], qin_[2], qin_[1], qin_[0]]
    return qout_

def noSwitchHandle(qin_):
    """Switch right joystick handle to left and viceversa. 

    Args:
        qin_ (list): list of q values

    Returns:
        list: updated list of q values
    """
    # qout_ = [qin_[3], qin_[2], qin_[1], qin_[0]]
    return qin_

# Generate random point to reach
def generateRandomPoint(currentTipRPoint):
    """Generate a random point outside of the limits of the reachibility in the relative body frame

    Args:
        currentTipRPoint (list): Current position of the target point used as goal

    Returns:
        list: New location of the target point in the relative body frame
    """
    point_ = np.array([0, 0, 0])
    # point_[0] = np.random.uniform(15, 30)  # X value range
    # point_[1] = currentTipRPoint[1]  # No change in the Y value
    # point_[2] = 70 + np.random.uniform(5, 15)  # Depth in Z
    # sel_ = np.random.uniform(10)
    # if sel_ > 7.5:  # depth
    offsetOnHeight = 5
    sel2_ = np.random.uniform(10)

    if sel2_ >= 5:  # from far
        point_[0] = int(np.random.uniform(0, 30))  # X value range
        point_[1] = int(np.random.uniform(5, 10))  # No change in the Y value
        point_[2] = int(np.random.uniform(75, 95))  # Depth in Z
    else:  # from close
        point_[0] = int(np.random.uniform(8, 30))  # X value range
        point_[1] = int(np.random.uniform(5, 10))  # No change in the Y value
        point_[2] = int(np.random.uniform(25, 35))  # Depth in Z
    # elif sel_ > 5:  # leftWS
    #     point_[0] = int(np.random.uniform(-10, 0))  # X value range
    #     point_[1] = currentTipRPoint[1]  # No change in the Y value
    #     point_[2] = int(np.random.uniform(40, 65))  # Depth in Z
    # else:
    #     #  sel_ > 2.5:  # rightWS
    #     point_[0] = int(np.random.uniform(40, 70))  # X value range
    #     point_[1] = currentTipRPoint[1]  # No change in the Y value
    #     point_[2] = int(np.random.uniform(40, 65))  # Depth in Z
    # else:
    #     point_[0] = int(np.random.uniform(-10, 0))  # X value range
    #     point_[1] = currentTipRPoint[1]  # No change in the Y value
    #     point_[2] = int(np.random.uniform(40, 65))  # Depth in Z

    return point_

# Generate random point to reach
def generateRandomPoint_v2(currentTipRPoint):
    """Extract point from the list of position to used in the experiment.

    Args:
        currentTipRPoint (list): 3D position of the right arm

    Returns:
        list: [Target position of the point, List of intented states to trigger]
    """
    point_ = np.array([0, 0, 0])
    sel2_ = np.random.uniform(15)
    listOfSel = []
    if sel2_ > 5:  
        val1_ = int(np.random.uniform(-20, 25))  # No change in the Y value# from far
        point_[1] = val1_
        point_[0] = abs(val1_)  # X value range
        point_[2] = int(75 + np.random.uniform(-5, 15))  # Depth in Z
        listOfSel.append('S_optimal_reach_pos')
    elif sel2_ > 10: 
        val1_ = int(np.random.uniform(-20, 25))  # No change in the Y value # from close
        point_[1] = val1_
        point_[0] = abs(val1_)  # X value range
        point_[2] = int(np.random.uniform(25, 31))  # Depth in Z
        listOfSel.append('S_optimal_reach_neg')
    elif sel2_ > 15:  # leftWS
        point_[0] = -int(np.random.uniform(-20, 10))  # X value range
        point_[1] = int(np.random.uniform(5, 10))  # No change in the Y value
        point_[2] = int(np.random.uniform(40, 60))  # Depth in Z
        listOfSel.append('S_left_WS')
    else:
        val1_ = int(np.random.uniform(-20, 25))  # No change in the Y value
        point_[1] = val1_
        point_[0] = abs(val1_)  # X value range
        point_[2] = int(np.random.uniform(40, 65))  # Depth in Z
        listOfSel.append('S_right_WS')
    # else:
    #     point_[0] = int(np.random.uniform(5, 35))  # X value range
    #     point_[1] = int(np.random.uniform(5, 10))  # No change in the Y value
    #     point_[2] = int(np.random.uniform(35,40))  # Depth in Z   
    #     listOfSel.append('S_right_fov')
    
    return point_

# Generate random point to reach
def generateRandomPoint_v3():
    """Extract point from the list of position to used in the experiment.

    Args:
        currentTipRPoint (list): 3D position of the right arm

    Returns:
        list: [Target position of the point, List of intented states to trigger]
    """
    point_ = np.array([0, 0, 0])
    sel2_ = np.random.randint(4)
    listOfSel = []
    if sel2_ == 0:                  # ------------- FAR
        val1_ = int(np.random.uniform(-20, 25))
        point_[1] = val1_
        point_[0] = abs(val1_)  # X value range
        point_[2] = int(75 + np.random.uniform(-5, 10))
        listOfSel.append('S_optimal_reach_pos')
    elif sel2_ == 1:                # ------------- BENDING LEFT
        point_[0] = int(np.random.uniform(-40, 0))
        point_[1] = int(np.random.uniform(5,   10))
        point_[2] = int(np.random.uniform(20,  25))
        listOfSel.append('S_optimal_reach_neg')
    elif sel2_ == 2:                # ------------- BENDING RIGHT
        point_[0] = -int(np.random.uniform(40, 55))
        point_[1] =  int(np.random.uniform(5,   10))
        point_[2] =  int(np.random.uniform(40,  60))
        listOfSel.append('S_left_WS')
    elif sel2_ == 3:                # ------------- CLOSE
        val1_ = int(np.random.uniform(-20, 25))
        point_[1] = val1_
        point_[0] = abs(val1_)
        point_[2] = int(np.random.uniform(-8, 30))
        listOfSel.append('S_right_WS')    
    return point_

def getPoint():
    return 0

def manipulibityIndex(jaco_):
    """Yoshikawa's manipulability index, a quality measure for redundant
    manipulators, which describes the distance to singular configurations. The
    measure is proportional to the volume of the manipulability ellipsoid.

    Args:
        jaco_ (array): Jacobian matrix

    Returns:
        float: Manipulability index scalar
    """
    return sqrt(np.linalg.det(np.dot(jaco_,jaco_.T)))

# To generate random q v2
def gen_random_q():
    """Generate a new updated q with a set of values that not that phisically extreme


    Returns:
        list: List of the random generated q
    """
    # curvature between -0.085 and 0.085 mm -1
    k_r = (0.085)*np.random.random_sample()
    k_l = (0.085)*np.random.random_sample()

    # angle between -90 and 90
    # a_r = (2.0*np.pi/2.0)*np.random.random_sample() + np.pi/2.0
    # a_l = (2.0*np.pi/2.0)*np.random.random_sample() - np.pi/2.0
    a_r = (2.0*np.pi/2.0)*0 + np.pi/1.0
    a_l = (2.0*np.pi/2.0)*0 + np.pi/1.0

    # insertion between 0 and 60
    d_r = 40*np.random.random_sample() + 30
    d_l = 40*np.random.random_sample() + 30

    # # endoscope steady for testing
    k_e1 = (0.005)*np.random.random_sample()
    k_e2 = (0.005)*np.random.random_sample()
    a_e = 0.0
    d_e = 10*np.random.random_sample()
    #
    # # test values
    # k_r = 0.085 * np.random.random_sample()  #
    # k_l = 0.085 * np.random.random_sample()  #
    # a_r = np.pi
    # a_l = 0.0
    return [k_r, a_r, d_r, k_l, a_l, d_l, k_e1, k_e2, a_e, d_e]

# To generate random q
def generate_random_q():
    # curvature between -0.085 and 0.085 mm -1
    k_r = (0.085)*np.random.random_sample()
    k_l = (0.085)*np.random.random_sample()

    # angle between -90 and 90
    a_r = (2.0*np.pi/2.0)*np.random.random_sample() + np.pi/2.0
    a_l = (2.0*np.pi/2.0)*np.random.random_sample() - np.pi/2.0

    # insertion between 0 and 60
    d_r = 60*np.random.random_sample()
    d_l = 60*np.random.random_sample()

    # # endoscope steady for testing
    k_e = (0.005)*np.random.random_sample()
    a_e = 0.0
    d_e = 10*np.random.random_sample()
    #
    # # test values
    # k_r = 0.085 * np.random.random_sample()  #
    # k_l = 0.085 * np.random.random_sample()  #
    # a_r = np.pi
    # a_l = 0.0
    return [k_r, a_r, d_r, k_l, a_l, d_l, k_e, a_e, d_e]

# To check if symmetric
def check_symmetric(a, rtol=1e-05, atol=1e-08):
    return np.allclose(a, a.T, rtol=rtol, atol=atol)

# To check if define semi-positive
def is_pos_def(x):
    return np.all(np.linalg.eigvals(x) >= 0)

# To check if define semi-negative
def is_neg_def(x):
    return np.all(np.linalg.eigvals(x) <= 0)

# ------------------------ TO VALIDATE
# To check if the list is within the min and max, return list of errors
def checkInRangeBounded(list_, mins_, maxs_):
    """Check if a list is within the mins and maxs ranges, if outsides bounds
    the output to the appropiate element of min_ or max_

    Args:
        list_ (list): List of values
        mins_ (list): List of mins
        maxs_ (list): List of maxs

    Returns:
        list: [result, errIdx_, errType_, err_]
    """
    res = []
    err_ = []
    errDof_ = []
    errIdx_ = []
    errType_ = []
    for i in range(len(list_)):
        if list_[i] < mins_[i]:
            # if qpOrder[i]!='ER' and qpOrder[i]!='':
            res.append(mins_[i])
            err_.append(list_[i])
            errDof_.append(qpOrder[i])
            errIdx_.append([i])
            errType_.append(['min'])
        elif list_[i] > maxs_[i]:
            res.append(maxs_[i])
            err_.append(list_[i])
            errDof_.append(qpOrder[i])
            errIdx_.append([i])
            errType_.append(['max'])
        else:
            res.append(list_[i])
    return res, errDof_, errType_, err_

# ------------------------ TO FILTER Q
# Filter q2cmd w.r.t. q_dot!=0 values
def filter_q2cmd(q_in_, labels_in_):
    """Filter q_in with respect to the labels_in

    Args:
        q_in_ (list): Joint values
        labels_in_ (list): Labels of the joint values

    Returns:
        list: list of list of commands to the STRAS follower
    """
    q2cmd_out_ = []
    for u_ in range(len(labels_in_)):
        idx_ = [k_ for k_ in range(len(qpOrder))
                if labels_in_[u_] == followerOrder[k_]][0]
        q2cmd_out_.append([labels_in_[u_],
                           q_in_[idx_],
                           maxRelativeDict[labels_in_[u_]]])
    return q2cmd_out_


# To check which control signal to update
def filter_q_dot(q_In_, q_Dot_, onlyDof_=None):
    """Filter the intented q_dot for the follower w.r.t. to the onlyDof list
    and the dictionary of minimum step tooCloseDict_up

    Args:
        q_In_ (list): Joint values
        q_Dot_ (list): Q dot of joint values
        onlyDof_ (list, optional): To filter to selected DOF. Defaults to None.

    Returns:
        list: The command list of list to send to the STRAS follower
    """
    cmd2Stras_internal = []
    dict1_ = dict(zip(followerOrder_local, q_In_))
    dict2_ = dict(zip(followerOrder_local, q_Dot_))
    if onlyDof_ is not None:
        dof2Use_ = onlyDof_.copy()
    else:
        dof2Use_ = followerOrder_local.copy()
    vals1_ = [dict1_[x_] for x_ in dof2Use_]
    vals2_ = [dict2_[x_] for x_ in dof2Use_]
    for i_ in range(len(dof2Use_)):
        if not(isclose(vals1_[i_], vals2_[i_],
               abs_tol=0)):
            #    abs_tol=tooCloseDict_qp[dof2Use_[i_]])):
            cmd2Stras_internal.append([dof2Use_[i_],
                                       q_Dot_[i_],
                                       maxRelativeDict[dof2Use_[i_]]])
    return cmd2Stras_internal


# From q2tele and raw q_follower activate the DOF that are close (leader-follower)
def defineActiveDof(q2Tele_, q_follower_, dof2use, dictActivation_, pOpt_=False):
    """Function to reactivate the DOF that are close to each other.
    The user regains control when the DOF of the leader match the DOF of the follower

    Args:
        q2Tele_ (list): Joint positions to send to the follower (from the LEADER)
        q_follower_ (list): Joint positions of the follower(ENDOSCOPE)
        dof2use (list): List of DOF to control
        dictActivation_ (dict): Dictionary of active DOF (e.g. ['TR','RR','BR'])
        pOpt_ (bool, optional): To print verbose. Defaults to False.

    Returns:
        dict: Dictionary of DOF to control with TRUE or FALSE
    """
    dictfollower = dict(zip(followerOrder_local, q_follower_))
    dictq2Tel = dict(zip(followerOrder_local, q2Tele_))
    valsfollower = [dictfollower[x_] for x_ in dof2use]
    valsTele = [dictq2Tel[x_] for x_ in dof2use]
    for i_ in range(len(dof2use)):
        if not(dictActivation_[dof2use[i_]]):
            if isclose(valsfollower[i_], valsTele[i_],
                       abs_tol=tooCloseDict_local[dof2use[i_]]):
                dictActivation_[dof2use[i_]] = True
                if pOpt_:
                    print('Retaking control of: ', dof2use[i_])
    return dictActivation_

# ------------------------------------------------------------------------------
# ------------------------ FOR INPUT-OUTPUT DATA
# ------------------------------------------------------------------------------
# To read data parameters
def read_controlParameters():
    # Reading Thresholds from file - FG
    configFileThs = 'controlParameters.txt'
    dThs = {}
    with open(configFileThs) as f:
        for line in f:
            (key, val) = line.split("=")
            dThs[key] = val
    return dThs

# To read data parameters
def read_thresholds():
    # Reading Thresholds from file - FG
    configFileThs = 'thresholds.txt'
    dThs = {}
    with open(configFileThs) as f:
        for line in f:
            (key, val) = line.split("=")
            dThs[key] = val
    return dThs

# ------------------------------------------------------------------------------
# ------------------------ FOR TELEOPERATION
# ------------------------------------------------------------------------------
# From the q2tele and the activation dictionary generate the commands
def teleoperateSTRAS(q2Tele_, dictActivation_local_):
    """Filter the list of desired joint goals with respect to the dictionary of active DOF

    Args:
        q2Tele_ (list): Joint values to send to the STRAS follower
        dictActivation_local_ (dict): Dictionary of DOF with TRUE or FALSE if DOF active in the desired teleoperation

    Returns:
        list: list of list to send to the STRAS follower
    """
    cmd2stras_ = []
    q2teleDict = dict(zip(followerOrder_local, q2Tele_))
    list_ = [i_ for i_ in range(
        len(dictActivation_local_)) if list(
            dictActivation_local_.values())[i_]]
    if len(list_) >= 1:
        for idx_ in list_:
            # print('followerOrder 2 control', followerOrder[idx_])
            # print('value2stras', q2teleDict[followerOrder[idx_]])
            # print('maxRelative', maxRelativeDict[followerOrder[idx_]])
            cmd2stras_.append([followerOrder_local[idx_],
                               q2teleDict[followerOrder_local[idx_]],
                               maxRelativeDict[followerOrder_local[idx_]]])
    return cmd2stras_

# To teleoperate endoscope from the [-1, 1] signals from the leader joysticks
def teleoperateSTRASendo(qEndo_in_, qfollower_Endo_):
    """Create the teleoperation command to send to the STRAS follower ENDOSCOPE dof

    Args:
        qEndo_in_ (list): List of DOF of endoscope joint positions to send
        qfollower_Endo_ (list): List of DOF of endoscope joint positions of follower

    Returns:
        list: [description]
    """
    qEndo_ = [qEndo_in_[0], qEndo_in_[1], qEndo_in_[3], qEndo_in_[2]]
    dofLabelList_ = ['EBV', 'EBH', 'ER', 'ET']
    qEndoOut_ = []
    # stepEndoTeleoperation
    for i_, dofEndo_ in enumerate(qEndo_):
        if int(dofEndo_) != 0:
            qEndoOut_.append(
                [dofLabelList_[i_],
                 qfollower_Endo_[i_]+(
                     stepEndoTeleoperation[dofLabelList_[i_]]*dofEndo_),
                 0.1])
    return qEndoOut_

# ------------------------ TO ORDER DICTIONARIES
# Order the values of a dictionary w.r.t. the orderListOuput
def orderRanges(rangeDict, orderListOutput):
    """Order a dictionary {rangeDict} values in a list order {orderListOutput}

    Args:
        rangeDict (dict): Dictionary with dof labels as keys and determined
        value
        orderListOutput (list): List of the labels of the dictionary in a
        specified order

    Returns:
        np.array: [description]
    """
    outputList = []
    for lbl in orderListOutput:
        outputList.append(rangeDict.get(lbl))
    return np.array(outputList)

# ------------------------------------------------------------------------------
# ------------------------ FOR TRANSFORMATION BETWEEN SPACES
# ------------------------------------------------------------------------------

# ------------------------ TO MAP BETWEEN SPACES
# To interpolate from [0, 1] to [mins, maxs] the desired output space
def map_norm2range(q_in_, listOfmins_, listOfmaxs_):
    """Normalize the input list w.r.t. to the desired mins and maxs

    Args:
        q_in_ (list): List of values to interpolate
        listOfmins_ (list): List of mins
        listOfmaxs_ (list): List of maxs

    Returns:
        list: Input in the correct range
    """
    # Input should be normalized and increasing between [0, 1]
    if type(listOfmins_) is np.ndarray:
        listOfmins_ = listOfmins_.tolist().copy()
    if type(listOfmaxs_) is np.ndarray:
        listOfmaxs_ = listOfmaxs_.tolist().copy()
    q_out_ = np.zeros(len(q_in_))
    for i_ in range(len(q_in_)):
        q_out_[i_] = np.interp(q_in_[i_],
                               (0, 1),
                               (listOfmins_[i_], listOfmaxs_[i_]))
    return q_out_

# To interpolate from [0, 1] to [mins, maxs] the QP q
def map_norm2follower_followerOrder(q_in_, listOfmins_, listOfmaxs_):
    """From the normalize input list map to the desired follower mins (listOfmins_) and maxs (listOfmaxs_).

    Args:
        q_in_ (list): List of values to interpolate
        listOfmins_ (list): List of mins
        listOfmaxs_ (list): List of maxs

    Returns:
        list: Input in the correct range
    """
    # Input should be normalized and increasing between [0, 1]
    if type(listOfmins_) is np.ndarray:
        listOfmins_ = listOfmins_.tolist().copy()
    if type(listOfmaxs_) is np.ndarray:
        listOfmaxs_ = listOfmaxs_.tolist().copy()
    q_out_ = np.zeros(len(q_in_))
    q_in_[4] = 1 - q_in_[4]  # Invert Right translation
    q_in_[7] = 1 - q_in_[7]  # Invert Left translation
    # q_in_[9] = 1 - q_in_[9]  # Invert Left bending
    for i_ in range(len(q_in_)):
        # if i_ == 4 or i_ == 6 or i_ == 7:
        #     q_out_[i_] = np.interp(q_in_[i_],
        #                            (0, 1),
        #                            (listOfmaxs_[i_], listOfmins_[i_]))
        # else:
        q_out_[i_] = np.interp(
            q_in_[i_],
            (0, 1),
            (listOfmins_[i_], listOfmaxs_[i_]))
    return q_out_

# ------------------------ TO NORMALIZE
# To normalize q (range:lim_down and lim_up) to [0,1], SAME ORDERS!!!
def normalize_q_qp(q_qp, lim_down_, lim_up_):
    """Normalize q (range:lim_down and lim_up) to [0,1], SAME ORDERS!!!

    Args:
        q_qp (list): Joint positions
        lim_down_ (list): List of mins
        lim_up_ (list): List of maxs

    Returns:
        list: list of range range [0, 1]
    """
    q_out = np.zeros(len(q_qp))
    for i_ in range(len(q_qp)):
        q_out[i_] = np.interp(q_qp[i_],
                              (lim_down_[i_], lim_up_[i_]),
                              (0, 1))
    return q_out

# ------------------------ TO CHANGE ORDER BETWEEN SPACES
# Change order from qp to follower
def order_qp2follower(q_in_):
    """Order the list(q_in_) from the qp order to the follower

    Args:
        q_in_ (list): Joint values in the corrected order

    Returns:
        list: List in the correct order
    """
    q_follower_endo = [q_in_[6], q_in_[7], q_in_[8], q_in_[9]]
    q_follower_R = [q_in_[2], q_in_[1], q_in_[0]]
    q_follower_L = [q_in_[5], q_in_[4], q_in_[3]]
    q_out_ = q_follower_endo + q_follower_R + q_follower_L
    return q_out_

# Change order from leader to qp
def order_leader2configspace(q_in_):
    """Order the list(q_in_) from the leader order to the qp

    Args:
        q_in_ (list): Joint values in the corrected order

    Returns:
        list: List in the correct order
    """
    q_R = [q_in_[6], q_in_[5], q_in_[4]]
    q_L = [q_in_[2], q_in_[1], q_in_[0]]
    q_e = [q_in_[8], q_in_[9], q_in_[11], q_in_[10]]  # CHECK
    q_out_ = q_R + q_L + q_e
    return q_out_

# To extract triggers from q_in
def takeTriggersFromLeader(q_in_):
    """Pass the trigger information from the original leader q.

    Args:
        q_in (list): _description_

    Returns:
        list: Lisf containing the trigger reads from the leader
    """
    qTrig_r = q_in_[3]
    qTrig_l = q_in_[5]
    return [qTrig_r, qTrig_l]

# Change order from follower to qp
def order_follower2configspace(q_in_):
    """Order the list(q_in_) from the leader order to the follower

    Args:
        q_in_ (list): Joint values in the corrected order

    Returns:
        list: List in the correct order
    """
    q_R = [q_in_[6], q_in_[5], q_in_[4]]
    q_L = [q_in_[9], q_in_[8], q_in_[7]]
    q_e = [q_in_[0], q_in_[1], q_in_[2], q_in_[3]]
    q_out_ = q_R + q_L + q_e
    return q_out_

# Mapping functions between normalized q's
def order_leader2follower(q_in_):
    """Order the list(q_in_) from the leader order to the follower

    Args:
        q_in_ (list): Joint values in the corrected order

    Returns:
        list: List in the correct order
    """
    q_E = [q_in_[8], q_in_[9], q_in_[11], q_in_[10]]
    q_R = [q_in_[4], q_in_[5], q_in_[6]]
    q_L = [q_in_[0], q_in_[1], q_in_[2]]
    q_out_ = q_E + q_R + q_L
    return q_out_

# ------------------------------------------------------------------------------
# ------------------------ GEOMETRIC FUNCTIONS
# ------------------------------------------------------------------------------

# Function to obtain if point is below or above the FOV line and the distance
def distance_point_line(p1, pl_1, pl_2, inwards_d, arm):
    """Obtain if point is below or above the FOV line and the distance

    Args:
        p1 (np.array): point of the tip of the instrument
        pl_1 (np.array): init point of the FOV segment to consider
        pl_2 (np.array): final point of the FOV segment to consider
        inwards_d (np.array): Normal distance inwards the FOV lines
        arm (str): Type of arm 'r','R' or 'l','L'

    Returns:
        list: 1-signed distance to the FOV lines
              2-position of the projected line over the FOV lines times inward
    """
    d = np.linalg.norm(np.cross(pl_2-pl_1, p1-pl_1))/np.linalg.norm(pl_2-pl_1)
    pos_in_line, _ = projection_over_line(pl_1, pl_2, p1, inwards_d, arm)
    pos_proj_goal_angle = (pos_in_line-p1)/np.linalg.norm(pos_in_line-p1)
    pos_proj_goal = pos_proj_goal_angle
    if (p1-pos_in_line)[-1] < 0:
        d_symbol = -1
    else:
        d_symbol = 1
    return d*d_symbol, pos_in_line, pos_proj_goal


# Function to obtain the projection to the closest FOV point
def projection_over_line(a, b, p, inwards_d, fov_plane):
    """Projection over the segment a-b the closest point to p by a inwards_d in 
    p direction, variable arm denote positive or negative X-axis

    Args:
        a (np.array): init point of the FOV segment to consider
        b (np.array): final point of the FOV segment to consider
        p (np.array): point of the tip of the instrument
        inwards_d (float): Normal distance inwards the FOV lines
        fov_plane (string): String of the arm 'r','R' or 'l','L'

    Returns:
        list: 1-projection by d(inwards) over the FOV lines
              2-point online closest to p
    """
    ap = p - a
    ab = b - a
    t = np.dot(ap, ab) / np.dot(ab, ab)
    # if you need the the closest point belonging to the segment
    t = max(0, min(1, t))
    result = a + t * ab
    result[1] = result[1] + inwards_d
    res_online = result
    if fov_plane == 'R' or fov_plane == 'r':
        result[0] = result[0] - inwards_d
    elif fov_plane == 'L' or fov_plane == 'l':
        result[0] = result[0] + inwards_d
    elif fov_plane == 'U' or fov_plane == 'u':
        result[0] = result[0] + inwards_d
    elif fov_plane == 'D' or fov_plane == 'd':
        result[0] = result[0] - inwards_d
    return result, res_online


# Function to get the distance to the FOV pyramid - Horizontal
def dist2PovHorizontal(point, quadrant,
                       angle=50, distanceInit=10, distanceEnd=80):
    """Function to get the distance to the FOV pyramid - Horizontal

    Args:
        point (list): List of current point location
        quadrant (str): Quadrant on the top or bottom
        angle (int, optional): Angle of vertical FOV. Defaults to 40.
        distanceInit (int, optional): Starting distance of the line that builds
        the FOV pyramid. Defaults to 20.
        distanceEnd (int, optional): End distance of the line that builds
        the FOV pyramid. Defaults to 50.

    Returns:
        [type]: [description]
    """
    posPointInit = Float(
        Float(tan(radians(angle)), 3)*distanceInit, 2)
    posPointEnd = Float(Float(tan(radians(angle)), 3)*distanceEnd, 2)
    # ------------------------ Defining the points of the boundary POV
    #     p0 = Point(0,0,0)
    pLeftInit = Point(-posPointInit, 0, distanceInit)
    pRightInit = Point(posPointInit, 0, distanceInit)
    pLeftEnd = Point(-posPointEnd, 0, distanceEnd)
    pRightEnd = Point(posPointEnd, 0, distanceEnd)
    # ------------------------ Shortest distance to POV boundary line
    povLeft = Line(pLeftInit, pLeftEnd)
    povRight = Line(pRightInit, pRightEnd)
    pActual = Point(point)
    if quadrant == 'R':
        return round(povRight.distance(pActual), 3)
    elif quadrant == 'L':
        return round(povLeft.distance(pActual), 3)
    else:
        print('Specify quadrant')
        return 'Error'


# Function to get the distance to the FOV pyramid - Vertical
def dist2PovVertical(point, quadrant,
                     angle=40, distanceInit=20, distanceEnd=50):
    """Function to get the distance to the FOV pyramid - Vertical

    Args:
        point (list): List of current point location
        quadrant (str): Quadrant on the top or bottom
        angle (int, optional): Angle of vertical FOV. Defaults to 40.
        distanceInit (int, optional): Starting distance of the line that builds
        the FOV pyramid. Defaults to 20.
        distanceEnd (int, optional): End distance of the line that builds
        the FOV pyramid. Defaults to 50.

    Returns:
        [type]: [description]
    """
    posPointInit = Float(
        Float(tan(radians(angle)), 3)*distanceInit, 2)
    posPointEnd = Float(Float(tan(radians(angle)), 3)*distanceEnd, 2)
    # ------------------------ Defining the points of the boundary POV
    #     p0 = Point(0,0,0)
    pLeftInit = Point(-posPointInit, 0, distanceInit)
    pRightInit = Point(posPointInit, 0, distanceInit)
    pLeftEnd = Point(-posPointEnd, 0, distanceEnd)
    pRightEnd = Point(posPointEnd, 0, distanceEnd)
    # ------------------------ Shortest distance to POV boundary line
    povLeft = Line(pLeftInit, pLeftEnd)
    povRight = Line(pRightInit, pRightEnd)
    pActual = Point(point)
    if quadrant == 'U':
        return round(povRight.distance(pActual), 3)
    elif quadrant == 'D':
        return round(povLeft.distance(pActual), 3)
    else:
        print('Specify quadrant')
        return 'Error'


# Function to obtain distance and direction between points
def consPoints(p1_, p2_):
    """Function to obtain distance and direction between points

    Args:
        p1_ (list): List of point 1
        p2_ (list): List of point 2

    Returns:
        list: distance between the points, direction
    """
    p1 = np.array(p1_)
    p2 = np.array(p2_)
    distanceBetweenPoints = np.linalg.norm(p1-p2)
    directionToP2fromP1 = (p2-p1)/distanceBetweenPoints
    return distanceBetweenPoints, directionToP2fromP1
