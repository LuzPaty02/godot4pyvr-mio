import sys
sys.path.insert(0, sys.path[0]+'module_main/')
import numpy as np
from module_main.model_utilities.robot_piecewise import Robot_piecewise_curvature
import module_main.model_utilities.transformations as tf
from liegroups import SE3
import copy
from math import cos, sin


class robot_model:
    """
    Class describing the model of a single piecewise constant curvature robot
    """

    def __init__(self, configfile=None):
        """
        Initialization of a general piecewise constant curvature robot model
        :param configfile: config file with the robot parameters (see below)
        """
        self.q = np.asarray([0, 0, 0])
        # Curvature, rotation angle, insertion (Webster's formalism)

        # If no config file is given, put some default parameters.
        # Useful for testing
        if not configfile:
            self.lengths = [60, 18, 5.1, 2.2, 4.5]
            self.diameters = [3.54, 3.54, 3.15, 1.78, 0.4]
            self.dth = -10  # degrees, nominal deviation between exit angle
            # and camera axis. rotation around y
            self.pos_canal = [-13.3,
                              6.2]  # position of the instrument channel wrt
            #    camera channel. z towards scene x down y right
            self.nsections = 5
            self.active = [0, 1, 0, 0, 0]
            # sections marked 1 are active (i.e. bending)

            self.robot = Robot_piecewise_curvature(self.nsections)
            self.robot.setLength([100]*self.nsections)  # simple initialization
            self.robot.setOrigin([0, 0, 0], [0, 0, 1], [1, 0, 0])  # same

        else:
            d = {}
            with open(configfile) as f:
                for line in f:
                    (key, val) = line.split("=")
                    d[key] = val

            self.diameters = np.array(
                d['diameters'][:-1].split(',')).astype(float)
            self.d = float(d['d'][:-1])
            self.lengths = np.array(
                d['L'][:-1].split(',')).astype(float)
            self.dth = float(d['dth'][:-1])
            self.pos_canal = np.array(
                d['pos_canal'][:-1].split(',')).astype(float)
            self.nsections = int(d['n_sec'][:-1])
            self.active = np.array(d['active'][:-1].split(',')).astype(int)

            self.robot = Robot_piecewise_curvature(self.nsections)
            self.robot.setLength([100]*self.nsections)  # simple initialization
            self.robot.setOrigin([0, 0, 0], [0, 0, 1], [1, 0, 0])  # same

        # Translation from reference frame to base of the robot.
        # For instance for the right robot this would be from the endoscope
        # frame to the channel frame.
        # For the endoscope this would be the identity (at initialization)
        self.T_0 = tf.translation_matrix(
            [self.pos_canal[0],
             self.pos_canal[1],
             -34.3]).dot(
            tf.rotation_matrix(np.deg2rad(self.dth), [0, 1, 0]))
        self.T_0_se3 = SE3.from_matrix(self.T_0)

    def setJointValues(self, q):
        self.q = q

        # First section length is 0<q[2]<lengths[0]
        # (i.e. value in the config file)
        if q[2] > self.lengths[0]:
            self.q[2] = self.lengths[0]
        if q[2] < 0:
            self.q[2] = 0

    def getJointValues(self):
        return self.q

    def computeSectionTransform(self, q_s):
        """
        Compute the transformation of a section characterized by joint
        values q_s
        :param q_s: joint values of the section in terms of curvature,
        rotation and lenght
        :return: Ts, the associated transformation matrixrotation
        """
        ksi = np.array(
            [0, 0, 1, -q_s[0]*np.sin(q_s[1]), q_s[0]*np.cos(q_s[1]), 0])
        T_s = SE3.exp(ksi*q_s[2])
        return T_s

    def computePose(self):
        """
        Compute the robot model
        :param q: joint values in the [curvature, rotation angle, insertion]
        format
        :return:
        """
        # initialize straight sections (non-zero curvature to avoid
        # singularities)
        curv = 0.00001*np.ones(self.nsections, dtype=float)
        rotation = np.zeros(self.nsections, dtype=float)  # initialize own
        # rotation at 0 for all
        lengths = list(self.lengths)  # length of the sections
        Transformations = []

        # Change the radius/rotation only of the active (i.e. bent) section
        for i in range(self.nsections):
            if self.active[i] == 1:
                curv[i] = self.q[0]
                rotation[i] = self.q[1]
            if i == 0:
                Ti = self.computeSectionTransform(
                    [curv[i], rotation[i], self.q[2]])
                Transformations.append(self.T_0_se3.dot(Ti))
            else:
                Ti = self.computeSectionTransform(
                    [curv[i], rotation[i], lengths[i]])
                Transformations.append(Transformations[i-1].dot(Ti))

        return Transformations

    def computeShape(self, q=None):
        """
        Compute the robot shape in its base frame
        :param q: (optional) joint positions
        :return: an array of arrays, each subarray being a 10-element shape
        vector for a given section
        """

        if q is not None:
            self.setJointValues(q)

        # initialize straight sections (non-zero curvature to avoid
        # singularities)
        curv = np.ones(self.nsections, dtype=float)*0.00001
        rotation = np.zeros(self.nsections, dtype=float)  # initialize own
        # rotation at 0 for all
        lengths = list(self.lengths)  # length of the sections
        Shape = []

        # Change the radius/rotation only of the active (i.e. bent) section
        Ti = copy.deepcopy(self.T_0_se3)

        for i in range(self.nsections):
            if i == 0:
                l0 = self.q[2]  # d is the insertion of the robot
            else:
                l0 = lengths[i]
            section_shape = []
            if self.active[i] == 1:
                curv[i] = self.q[0]
                rotation[i] = self.q[1]

            # Use the shared piecewise-constant-curvature arc implementation
            # for the section backbone, then place it in this robot's frame.
            # The +pi aligns its normal convention with computeSectionTransform.
            radius = np.inf if curv[i] == 0 else 1.0 / curv[i]
            local_shape, _, _ = self.robot.computeCircArc(
                np.zeros(3), np.array([0.0, 0.0, 1.0]),
                np.array([1.0, 0.0, 0.0]), rotation[i] + np.pi,
                radius, l0, n_samples=10)
            base = Ti.as_matrix()
            section_shape = (local_shape @ base[:3, :3].T) + base[:3, 3]
            Ti = Ti.dot(self.computeSectionTransform(
                [curv[i], rotation[i], l0]))
            Shape.append(section_shape)

        return Shape

    def computeModel(self, q=None, jac=False, Tb_w=None):
        """
        Compute the overall robot model
        :param q: (optional) joint values
        :param jac: boolean flag to compute the Jacobian as well
        :return: Transformation matrix to the robot tip, plus tip Jacobian in
        the base frame if jac=True
        """
        if q is not None:
            self.setJointValues(q)
        if jac is False:
            return self.computePose()[self.nsections-1]
        else:
            return self.computeJacobian_new(Tb_w)

    def computeJacobian(self):
        """
        Compute the Jacobian of the robot. This code assumes that the robot
        has a first straight section with  has a variable length, a second
        bended/rotated section, and subsequent straight sections. The code
        computes the Jacobian in terms of screw velocity, i.e. translational
        speed and rotational velocity
        :return: Tip transformation and Jacobian expressed in the base frame
        """
        k = self.q[0]
        phi = self.q[1]

        Jv = np.zeros((6, 3))
        J = np.zeros((6, 3))

        Trobot = self.computePose()

        # Jacobian of the first portion. We are only interested in the third
        # column, since curvature and rotation are 0 (straight portion)
        Jv[:, 2] = SE3.adjoint(self.T_0_se3).dot(np.array([0, 0, 1, 0, 0, 0]))
        J[:, 2] = SE3.adjoint(self.T_0_se3).dot(np.array([0, 0, 1, 0, 0, 0]))

        # Jacobian of the second portion. Here l is fixed, so we'll only keep
        # the two first columns
        l = self.lengths[1]
        l2 = self.lengths[2]
        cp = np.cos(phi)
        sp = np.sin(phi)

        if 0 == k:
            vs_1 = np.array([[-cp * l * l / 2.0, 0, 0],
                             [-sp * l * l / 2.0, 0, 0],
                             [0, 0, 1],
                             [-l * sp, 0, 0],
                             [l * cp, 0, 0],
                             [0, 1, 0]])
        else:
            dkk = 1.0 / (k * k)  # divider by k squared
            ckl = np.cos(k * l)
            skl = np.sin(k * l)

            # velocity screw of the section wrt to kinematic variables
            vs_1 = np.array([[cp * (ckl - 1.0) * dkk, 0, 0],
                             [sp * (ckl - 1.0) * dkk, 0, 0],
                             [-(skl - (k * l)) * dkk, 0, 1],
                             [-l * sp, 0, -k * sp],
                             [l * cp, 0, k * cp],
                             [0, 1, 0]])

        # screw is multiplied by the adjoint transform to the base of the
        # bendable/rotated section,
        # to express it in the correct frame
        Jv[:, :2] = SE3.adjoint(Trobot[0]).dot(vs_1[:, :2])

        # Put the Jacobian in the correct frame by multiplying by the adjoint
        # of the inverse translation from tip to base
        T_tb = np.eye(4)
        T_tb[:3, 3] = -Trobot[2].as_matrix()[:3, 3]

        J[:, :2] = SE3.adjoint(SE3.from_matrix(T_tb)) @ Jv[:, :2]
        # J = Trobot[1].inv().adjoint() @ J

        return Trobot[self.nsections-1], J, Jv

    def computeJacobian_new(self, Tb_w=None):
        """
        Compute the robot jacobian, assuming a 3 links robot with :
            - a first variable length section
            - a second bendable/rotated section with fixed length
            - a third fixed length section
        :param Tb_w: (optional) Transform from world frame to base frame as a
        SE3 liegroup object
        :return: Tr_b, Tr_w, Xr_b, Xr_w, Jx_b, Jx_w, zrb, zrw, Jz_b, Jz_w:
            T is a transform in SE3 to the tip of the robot
            X is the position
            Jx is the position Jacobian
            z is the tip tangent vector
            Jz is the tip tangent Jacobian

            b is expressed in the robot base frame
            w in the world frame
        """

        if Tb_w is None:
            Tb_w = SE3.from_matrix(np.eye(4))

        Hb_w = Tb_w.as_matrix()

        k = self.q[0]
        if k == 0:
            k = 0.0000000000000001
        phi = self.q[1]
        l1 = self.q[2]
        l = self.lengths[1]
        l3 = self.lengths[2]

        Trobot = self.computePose()
        Tr_b = Trobot[2]
        Tr_w = Tb_w.dot(Tr_b)

        # Tip position in robot frame
        Xr_b = self.T_0 @ np.array(
            [[cos(phi)*(l3 * sin(k*l) + (1-cos(k*l))/k)],
             [sin(phi)*(l3 * sin(k*l) + (1-cos(k*l))/k)],
             [l1+l3*cos(k*l) + sin(k*l)/k],
             [1]])

        Xr_w = Hb_w @ Xr_b

        # Tip tangent vector in robot frame
        zr_b = self.T_0 @ np.array([[cos(phi)*sin(k*l)],
                                    [sin(phi)*sin(k*l)],
                                    [cos(k*l)],
                                    [0]])
        zr_w = Hb_w @ zr_b

        # Jacobian in position and tip tangent orientation
        Jx_b = np.zeros((3, 3))
        Jz_b = np.zeros((3, 3))

        Jx_b[:, 0] = [(l3 * l * cos(k * l) +
                      (k * l * sin(k * l) -
                      (1 - cos(k * l))) / (k * k)) * cos(phi),
                      (l3 * l * cos(k * l) + (k * l * sin(k * l) -
                       (1 - cos(k * l))) / (k * k)) * sin(phi),
                      -l*l3*sin(k*l) + (k*l*cos(k*l)-sin(k*l))/(k*k)]
        Jx_b[:, 1] = [-l3 * sin(phi) * sin(k * l) -
                      sin(phi) * (1 - cos(k * l)) / k,
                      l3 * cos(phi) * sin(k * l) +
                      cos(phi) * (1 - cos(k * l)) / k,
                      0]
        Jx_b[:, 2] = [0, 0, 1]

        # Jz_b computation TODO

        # Expressing in the base frame
        Jx_b = self.T_0[:3, :3] @ Jx_b
        Jz_b = self.T_0[:3, :3] @ Jz_b
        # Expressing the world frame
        Jx_w = Hb_w[:3, :3] @ Jx_b
        Jz_w = Hb_w[:3, :3] @ Jz_b

        return Tr_b, Tr_w, Xr_b, Xr_w, Jx_b, Jx_w, zr_b, zr_w, Jz_b, Jz_w


class robot_twobending:
    """
        Class describing the model of a single piecewise constant curvature
        robot with two bendings (i.e. endoscope robot in STRAS)
        """

    def __init__(self, configfile=None):
        """
        Initialization of a general piecewise constant curvature robot model
        :param configfile: config file with the robot parameters (see below)
        """
        self.q = np.asarray([0, 0, 0, 0])
        # curvature1, curvature2, rotation, insertion

        # If no config file is given, put some default parameters.
        # Useful for testing
        if not configfile:
            self.lengths = [60, 18, 5.1, 2.2, 4.5]
            self.diameters = [3.54, 3.54, 3.15, 1.78, 0.4]
            self.dth = -10  # degrees, nominal deviation between exit angle
            # and camera axis. rotation around y
            self.pos_canal = [-13.3,
                              6.2]  # position of the instrument channel wrt
            #   camera channel. z towards scene x down y right
            self.nsections = 5
            # sections marked 1 are active (i.e. bending)
            self.active = [0, 1, 0, 0, 0]

            self.robot = Robot_piecewise_curvature(self.nsections)
            # simple initialization
            self.robot.setLength([100] * self.nsections)
            self.robot.setOrigin([0, 0, 0], [0, 0, 1], [1, 0, 0])  # same

        else:
            d = {}
            with open(configfile) as f:
                for line in f:
                    (key, val) = line.split("=")
                    d[key] = val

            self.diameters = np.array(
                d['diameters'][:-1].split(',')).astype(float)
            self.d = float(d['d'][:-1])
            self.lengths = np.array(d['L'][:-1].split(',')).astype(float)
            self.dth = float(d['dth'][:-1])
            self.pos_canal = np.array(
                d['pos_canal'][:-1].split(',')).astype(float)
            self.nsections = int(d['n_sec'][:-1])
            self.active = np.array(d['active'][:-1].split(',')).astype(int)

            self.robot = Robot_piecewise_curvature(self.nsections)
            # simple initialization
            self.robot.setLength([100] * self.nsections)
            # Fernando change up 150 to 75
            self.robot.setOrigin([0, 0, 0], [0, 0, 1], [1, 0, 0])  # same

        # Translation from reference frame to base of the robot.
        # For instance for the right robot this would be from the endoscope
        # frame to the channel frame.
        # For the endoscope this would be the identity (at initialization)
        # FERNANDO added '+12.5' on the translation matrix for the endoscope
        # two_bending model
        self.T_0 = tf.translation_matrix(
            [self.pos_canal[0],
             self.pos_canal[1],
             0]).dot(
            tf.rotation_matrix(np.deg2rad(self.dth), [0, 1, 0]))
        self.T_0_se3 = SE3.from_matrix(self.T_0)

    def setJointValues(self, q):
        # if we don't care about endoscope rotation, we provide only 3 numbers
        # which are two bendings and the insertion
        if len(q) < 4:
            self.q[0] = q[0]
            self.q[1] = q[1]
            self.q[3] = q[2]
        else:
            self.q = q

    def getJointValues(self):
        return self.q

    def computeSectionTransform(self, q_s):
        """
        Compute the transformtion of a section characterized byjoint values q_s
        :param q_s: joint values of the section in terms of curvature,
        rotation and length
        :return: Ts, the associated transformation matrix
        """
        cr = np.cos(q_s[2])
        sr = np.sin(q_s[2])
        # expression with two curvatures and a rotation
        ksi = np.array([0, 0, 1, q_s[0]*cr - q_s[1] *
                        sr, q_s[0]*sr + q_s[1]*cr, 0])
        T_s = SE3.exp(ksi * q_s[3])
        return T_s

    def computePose(self):
        """
        Compute the robot model
        :param q: joint values in the [curvature, rotation angle, insertion]
        format
        :return:
        """
        # initialize straight sections (non-zero curvature to avoid
        # singularities)
        curv = 0.00001 * np.ones(self.nsections, dtype=float)
        curv2 = 0.00001 * np.ones(self.nsections, dtype=float)
        # initialize own rotation at 0 for all
        rotation = np.zeros(self.nsections, dtype=float)
        lengths = list(self.lengths)  # length of the sections
        Transformations = []

        # Change the radius/rotation only of the active (i.e. bent) section
        for i in range(self.nsections):
            if self.active[i] == 1:
                curv[i] = self.q[0]
                curv2[i] = self.q[1]
                rotation[i] = self.q[2]
            if i == 0:
                Ti = self.computeSectionTransform(
                    [curv[i], curv2[i], rotation[i], self.q[3]])
                Transformations.append(self.T_0_se3.dot(Ti))
            else:
                Ti = self.computeSectionTransform(
                    [curv[i], curv2[i], rotation[i], lengths[i]])
                Transformations.append(Transformations[i - 1].dot(Ti))

        return Transformations

    def computeShape(self, q=None):
        """
        Compute the robot shape in its base frame
        :param q: (optional) joint positions
        :return: an array of arrays, each subarray being a 10-element shape
        vector for a given section
        """

        if q is not None:
            self.setJointValues(q)

        # initialize straight sections (non-zero curvature to avoid
        # singularities)
        curv = np.ones(self.nsections, dtype=float) * 0.00001
        curv2 = 0.00001 * np.ones(self.nsections, dtype=float)
        # initialize own rotation at 0 for all
        rotation = np.zeros(self.nsections, dtype=float)
        lengths = list(self.lengths)  # length of the sections
        Shape = []

        # Change the radius/rotation only of the active (i.e. bent) section
        Ti = copy.deepcopy(self.T_0_se3)

        for i in range(self.nsections):
            if i == 0:
                l0 = self.q[3]  # d is the insertion of the robot
            else:
                l0 = lengths[i]
            section_shape = []
            if self.active[i] == 1:
                curv[i] = self.q[0]
                curv2[i] = self.q[1]
                rotation[i] = self.q[2]

            for lj in np.linspace(start=0, stop=l0, num=10):
                Tc = Ti.dot(self.computeSectionTransform(
                    [curv[i], curv2[i], rotation[i], lj]))
                p = Tc.as_matrix()[:-1, 3]
                section_shape.append(p)
            Ti = Ti.dot(self.computeSectionTransform(
                [curv[i], curv2[i], rotation[i], l0]))
            Shape.append(section_shape)

        return Shape

    def computeModel(self, q=None, jac=False, Tb_w=None):
        """
        Compute the overall robot model
        :param q: (optional) joint values
        :param jac: boolean flag to compute the Jacobian as well
        :return: Transformation matrix to the robot tip,
                 plus tip Jacobian in the base frame if jac=True
        """
        if q is not None:
            self.setJointValues(q)
        if jac is False:
            return self.computePose()[self.nsections - 1]
        else:
            return self.computeJacobian_new(Tb_w)

    def computeJacobian(self):
        """
        Compute the Jacobian of the robot. This code assumes that the robot has
        a first straight section with  has a variable length, a second
        bended/rotated section, and subsequent straight sections. The code
        computes the Jacobian in terms of screw velocity, i.e. translational
        speed and rotational velocity
        :return: Tip transformation and Jacobian expressed in the base frame
        """
        k = self.q[0]
        phi = self.q[1]

        Jv = np.zeros((6, 3))
        J = np.zeros((6, 3))

        Trobot = self.computePose()

        # Jacobian of the first portion.
        # We are only interested in the third column,
        # since curvature and rotation are 0 (straight portion)
        Jv[:, 2] = SE3.adjoint(self.T_0_se3).dot(np.array([0, 0, 1, 0, 0, 0]))
        J[:, 2] = SE3.adjoint(self.T_0_se3).dot(np.array([0, 0, 1, 0, 0, 0]))

        # Jacobian of the second portion.
        # Here l is fixed, so we'll only keep the two first columns
        l = self.lengths[1]
        l2 = self.lengths[2]
        cp = np.cos(phi)
        sp = np.sin(phi)

        if 0 == k:
            vs_1 = np.array([[-cp * l * l / 2.0, 0, 0],
                             [-sp * l * l / 2.0, 0, 0],
                             [0, 0, 1],
                             [-l * sp, 0, 0],
                             [l * cp, 0, 0],
                             [0, 1, 0]])
        else:
            dkk = 1.0 / (k * k)  # divider by k squared
            ckl = np.cos(k * l)
            skl = np.sin(k * l)

            # velocity screw of the section wrt to kinematic variables
            vs_1 = np.array([[cp * (ckl - 1.0) * dkk, 0, 0],
                             [sp * (ckl - 1.0) * dkk, 0, 0],
                             [-(skl - (k * l)) * dkk, 0, 1],
                             [-l * sp, 0, -k * sp],
                             [l * cp, 0, k * cp],
                             [0, 1, 0]])

        # screw is multiplied by the adjoint transform to the base of the
        # bendable/rotated section, to express it in the correct frame
        Jv[:, :2] = SE3.adjoint(Trobot[0]).dot(vs_1[:, :2])

        # Put the Jacobian in the correct frame by multiplying by the adjoint
        # of the inverse translation from tip to base
        T_tb = np.eye(4)
        T_tb[:3, 3] = -Trobot[2].as_matrix()[:3, 3]

        J[:, :2] = SE3.adjoint(SE3.from_matrix(T_tb)) @ Jv[:, :2]
        # J = Trobot[1].inv().adjoint() @ J

        return Trobot[self.nsections - 1], J, Jv

    def computeJacobian_new(self, Tb_w=None):
        """
        Compute the robot jacobian, assuming a 3 links robot with :
            - a first variable length section
            - a second bendable/rotated section with fixed length
            - a third fixed length section
        :param Tb_w: (optional) Transform from world frame to base frame as a
                     SE3 liegroup object
        :return: Tr_b, Tr_w, Xr_b, Xr_w, Jx_b, Jx_w, zrb, zrw, Jz_b, Jz_w:
            T is a transform in SE3 to the tip of the robot
            X is the position
            Jx is the position Jacobian
            z is the tip tangent vector
            Jz is the tip tangent Jacobian

            b is expressed in the robot base frame
            w in the world frame
        """

        if Tb_w is None:
            Tb_w = SE3.from_matrix(np.eye(4))

        Hb_w = Tb_w.as_matrix()

        Trobot = self.computePose()
        Tr_b = Trobot[2]
        Tr_w = Tb_w.dot(Tr_b)

        Xr_b = Tr_b.as_matrix()[:3, 3]
        zr_b = Tr_b.as_matrix()[:3, 2]
        Xr_w = Tr_w.as_matrix()[:3, 3]
        zr_w = Tr_w.as_matrix()[:3, 2]

        # Computing Jacobians using finite difference for quick results.
        # @TODO later: implement analytical jacobian
        q_ = self.q
        dq = 0.00001
        Jx_b = np.zeros((3, 4))
        Jz_b = np.zeros((3, 4))
        Jx_w = np.zeros((3, 4))
        Jz_w = np.zeros((3, 4))

        for i in range(len(self.q)):
            self.q[i] += dq
            Tpert_b = self.computePose()[2]
            Tpert_w = Tb_w.dot(Tpert_b)
            Hpert_b = Tpert_b.as_matrix()
            Hpert_w = Tpert_w.as_matrix()

            Xpert_b = Hpert_b[:3, 3]
            zpert_b = Hpert_b[:3, 2]
            Xpert_w = Hpert_w[:3, 3]
            zpert_w = Hpert_w[:3, 2]

            Jx_b[:, i] = (Xpert_b - Xr_b) / dq
            Jx_w[:, i] = (Xpert_w - Xr_w) / dq
            Jz_b[:, i] = (zpert_b - zr_b) / dq
            Jz_w[:, i] = (zpert_w - zr_b) / dq
            self.q[i] -= dq

        return Tr_b, Tr_w, Xr_b, Xr_w, Jx_b, Jx_w, zr_b, zr_w, Jz_b, Jz_w
