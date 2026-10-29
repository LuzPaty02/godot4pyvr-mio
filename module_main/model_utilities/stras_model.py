import numpy as np
from numpy.lib.stride_tricks import as_strided
import module_main.model_utilities.robot_model
from liegroups import SE3
hardwareLimits = False


class StrasModel:
    """_summary_
    """
    def __init__(self, robot_right, robot_left, endoscope):
        """STRAS instance to pass STRAS-EASE parameters.

        Args:
            robot_right (instance): Instance of right arm
            robot_left (instance): Instance of left arm
            endoscope (instance): Instance of endoscope body
        """
        self.robright = robot_right
        self.robleft = robot_left
        self.robend = endoscope
        self.render = None
        self.tipRpos = None
        self.tipLpos = None
        self.tipEpos = None
        self.endoscopeShapeFlag = False

    def setRender(self, render):
        """Setting the render instance

        Args:
            render (instance): render instance to be used by VTK visualization 
        """
        self.render = render

    def defineEndoscopeShapeOn(self):
        self.endoscopeShapeFlag = True

    def computemodels(self, q):
        """Compute the robot models for a given joint position configuration space q.

        Args:
            q (list): joint positions in the form [qR, qL, qE]

        Returns:
            list: Tr_e, Jr_e, Tl_e, Jl_e, Tr_0, Jr_0, Tl_0, Jl_0, Te, Je 
            (Translation/Jacobian for right (r) and left (l) instruments and the endoscope (e), 
            in the endoscope frame (e) or in the fixed frame (0)
        """

        q_right = q[0:3]
        q_left = q[3:6]
        q_end = q[6:]

        # q_right_len = len(q_right)
        # q_left_len = len(q_left)
        # q_end_len = len(q_end)

        L_arms = 18.5
        L_endo = 185

        # Fernando added
        if hardwareLimits:
            # Hardware limits on simulation
            # Right Bending limits
            if q_right[0] > (np.pi/(2*L_arms)):
                q_right[0] = (np.pi/(2*L_arms))
            if q_right[0] < -(np.pi/(2*L_arms)):
                q_right[0] = -(np.pi/(2*L_arms))
            # Right Translation limits
            if q_right[2] > 75:
                q_right[2] = 75
            if q_right[2] < 0:
                q_right[2] = 0

            # Left Bending limits
            if q_left[0] > (np.pi/(2*L_arms)):
                q_left[0] = (np.pi/(2*L_arms))
            if q_left[0] < -(np.pi/(2*L_arms)):
                q_left[0] = -(np.pi/(2*L_arms))
            # Left Translation limits
            if q_left[2] > 75:
                q_left[2] = 75
            if q_left[2] < 0:
                q_left[2] = 0

            # Endoscope Bending limits
            if q_end[0] > (np.pi/(2*L_endo)):
                q_end[0] = (np.pi/(2*L_endo))
            if q_end[0] < -(np.pi/(2*L_endo)):
                q_end[0] = -(np.pi/(2*L_endo))
            if q_end[1] > (np.pi/(2*L_endo)):
                q_end[1] = (np.pi/(2*L_endo))
            if q_end[1] < -(np.pi/(2*L_endo)):
                q_end[1] = -(np.pi/(2*L_endo))
            # Endoscope TRANSLATION limits
            if q_end[3] > 75:
                q_end[3] = 75
            if q_end[3] < -25:
                q_end[3] = -25

        # Compute left and right robot pose and Jacobian in camera frame,
        # and endoscope pose and Jacobian in fixed frame

        # Endoscope
        foo1 = self.robend.computeModel(q_end, jac=True)
        Te = foo1[0]
        ign = foo1[1]
        Xe = foo1[2]
        ign = foo1[3]
        Je = foo1[4]
        ign = foo1[5]
        ze = foo1[6]
        ign = foo1[7]
        Jze = foo1[8]
        ign = foo1[9]

        # Right arm
        foo2 = self.robright.computeModel(q_right, jac=True, Tb_w=Te)
        Tr_e = foo2[0]
        Tr_0 = foo2[1]
        Xr_e = foo2[2]
        Xr_0 = foo2[3]
        Jr_e = foo2[4]
        Jr_0 = foo2[5]
        zr_e = foo2[6]
        zr_0 = foo2[7]
        Jzr_e = foo2[8]
        Jzr_0 = foo2[0]

        # Left arm
        foo3 = self.robleft.computeModel(q_left, jac=True, Tb_w=Te)
        Tl_e = foo3[0]
        Tl_0 = foo3[1]
        Xl_e = foo3[2]
        Xl_0 = foo3[3]
        Jl_e = foo3[4]
        Jl_0 = foo3[5]
        zl_e = foo3[6]
        zl_0 = foo3[7]
        Jzl_e = foo3[8]
        Jzl_0 = foo3[9]

        # # Compute left and right robot pose in fixed frame
        # Tr_0 = Te.dot(Tr_e)
        # Tl_0 = Te.dot(Tl_e)

        # Express robot jacobian in the base frame as well

        # # Compute left and right robot twist Jacobian in fixed frame
        # Jr_0_v = SE3.adjoint(Te) @ Jr_e_v
        # Jl_0_v = SE3.adjoint(Te) @ Jl_e_v
        #
        # # Express it as a velocity jacobian
        # T_tb = np.eye(4)
        # T_tb[:3, 3] = Tr_0.as_matrix()[:3, 3]
        # Jr_0 = SE3.adjoint(Tr_0.inv()) @ Jr_0_v
        #
        #
        # T_tb[:3, 3] = -Tl_0.as_matrix()[:3, 3]
        # #Jl_0 = SE3.adjoint(SE3.from_matrix(T_tb)) @ Jl_0_v
        #
        # Rl_e = Te.as_matrix()
        # Rl_e[:3,3] = 0
        # Jl_0 = SE3.adjoint(SE3.from_matrix(Rl_e).inv()) @ Jl_e

        # T_tb = np.eye(4)
        # T_tb[:3, 3] = -Tr_0.as_matrix()[:3, 3]
        # Jr_0 = SE3.adjoint(Tr_0.inv()) @ Jr_0
        #
        # T_tb[:3, 3] = -Tl_0.as_matrix()[:3, 3]
        # Jl_0 = SE3.adjoint(SE3.from_matrix(T_tb)) @ Jl_0

        return Tr_e.as_matrix(), Jr_e, Tl_e.as_matrix(), Jl_e,\
            Tr_0.as_matrix(), Jr_0, Tl_0.as_matrix(), Jl_0, Te.as_matrix(), Je, \
            Jzr_e, zr_e

 

    def computeShape(self, q):
        """Compute the shape of the stras robot arms. NOT USED.

        Args:
            q (list): configuration space q value list

        Returns:
            _type_: _description_
        """
        q_right = q[0:3]
        q_left = q[3:6]
        q_end = q[6:]

        # Get Shape of left and right arm, and endoscope tip pose
        Shape_R = self.robright.computeShape(q_right)
        Shape_L = self.robleft.computeShape(q_left)
        if self.endoscopeShapeFlag:
            Shape_Endo = self.robend.computeShape(q_end)
        Tip_e = self.robend.computeModel(q=q_end, jac=False)

        if self.endoscopeShapeFlag:
            return Shape_R, Shape_L, Tip_e, Shape_Endo
        else:
            return Shape_R, Shape_L, Tip_e,

    def currentFramesPositions(self, q, excludeRotation=True):
        """NOT USED.

        Args:
            q (list): configuration space q value list
            excludeRotation (bool, optional): _description_. Defaults to True.
        """
        # For compatibility purposes
        q_right = q[0:3]
        q_left = q[3:6]
        q_end = q[6:]

        # Get Shape of left and right arm, and endoscope tip pose
        Shape_R = self.robright.computeShape(q_right)
        Shape_L = self.robleft.computeShape(q_left)
        Tip_e = self.robend.computeModel(q=q_end, jac=False)

        # Get tip position of both arms and endoscope as np.matrix
        self.tipRpos = np.append(Shape_R[-1][-1], 1)
        self.tipLpos = np.append(Shape_L[-1][-1], 1)
        pretipEpos = Tip_e.as_matrix()

        # If rotation is included, else identity matrix
        if excludeRotation:
            pretipEpos[:3, :3] = np.eye(3)

        self.tipEpos = pretipEpos