import numpy as np


class StrasModel:
    """Forward-kinematics model for the two arms and endoscope."""
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
        self.endoscopeShapeFlag = False

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

        Te, _, _, _, Je, _, _, _, _, _ = self.robend.computeModel(q_end, jac=True)
        (Tr_e, Tr_0, _, _, Jr_e, Jr_0, zr_e, _, Jzr_e, _) = (
            self.robright.computeModel(q_right, jac=True, Tb_w=Te))
        (Tl_e, Tl_0, _, _, Jl_e, Jl_0, _, _, _, _) = (
            self.robleft.computeModel(q_left, jac=True, Tb_w=Te))

        return Tr_e.as_matrix(), Jr_e, Tl_e.as_matrix(), Jl_e,\
            Tr_0.as_matrix(), Jr_0, Tl_0.as_matrix(), Jl_0, Te.as_matrix(), Je, \
            Jzr_e, zr_e

 

    def computeShape(self, q):
        """Compute arm and endoscope polylines for a joint configuration.

        Args:
            q (list): configuration space q value list

        Returns:
            Arm section polylines, the endoscope tip pose, and optionally its
            section polylines.
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