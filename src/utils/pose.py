from math import cos, sin

import numpy as np
from common import DEG, rotation_z, translation
from geometry_msgs.msg import Pose2D

# Matrix to convert between XYZ and XY
F = np.array([[1, 0, 0], [0, 1, 0]])


class Pose3D(np.ndarray):
    """
    Definition of a robot pose in 3 DOF (x, y, yaw). The class inherits from a ndarray.
    This class extends the ndarray with the :math:`oplus` and :math:`ominus` operators and the corresponding Jacobians.
    """

    def __new__(cls, input_array=np.array([[0.0, 0.0, 0.0]]).T):
        """
        Constructor of the class. It is called when the class is instantiated. It is required to extend the ndarry numpy class.

        :param input_array: array used to initialize the class
        :returns: the instance of a Pose4D class object
        """

        # Input array is an already formed ndarray instance
        # We first cast to be our class type
        obj = np.asarray(input_array).view(cls)
        # Finally, we must return the newly created object:
        return obj

    def __init__(self, input_array=np.array([[0.0, 0.0, 0.0]]).T):

        assert input_array.shape == (3, 1), "mean must be a 3x1 vector"

    def distance(self, point):
        return np.linalg.norm([self.x - point[0], self.y - point[1]])

    @staticmethod
    def from_values(x, y, theta):
        return Pose3D(np.array([[x, y, theta]]).T)

    def clone(self):
        return Pose3D.from_values(self.x, self.y, self.theta)

    def to_Pose2D(self):
        return Pose2D(self.x, self.y, self.theta)

    def to_Pose2D_array(self):
        return np.array([self.x, self.y])

    @staticmethod
    def from_Pose2D(pose: Pose2D):
        return Pose3D.from_values(pose.x, pose.y, pose.theta)

    @property
    def x(self):
        return self[0][0]

    @property
    def y(self):
        return self[1][0]

    @property
    def theta(self):
        return self[2][0]

    def __str__(self) -> str:
        return f"x: {self.x:.2f} y: {self.y:.2f} θ: {np.degrees(self.theta):.2f}{DEG}"

    def oplus(AxB, BxC):
        """
        Given a Pose3D object *AxB* (the self object) and a Pose3D object *BxC*, it returns the Pose3D object *AxC*.

        .. math::
            \\mathbf{{^A}x_B} &= \\begin{bmatrix} ^Ax_B & ^Ay_B & ^A\\psi_B \\end{bmatrix}^T \\\\
            \\mathbf{{^B}x_C} &= \\begin{bmatrix} ^Bx_C & ^By_C & & ^B\\psi_C \\end{bmatrix}^T \\\\

        The operation is defined as:

        .. math::
            \\mathbf{{^A}x_C} &= \\mathbf{{^A}x_B} \\oplus \\mathbf{{^B}x_C} =
            \\begin{bmatrix}
                ^Ax_B + ^Bx_C  \\cos(^A\\psi_B) - ^By_C  \\sin(^A\\psi_B) \\\\
                ^Ay_B + ^Bx_C  \\sin(^A\\psi_B) + ^By_C  \\cos(^A\\psi_B) \\\\
                ^A\\psi_B + ^B\\psi_C
            \\end{bmatrix}
            :label: eq-oplus3dof

        :param BxC: C-Frame pose expressed in B-Frame coordinates
        :returns: C-Frame pose expressed in A-Frame coordinates

        ** To be completed by the student **
        """

        a_x_b, a_y_b, a_psi_b = np.squeeze(AxB)
        b_x_c, b_y_c, b_psi_c = np.squeeze(BxC)

        return Pose3D(
            np.array(
                [
                    [
                        a_x_b + b_x_c * cos(a_psi_b) - b_y_c * sin(a_psi_b),
                        a_y_b + b_x_c * sin(a_psi_b) + b_y_c * cos(a_psi_b),
                        a_psi_b + b_psi_c,
                    ]
                ]
            ).T
        )

    def J_1oplus(AxB, BxC):
        """
        Jacobian of the pose compounding operation (eq. :eq:`eq-oplus3dof`) with respect to the first pose:

        .. math::
            J_{1\\oplus}=\\frac{\\partial  ^Ax_B \\oplus ^Bx_C}{\\partial ^Ax_B} =
            \\begin{bmatrix}
                1 & 0 &  -^Bx_C \\sin(^A\\psi_B) - ^By_C \\cos(^A\\psi_B) \\\\
                0 & 1 &  ^Bx_C \\cos(^A\\psi_B) - ^By_C \\sin(^A\\psi_B) \\\\
                0 & 0 & 1
            \\end{bmatrix}
            :label: eq-J1oplus3dof

        The method returns a numerical matrix containing the evaluation of the Jacobian for the pose *AxB* (the self object) and the :math:`2^{nd}` posepose *BxC*.

        :param BxC: 2nd pose
        :returns: Evaluation of the :math:`J_{1\\oplus}` Jacobian of the pose compounding operation with respect to the first pose (eq. :eq:`eq-J1oplus3dof`)

        ** To be completed by the student **
        """
        _, _, psi = np.squeeze(AxB)
        x, y, _ = np.squeeze(BxC)

        return np.array(
            [
                [1, 0, -x * sin(psi) - y * cos(psi)],
                [0, 1, x * cos(psi) - y * sin(psi)],
                [0, 0, 1],
            ]
        )

    def J_2oplus(AxB):
        """
        Jacobian of the pose compounding operation (:eq:`eq-oplus3dof`) with respect to the second pose:

        .. math::
            J_{2\\oplus}=\\frac{\\partial  ^Ax_B \\oplus ^Bx_C}{\\partial ^Bx_C} =
            \\begin{bmatrix}
                \\cos(^A\\psi_B) & -\\sin(^A\\psi_B) & 0  \\\\
                \\sin(^A\\psi_B) & \\cos(^A\\psi_B) & 0  \\\\
                0 & 0 & 1
            \\end{bmatrix}
            :label: eq-J2oplus3dof

        The method returns a numerical matrix containing the evaluation of the Jacobian for the :math:`1^{st} posepose *AxB* (the self object).

        :returns: Evaluation of the :math:`J_{2\\oplus}` Jacobian of the pose compounding operation with respect to the second pose (eq. :eq:`eq-J2oplus3dof`)

        ** To be completed by the student **
        """
        _, _, psi = np.squeeze(AxB)
        return np.array(
            [
                [cos(psi), -sin(psi), 0],
                [sin(psi), cos(psi), 0],
                [0, 0, 1],
            ]
        )

    def ominus(AxB):
        """
        Inverse pose compounding of the *AxB* pose (the self objetc):

        .. math::
            ^Bx_A = \\ominus ^Ax_B =
            \\begin{bmatrix}
                -^Ax_B \\cos(^A\\psi_B) - ^Ay_B \\sin(^A\\psi_B) \\\\
                ^Ax_B \\sin(^A\\psi_B) - ^Ay_B \\cos(^A\\psi_B) \\\\
                -^A\\psi_B
            \\end{bmatrix}
            :label: eq-ominus3dof

        :returns: A-Frame pose expressed in B-Frame coordinates (eq. :eq:`eq-ominus3dof`)

        ** To be completed by the student **
        """
        x, y, psi = np.squeeze(AxB)
        return Pose3D(
            np.array(
                [
                    [
                        -x * cos(psi) - y * sin(psi),
                        x * sin(psi) - y * cos(psi),
                        -psi,
                    ]
                ]
            ).T
        )

    def J_ominus(AxB):
        """
        Jacobian of the inverse pose compounding operation (:eq:`eq-oplus3dof`) with respect the pose *AxB* (the self object):

        .. math::
            J_{\\ominus}=\\frac{\\partial  \\ominus ^Ax_B}{\\partial ^Ax_B} =
            \\begin{bmatrix}
                -\\cos(^A\\psi_B) & -\\sin(^A\\psi_B) &  ^Ax_B \\sin(^A\\psi_B) - ^Ay_B \\cos(^A\\psi_B) \\\\
                \\sin(^A\\psi_B) & -\\cos(^A\\psi_B) &  ^Ax_B \\cos(^A\\psi_B) + ^Ay_B \\sin(^A\\psi_B) \\\\
                0 & 0 & -1
            \\end{bmatrix}
            :label: eq-Jominus3dof

        Returns the numerical matrix containing the evaluation of the Jacobian for the pose *AxB* (the self object).

        :returns: Evaluation of the :math:`J_{\\ominus}` Jacobian of the inverse pose compounding operation with respect to the pose (eq. :eq:`eq-Jominus3dof`)
        **To be completed by the student**
        """
        x, y, psi = np.squeeze(AxB)
        s, c = sin(psi), cos(psi)
        return np.array(
            [
                [-c, -s, x * s - y * c],
                [s, -c, x * c + y * s],
                [0, 0, -1],
            ]
        )

    def boxplus(AxB, BxF):
        assert BxF.shape == (2, 1)

        return np.array(F @ (AxB.oplus(F.T @ BxF)))

    def J_1boxplus(AxB, BxF):
        assert BxF.shape == (2, 1)

        return np.array(F @ AxB.J_1oplus(F.T @ BxF))

    def J_2boxplus(AxB, BxF):
        assert BxF.shape == (2, 1)

        return np.array(F @ AxB.J_2oplus() @ F.T)

    def as_transformation(self):
        return translation(self.x, self.y) @ rotation_z(self.theta)


if __name__ == "__main__":
    robot_pose = Pose3D(np.array([1, 1, 0]).reshape((3, 1)))
    BxF = np.array([1, 1]).reshape((2, 1))
    print(robot_pose.boxplus(BxF))
    print(robot_pose.J_1boxplus(BxF))
    print(robot_pose.J_2boxplus(BxF))

    robot_pose = Pose3D(np.array([1, 1, np.pi / 2]).reshape((3, 1)))
    print(robot_pose.boxplus(BxF))
