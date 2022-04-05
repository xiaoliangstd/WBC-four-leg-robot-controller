import numpy as np
import math

def rotx(x):
    s = math.sin(x)
    c = math.cos(x)

    result = np.mat([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

    return result


def roty(y):
    s = math.sin(y)
    c = math.cos(y)

    result = np.mat([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])

    return result


def rotz(z):
    s = math.sin(z)
    c = math.cos(z)

    result = np.mat([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])

    return result


class GroundEstimator:
    def __init__(self, robot):
        self._robot = robot
        self._equation_coeff = np.mat([[0.0, 0.0 ,0.0]]).transpose()
        self._ground_yaw = 0.0
        self._ground_pitch = 0.0
        self._ground_roll = 0.0
        self._ground_posture_mat = np.mat(np.eye(3))

    def run(self,current_time):
        leg_contact_state = self._robot.getFootContactState()
        foot_pos_in_gravity_frame = self._robot.getFootPosInGravityFrame()

        num_legs_contact_on_ground = sum(leg_contact_state)

        if num_legs_contact_on_ground < 3:
            return

        coeff_mat = np.mat(np.zeros([num_legs_contact_on_ground, 3]))
        result_mat = np.mat(np.zeros([num_legs_contact_on_ground, 1]))

        row_index = 0
        for i in range(4):
            if leg_contact_state[i] == 1:
                for j in range(3):
                    coeff_mat[row_index, j] = foot_pos_in_gravity_frame[i][j]
                result_mat[row_index, 0] = -1
                row_index += 1

        self._equation_coeff = np.linalg.pinv(coeff_mat) * result_mat

        ground_z_aixs = [self._equation_coeff[0, 0], self._equation_coeff[1, 0], self._equation_coeff[2, 0]]
        ground_z_aixs /= np.linalg.norm(ground_z_aixs)

        roll, pitch, _= self._robot.getRobotEuler()
        robot_positure_mat = rotz(0.0) * roty(pitch) * rotx(roll)

        robot_x_axis = [robot_positure_mat[0, 0], robot_positure_mat[1, 0], robot_positure_mat[2, 0]]
        robot_x_axis /= np.linalg.norm(robot_x_axis)


        ground_y_axis = np.cross(ground_z_aixs, robot_x_axis)
        ground_y_axis /= np.linalg.norm(ground_y_axis)

        ground_x_axis = np.cross(ground_y_axis, ground_z_aixs)
        ground_x_axis /= np.linalg.norm(ground_x_axis)

        for i in range(3):
            self._ground_posture_mat[i, 0] = ground_x_axis[i]
            self._ground_posture_mat[i, 1] = ground_y_axis[i]
            self._ground_posture_mat[i, 2] = ground_z_aixs[i]

        self._ground_yaw = math.atan2(self._ground_posture_mat[1, 0], self._ground_posture_mat[0, 0])
        self._ground_pitch = math.atan2(-self._ground_posture_mat[2, 0],
                                        math.sqrt(self._ground_posture_mat[2, 1]**2 + self._ground_posture_mat[2, 2]**2))
        self._ground_roll = math.atan2(self._ground_posture_mat[2, 1], self._ground_posture_mat[2, 2])














