import math
import sys

import gait
import copy
import numpy as np

_HIP_POSITION_IN_BODY_FRAME = [0.17, -0.13, 0.0,
                              0.17, 0.13, 0.0,
                              -0.17, -0.13, 0.0,
                              -0.17, 0.13, 0.0]

class SwingLeg:
    def __init__(self, robot, gait_generator, state_estimator, ground_estimator):
        self._robot = robot
        self._ground_estimator = ground_estimator
        self._state_estimator = state_estimator
        self._gait_generator = gait_generator
        self._last_leg_state = [gait.STANCE]*4


        self._phase_switch_foot_gravity_position = [(0.17, -0.13, -0.2), (0.17, 0.13, -0.2), (-0.17, -0.13, -0.2),
                                                    (-0.17, 0.13, -0.2)]

        self._desired_foot_pos = [0.0]*12





    def swingTrajectoryGenerate(self, start, end, lift_height,phase):

        phase = 2 * math.pi * phase

        trajectory_point = np.mat([0.0, 0.0, 0.0]).transpose()

        trajectory_point = (end - start) * (phase - math.sin(phase)) / (2 * math.pi) + start



        z_lo = max(start[2][0], end[2][0])

        trajectory_point[2][0] = lift_height * (1 - math.cos(phase)) / 2 + z_lo


        return trajectory_point


    def run(self, current_time, desired_x_speed, desired_y_speed, desired_twist_speed):
        new_leg_state = self._gait_generator._desired_leg_state

        desired_linear_velocity = self._ground_estimator._ground_posture_mat * np.mat([[desired_x_speed,
                                                                                        desired_y_speed,
                                                                                        0]]).transpose()

        desired_linear_velocity = [desired_linear_velocity[0,0],
                                   desired_linear_velocity[1,0],
                                   desired_linear_velocity[2,0]]

        desired_angular_velocity = [0.0, 0.0, desired_twist_speed]

        for leg_id, state in enumerate(new_leg_state):
            if (state == gait.SWING
                    and state != self._last_leg_state[leg_id]):

                foot_pos_in_g_frame = self._robot.getFootPosInGravityFrame()[leg_id]

                self._phase_switch_foot_gravity_position[leg_id] = foot_pos_in_g_frame

        self._last_leg_state = copy.deepcopy(new_leg_state)



        linear_velocity = self._state_estimator.getEstimatedVelocity()
        # linear_velocity = self._robot.getRobotLinearVelocity()

        angular_velocity = self._robot.getRobotAngulurVelocity()


        euler_angle = self._robot.getRobotEuler()
        euler_angle = list(euler_angle)
        euler_angle[2] = 0.0
        quat_from_euler_angle = self._robot._pybullet_interface.getQuaternionFromEuler(euler_angle)

        for leg_id, leg_state in enumerate(self._gait_generator._leg_state):
            if leg_state in (gait.STANCE,
                             gait.EARLY_CONTACT,
                             gait.LOSE_CONTACT):
                continue


            hip_position_in_body_frame = [_HIP_POSITION_IN_BODY_FRAME[3*leg_id], _HIP_POSITION_IN_BODY_FRAME[3*leg_id + 1],
                                          _HIP_POSITION_IN_BODY_FRAME[3*leg_id + 2]]


            hip_position_in_gravity_frame, _ = \
                self._robot._pybullet_interface.multiplyTransforms([0.0, 0.0, 0.0],
                                                                    self._robot._pybullet_interface.invertTransform([0, 0, 0],quat_from_euler_angle)[1],
                                                                   hip_position_in_body_frame,
                                                                   [0, 0, 0, 1])





            linear_velocity_mat = np.mat([linear_velocity[0],
                                      linear_velocity[1],
                                      linear_velocity[2]]).transpose()


            hip_position_in_gravity_frame = np.mat([hip_position_in_gravity_frame[0],
                                                    hip_position_in_gravity_frame[1],
                                                    hip_position_in_gravity_frame[2]]).transpose()


            desired_foot_placemant = hip_position_in_gravity_frame + \
                                     linear_velocity_mat * self._gait_generator._stance_duration[leg_id] / 2




            desired_foot_placemant[2] = (-1 - self._ground_estimator._equation_coeff[0, 0] * desired_foot_placemant[0] - \
                                        self._ground_estimator._equation_coeff[1, 0] * desired_foot_placemant[1]) / self._ground_estimator._equation_coeff[2, 0]




            start = np.mat([self._phase_switch_foot_gravity_position[leg_id][0],
                            self._phase_switch_foot_gravity_position[leg_id][1],
                            self._phase_switch_foot_gravity_position[leg_id][2]]).transpose()





            lift_height = 0.15
            trajectory = self.swingTrajectoryGenerate(start,
                                                      desired_foot_placemant,
                                                      lift_height,
                                                      self._gait_generator._normalized_phase[leg_id])



            for i in range(3):
                self._desired_foot_pos[3*leg_id + i] = float(trajectory[i][0])





















