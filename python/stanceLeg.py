import math
import sys

import numpy as np

import WBC_CONTROLLER

import gait



class StanceLeg:
    def __init__(self, robot, gait_generator, state_estimator, ground_estimator):
        self._robot = robot
        self._ground_estimator = ground_estimator
        self._state_estimator = state_estimator
        self._gait_generator = gait_generator

        self._desired_euler = [0.0]*3
        self._desired_pos_in_gravity_frame = [0.0] * 3
        self._desired_angular_velocity_in_gravity_frame = [0.0]*3
        self._desired_linear_velocity_in_gravity_frame = [0.0]*3


    def run(self, current_time, desired_x_speed, desired_y_speed, desired_twist_speed, desired_robot_height):
        desired_linear_velocity = self._ground_estimator._ground_posture_mat * np.mat([[desired_x_speed,
                                                                                        desired_y_speed,
                                                                                        0]]).transpose()

        self._desired_linear_velocity_in_gravity_frame = [float(desired_linear_velocity[0][0]),
                                                          float(desired_linear_velocity[1][0]),
                                                          float(desired_linear_velocity[2][0])]

        self._desired_angular_velocity_in_gravity_frame = [0.0, 0.0, desired_twist_speed]
        self._desired_euler = [self._ground_estimator._ground_roll,
                               self._ground_estimator._ground_pitch,
                               0.0]

        self._desired_pos_in_gravity_frame = [0.0, 0.0, desired_robot_height]









