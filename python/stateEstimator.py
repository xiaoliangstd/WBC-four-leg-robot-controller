import numpy as np
import KFStateEstimator

class StateEstimator:
    def __init__(self, robot):
        self._state_estimator = KFStateEstimator.KFStateEstimator(0.005, 1e10, 1e-4)
        self._robot = robot

    def run(self, current_time):
        foot_pos_in_gravity_frame = self._robot.getFootPosInGravityFrame()
        foot_pos_in_gravity_frame = np.asarray(foot_pos_in_gravity_frame).flatten()

        foot_vel_in_gravity_frame = self._robot.getFootVelocityInGravityFrame()
        foot_vel_in_gravity_frame = np.asarray(foot_vel_in_gravity_frame).flatten()


        angular_vel = self._robot.getRobotAngulurVelocity()

        leg_contact_state = self._robot.getFootContactState()

        self._state_estimator.run(foot_pos_in_gravity_frame,
                                  foot_vel_in_gravity_frame,
                                  angular_vel,
                                  leg_contact_state)

    def getEstimatedVelocity(self):
        estimated_velocity = self._state_estimator.getEstimatedVelocity()

        return estimated_velocity

    def getEstimatedHeight(self):
        estimated_height = self._state_estimator.getEstimatedHeight()

        return estimated_height











