import gait
import stanceLeg
import swingLeg
import stateEstimator
import groundEstimator
import WbcController

class Locomotion:
    def __init__(self, robot):
        self._ground_estimator = groundEstimator.GroundEstimator(robot)
        self._state_estimator = stateEstimator.StateEstimator(robot)
        self._gait_generator = gait.GaitGenerator(robot)
        self._swing_leg = swingLeg.SwingLeg(robot, self._gait_generator, self._state_estimator, self._ground_estimator)
        self._stance_leg = stanceLeg.StanceLeg(robot, self._gait_generator, self._state_estimator, self._ground_estimator)
        self._wbc_controller = WbcController.WbcController(robot, self._swing_leg, self._stance_leg, self._gait_generator, self._state_estimator)

    def run(self, current_time, desired_x_speed, desired_y_speed, desired_twist_speed, desired_robot_height):
        self._ground_estimator.run(current_time)
        self._state_estimator.run(current_time)
        self._gait_generator.run(current_time)
        self._swing_leg.run(current_time, desired_x_speed, desired_y_speed, desired_twist_speed)
        self._stance_leg.run(current_time, desired_x_speed, desired_y_speed, desired_twist_speed, desired_robot_height)
        self._wbc_controller.run(current_time)
