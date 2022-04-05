import WBC_CONTROLLER
import gait
import math


alpha = 1e-5
friction_coeff = 0.4

kp_ori = 1.0
kd_ori = 1.0

kp_pos = 1.0
kd_pos = 1.0

kp_foot = 1.0
kd_foot = 1.0

weights = [20.0, 20.0, 5.0, 1.0, 1.0, 0.2]


class WbcController:
    def __init__(self, robot, swing_leg, stance_leg, gait_generator, state_estimator):
        self._robot = robot
        self._wbc_controller = WBC_CONTROLLER.WBC(kp_ori, kd_ori, kp_pos, kd_pos,
                                             kp_foot, kd_foot, weights,
                                             alpha, friction_coeff)
        self._swing_leg = swing_leg
        self._stance_leg = stance_leg
        self._gait_generator = gait_generator
        self._state_estimator = state_estimator

    def run(self, current_time):
        leg_contact_state = [int(0)]*4

        for leg_id, leg_state in enumerate(self._gait_generator._leg_state):
            if leg_state  in (gait.STANCE,
                             gait.EARLY_CONTACT,
                             gait.LOSE_CONTACT):
                leg_contact_state[leg_id] = int(1)

        linear_velocity_in_gravity_frame = self._state_estimator.getEstimatedVelocity()
        # linear_velocity_in_gravity_frame = self._robot.getRobotLinearVelocity()
        angular_velocity_in_gravity_frame = self._robot.getRobotAngulurVelocity()
        euler_angle = self._robot.getRobotEuler()
        euler_angle = list(euler_angle)
        euler_angle[2] = 0.0
        # foot_pos_in_gravity_frame = self._robot.getFootPosInGravityFrame()
        # foot_pos_in_gravity_frame = np.asarray(foot_pos_in_gravity_frame).flatten()

        joint_pos = self._robot.getJointAngles()
        joint_vel = self._robot.getJointVelocity()





        joint_cmd = self._wbc_controller.getJointCmd(self._stance_leg._desired_euler,
                                         self._stance_leg._desired_pos_in_gravity_frame,
                                         self._swing_leg._desired_foot_pos,

                                         self._stance_leg._desired_angular_velocity_in_gravity_frame,
                                         self._stance_leg._desired_linear_velocity_in_gravity_frame,
                                         [0.0]*12,

                                         [0.0]*3,
                                         [0.0]*3,
                                         [0.0]*12,

                                         leg_contact_state,

                                         euler_angle,
                                         angular_velocity_in_gravity_frame,
                                         linear_velocity_in_gravity_frame,

                                         joint_pos,
                                         joint_vel)

        # for i in range(12):
        #     print(joint_cmd[3*i: 3*i + 3], '\n')
        # print('\n\n')


        for i in range(12):
            self._robot._joint_action[i][0] = joint_cmd[1 + 3*i]
            self._robot._joint_action[i][1] = joint_cmd[2 + 3*i]
            self._robot._joint_action[i][2] = 100.0
            self._robot._joint_action[i][3] = 1.0
            self._robot._joint_action[i][4] = joint_cmd[0 + 3*i]
            # self._robot._joint_action[i][4] = 0.0
