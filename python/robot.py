import math
import numpy as np
MOTOR_ID_LIST = [1, 3, 4, 6, 8, 9, 11, 13, 14, 16, 18, 19]
_LEG_END_LINK_ID = {'FR': 5, 'FL': 10, 'RR': 15, 'RL': 20}




class Robot:
    def __init__(self, pybullet_interface,robot):
        self._robot = robot
        self._pybullet_interface = pybullet_interface
        self._joint_action = [[0, 0, 0, 0, 0]]*12

    def getRobotQuaternion(self):
        _, orn = self._pybullet_interface.getBasePositionAndOrientation(self._robot)
        return orn

    def getRobotEuler(self):
        _, orn = self._pybullet_interface.getBasePositionAndOrientation(self._robot)
        return self._pybullet_interface.getEulerFromQuaternion(orn)

    def getRobotPos(self):
        pos,_ = self._pybullet_interface.getBasePositionAndOrientation(self._robot)
        return pos

    def getJointAngles(self):

        joint_states = self._pybullet_interface.getJointStates(self._robot, MOTOR_ID_LIST)
        joint_angles = [0]*12

        for i in range(12):
            joint_angles[i] = joint_states[i][0]

        return joint_angles

    def getJointVelocity(self):
        joint_states = self._pybullet_interface.getJointStates(self._robot, MOTOR_ID_LIST)
        joint_velocity = [0] * 12

        for i in range(12):
            joint_velocity[i] = joint_states[i][1]

        return joint_velocity

    def printJointInfo(self):
        for joint_index in range(self._pybullet_interface.getNumJoints(self._robot)):
            print(self._pybullet_interface.getJointInfo(self._robot, joint_index))


    def getRobotLinearVelocity(self):
        robot_linear_velocity_in_world_frame = self._pybullet_interface.getBaseVelocity(self._robot)[0]
        yaw_angle = -self.getRobotEuler()[2]
        robot_linear_velocity_in_gravity_aligned_frame = \
            self._pybullet_interface.multiplyTransforms([0, 0, 0],
                                                        self._pybullet_interface.getQuaternionFromEuler([0, 0, yaw_angle]),
                                                        robot_linear_velocity_in_world_frame,
                                                        [0, 0, 0, 1])[0]

        return robot_linear_velocity_in_gravity_aligned_frame

    def getRobotAngulurVelocity(self):
        robot_angular_velocity_in_world_frame = self._pybullet_interface.getBaseVelocity(self._robot)[1]
        yaw_angle = -self.getRobotEuler()[2]
        robot_angular_velocity_in_gravity_aligned_frame = \
            self._pybullet_interface.multiplyTransforms([0, 0, 0],
                                                        self._pybullet_interface.getQuaternionFromEuler(
                                                            [0, 0, yaw_angle]),
                                                        robot_angular_velocity_in_world_frame,
                                                        [0, 0, 0, 1])[0]

        return robot_angular_velocity_in_gravity_aligned_frame

    def getFootPosInBodyFrame(self):
        leg_pos_in_body_frame = [None] * 4
        base_position, base_orientation = self._pybullet_interface.getBasePositionAndOrientation(self._robot)
        inverse_translation, inverse_rotation = self._pybullet_interface.invertTransform(
            base_position, base_orientation)

        for leg_id in range(4):
            leg_pos_in_world_frame = self._pybullet_interface.getLinkState(self._robot, (leg_id + 1) * 5)[0]
            link_local_position, _ = self._pybullet_interface.multiplyTransforms(
                inverse_translation, inverse_rotation, leg_pos_in_world_frame, (0, 0, 0, 1))
            leg_pos_in_body_frame[leg_id] = link_local_position

        return leg_pos_in_body_frame

    def getFootVelocityInBodyFrame(self):
        joint_vel = self.getJointVelocity()
        foot_vel = [0.0]*12

        for leg_id in range(4):
            single_joint_vel_mat = np.mat([[joint_vel[3*leg_id],
                                            joint_vel[3*leg_id + 1],
                                            joint_vel[3*leg_id + 2]]]).transpose()

            all_joint_angles = self.getJointAngles()
            contact_jacobian = self._pybullet_interface.calculateJacobian(self._robot, 5 * (leg_id + 1),
                                                                          [0, 0, 0],
                                                                          all_joint_angles,
                                                                          [0] * 12,
                                                                          [0] * 12)[0]
            contact_jacobian_mat = np.mat(np.zeros([3, 3]))

            for row in range(3):
                for col in range(3):
                    contact_jacobian_mat[row, col] = contact_jacobian[row][col + 6 + leg_id * 3]

            single_foot_vel_mat = contact_jacobian_mat * single_joint_vel_mat

            for i in range(3):
                foot_vel[3*leg_id+i] = single_foot_vel_mat[i, 0]

        return foot_vel

    def getFootVelocityInGravityFrame(self):
        foot_vel_in_gravity_frame = [0.0] * 12

        base_position, base_orientation = self._pybullet_interface.getBasePositionAndOrientation(self._robot)
        base_orientation = self._pybullet_interface.getEulerFromQuaternion(base_orientation)
        base_orientation = list(base_orientation)
        base_orientation[2] = 0.0
        base_orientation = self._pybullet_interface.getQuaternionFromEuler(base_orientation)

        foot_vel_in_body_frame = self.getFootVelocityInBodyFrame()

        for leg_id in range(4):
            single_foot_vel, _ = self._pybullet_interface.multiplyTransforms([0.0, 0.0, 0.0], base_orientation,
                                                                             foot_vel_in_body_frame[3*leg_id:3*leg_id+3],
                                                                             [0, 0, 0, 1])

            foot_vel_in_gravity_frame[3*leg_id: 3*leg_id+3] = single_foot_vel[:]

        return foot_vel_in_gravity_frame













    def getFootPosInGravityFrame(self):
        leg_pos_in_gravity_frame = [None] * 4
        base_position, base_orientation = self._pybullet_interface.getBasePositionAndOrientation(self._robot)
        base_orientation = self._pybullet_interface.getEulerFromQuaternion(base_orientation)
        base_orientation = list(base_orientation)
        base_orientation[0] = 0.0
        base_orientation[1] = 0.0
        base_orientation = self._pybullet_interface.getQuaternionFromEuler(base_orientation)
        inverse_translation, inverse_rotation = self._pybullet_interface.invertTransform(
            base_position, base_orientation)

        for leg_id in range(4):
            leg_pos_in_world_frame = self._pybullet_interface.getLinkState(self._robot, (leg_id + 1) * 5)[0]
            link_local_position, _ = self._pybullet_interface.multiplyTransforms(
                inverse_translation, inverse_rotation, leg_pos_in_world_frame, (0, 0, 0, 1))
            leg_pos_in_gravity_frame[leg_id] = link_local_position

        return leg_pos_in_gravity_frame

    def caculateJointAnglesFromFootPosInBaseFrame(self, leg_id, foot_pos_in_base_frame):
        base_position, base_orientation = self._pybullet_interface.getBasePositionAndOrientation(self._robot)
        foot_pos_in_world_frame, _ = self._pybullet_interface.multiplyTransforms(
            base_position, base_orientation, foot_pos_in_base_frame, (0, 0, 0, 1)
        )

        all_joint_angles = self._pybullet_interface.calculateInverseKinematics(
            self._robot, 5 * (leg_id + 1), foot_pos_in_world_frame)

        return all_joint_angles

    def caculateJointAnglesFromFootPosInGravityFrame(self, leg_id, foot_pos_in_gravity_frame):
        base_position, base_orientation = self._pybullet_interface.getBasePositionAndOrientation(self._robot)
        base_orientation = self._pybullet_interface.getEulerFromQuaternion(base_orientation)
        base_orientation = list(base_orientation)
        base_orientation[0] = 0.0
        base_orientation[1] = 0.0
        base_orientation = self._pybullet_interface.getQuaternionFromEuler(base_orientation)

        foot_pos_in_world_frame, _ = self._pybullet_interface.multiplyTransforms(base_position,
                                                                                 base_orientation,
                                                                                 foot_pos_in_gravity_frame,
                                                                                 [0, 0, 0, 1])
        all_joint_angles = self._pybullet_interface.calculateInverseKinematics(
            self._robot, 5 * (leg_id + 1), foot_pos_in_world_frame)

        return all_joint_angles

    def setFootPosInBaseFrame(self, leg_id, foot_pos_in_base_frame):
        desired_all_joints_angles = self.caculateJointAnglesFromFootPosInBaseFrame(leg_id, foot_pos_in_base_frame)
        desired_leg_joints_angles = desired_all_joints_angles[leg_id*3: (leg_id+1)*3]

        # motor_list = MOTOR_ID_LIST[leg_id*3: (leg_id+1)*3]

        for i in range(3):
            kp = 100.0
            if i == 0:
                kd = 1.0
            else:
                kd = 2.0
            self._joint_action[3 * leg_id + i] = [desired_leg_joints_angles[i], 0.0, kp, kd, 0.0]


        # self._pybullet_interface.setJointMotorControlArray(self._robot, motor_list,
        #                                                    controlMode = self._pybullet_interface.POSITION_CONTROL,
        #                                                    targetPositions = desired_leg_joints_angles)

    def setFootPosInGravityFrame(self, leg_id, foot_pos_in_gravity_frame):
        desired_all_joints_angles = self.caculateJointAnglesFromFootPosInGravityFrame(leg_id,
                                                                                      foot_pos_in_gravity_frame)

        desired_leg_joints_angles = desired_all_joints_angles[leg_id * 3: (leg_id + 1) * 3]

        # for i in range(3):
        #     kp = 1000.0
        #     if i == 0:
        #         kd = math.sqrt(10)
        #     else:
        #         kd = 2.0 * math.sqrt(10)
        #     self._joint_action[3*leg_id+i] = [desired_leg_joints_angles[i], 0.0, kp, kd, 0.0]

        for i in range(3):
            kp = 100.0
            if i == 0:
                kd = 1.0
            else:
                kd = 2.0
            self._joint_action[3*leg_id+i] = [desired_leg_joints_angles[i], 0.0, kp, kd, 0.0]

        # motor_list = MOTOR_ID_LIST[leg_id * 3: (leg_id + 1) * 3]

        # self._pybullet_interface.setJointMotorControlArray(self._robot, motor_list,
        #                                                    controlMode=self._pybullet_interface.POSITION_CONTROL,
        #                                                    targetPositions=desired_leg_joints_angles)

    def setFootPosInWorldFrame(self, leg_id, foot_pos_in_world_frame):
        all_joint_angles = self._pybullet_interface.calculateInverseKinematics(
            self._robot, 5 * (leg_id + 1), foot_pos_in_world_frame)

        desired_leg_joints_angles = all_joint_angles[leg_id * 3: (leg_id + 1) * 3]

        for i in range(3):
            kp = 100.0
            if i == 0:
                kd = 1.0
            else:
                kd = 2.0
            self._joint_action[3*leg_id+i] = [desired_leg_joints_angles[i], 0.0, kp, kd, 0.0]

        # motor_list = MOTOR_ID_LIST[leg_id * 3: (leg_id + 1) * 3]
        #
        # self._pybullet_interface.setJointMotorControlArray(self._robot, motor_list,
        #                                                    controlMode=self._pybullet_interface.POSITION_CONTROL,
        #                                                    targetPositions=desired_leg_joints_angles)


    def setFootForceInBaseFrame(self, leg_id, foot_force_in_base_frame):
        all_joint_angles = self.getJointAngles()
        contact_jacobian = self._pybullet_interface.calculateJacobian(self._robot, 5*(leg_id + 1),
                                                                     [0, 0, 0],
                                                                     all_joint_angles,
                                                                      [0]*12,
                                                                      [0]*12)[0]
        contact_jacobian_mat = np.mat(np.zeros([3, 3]))

        for row in range(3):
            for col in range(3):
                contact_jacobian_mat[row, col] = contact_jacobian[row][col+6+leg_id*3]

        foot_force_in_base_frame_mat = np.mat([0.0, 0.0, 0.0]).transpose()
        for i in range(3):
            foot_force_in_base_frame_mat[i, 0] = foot_force_in_base_frame[i]

        desired_torques_mat = contact_jacobian_mat.transpose() * foot_force_in_base_frame_mat

        for i in range(3):
            self._joint_action[3*leg_id+i] = [0, 0, 0, 0, desired_torques_mat[i, 0]]

        # print(desired_torques_mat, '\n\n')

        # motor_list = MOTOR_ID_LIST[leg_id * 3: (leg_id + 1) * 3]

        # self._pybullet_interface.setJointMotorControlArray(self._robot, motor_list,
        #                                                    controlMode=self._pybullet_interface.VELOCITY_CONTROL,
        #                                                    forces=[0.0, 0.0, 0.0])
        #
        # self._pybullet_interface.setJointMotorControlArray(self._robot, motor_list,
        #                                                    controlMode=self._pybullet_interface.TORQUE_CONTROL,
        #                                                    forces=desired_torques_mat)

    def setFootForceInGravityFrame(self, leg_id, foot_force_in_gravity_frame):
        base_position, base_orientation = self._pybullet_interface.getBasePositionAndOrientation(self._robot)
        base_orientation = self._pybullet_interface.getEulerFromQuaternion(base_orientation)
        base_orientation = list(base_orientation)
        base_orientation[2] = 0.0
        base_orientation = self._pybullet_interface.getQuaternionFromEuler(base_orientation)

        inv_position, inv_orientation = self._pybullet_interface.invertTransform(base_position, base_orientation)
        foot_force_in_base_frame, _ = self._pybullet_interface.multiplyTransforms([0, 0, 0],
                                                                                  inv_orientation,
                                                                                  foot_force_in_gravity_frame,
                                                                                  [0, 0, 0, 1])




        self.setFootForceInBaseFrame(leg_id, foot_force_in_base_frame)

    def getFootContactState(self):
        all_contacts = self._pybullet_interface.getContactPoints(bodyA = self._robot)

        leg_contact_state = [int(0), int(0), int(0), int(0)]

        for contacts in all_contacts:
           if contacts[2] == self._robot:
               continue

           if contacts[3] == 5:
               leg_contact_state[0] = int(1)
           elif contacts[3] == 10:
               leg_contact_state[1] = int(1)
           elif contacts[3] == 15:
               leg_contact_state[2] = int(1)
           elif contacts[3] == 20:
               leg_contact_state[3] = int(1)

        return leg_contact_state

    def stepJointServo(self):
        desired_tau = [0.0]*12

        joint_angles = self.getJointAngles()
        joint_velocity = self.getJointVelocity()

        for i in range(12):
            desired_tau[i] = self._joint_action[i][4] + \
                             self._joint_action[i][2] * (self._joint_action[i][0] - joint_angles[i]) + \
                             self._joint_action[i][3] * (self._joint_action[i][1] - joint_velocity[i])

        self._pybullet_interface.setJointMotorControlArray(self._robot, MOTOR_ID_LIST,
                                                           controlMode=self._pybullet_interface.VELOCITY_CONTROL,
                                                           forces=[0.0]*12)

        self._pybullet_interface.setJointMotorControlArray(self._robot, MOTOR_ID_LIST,
                                                           controlMode=self._pybullet_interface.TORQUE_CONTROL,
                                                           forces=desired_tau)

    def step(self):
        for i in range(5):
            self.stepJointServo()
            self._pybullet_interface.stepSimulation()
































