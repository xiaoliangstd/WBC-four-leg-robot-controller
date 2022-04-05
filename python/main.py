import numpy as np
import pybullet as p
import pybullet_data as pd
import robot as rob

import time
import math

import sys

import locomotion

import WBC_CONTROLLER


physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
p.setAdditionalSearchPath('/home/luobin/anaconda3/lib/python3.7/site-packages/pybullet_data')
p.setPhysicsEngineParameter(numSolverIterations=30)
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.4]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
p.setAdditionalSearchPath('/home/luobin/anaconda3/lib/python3.7/site-packages/pybullet_data/a1')
robot_id = p.loadURDF("a1.urdf",cubeStartPos, cubeStartOrientation)
# p.setAdditionalSearchPath('./')
# p.loadURDF("slope.urdf", [2.0, 0, 0.0], [0, 0, 0, 1])

robot = rob.Robot(p, robot_id)





time_step = 0.001
p.setTimeStep(time_step)



p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)






locomotion_controller = locomotion.Locomotion(robot)

desired_linear_velocity = [0.0, 0.0, 0.0]
desired_angular_velocity = [0.0, 0.0, 0.0]
desired_euler_angle= [0.0, 0.0, 0.0]
desired_robot_height = 0.3

period = 2.0


phase = 0.0

foot_pos_record = []





np.set_printoptions(suppress=True)



current_time = 0.0
step_count = 0


alpha = 1e-5
friction_coeff = 0.4

kp_ori = 0.0
kd_ori = 0.0

kp_pos = 0.0
kd_pos = 0.0

kp_foot = 100.0
kd_foot = 10.0

weights = [20.0, 20.0, 5.0, 1.0, 1.0, 0.2]

wbc_controller = WBC_CONTROLLER.WBC(kp_ori, kd_ori, kp_pos, kd_pos,
                                             kp_foot, kd_foot, weights,
                                             alpha, friction_coeff)

while current_time <= 1000:
    location, _ = p.getBasePositionAndOrientation(robot_id)
    p.resetDebugVisualizerCamera(
        cameraDistance=1,
        cameraYaw=30,
        cameraPitch=-30,
        cameraTargetPosition=location
    )

    # p.resetBasePositionAndOrientation(robot_id, [0, 0, 1], [0, 0, 0, 1])




    if step_count < 3000:
        robot.setFootPosInBaseFrame(0, [0.17, -0.13, -0.3])
        robot.setFootPosInBaseFrame(1, [0.17, 0.13, -0.3])
        robot.setFootPosInBaseFrame(2, [-0.17, -0.13, -0.3])
        robot.setFootPosInBaseFrame(3, [-0.17, 0.13, -0.3])

    else:



        desired_x_speed = 0.0 * math.sin(2 * math.pi / 50.0 * current_time)

        # desired_x_speed = 0.5






        desired_y_speed = 0.0* math.sin(2 * math.pi / 50.0 * current_time)
        desired_twist_speed= 0.0 * math.sin(2 * math.pi / 10.0 * current_time)

        desired_robot_height = 0.0 * math.sin(2 * math.pi / 5.0 * current_time) + 0.3

        locomotion_controller.run(current_time, desired_x_speed, desired_y_speed, desired_twist_speed, desired_robot_height)



        current_time += 5* time_step





    step_count += 1

    robot.step()





p.disconnect()










