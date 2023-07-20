"""
Raven 2 Control - Control software for the Raven II robot
Copyright (C) 2005-2022  Andrew Lewis, Yun-Hsuan Su, Haonan Peng, Blake Hannaford,
and the University of Washington BioRobotics Laboratory
This file is part of Raven 2 Control.
Raven 2 Control is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Raven 2 Control is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public License
along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
raven_py_controller.py
Python controller for RAVEN II using CRTK API
date Apr 11, 2023
author Haonan Peng, Dun-Tin Chiang, Yun-Hsuan Su, Andrew Lewis

[!!!IMPORTANT!!!]: This is a hacking code to use the RAVEN II right arm control board to control RAVEN's motor providing desired torque, but not controlling the actual right arm
[!!!IMPORTANT!!!]: This code hack into CRTK 'servo_jp' control and replace its function into torque control
[!!!IMPORTANT!!!]: The usage of this code requires change in RAVEN's src code, if you would like to use this code, please contact authors at penghn@uw.edu
"""

# This force actuator controller use raven2 CRTK torque control to provide a desired force to RAVEN's left arm end-effector

testing = False

import time


import numpy as np
import math
from scipy.optimize import minimize
# from scipy.spatial.transform import Rotation as sp_rot

if not testing:
    import rospy
    import std_msgs.msg
    import geometry_msgs.msg
    import sensor_msgs.msg

    import crtk_msgs.msg # crtk_msgs/operating_state
    #todo import the msgs.type of raven state
    from raven_2.msg import raven_state
    import raven2_CRTK_torque_controller


class raven2_crtk_force_controller():

    # ros node is not initialized here, it should be initialized and named by the program that use this controller
    def __init__(self):

        self.motor_loc = np.array([[1000, 1000, 1000],     # (x, y, z) motor locations in mm, in RAVEN's frame 0, transform might be needed, [0] not used, [1] for motor 1 
                                  [500, 0, 0],
                                  [-500, 0, 0],
                                  [0, 500, 0],
                                  [0, -500, 0],
                                  [0, 0, 500],
                                  [0, 0, -500]
                                  ])

        

        self.motor_dir = np.array([[1, 0, 0],     # unit vector of the direction of each motor, computed by motor location and end-effector location, transform might be needed, [0] not used, [1] for motor 1 
                                  [1, 0, 0],
                                  [-1, 0, 0],
                                  [0, 1, 0],
                                  [0, -1, 0],
                                  [0, 0, 1],
                                  [0, 0, -1]
                                  ])


        #variable for end_effector location
        self.end_effector_loc = None 
        # self.end_effector_loc = np.array([-77.0, -25.0, 14.0])

        self.force_d = np.array([0.0,0.0,0.0])  # The desired force, will be updated by callback
        self.force_d_static = np.array([0.0,0.0,0.0])  # The desired force, will not be update by callback, will stay static during solving the 6 forces
        self.new_force_cmd = False

        self.force_max = 5.0 * np.ones(7)  # [0] not used, [1] for motor 1 
        self.force_min =  -5.0 * np.ones(7)  # [0] not used, [1] for motor 1 

        self.last_solution = np.ones(6) * 2.2  # this is the the last solution,  [0] for motor 1 


        self.r2_tor_ctl = raven2_CRTK_torque_controller.raven2_crtk_torque_controller(name_space = ' ', robot_name_1 = 'arm1', robot_name_2 = 'arm2', grasper_name = 'grasp1', use_load_cell = True)
        self.tor_cmd = np.zeros(7)  # [0] is not used, [1] for motor 1

        self.y_force_only = True  # [TEST] This is only used for a temprary test, where only y axis force is given, by motor 4 and 5

        
        self.prev_tor_cmd = None

        if not testing:
            self.__init_pub_sub()

        return None


    # setup ros publishers and subscribers
    def __init_pub_sub(self):

        # force and torque publisher, 
        # msg.position - (6,) x,y,z force and x, y, z torque (if applicable)
        # msg.velocity - (7,) desired torque command on motor 1-6, [0] is not used, [1] is motor 1
        self.__publisher_force_applied = rospy.Publisher('force_applied', sensor_msgs.msg.JointState, latch = True, queue_size = 1)
        #TODO: create a publisher that publish the torque command
        self.__publisher_torque_cmd = rospy.Publisher('torque_cmd', sensor_msgs.msg.JointState, latch = True, queue_size = 1)
        #TODO end
        self.__subscriber_force_cmd = rospy.Subscriber('force_cmd', sensor_msgs.msg.JointState, self.__callback_force_cmd)
        #TODO: create a subscriber to get the end effector position
        self.__subscriber_ravenstate = rospy.Subscriber('ravenstate', raven_state, self.__callback_ravenstate)
        #TODO end

        return None

    def __callback_force_cmd(self, msg):
        self.force_d[0] = msg.position[0]
        self.force_d[1] = msg.position[1]
        self.force_d[2] = msg.position[2]
        self.new_force_cmd = True
        #print('force_cmd received')
        return None
    
    def __callback_ravenstate(self, msg):
        #isolate the loaction of gold arm grasper
        end_effector_pos_raw = np.array(msg.pos[0:3]) #unit: um
        #print("For debug - end_effector_pos_raw = ", end_effector_pos_raw, np.shape(end_effector_pos_raw))
        self.end_effector_loc = end_effector_pos_raw / 1000.0 #unit: mm
        #print("For debug - self.end_effector_loc = ", self.end_effector_loc)


    def compute_motor_dir(self):
        self.motor_dir = self.end_effector_loc - self.motor_loc 
        for idx in range(1,7):
            self.motor_dir[idx] = self.motor_dir[idx] / np.linalg.norm(self.motor_dir[idx]) # normalize the direcrtion vector so that they all have a lenth of 1

    # force_cmd should be (3,), (x, y, z) component of the force in raven's frame 0
    # the force controller will always listen to topic 'force_cmd', and if there are commands published to this topic, input 'force_cmd' should be None
    def pub_force_cmd(self, force_cmd = None):
        if force_cmd is not None:
            self.force_d = force_cmd
            self.new_force_cmd = True

        if not self.new_force_cmd:
            return None
        
        if self.end_effector_loc == None:
            return None
        self.compute_motor_dir()
        # bounds = [(2.2, 5.0)] * 6  # No bounds on the variables
        bounds = [(0.5, 6.0)] * 6 #unit: N

        self.force_d_static[:] =  self.force_d[:]
        #TODO:add a mechanism that use the previous result as initial guess
        if self.prev_tor_cmd is None:
            self.prev_tor_cmd = np.random.rand(6)   
        solution = minimize(self.objective, self.prev_tor_cmd, bounds=bounds)
        # TODO end
        # solution = minimize(self.objective, np.random.rand(6), bounds=bounds)
        self.tor_cmd[1:] = solution.x
        # print("Solution: ", solution.x)  # [Test] Print the solution
        
        if self.y_force_only:
            self.force_d_static[0] = 0
            self.force_d_static[2] = 0
            self.tor_cmd = np.zeros(7)
            if self.force_d_static[1] >=0:
                self.tor_cmd[4] = 22
                self.tor_cmd[5] = int(10* self.force_d_static[1] + 22)
            else:
                self.tor_cmd[5] = 22
                self.tor_cmd[4] = int(-10* self.force_d_static[1] + 22)
            #print(self.tor_cmd)

        #TODO:publish the list of torque command to /torque_cmd topic
        #self.r2_tor_ctl.pub_torque_command_with_comp(self.tor_cmd) #original
        tor_cmd_msg = sensor_msgs.msg.JointState()
        tor_cmd_msg.position = self.tor_cmd
        self.__publisher_torque_cmd.publish(tor_cmd_msg)
        #TODO end
        
        tor_vec = np.ones((6,1))
        tor_vec[:,0] =  self.tor_cmd[1:].T / 10
        force_applied = np.sum(self.motor_dir[1:] * tor_vec, axis = 0)
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position[:] = force_applied.flat
        msg.position[:] = self.force_d_static.flat # [test] this line should be muted
        self.__publisher_force_applied.publish(msg)
        self.new_force_cmd = False
        #TODO: update the current tor_cmd to the prev_tor_cmd
        self.prev_tor_cmd = solution.x


        return None

    # Define the objective function
    def objective(self, force_value):
        force_vec = np.ones((6,1))
        force_vec[:,0] =  force_value.T
        #print(np.sum(self.motor_dir * force_vec))

        diff = np.sum(self.motor_dir[1:] * force_vec, axis = 0) - self.force_d_static
        obj = diff[0]**2 + diff[1]**2 + diff[2]**2    # this is much faster than using np.linalg.norm()
        #obj = np.linalg.norm(np.sum(self.motor_dir[1:] * force_vec, axis = 0) - self.force_d)
        #print(obj)
        return obj




    
