# -*- coding: utf-8 -*-
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

date Jan 10, 2022
author Haonan Peng, Yun-Hsuan Su, Andrew Lewis, 
"""

import time

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as sp_rot
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import crtk_msgs.msg # crtk_msgs/operating_state

import utils_r2_py_controller as utils


Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

class raven2_runtime_monitor():

    # ros node is not initialized here, it should be initialized and named by the program that use this controller
    def __init__(self, name_space, robot_name, grasper_name, sample_rate = 20, output_rate = 1):
        self.name_space = name_space
        self.robot_name = robot_name
        self.grasper_name = grasper_name
        
        self.intv_sample = 1.0 / sample_rate
        self.intv_output = 1.0 / output_rate

        #self.joint_velocity_factor = np.array([1e-5, 1e-5 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5]) # 1e-5 means target speed is 1.0cm (1e-5 m) per second

        self.measured_cpos_tranform = np.zeros((4,4)) # np.array 4x4 transform matrix of the end-effector measured position
        
        self.measured_jpos = np.zeros((1,15)) # (15,) array of measured joint position
        # self.measured_jvel = None # (15,) array of measured joint velocity
        # self.measured_jeff = None # (15,) array of measured joint effort
        self.rec_cpos = np.zeros((1,4))
        self.rec_jpos = np.zeros((1,16))
        self.rec_control_jr = np.zeros((1,16))
        
        self.time_start = rospy.get_time()
        self.time_last_sample_servo_jr = rospy.get_time()
        self.time_last_sample_jpos = rospy.get_time()
        self.time_last_sample_cpos = rospy.get_time()
        self.time_last_output = rospy.get_time()

        
        self.__init_pub_sub()

        return None

    # setup ros publishers and subscribers
    def __init_pub_sub(self):

        # Subscriber and publisher of robot operation state
        topic = "/" + self.robot_name + "/operating_state"
        self.__subscriber_operating_state = rospy.Subscriber(topic, crtk_msgs.msg.operating_state, self.__callback_operating_state)

        # Subscribers of joint state and cartisian position
        topic = "/" + self.robot_name + "/measured_cp"
        self.__subscriber_measured_cp = rospy.Subscriber(topic, geometry_msgs.msg.TransformStamped, self.__callback_measured_cp)

        # topic = "/" + self.robot_name + "/measured_js" # [IMPT] this should be the usual case
        topic = "/arm2/measured_js" # [IMPT] This line is because the RAVEN that I use has a mismatch that the arm1's jpos is published on arm2
        self.__subscriber_measured_js = rospy.Subscriber(topic, sensor_msgs.msg.JointState, self.__callback_measured_jp)
        
        # Subscribers of joint state and cartisian position
        topic = "/" + self.robot_name + "/servo_jr"
        self.__subscriber_servo_jr = rospy.Subscriber(topic, sensor_msgs.msg.JointState, self.__callback_servo_jr)

        return None

    def __check_max_jpose_command(self, joint_command):
        diff = self.max_jr - np.abs(joint_command)
        idx = np.array(np.where(diff<0))

        return idx[0]

    def __callback_measured_cp(self, msg):
        rot = sp_rot.from_quat([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])

        # self.measured_cpos_tranform[0:3,0:3] = rot.as_matrix() # there is an error, 'no method called as_matrix'
        self.measured_cpos_tranform[0,3] = msg.transform.translation.x
        self.measured_cpos_tranform[1,3] = msg.transform.translation.y
        self.measured_cpos_tranform[2,3] = msg.transform.translation.z
        self.measured_cpos_tranform[3,3] = 1.0
        
        if rospy.get_time() - self.time_last_sample_cpos >= self.intv_sample:
            new_line = np.zeros((1,4))
            new_line[0, 0] = rospy.get_time()
            new_line[0, 1] = msg.transform.translation.x
            new_line[0, 2] = msg.transform.translation.y
            new_line[0, 3] = msg.transform.translation.z        
            self.rec_cpos = np.append(self.rec_cpos, new_line, axis = 0)

        return None

    def __callback_measured_jp(self, msg):
        self.measured_jpos = np.array(msg.position)
        self.measured_jvel = np.array(msg.velocity)
        self.measured_jeff = np.array(msg.effort)
        
        if rospy.get_time() - self.time_last_sample_jpos >= self.intv_sample:
            new_line = np.array([np.append(rospy.get_time(), self.measured_jpos)])
            self.rec_jpos = np.append(self.rec_jpos, new_line, axis = 0)
        
        return None
    
    def __callback_servo_jr(self, msg):
        self.servo_jr_jpos = np.array(msg.position)
        self.servo_jr_jvel = np.array(msg.velocity)
        self.servo_jr_jeff = np.array(msg.effort)
        
        if rospy.get_time() - self.time_last_sample_servo_jr >= self.intv_sample:
            new_line = np.array([np.append(rospy.get_time(), self.servo_jr_jpos)])
            self.control_jr = np.append(self.control_jr, new_line, axis = 0)
        
        return None

    def __callback_operating_state(self, msg):

        self.operate_state = msg.state

        return None
    
    def start_raven_monitor(self):
        working = True
        
        
        
        while working:
            if rospy.get_time() - self.time_last_output >= self.intv_output:
                # Fig 1: 3D traj in joint space
                fig1 = plt.figure(1)
                ax1 = fig1.gca(projection='3d')
                ax1.xlabel('joint 1(Deg)')
                ax1.ylabel('joint 2(Deg)')
                ax1.zlabel('joint 3(m)')
                ax1.plot(self.rec_jpos[1:,1]*Rad2Deg, self.rec_jpos[1:,2]*Rad2Deg, self.rec_jpos[1:,3])
                ax1.scatter(self.rec_jpos[-1,1]*Rad2Deg, self.rec_jpos[-1:,2]*Rad2Deg, self.rec_jpos[-1:,3], label=str("%.4f" % (self.rec_jpos[-1,1]*Rad2Deg)) + ', ' + str("%.4f" % (self.rec_jpos[-1,2]*Rad2Deg)) + ', ' + str("%.4f" % (self.rec_jpos[-1,3])))         
                ax1.legend()
                
                # Fig 2: 2D traj in joint space
                fig1 = plt.figure(2)
                plt.subplot(3, 1, 1)
                plt.plot(self.rec_jpos[1:,0], self.rec_jpos[1:,1]*Rad2Deg)
                plt.ylabel('joint 1(Deg)')
                
                plt.subplot(3, 1, 1)
                plt.plot(self.rec_jpos[1:,0], self.rec_jpos[1:,2]*Rad2Deg)
                plt.ylabel('joint 2(Deg)')
                
                plt.subplot(3, 1, 1)
                plt.plot(self.rec_jpos[1:,0], self.rec_jpos[1:,3])
                plt.ylabel('joint 3(m)')
                
                # Fig 3: joint jr motion command
                fig1 = plt.figure(3)
                plt.subplot(3, 1, 1)
                plt.plot(self.rec_control_jr[1:,0], self.rec_control_jr[1:,1]*Rad2Deg*1e3)
                plt.ylabel('joint 1(Deg/s)')
                
                plt.subplot(3, 1, 1)
                plt.plot(self.rec_control_jr[1:,0], self.rec_control_jr[1:,2]*Rad2Deg*1e3)
                plt.ylabel('joint 2(Deg/s)')
                
                plt.subplot(3, 1, 1)
                plt.plot(self.rec_control_jr[1:,0], self.rec_control_jr[1:,3]*1e3)
                plt.ylabel('joint 3(m/s)')
                
                plt.show()
                
        
        return None