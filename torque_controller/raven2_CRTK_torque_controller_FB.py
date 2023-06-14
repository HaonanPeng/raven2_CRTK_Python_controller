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

# This is the version using load cell feedback control, not load cell slope compensation

import time

import rospy
import numpy as np
# from scipy.spatial.transform import Rotation as sp_rot
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg

import crtk_msgs.msg # crtk_msgs/operating_state
#todo import the msgs.type of raven state
from raven_2.msg import raven_state
import math

class raven2_crtk_torque_controller():

    # ros node is not initialized here, it should be initialized and named by the program that use this controller
    def __init__(self, name_space, robot_name_1, robot_name_2, grasper_name, use_load_cell = False):
        self.name_space = name_space
        self.robot_name_1 = robot_name_1  # This is the real RAVEN arm
        self.robot_name_2 = robot_name_2  # This is the hacked arm to control torque
        self.grasper_name = grasper_name

        #self.joint_velocity_factor = np.array([1e-5, 1e-5 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5]) # 1e-5 means target speed is 1.0cm (1e-5 m) per second

        self.max_torque = 80.0 * np.ones(15) # 50 mNm, This is the max torque  
        self.rate_pub = 500 # !IMPT This is the publish rate of the motion command publisher. It must be tested, because it will affect the real time factor and thus affect the speed. If you are not sure or cannot test, use a large rate (such as 1000) can be safer.
        self.max_rate_move = 500 # This is a protection, if the time interval between 2 move command is shorter than 1/rate, the publisher will wait util 1/rate
        self.min_interval_move = 1.0/self.max_rate_move
        self.pos_scaler = 1000.0/self.rate_pub # This is to scale the pos command to meet the target velocity
        self.time_last_pub_move = -1.0

        self.operate_state = None # [String] current robot operation state, according the CRTK standard - "DISABLED", "ENABLED", "PAUSED", "FAULT", robot can only be controlled when "ENABLED"
        self.is_homed = None 
        self.is_busy = None

        self.measured_cpos_tranform_1 = np.zeros((4,4)) # [LEFT ARM] np.array 4x4 transform matrix of the end-effector measured position
        self.measured_jpos_1 = None # [LEFT ARM] (15,) array of measured joint position
        self.measured_jvel_1 = None # [LEFT ARM] (15,) array of measured joint velocity
        self.measured_jeff_1 = None # [LEFT ARM] (15,) array of measured joint effort

        self.measured_mpos_2 = None # [RIGHT ARM] (15,) array of measured motor pose
        self.measured_torque_2 = None # [RIGHT ARM] (15,) array of motor torque in ravenstate, derived from current command

        self.ravenstate_cur = None
        self.first_ravenstate = False

        self.pub_count_motion = 0 # The counts or how many torque command messages are sent
        
        self.tau_cmd_cur = np.zeros(16)
        self.force_pid_p = 1.0  # p factor of force PID feedback control using load cell
        
        self.load_cell_force = None  # (7,) array, [0] will not be used, [1] for motor 1, [2] for motor 2, so on and so forth
        self.load_cell_first_call = False

        self.__init_pub_sub()

        return None

    def __del__(self):
        self.pub_state_command('pause') 
        return None

    # setup ros publishers and subscribers
    def __init_pub_sub(self):

        # Subscriber and publisher of robot operation state
        topic = "/" + self.robot_name_1 + "/operating_state"
        #self.__subscriber_operating_state = rospy.Subscriber(topic, crtk_msgs.msg.operating_state, self.__callback_operating_state)

        topic = "/arm2/measured_js" # [IMPT] This is actually listening to left arm, because the RAVEN I use has a mismatch that the arm1's jpos is published on arm2
        self.__subscriber_measured_js = rospy.Subscriber(topic, sensor_msgs.msg.JointState, self.__callback_measured_jp_1)

        topic = "ravenstate" # [IMPT] This is actually listening to left arm, because the RAVEN I use has a mismatch that the arm1's jpos is published on arm2
        self.__subscriber_ravenstate = rospy.Subscriber(topic, raven_state, self.__callback_ravenstate)

        self.subscriber_load_cell_force = rospy.Subscriber('/load_cell_forces', sensor_msgs.msg.JointState, self.__callback_load_cell_force)


        # torque publishers

        topic = "/" + self.robot_name_2 + "/servo_jp"
        self.__publisher_servo_jp = rospy.Publisher(topic, sensor_msgs.msg.JointState, latch = True, queue_size = 1)

        return None

    def __callback_measured_jp_1(self, msg):
        self.measured_jpos_1 = np.array(msg.position)
        self.measured_jvel_1 = np.array(msg.velocity)
        self.measured_jeff_1 = np.array(msg.effort)
        return None

    def __callback_ravenstate(self, msg):
        self.ravenstate_cur = msg
        if not self.first_ravenstate:
            self.first_ravenstate = True
        return None

    def __callback_load_cell_force(self, msg):
        if not self.load_cell_first_call:
            self.load_cell_first_call = True
            self.load_cell_force = np.zeros(7)
        else:
            self.load_cell_force[1:] = msg.position[:]

        return None

    # [IMPT]: the joint_command is an np.array of dimension 16, please notice that the first entry is always 0 and does nothing, 
    #         this is to make the command consistent and intuitive - command[1] is joint 1 and [2] is joint 2, so on and so forth.
    #         in the controller code, this only applies to the input joint_command here, nowhere else
    # [Input ]: joint_command - an np.array of dimension 16, joint_command[1] should be the expected velocity of joint 1 (rad and m /sec)
    # [Return]: -1 if command not published, 0 if command published normally
    # [Note]: There is no clear reason why the dimension of the joint command is 15 in CRTK RAVEN. But it is confirmed that this is not to control 2 arms. Each arm should have its own controller node
    def pub_torque_command(self, torque_command):

        torque_command = torque_command[1:] # This is to meet the format of CRTK, where joint 1 is at index 0
        
        max_check = self.__check_max_torque_command(torque_command)
        if max_check.size != 0:
            print('Command velocity too fast, joints: ')
            print(max_check)
            print('Command not sent')
            
            return -1

        msg = sensor_msgs.msg.JointState()
        msg.position[:] = torque_command.flat
        
        max_check = self.__check_max_torque_command(torque_command)

        interval_pub = time.time() - self.time_last_pub_move
        #print(str(interval_pub)) # [debug]
        if (self.time_last_pub_move != -1.0) & (interval_pub < self.min_interval_move):
            time.sleep(self.min_interval_move-interval_pub) # If the time interval is too short, wait util do not exceed the max rate
            #print('time sleep:' + str(self.min_interval_move-interval_pub)) #[debug]
        self.__publisher_servo_jp.publish(msg)
        self.time_last_pub_move = time.time()
        self.pub_count_motion += 1

        #print('Command pub count: ' + str(self.pub_count_motion) + ' | msg: ' + str(joint_command)) # [debug]
        return 0


    # [Input ]: (7,) array, which is the target torque of the 6 motors, [0] will not be used, [1] for motor 1, [2] for motor 2, so on and so forth
    # [Return]: -1 if command not published, 0 if command published normally
    def pub_torque_command_with_comp(self, torque_command):
        if self.first_ravenstate == False:
            print('No ravenstate yet, command not sent.')
            return -1

        cmd_comp = np.zeros((16))

        for i in range(1,7):   # 1 for motor 1, 6 for motor 6
            target_torque = torque_command[i]
            if target_torque == 0:
                cmd_comp[i] = 0
                continue
            coulomb_offset_list = np.array(self.coulomb_factors[i])
            coulomb_offset = coulomb_offset_list[np.absolute(coulomb_offset_list[:,0]-target_torque).argmin(),1] # this is to find the closest factor
            if i < 3:
                motor_velocity = self.ravenstate_cur.mvel[i + 7]
            else:
                motor_velocity = self.ravenstate_cur.mvel[i + 8]

            if np.abs(motor_velocity) > 0 and  time.time() - self.vel_last_comp_time[i] >= self.vel_lock: #forward drive

                control_torque = target_torque + np.sign(motor_velocity) * coulomb_offset
                self.vel_last_comp_val[i] = motor_velocity
                self.vel_last_comp_time[i] = time.time()
                print('1111111111111111111111111111111111')

            elif np.abs(motor_velocity) > 0 and  time.time() - self.vel_last_comp_time[i] < self.vel_lock: 
                control_torque = target_torque + np.sign(self.vel_last_comp_val[i]) * coulomb_offset
                print('2222222222222222222222222222222222')

            else: #static
                #if motor_velocity > 0:
                    #control_torque = target_torque + coulomb_offset
                #elif motor_velocity < 0:
                    #control_torque = (target_torque + np.sign(motor_velocity) * coulomb_offset) - (200 - math.fabs(motor_velocity))*((target_torque + np.sign(motor_velocity) * coulomb_offset) - 12)/200
                    #control_torque = target_torque - coulomb_offset
                #else:
                a = 1
                print('333333333333333333333333333333333')
                if a ==1:
                    if time.time() - self.load_cell_last_comp_time[i] >= self.load_cell_lock:
                        if self.load_cell_slope[i] > 600:  # moving toward the motor
                            control_torque = target_torque + coulomb_offset
                        elif self.load_cell_slope[i] < -600:  # moving agains the motor
                            control_torque = target_torque - coulomb_offset
                        else:
                            control_torque = target_torque + 0
                        self.load_cell_last_comp_time[i] = time.time()
                        self.load_cell_last_comp_slope[i] = self.load_cell_slope[i]
                    else:   # if the last load cell compensation time is shorter than the threshold, lock and use the same slope
                        if self.load_cell_last_comp_slope[i] > 600:  # moving toward the motor
                            control_torque = target_torque + coulomb_offset
                        elif self.load_cell_last_comp_slope[i] < -600:  # moving agains the motor
                            control_torque = target_torque - coulomb_offset
                        else:
                            control_torque = target_torque + 0
            if abs(control_torque) > self.max_torque[i]:
                print('[ERROR] control torque too large, motor ' + str(i) + ', control torque: ' + str(control_torque))
                print('Command not sent.')
                return -1
            cmd_comp[i] =  control_torque

        self.pub_torque_command(cmd_comp.astype(int))
        print(cmd_comp.astype(int))
        return 0
    
    # [Input ]: (7,) array, which is the target **force** of the 6 motors, [0] will not be used, [1] for motor 1, [2] for motor 2, so on and so forth
    # [Return]: -1 if command not published, 0 if command published normally
    def pub_tau_cmd_with_FB(self, force_command):
        if self.first_ravenstate == False or self.load_cell_first_call == False:
            print('No ravenstate or load cell force yet, command not sent.')
            return -1
        cmd_comp = np.zeros((16))
        for i in range(1,7):
            cmd_comp[i] = self.tau_cmd_cur[i] + self.force_pid_p * (force_command[i] - self.load_cell_force[i])
            
            if abs(cmd_comp[i]) > self.max_torque[i]:
                print('[ERROR] control torque too large, motor ' + str(i) + ', control torque: ' + str(cmd_comp[i]))
                print('Zero command is sent to all motor.')
                cmd_comp = np.zeros((16))
                return -1  
        cmd_comp = cmd_comp.clip(-8.0, 80.0)
        self.tau_cmd_cur = cmd_comp[:]
        self.pub_torque_command(cmd_comp.astype(int))
        print(cmd_comp.astype(int))
        return 0

    def __check_max_torque_command(self, torque_command):
        diff = self.max_torque - np.abs(torque_command)
        idx = np.array(np.where(diff<0))

        return idx[0]+1

