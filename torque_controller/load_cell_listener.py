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

time_window = 8  # this is the time window of 
class r2_load_cell():

    # ros node is not initialized here, it should be initialized and named by the program that use this controller
    def __init__(self):

        self.load_cell_slope = np.array([0,0,0,0,0,0])
        self.load_cell_val = []
        self.subscriber_measured_js = rospy.Subscriber('/load_cells', sensor_msgs.msg.JointState, self.callback_load_cell)
    def callback_load_cell(self, msg):
        load_cell_cur = np.array(msg.position)
        self.load_cell_val.append(load_cell_cur)
        self.load_cell_val = self.load_cell_val[-time_window:]
        load_cell_arr = np.array(self.load_cell_val)
        if load_cell_arr.shape[0] == time_window:
            for i in range(0,6):    
                self.load_cell_slope[i] = average_slope(load_cell_arr[:, i])
        #print(self.load_cell_slope[0])
            
        return None

def linear_regression(y):
    x = np.arange(np.size(y))
    ones = np.ones_like(x)
    A = np.column_stack((x, ones))
    slope, _ = np.linalg.lstsq(A, y, rcond=None)[0]
    return slope

def average_slope(numbers):
    slopes = []
    for i in range(len(numbers) - 1):
        slope = (numbers[i+1] - numbers[i])
        slopes.append(slope)
    avg_slope = sum(slopes) / len(slopes)
    return avg_slope

if __name__ == "__main__":
    load_cell = r2_load_cell()
    rospy.init_node('load_cell_listener', anonymous=True)
    rospy.sleep(2)
    r = rospy.Rate(50)
    max_abs = 0
    while True:
        #print(load_cell.load_cell_slope[0])
        if abs(load_cell.load_cell_slope[0]) > max_abs:
            max_abs = abs(load_cell.load_cell_slope[0])
            print(max_abs)
        
        r.sleep()

