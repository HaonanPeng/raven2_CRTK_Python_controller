"""
Raven 2 Control - Control software for the Raven II robot
Copyright (C) 2005-2023  Andrew Lewis, Yun-Hsuan Su, Haonan Peng, Blake Hannaford,
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
raven_keyboard_controller.py
Python controller for RAVEN II using CRTK API
date Apr 12, 2023
author Haonan Peng, Dun-Tin Chiang, Yun-Hsuan Su, Andrew Lewis, 
"""

"""
This piece of code is for testing the single force unit. 
The code reads in raven state, and compensates coulomb and viscous friction based on motor velocity 
"""

import sys
import time
import utils_r2_torque_keyboard_controller as utils
import rospy
import numpy as np
import raven2_CRTK_torque_controller
import copy

 
target_torques = np.zeros(7)  #assume these parameters are assignend by other higher controller 
target_torques[4] = 40
target_torques[5] = 40



rospy.init_node('force_unit_joint45', anonymous=True)
rospy.loginfo("Node is created")

#set the rate to 100 Hz
r = rospy.Rate(100)

r2_tor_ctl = raven2_CRTK_torque_controller.raven2_crtk_torque_controller(name_space = ' ', robot_name_1 = 'arm1', robot_name_2 = 'arm2', grasper_name = 'grasp1', use_load_cell = True)



while not rospy.is_shutdown():
    r2_tor_ctl.pub_torque_command_with_comp(target_torques)
    r.sleep()

        

sys.exit()
