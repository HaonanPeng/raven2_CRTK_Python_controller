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
import rospy
import numpy as np
import raven2_CRTK_force_actuator_controller
import copy

rospy.init_node('force_controller', anonymous=True)

#set the rate to 100 Hz
r = rospy.Rate(100)

r2_force_ctl = raven2_CRTK_force_actuator_controller.raven2_crtk_force_controller()



while not rospy.is_shutdown():
    r2_force_ctl.pub_force_cmd()  # no force is given here, the node will listen to topic 'force_cmd' for force commands
    r.sleep()

        

sys.exit()
