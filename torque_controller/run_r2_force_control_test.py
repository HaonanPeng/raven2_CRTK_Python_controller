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
This piece of code is for testing the single/multiple force unit. 
Specify the desired torque command in target_torques array 
"""

import sys
import time
import utils_r2_torque_keyboard_controller as utils
import rospy
import sensor_msgs.msg
import numpy as np
import copy


def force_test():
    pub = rospy.Publisher('torque_cmd', sensor_msgs.msg.JointState, latch = True, queue_size = 1)
    rospy.init_node('force_unit_joint45', anonymous=True)
    rospy.loginfo("Node is created") 


    target_torques = np.zeros(7)  #assume these parameters are assignend by other higher controller 
    #the index here start's from 1-7
    target_torques[4] = 10.0 
    target_torques[5] = 10.0

    #create message for publishing
    tor_cmd_msg = sensor_msgs.msg.JointState()
    tor_cmd_msg.position = target_torques
    pub.publish(tor_cmd_msg)
    #set the rate to 100 Hz
    r = rospy.Rate(100)

    #while not rospy.is_shutdown():
        #pub.publish(tor_cmd_msg)
        #r.sleep()



if __name__ == '__main__':
    try:
        force_test()
    except rospy.ROSInterruptException:
        pass









        

sys.exit()
