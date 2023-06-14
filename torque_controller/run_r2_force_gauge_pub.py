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

import serial
import numpy as np
import rospy
# from std_msgs.msg import Float32
import sensor_msgs.msg

rospy.init_node('force_gauge_driver', anonymous=True)
fg_pub = rospy.Publisher('force_gauge', sensor_msgs.msg.JointState, queue_size=1)

ser = serial.Serial('/dev/ttyUSB0', 2400, timeout = 5)
#ser.set_buffer_size(rx_size = 4096, tx_size = 4096)
rospy.sleep(3)

# Read and record the data
count = 0

while True:
    try:
    #print('111--------------------------------------')
    #b = ser.readline()         # read a byte string
        b = ser.read(6) 
    #print(b)
    #print('222--------------------------------------')
        string_n = b.decode()      # decode byte string into Unicode
    #print('333--------------------------------------')
        #string = string_n.rstrip() # remove \n and \r
    #print('444--------------------------------------')
    
    


        force_val = float(string_n)    # convert string to float
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = [force_val]
        fg_pub.publish(msg)
        count += 1
        print('Force: ' + str(force_val) + '    Count: ' + str(count))

    except:
        continue

    

