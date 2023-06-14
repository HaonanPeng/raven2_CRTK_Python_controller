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

import tty, sys, termios
import time
import utils_r2_torque_keyboard_controller as utils
import rospy
import numpy as np
import raven2_CRTK_torque_controller

command_interval = 0.2 # [IMPT]: 0.2 is usually good, increase this factor will cause a larger control delay, but may increase the publishing rate

torque_joint_1 = 3 # Nm
torque_joint_2 = 3 # Nm
torque_joint_3 = 3 # Nm
torque_joint_4 = 3 # Nm
torque_joint_5 = 3 # Nm
torque_joint_6 = 3 # Nm
torque_joint_7 = 3 # Nm


filedescriptors = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)

rospy.init_node('raven_torque_keyboard_controller', anonymous=True)

r2_tor_ctl = raven2_CRTK_torque_controller.raven2_crtk_torque_controller(name_space = ' ', robot_name_1 = 'arm1', robot_name_2 = 'arm2', grasper_name = 'grasp1')

torque_cur_j1 = float(0.0)
torque_cur_j2 = float(0.0)
torque_cur_j3 = float(0.0)
torque_cur_j4 = float(0.0)
torque_cur_j5 = float(0.0)
torque_cur_j6 = float(0.0)
torque_cur_j7 = float(0.0)

max_torque = 50.0

x = 0
working = 1
utils.print_manu()

time_last_key_command = 0
count_interval_cmd = 101
while working==1:

    # if time.time() - time_last_key_command >= command_interval:
    #     print('--------------------------------------------------------------------------------')
    #     print('--------------------------------------------------------------------------------')
    #     print('--------------------------------------------------------------------------------')
    #     time_last_key_command = time.time()
    #     input_key = sys.stdin.read(1)[0]


    input_key = sys.stdin.read(1)[0]
    termios.tcflush(sys.stdin, termios.TCIOFLUSH)
    if input_key == 'k':
        sys.exit('Closing RAVEN 2 Torque Keyboard controller')
    count_interval_cmd = 0


    if input_key == '1':
        utils.print_no_newline(" Torque: Joint 1 +++         ")
        cmd = np.zeros((16))
        cmd[1] = torque_cur_j1 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == 'q':
        utils.print_no_newline(" Torque: Joint 1 ---         ")
        cmd = np.zeros((16))
        cmd[1] = -torque_cur_j1 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == '2':
        utils.print_no_newline(" Torque: Joint 2 +++         ")
        cmd = np.zeros((16))
        cmd[2] = torque_cur_j2 
        r2_tor_ctl.pub_torque_command(cmd)
              
    elif input_key == 'w':
        utils.print_no_newline(" Torque: Joint 2 ---         ")
        cmd = np.zeros((16))
        cmd[2] = -torque_cur_j2 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == '3':
        utils.print_no_newline(" Torque: Joint 3 +++         ")
        cmd = np.zeros((16))
        cmd[3] = torque_cur_j3 
        r2_tor_ctl.pub_torque_command(cmd)
        
    elif input_key == 'e':
        utils.print_no_newline(" Torque: Joint 3 ---         ")
        cmd = np.zeros((16))
        cmd[3] = -torque_cur_j3 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == '4':
        utils.print_no_newline(" Torque: Joint 4 +++         ")
        cmd = np.zeros((16))
        cmd[4] = torque_cur_j4 
        r2_tor_ctl.pub_torque_command(cmd)
        
    elif input_key == 'r':
        utils.print_no_newline(" Torque: Joint 4 ---         ")
        cmd = np.zeros((16))
        cmd[4] = -torque_cur_j4 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == '5':
        utils.print_no_newline(" Torque: Joint 5 +++         ")
        cmd = np.zeros((16))
        cmd[5] = torque_cur_j5 
        r2_tor_ctl.pub_torque_command(cmd)
        #print(torque_cur_j5)
        
    elif input_key == 't':
        utils.print_no_newline(" Torque: Joint 5 ---         ")
        cmd = np.zeros((16))
        cmd[5] = -torque_cur_j5 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == '6':
        utils.print_no_newline(" Torque: Joint 6 +++         ")
        cmd = np.zeros((16))
        cmd[6] = torque_cur_j6 
        r2_tor_ctl.pub_torque_command(cmd)
        
    elif input_key == 'y':
        utils.print_no_newline(" Torque: Joint 6 ---         ")
        cmd = np.zeros((16))
        cmd[6] = -torque_cur_j6 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == '7':
        utils.print_no_newline(" Torque: Joint 7 +++         ")
        cmd = np.zeros((16))
        cmd[7] = torque_cur_j7 
        r2_tor_ctl.pub_torque_command(cmd)
        
    elif input_key == 'u':
        utils.print_no_newline(" Torque: Joint 7 ---         ")
        cmd = np.zeros((16))
        cmd[7] = -torque_cur_j7 
        r2_tor_ctl.pub_torque_command(cmd)
        
    # multi control----------------------------------------------
    elif input_key == '8':
        utils.print_no_newline(' Troque: Joint 123 +++       ')
        cmd = np.zeros((16))
        cmd[1] = torque_cur_j1 
        cmd[2] = torque_cur_j2 
        cmd[3] = torque_cur_j3 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == 'i':
        utils.print_no_newline(' Torque: Joint 123 ---       ')
        cmd = np.zeros((16))
        cmd[1] = -torque_cur_j1 
        cmd[2] = -torque_cur_j2 
        cmd[3] = -torque_cur_j3 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == '9':
        utils.print_no_newline(' Torque: Joint 456 +++       ')
        cmd = np.zeros((16))
        cmd[4] = torque_cur_j4 
        cmd[5] = torque_cur_j5 
        cmd[6] = torque_cur_j6 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == 'o':
        utils.print_no_newline(' Torque: Joint 456 ---       ')
        cmd = np.zeros((16))
        cmd[4] = -torque_cur_j4 
        cmd[5] = -torque_cur_j5 
        cmd[6] = -torque_cur_j6 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == '0':
        utils.print_no_newline(' Torque: Joint 135 +++       ')
        cmd = np.zeros((16))
        cmd[1] = torque_cur_j1 
        cmd[3] = torque_cur_j3 
        cmd[5] = torque_cur_j5 
        r2_tor_ctl.pub_torque_command(cmd)

    elif input_key == 'p':
        utils.print_no_newline(' Torque: Joint 135 ---       ')
        cmd = np.zeros((16))
        cmd[1] = -torque_cur_j1 
        cmd[3] = -torque_cur_j3 
        cmd[5] = -torque_cur_j5 
        r2_tor_ctl.pub_torque_command(cmd)

    # change joint torque-----------------------------------------
    # Joint 1
    elif input_key == 'a':
        torque_cur_j1 = np.clip(torque_cur_j1 + 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 1 increased to: ' + str(torque_cur_j1) + '            ')
        
        if abs(torque_cur_j1)<0.09:
            torque_cur_j1 = 0.0

    elif input_key == 'z':
        torque_cur_j1 = np.clip(torque_cur_j1 - 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 1 decreased to: ' + str(torque_cur_j1) + '            ')
        
        if abs(torque_cur_j1)<0.09:
            torque_cur_j1 = 0.0

    # Joint 2
    elif input_key == 's':
        torque_cur_j2 = np.clip(torque_cur_j2 + 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 2 increased to: ' + str(torque_cur_j2) + '            ')
        if abs(torque_cur_j2)<0.09:
            torque_cur_j2 = 0.0

    elif input_key == 'x':
        torque_cur_j2 = np.clip(torque_cur_j2 - 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 2 decreased to: ' + str(torque_cur_j2) + '            ')
        if abs(torque_cur_j2)<0.09:
            torque_cur_j2 = 0.0

    # Joint 3
    elif input_key == 'd':
        torque_cur_j3 = np.clip(torque_cur_j3 + 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 3 increased to: ' + str(torque_cur_j3) + '            ')
        if abs(torque_cur_j3)<0.09:
            torque_cur_j3 = 0.0

    elif input_key == 'c':
        torque_cur_j3 = np.clip(torque_cur_j3 - 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 3 decreased to: ' + str(torque_cur_j3) + '            ')
        if abs(torque_cur_j3)<0.09:
            torque_cur_j3 = 0.0

    # Joint 4
    elif input_key == 'f':
        torque_cur_j4 = np.clip(torque_cur_j4 + 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 4 increased to: ' + str(torque_cur_j4) + '            ')
        if abs(torque_cur_j4)<0.09:
            torque_cur_j4 = 0.0

    elif input_key == 'v':
        torque_cur_j4 = np.clip(torque_cur_j4 - 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 4 decreased to: ' + str(torque_cur_j4) + '            ')
        if abs(torque_cur_j4)<0.09:
            torque_cur_j4 = 0.0

    # Joint 5
    elif input_key == 'g':
        torque_cur_j5 = np.clip(torque_cur_j5 + 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 5 increased to: ' + str(torque_cur_j5) + '            ')
        if abs(torque_cur_j5)<0.09:
            torque_cur_j5 = 0.0

    elif input_key == 'b':
        torque_cur_j5 = np.clip(torque_cur_j5 - 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 5 decreased to: ' + str(torque_cur_j5) + '            ')
        if abs(torque_cur_j5)<0.09:
            torque_cur_j5 = 0.0

    # Joint 6
    elif input_key == 'h':
        torque_cur_j6 = np.clip(torque_cur_j6 + 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 6 increased to: ' + str(torque_cur_j6) + '            ')
        if abs(torque_cur_j6)<0.09:
            torque_cur_j6 = 0.0

    elif input_key == 'n':
        torque_cur_j6 = np.clip(torque_cur_j6 - 0.1, -max_torque, max_torque)
        utils.print_no_newline(' Torque 6 decreased to: ' + str(torque_cur_j6) + '            ')
        if abs(torque_cur_j6)<0.09:
            torque_cur_j6 = 0.0

    # Joint 7
    elif input_key == 'j':
        torque_cur_j7 = np.clip(torque_cur_j7 + 0.1, -max_torque, max_torque)
        if abs(torque_cur_j7)<0.09:
            torque_cur_j7 = 0.0
        utils.print_no_newline(' Torque 7 increased to: ' + str(torque_cur_j7) + '            ')


    elif input_key == 'm':
        torque_cur_j7 = np.clip(torque_cur_j7 - 0.1, -max_torque, max_torque)
        if abs(torque_cur_j7)<0.09:
            torque_cur_j7 = 0.0
        utils.print_no_newline(' Torque 7 decreased to: ' + str(torque_cur_j7) + '            ')

        

    else:
        utils.print_no_newline(' Unknown command                            ')

termios.tcsetattr(sys.stdin, termios.TCSADRAIN,filedescriptors)
