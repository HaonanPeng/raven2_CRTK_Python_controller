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



raven_keyboard_controller.py

Python controller for RAVEN II using CRTK API

date Jan 10, 2022
author Haonan Peng, Yun-Hsuan Su, Andrew Lewis, 
"""

import tty, sys, termios
import time
import utils_r2_keyboard_controller as utils
import rospy
import numpy as np
import raven_py_controller

command_interval = 0.2 # [IMPT]: 0,2 is usually good, increase this factor will cause a larger control delay, but may increase the publishing rate

velocity_joint_1 = 3 # degree/s
velocity_joint_2 = 3 # degree/s
velocity_joint_3 = 0.01 # m/s
velocity_joint_4 = 10 # degree/s
velocity_joint_5 = 10 # degree/s
velocity_joint_6 = 10 # degree/s
velocity_joint_7 = 10 # degree/s

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

filedescriptors = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)

rospy.init_node('raven_keyboard_controller', anonymous=True)

r2py_ctl = raven_py_controller.raven2_py_controller(name_space = ' ', robot_name = 'arm1', grasper_name = 'grasp1')
r2py_ctl.pub_state_command('resume')

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

    if count_interval_cmd >= 100: 
        input_key = sys.stdin.read(1)[0]
        termios.tcflush(sys.stdin, termios.TCIOFLUSH)
        if input_key == '9':
            r2py_ctl.pub_state_command('pause')
            sys.exit('Closing RAVEN 2 Keyboard controller')
        count_interval_cmd = 0
    count_interval_cmd += 1

    if input_key == '1':
        utils.print_no_newline(" Moving: Joint 1 +++         ")
        cmd = np.zeros((16))
        cmd[1] = velocity_joint_1 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)

    elif input_key == 'q':
        utils.print_no_newline(" Moving: Joint 1 ---         ")
        cmd = np.zeros((16))
        cmd[1] = -velocity_joint_1 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)

    elif input_key == '2':
        utils.print_no_newline(" Moving: Joint 2 +++         ")
        cmd = np.zeros((16))
        cmd[2] = velocity_joint_2 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)
              
    elif input_key == 'w':
        utils.print_no_newline(" Moving: Joint 2 ---         ")
        cmd = np.zeros((16))
        cmd[2] = -velocity_joint_2 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)

    elif input_key == '3':
        utils.print_no_newline(" Moving: Joint 3 +++         ")
        cmd = np.zeros((16))
        cmd[3] = velocity_joint_3 * 1e-3
        r2py_ctl.pub_jr_command(cmd)
        
    elif input_key == 'e':
        utils.print_no_newline(" Moving: Joint 3 ---         ")
        cmd = np.zeros((16))
        cmd[3] = -velocity_joint_3 * 1e-3
        r2py_ctl.pub_jr_command(cmd)

    elif input_key == '4':
        utils.print_no_newline(" Moving: Joint 4 +++         ")
        cmd = np.zeros((16))
        cmd[4] = velocity_joint_4 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)
        
    elif input_key == 'r':
        utils.print_no_newline(" Moving: Joint 4 ---         ")
        cmd = np.zeros((16))
        cmd[4] = -velocity_joint_4 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)

    elif input_key == '5':
        utils.print_no_newline(" Moving: Joint 5 +++         ")
        cmd = np.zeros((16))
        cmd[5] = velocity_joint_5 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)
        
    elif input_key == 't':
        utils.print_no_newline(" Moving: Joint 5 ---         ")
        cmd = np.zeros((16))
        cmd[5] = -velocity_joint_5 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)

    elif input_key == '6':
        utils.print_no_newline(" Moving: Joint 6 +++         ")
        cmd = np.zeros((16))
        cmd[6] = velocity_joint_6 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)
        
    elif input_key == 'y':
        utils.print_no_newline(" Moving: Joint 6 ---         ")
        cmd = np.zeros((16))
        cmd[6] = -velocity_joint_6 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)

    elif input_key == '7':
        utils.print_no_newline(" Moving: Joint 7 +++         ")
        cmd = np.zeros((16))
        cmd[7] = velocity_joint_7 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)
        
    elif input_key == 'u':
        utils.print_no_newline(" Moving: Joint 7 ---         ")
        cmd = np.zeros((16))
        cmd[7] = -velocity_joint_7 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)

    elif input_key == 'o':
        utils.print_no_newline(" Moving: Grasper open         ")
        cmd = np.zeros((16))
        cmd[6] = velocity_joint_6 * Deg2Rad * 1e-3
        cmd[7] = velocity_joint_7 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)

    elif input_key == 'p':
        utils.print_no_newline(" Moving: Grasper close         ")
        cmd = np.zeros((16))
        cmd[6] = -velocity_joint_6 * Deg2Rad * 1e-3
        cmd[7] = -velocity_joint_7 * Deg2Rad * 1e-3
        r2py_ctl.pub_jr_command(cmd)
        
    # Cartisian space control----------------------------------------------
    elif input_key == 's':
        utils.print_no_newline(' Moving: Grasper forward    ')

    elif input_key == 'x':
        utils.print_no_newline(' Moving: Grasper backward    ')

    elif input_key == 'z':
        utils.print_no_newline(' Moving: Grasper left    ')

    elif input_key == 'c':
        utils.print_no_newline(' Moving: Grasper right    ')

    elif input_key == 'f':
        utils.print_no_newline(' Moving: Grasper up    ')

    elif input_key == 'v':
        utils.print_no_newline(' Moving: Grasper down    ')

    elif input_key == 'h':
        utils.print_no_newline(' Moving: Grasper P -    ')

    elif input_key == 'n':
        utils.print_no_newline(' Moving: Grasper P +    ')

    elif input_key == 'b':
        utils.print_no_newline(' Moving: Grasper R -    ')

    elif input_key == 'm':
        utils.print_no_newline(' Moving: Grasper R +    ')

    elif input_key == 'g':
        utils.print_no_newline(' Moving: Grasper Y -    ')

    elif input_key == 'j':
        utils.print_no_newline(' Moving: Grasper Y +    ')

    else:
        utils.print_no_newline(' Unknown command         ')

termios.tcsetattr(sys.stdin, termios.TCSADRAIN,filedescriptors)
