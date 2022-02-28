# -*- coding: utf-8 -*-
"""
Created on Tue Sep 28 16:02:04 2021

@author: 75678
"""
import numpy as np
import func_raven_joint_encodersIII

# encoders = func_raven_joint_encodersIII.raven_joint_encodersIII()

# encoders.init_serial(com = '/dev/ttyACM0', baud = 115200)

# encoders.zero_calibration()

# encoders.ros_pub()

deg2rad = np.pi / 180.0
rad2deg = 180.0 / np.pi

encoders = func_raven_joint_encodersIII.raven_joint_encodersIII(encoder_unit_resolution = [2*np.pi/80000.0, 2*np.pi/80000.0, 5e-6], 
                                                                joint_zero_pos = [0.0*deg2rad, 130*deg2rad, 0.572467])

encoders.init_serial(com = '/dev/ttyACM0', baud = 115200)

encoders.calibration(cali_time = 60, 
                     direction = [-1, 1, 1], 
                     peak_thresh = [50, 20, 20])

encoders.ros_pub()

