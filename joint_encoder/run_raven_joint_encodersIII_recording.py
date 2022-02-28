# -*- coding: utf-8 -*-
"""
Created on Tue Sep 28 16:02:04 2021

@author: 75678
"""

import func_raven_joint_encodersIII


encoders = func_raven_joint_encodersIII.raven_joint_encodersIII()

encoders.init_serial(com = 'COM4', baud = 115200)

encoders.zero_calibration()

encoders.recording(record_time = 50)

encoders.save_data(file_path = 'test_trail_2.txt')

encoders.plot_data()