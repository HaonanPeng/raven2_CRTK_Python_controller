# -*- coding: utf-8 -*-
"""
Created on Mon Oct  4 16:29:38 2021

@author: 75678
"""

import func_raven_joint_encodersIII

encoders = func_raven_joint_encodersIII.raven_joint_encodersIII()

# encoders.load_data('trail_with_multi_init.txt')
encoders.load_data('trail_with_one_init_2.txt')

encoders.calibration(cali_time = 60, direction = [-1, 1, 1], peak_thresh = [50, 20, 20])

encoders.Gauss_filter()

# encoders.save_data(file_path = 'trail_with_one_init_2.txt', time_interval = [130,190])

# encoders.plot_data()