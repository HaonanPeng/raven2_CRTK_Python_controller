# -*- coding: utf-8 -*-
"""
Created on Fri Feb 25 08:33:03 2022

@author: 75678
"""

import numpy as np

def rms_error(array1, array2):
    rms = np.sqrt(np.mean(np.square(array1 - array2)))
    
    return rms