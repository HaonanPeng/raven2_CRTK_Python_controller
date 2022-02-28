# -*- coding: utf-8 -*-
"""
Created on Thu Feb 17 12:12:28 2022

@author: 75678
"""

import numpy as np

Deg2Rad = np.pi / 180.0
Rad2Deg = 180.0 / np.pi

print(0.389660 * Rad2Deg)

dir_1 = np.sign(np.random.randn())
print(dir_1)

diff = np.array([1,1,1,1]) - np.array([0,2,0,0])
idx = np.array(np.where(diff<0))
if idx.size != 0:
    print('not empty')
    
print(idx[0])

for i in range(0,100):
    print(i)
