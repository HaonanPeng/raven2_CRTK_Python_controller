# -*- coding: utf-8 -*-
"""
Created on Fri Feb 12 15:11:29 2021

@author: 75678
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt

file_name = 'demo_trail_1_4.txt'
record_time = 50

# set up the serial line
ser = serial.Serial('COM7', 115200)
# time.sleep(2)


start_time = time.time()
# Read and record the data
data =[]                       # empty list to store the data
while time.time() - start_time < record_time:
    b = ser.readline()         # read a byte string
    try:
        string_n = b.decode()      # decode byte string into Unicode  
        string = string_n.rstrip() # remove \n and \r
    except:
        continue
    
    try:
        [reading_1, reading_2, reading_3] = string.split('_')
    except:
        continue

    try:
        enc_1 = float(reading_1)
        enc_2 = float(reading_2)      # convert string to float
        enc_3 = float(reading_3)
    except:
        continue
    
    time_cur = time.time() - start_time
    new_line = [time_cur, enc_1, enc_2, enc_3]
    # print('Time: ' + str(time_cur))
    
    # try:
    #     ppg = float(ppg_str)
    #     ecg = float(ecg_str)      # convert string to float
    # except:
    #     continue
    # print([ppg, ecg])
    data.append(new_line)           # add to the end of data list
    # time.sleep(0.1)            # wait (sleep) 0.1 seconds

ser.close()

data_array = np.array(data)
print('Record ended, data shape: ' + str(data_array.shape))
print('Data saved at:' + file_name)
np.savetxt(file_name, data_array)


# show the data

# for line in data:
#     print(line)
    
plt.figure(1)
plt.subplot(311)
plt.plot(data_array[:,0], data_array[:,1])
plt.xlabel('Time (seconds)')
plt.ylabel('Reading')
plt.title('ENC1')


plt.subplot(312)
plt.plot(data_array[:,0],data_array[:,2])
plt.xlabel('Time (seconds)')
plt.ylabel('Reading')
plt.title('ENC2')
plt.show()

plt.subplot(313)
plt.plot(data_array[:,0],data_array[:,2])
plt.xlabel('Time (seconds)')
plt.ylabel('Reading')
plt.title('ENC2')
plt.show()