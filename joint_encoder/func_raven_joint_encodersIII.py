# -*- coding: utf-8 -*-
"""
Created on Wed Sep 22 12:37:58 2021

@author: 75678
"""

import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import scipy
from scipy import signal
import rospy
import sensor_msgs.msg

from utils import *

deg2rad = np.pi / 180.0
rad2deg = 180.0 / np.pi

class raven_joint_encodersIII():
    unit_resolution_encoder_1 = None
    unit_resolution_encoder_2 = None
    unit_resolution_encoder_3 = None
    
    zero_pos_joint_1 = None
    zero_pos_joint_2 = None
    zero_pos_joint_3 = None
    
    enc1_cur = 0
    enc2_cur = 0
    enc3_cur = 0
    
    enc1_cali = 0
    enc2_cali = 0
    enc3_cali = 0
    
    data_cali_ = None
    data_ = None
    
    data_filtered_ = None
    
    initial_time = 0
    
    ser = None # Serial reader
    
    def __init__(self, encoder_unit_resolution, joint_zero_pos = [0,0,0]):
        self.unit_resolution_encoder_1 = encoder_unit_resolution[0]
        self.unit_resolution_encoder_2 = encoder_unit_resolution[1]
        self.unit_resolution_encoder_3 = encoder_unit_resolution[2]
        
        self.zero_pos_joint_1 = joint_zero_pos[0]
        self.zero_pos_joint_2 = joint_zero_pos[1]
        self.zero_pos_joint_3 = joint_zero_pos[2]
        
        return None
    
    def init_serial(self, com, baud, ros = True):
        self.ser = serial.Serial(com, baud)
        self.initial_time = time.time()
        
        if ros == True:
            import rospy
            import sensor_msgs.msg
        return None
    
    def recording(self, record_time):
        rj_print_sep_line()
        rj_print('Start recording.............')
        self.data_ =[]                       # empty list to store the data
        start_time = time.time()
        
        print_count = 0
        
        while time.time() - start_time < record_time:
            b = self.ser.readline()         # read a byte string
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
                enc_1 = float(reading_1) - self.enc1_cali
                enc_2 = float(reading_2) - self.enc2_cali     # convert string to float
                enc_3 = float(reading_3) - self.enc3_cali
            except:
                continue
            
            time_cur = time.time() - start_time
            new_line = [time_cur, enc_1, enc_2, enc_3]
            self.data_.append(new_line)           # add to the end of data list
            
            if time_cur - print_count > 1:
                rj_print('Recording: Time ' + str(round(time_cur, 1)) + ' | Enc_1: ' + str(enc_1) + ' | Enc_2: ' + str(enc_2) + ' | Enc_3: ' + str(enc_3))
                print_count = print_count + 1
        rj_print('End recording.............')
        rj_print_sep_line()
      
        return self.data_
    
    def ros_pub(self, node_name = 'r2_joint_encoder'):
        
        rospy.init_node(node_name)
        #rospy.init_node('talker', anonymous=True)
        enc_pub = rospy.Publisher('joint_enc', sensor_msgs.msg.JointState, queue_size=1)
        ext_jpos_pub = rospy.Publisher('ext_jpos', sensor_msgs.msg.JointState, queue_size=1)
        
        rj_print_sep_line()
        rj_print('Start publishing.............')
        start_time = time.time()
        
        print_count = 0
        
        while not rospy.is_shutdown():
            
            b = self.ser.readline()         # read a byte string
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
                enc_1 = float(reading_1) - self.enc1_cali
                enc_2 = float(reading_2) - self.enc2_cali     # convert string to float
                enc_3 = float(reading_3) - self.enc3_cali
		jpos_1 = enc_1 * self.unit_resolution_encoder_1 + self.zero_pos_joint_1
                jpos_2 = enc_2 * self.unit_resolution_encoder_2 + self.zero_pos_joint_2
                jpos_3 = enc_3 * self.unit_resolution_encoder_3 + self.zero_pos_joint_3

            except:
		print('error')
                continue

            
            # time_cur = time.time() - start_time
            # Publish enc values
            new_line = np.array([enc_1, enc_2, enc_3])
            msg = sensor_msgs.msg.JointState()
            msg.header.stamp = rospy.Time.now()
            msg.position[:] = new_line.flat
            enc_pub.publish(msg)
            
            # Publish jpos values
            new_line = np.array([jpos_1, jpos_2, jpos_3])
            msg = sensor_msgs.msg.JointState()
            msg.header.stamp = rospy.Time.now()
            msg.position[:] = new_line.flat
            ext_jpos_pub.publish(msg)


            time_cur = time.time() - start_time
            if time_cur - print_count > 1:
                rj_print('Publishing ' + str(round(time_cur, 1)) + ' | Enc_1: ' + str(enc_1) + ' | Enc_2: ' + str(enc_2) + ' | Enc_3: ' + str(enc_3))
		rj_print('Publishing ' + str(round(time_cur, 1)) + ' | Jpos_1: ' + str(jpos_1) + ' | Jpos_2: ' + str(jpos_2) + ' | Jpos_3: ' + str(jpos_3))
                print_count = print_count + 1
      
        return None
    
    def calibration(self, cali_time, direction = [-1, 1, 1], peak_thresh = [50, 20, 20], show_plot = False):
        rj_print('Start calibration.............')
        rj_print('Time: ')
        
        # self.data_cali_ = self.data_ # for debuging
        
        self.data_cali_ =[]                       # empty list to store the data
        start_time = time.time()
        
        print_count = 0
        
        while time.time() - start_time < cali_time:
            
            b = self.ser.readline()         # read a byte string
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
                enc_1 = float(reading_1) - self.enc1_cali
                enc_2 = float(reading_2) - self.enc2_cali     # convert string to float
                enc_3 = float(reading_3) - self.enc3_cali
            except:
                continue
            
            time_cur = time.time() - start_time
            new_line = [time_cur, enc_1, enc_2, enc_3]
            self.data_cali_.append(new_line)           # add to the end of data list
            
            if time_cur - print_count > 1:
                rj_print('Calibrating: Time ' + str(round(time_cur, 1)) + ' | Enc_1: ' + str(enc_1) + ' | Enc_2: ' + str(enc_2) + ' | Enc_3: ' + str(enc_3))
                print_count = print_count + 1
                
        self.data_cali_ = np.array(self.data_cali_)
                
        #-------------------------------------------------------------
        enc_1_ = self.data_cali_[:,1] * direction[0]
        enc_2_ = self.data_cali_[:,2] * direction[1]
        enc_3_ = self.data_cali_[:,3] * direction[2]
        
        peak_enc_1 = np.max(enc_1_)
        peak_enc_2 = np.max(enc_2_)
        peak_enc_3 = np.max(enc_3_)
        
        self.enc1_cali = np.mean(enc_1_[enc_1_ > (peak_enc_1 - peak_thresh[0])]) * direction[0]
        self.enc2_cali = np.mean(enc_2_[enc_2_ > (peak_enc_2 - peak_thresh[1])]) * direction[1]
        self.enc3_cali = np.mean(enc_3_[enc_3_ > (peak_enc_3 - peak_thresh[2])]) * direction[2]
        
        rj_print_sep_line()
        rj_print('Calibration complete.')
        rj_print('Calibrated at: Enc_1: ' + str(self.enc1_cali) + ' | Enc_2: ' + str(self.enc2_cali) + ' | Enc_3: ' + str(self.enc3_cali))
        rj_print_sep_line()
        
        data_array = np.array(self.data_)
        
        if show_plot == True:
            plt.figure(1, figsize=(15, 12))
            plt.subplot(311)
            plt.plot(data_array[:,0], data_array[:,1])
            plt.plot(data_array[:,0], self.enc1_cali * np.ones(np.shape(data_array[:,1])))
            plt.legend(('raw','cali'))
            # plt.xlabel('Time (seconds)')
            plt.ylabel('Reading')
            plt.title('ENC1')
            
            
            plt.subplot(312)
            plt.plot(data_array[:,0],data_array[:,2])
            plt.plot(data_array[:,0], self.enc2_cali * np.ones(np.shape(data_array[:,2])))
            # plt.xlabel('Time (seconds)')
            plt.ylabel('Reading')
            plt.title('ENC2')
            
            plt.subplot(313)
            plt.plot(data_array[:,0],data_array[:,3])
            plt.plot(data_array[:,0], self.enc3_cali * np.ones(np.shape(data_array[:,3])))
            plt.xlabel('Time (seconds)')
            plt.ylabel('Reading')
            plt.title('ENC3')
            plt.show()
        
        return None
    
    def zero_calibration(self):
        data_cali_ = self.recording(2)
        _, self.enc1_cali, self.enc2_cali, self.enc3_cali = data_cali_[-5]
        
        print(self.enc1_cali)
        
        rj_print_sep_line()
        rj_print('Zero calibration complete.')
        rj_print('Calibrated at: Enc_1: ' + str(self.enc1_cali) + ' | Enc_2: ' + str(self.enc2_cali) + ' | Enc_3: ' + str(self.enc3_cali))
        rj_print_sep_line()
        
        return None
    
    def save_data(self, file_path, time_interval = None):
        data_array = np.array(self.data_)
        
        if time_interval == None:
            np.savetxt(file_path, data_array)
            rj_print_sep_line()
            rj_print('Data saved. Path: ' + file_path)
            rj_print('Time interval(s): All')
            rj_print('Size: ' + str(np.shape(data_array)))
            rj_print_sep_line()
        else:
            data_array = data_array[(data_array[:,0]>time_interval[0]) & (data_array[:,0]<time_interval[1]), :]
            np.savetxt(file_path, data_array)
            rj_print_sep_line()
            rj_print('Data saved. Path: ' + file_path)
            rj_print('Time interval(s): ' + str(time_interval))
            rj_print('Size: ' + str(np.shape(data_array)))
            rj_print_sep_line()
        
        return None
    
    def Gauss_filter(self):
        ## ----------------------------------------------------------
        ## This is a butter filter, which can cause ringing
        # fs = 100 # frequency of signal
        # fc = 2  # Cut-off frequency of the filter
        # w = fc / (fs / 2) # Normalize the frequency
        # b, a = signal.butter(5, w, 'low')
        # signal_r = self.data_[:,1:4]
        # time_line = np.array([self.data_[:,0]]).T
        # signal_f = signal.filtfilt(b, a, signal_r, axis = 0)
        # self.data_filtered_ = np.hstack((time_line,signal_f))
        ## ----------------------------------------------------------
        
        signal_r = self.data_[:,1:4]
        sigma = 10
        
        time_line = np.array([self.data_[:,0]]).T
        enc_1 = self.data_[:,1]
        enc_2 = self.data_[:,2]
        enc_3 = self.data_[:,3]
        
        enc_1_f = scipy.ndimage.gaussian_filter(enc_1, sigma=sigma, order=0)
        enc_2_f = scipy.ndimage.gaussian_filter(enc_2, sigma=sigma, order=0)
        enc_3_f = scipy.ndimage.gaussian_filter(enc_3, sigma=sigma, order=0)
        
        self.data_filtered_ = np.hstack((time_line, np.array([enc_1_f]).T, np.array([enc_2_f]).T, np.array([enc_3_f]).T))
        
        return None
    
    def load_data(self, file_path):
        
        self.data_ = np.loadtxt(file_path)
        rj_print_sep_line()
        rj_print('Data loaded: ' + file_path)
        rj_print('Size: ' + str(np.shape(self.data_)))
        rj_print_sep_line()
        
        return None
        
    
    def plot_data(self):
        data_array = np.array(self.data_)
        data_filtered_array = np.array(self.data_filtered_)
        
        plt.figure(1, figsize=(15, 12))
        plt.subplot(311)
        plt.plot(data_array[:,0], data_array[:,1])
        plt.plot(data_filtered_array[:,0], data_filtered_array[:,1])
        plt.legend(('raw','filtered'))
        # plt.xlabel('Time (seconds)')
        plt.ylabel('Reading')
        plt.title('ENC1')
        
        
        plt.subplot(312)
        plt.plot(data_array[:,0],data_array[:,2])
        plt.plot(data_filtered_array[:,0], data_filtered_array[:,2])
        # plt.xlabel('Time (seconds)')
        plt.ylabel('Reading')
        plt.title('ENC2')
        plt.show()
        
        plt.subplot(313)
        plt.plot(data_array[:,0],data_array[:,3])
        plt.plot(data_filtered_array[:,0], data_filtered_array[:,3])
        plt.xlabel('Time (seconds)')
        plt.ylabel('Reading')
        plt.title('ENC3')
        plt.show()
        return None
    
