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

# This code will subscribe to the force gauge topic and ravenstate, and then record the data and save to txt.

import numpy as np
import rospy
# from std_msgs.msg import Float32
import sensor_msgs.msg
from raven_2.msg import raven_state
import sys
import signal
import argparse
import matplotlib.pyplot as plt

class raven2_froce_gauge_recorder():

    def __init__(self, name_surfix):
        self.record_file_name = 'force_gauge_records/force_gauge_record_' + name_surfix + '.txt'
        self.record_plot_name = 'force_gauge_records/force_gauge_record_' + name_surfix + '.jpg'
        rospy.init_node('force_gauge_recorder', anonymous=True)
        rospy.sleep(1)

        self.first_rs = True
        self.data = [[]]

        self.new_tau = False
        self.new_ravenstate = False

        self.__sub_force_gauge = rospy.Subscriber('force_gauge', sensor_msgs.msg.JointState, self.__callback_force_gauge)
        self.__sub_ravenstate = rospy.Subscriber('ravenstate', raven_state, self.__callback_ravenstate)
        
    def __callback_force_gauge(self, msg):
        self.tau_cur = msg.position[0]
        self.new_tau = True
        return None
    def __callback_ravenstate(self, msg):
        self.ravenstate_cur = msg
        self.new_ravenstate = True
        if self.first_rs:
            self.start_time = msg.hdr.stamp.to_sec()
            self.first_rs = False

    def record(self):
        while True:
            if self.new_tau and self.new_ravenstate:
                self.data.append([self.ravenstate_cur.hdr.stamp.to_sec() - self.start_time, self.tau_cur, self.ravenstate_cur.tau[13], self.ravenstate_cur.mvel[13]])
                self.new_tau = False
                self.new_ravenstate = False

    def save_data(self):
        data_arr = np.array(self.data[1:])
        np.savetxt(self.record_file_name, data_arr)
        print('Recording finished, data saved at: ' + self.record_file_name)
        print('Data size: ' + str(data_arr.shape))

    def save_plot(self):
        fig, ax = plt.subplots()
        data_arr = np.array(self.data[1:])

        #compute statistics
        mean_force_neg = np.mean(data_arr[data_arr[:,3]<-5][:,1])
        mean_force_posi = np.mean(data_arr[data_arr[:,3]>5][:,1])
        stdv_force_neg = np.std(data_arr[data_arr[:,3]<-5][:,1])
        stdv_force_posi = np.std(data_arr[data_arr[:,3]>5][:,1])

        print('Result: ')
        print(str(round(stdv_force_posi, 6)) + '\t' + str(round(stdv_force_neg, 6)) + '\t' + str(round(mean_force_posi, 6)) + '\t' + str(round(mean_force_neg, 6)))
        #print(str(round(mean_force_posi, 6)))
        #print(str(round(mean_force_neg, 6)))
        #print(str(round(stdv_force_posi, 6)))
        #print(str(round(stdv_force_neg, 6)))

        # # draw plots
        # plt.scatter(data_arr[:,3], data_arr[:,1], c='b')
        # ax.axvline(x=0, color = 'g')
        # ax.axhline(y=100 * np.mean(data_arr[:,2]), color = 'r')
        # ax.set_ylim([-1,6])
        # plt.xlabel('velocity')
        # plt.ylabel('force')
        # plt.title('negative mean: ' + str(round(mean_force_neg, 6)) + '    positive mean: ' + str(round(mean_force_posi, 6)) + '\n negative STDV: ' + str(round(stdv_force_neg, 6)) + '    positive STDV: ' + str(round(stdv_force_posi, 6)))
        # #ax.text(0.5, -0.2, 'negative mean: ' + str(round(mean_force_neg, 6)) + '    positive mean: ' + str(round(mean_force_posi, 6)) + '\n negative STDV: ' + str(round(stdv_force_neg, 6)) + '    positive STDV: ' + str(round(stdv_force_posi, 6)), ha='center', va='center', transform=ax.transAxes)
        # plt.savefig(self.record_plot_name)

        # draw plots
        fig, (ax1, ax2, ax3) = plt.subplots(3)
        fig.suptitle('negative mean: ' + str(round(mean_force_neg, 6)) + '    positive mean: ' + str(round(mean_force_posi, 6)) + '\n negative STDV: ' + str(round(stdv_force_neg, 6)) + '    positive STDV: ' + str(round(stdv_force_posi, 6)))
        ax1.scatter(data_arr[:,3], data_arr[:,1], c='b')
        ax1.axvline(x=0, color = 'g')
        ax1.axhline(y=100 * np.mean(data_arr[:,2]), color = 'r')
        ax1.set_ylim([-1,6])
        ax1.set(xlabel='velocity', ylabel='force')
        ax2.plot(data_arr[:,0], data_arr[:,1])
        ax2.set(xlabel='time', ylabel='force')
        ax2.set_ylim([-1,6])

        ax3.plot(data_arr[:,0], data_arr[:,3])
        ax3.set(xlabel='time', ylabel='velocity')
        fig.set_figheight(15)
        
        #ax.text(0.5, -0.2, 'negative mean: ' + str(round(mean_force_neg, 6)) + '    positive mean: ' + str(round(mean_force_posi, 6)) + '\n negative STDV: ' + str(round(stdv_force_neg, 6)) + '    positive STDV: ' + str(round(stdv_force_posi, 6)), ha='center', va='center', transform=ax.transAxes)
        plt.savefig(self.record_plot_name)


    
    def __del__(self):
        data_arr = np.array(self.data[1:])
        np.savetxt(self.record_file_name, data_arr)
        print('Recording finished, data saved at: ' + self.record_file_name)
        print('Data size: ' + str(data_arr.shape))





parser = argparse.ArgumentParser()
parser.add_argument('suffix', type=str, default='00')
args = parser.parse_args()
r2_fgr = raven2_froce_gauge_recorder(args.suffix)
def main():
    
    
    def signal_handler(signal, frame):
        global r2_fgr
        r2_fgr.save_data()
        r2_fgr.save_plot()
        rospy.sleep(3)
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)


    r2_fgr.record()

if __name__ == "__main__":
    main()





    

