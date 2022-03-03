import roslib   #roslib.load_manifest(PKG)
import rosbag
import rospy
import sys
import numpy as np
import sensor_msgs.msg

def read_bag_ravenstate(bag_name, output_name = None):
    with rosbag.Bag(bag_name , 'r') as bag:

        ravenstate_ = np.zeros((1,240))
        counter = 0
        for topic,msg,t in bag.read_messages():

            timestr = "%.6f" %  msg.hdr.stamp.to_sec()
            newline = np.zeros((1,240))
            newline[0] = timestr
            idx_count = 1

            for index in range(0,16):
                newline[0, idx_count] = ("%.6f" % msg.jpos[index])
                idx_count += 1

            newline[0, idx_count] = ("%.6f" % msg.runlevel)
            idx_count += 1
            newline[0, idx_count] = ("%.6f" % msg.sublevel)
            idx_count += 1
            newline[0, idx_count] = ("%.6f" % msg.last_seq)
            idx_count += 1

            for index in range(0,2):
                newline[0, idx_count] = ("%.6f" % msg.type[index])
                idx_count += 1

            for index in range(0,6):
                newline[0, idx_count] = ("%.6f" % msg.pos[index])
                idx_count += 1

            for index in range(0,18):
                newline[0, idx_count] = ("%.6f" % msg.ori[index])
                idx_count += 1

            for index in range(0,18):
                newline[0, idx_count] = ("%.6f" % msg.ori_d[index])
                idx_count += 1

            for index in range(0,6):
                newline[0, idx_count] = ("%.6f" % msg.pos_d[index])
                idx_count += 1

            # newline[0, idx_count] = (msg.dt)
            # idx_count += 1

            for index in range(0,16):
                newline[0, idx_count] = ("%.6f" % msg.encVals[index])
                idx_count += 1

            for index in range(0,16):
                newline[0, idx_count] = ("%.6f" % msg.dac_val[index])
                idx_count += 1

            for index in range(0,16):
                newline[0, idx_count] = ("%.6f" % msg.tau[index])
                idx_count += 1

            for index in range(0,16):
                newline[0, idx_count] = ("%.6f" % msg.mpos[index])
                idx_count += 1

            for index in range(0,16):
                newline[0, idx_count] = ("%.6f" % msg.mvel[index])
                idx_count += 1

            for index in range(0,16):
                newline[0, idx_count] = ("%.6f" % msg.jvel[index])
                idx_count += 1

            for index in range(0,16):
                newline[0, idx_count] = ("%.6f" % msg.mpos_d[index])
                idx_count += 1

            for index in range(0,16):
                newline[0, idx_count] = ("%.6f" % msg.jpos_d[index])
                idx_count += 1

            for index in range(0,2):
                newline[0, idx_count] = ("%.6f" % msg.grasp_d[index])
                idx_count += 1

            for index in range(0,16):
                newline[0, idx_count] = ("%.6f" % msg.encoffsets[index])
                idx_count += 1

            for index in range(0,12):
                newline[0, idx_count] = ("%.6f" % msg.jac_vel[index])
                idx_count += 1

            for index in range(0,12):
                newline[0, idx_count] = ("%.6f" % msg.jac_f[index])
                idx_count += 1

            ravenstate_ = np.append(ravenstate_, newline, axis=0)
            counter +=1
            print(counter)

        np.savetxt(output_name, ravenstate_[1:,:], delimiter=",")
        print('ravenstate bag converted to csv, shape: ' + str(ravenstate_[1:,:].shape))

    return None

def read_bag_CRTK_measured_js(bag_name, output_name = None):
    with rosbag.Bag(bag_name , 'r') as bag:

        result_ = np.zeros((1,8))
        counter = 0
        for topic,msg,t in bag.read_messages():

            timestr = "%.6f" %  msg.header.stamp.to_sec()
            newline = np.zeros((1,8))
            newline[0] = timestr
            idx_count = 1

            for index in range(0,7):
                newline[0, idx_count] = ("%.6f" % msg.position[index])
                idx_count += 1

            result_ = np.append(result_, newline, axis=0)
            counter += 1
            print(counter)

        np.savetxt(output_name, result_[1:,:], delimiter=",")
        print('CRTK_measured_js bag converted to csv, shape: ' + str(result_[1:,:].shape))

    return None

def read_bag_ext_joint_encoder(bag_name, output_name = None):
    with rosbag.Bag(bag_name , 'r') as bag:

        result_ = np.zeros((1,4))
        counter = 0
        for topic,msg,t in bag.read_messages():

            timestr = "%.6f" %  msg.header.stamp.to_sec()
            newline = np.zeros((1,4))
            newline[0] = timestr
            idx_count = 1

            for index in range(0,3):
                newline[0, idx_count] = ("%.6f" % msg.position[index])
                idx_count += 1

            result_ = np.append(result_, newline, axis=0)
            counter += 1
            print(counter)

        np.savetxt(output_name, result_[1:,:], delimiter=",")
        print('External joint encoder bag converted to csv, shape: ' + str(result_[1:,:].shape))

    return None



