import roslib   #roslib.load_manifest(PKG)
import rosbag
import rospy
import sys
import numpy as np
import sensor_msgs.msg

def read_bag_ravenstate(bag_name, save_inter_rst = False, output_name = None):
    with rosbag.Bag(bag_name , 'r') as bag:

        ravenstate_ = [[0.0]*240]
        counter = 0
        for topic,msg,t in bag.read_messages():

            timestr = float("%.6f" %  msg.hdr.stamp.to_sec())
            newline = [0.0]*240
            newline[0] = timestr
            idx_count = 1

            for index in range(0,16):
                newline[ idx_count] = float("%.6f" % msg.jpos[index])
                idx_count += 1

            newline[ idx_count] = float("%.6f" % msg.runlevel)
            idx_count += 1
            newline[ idx_count] = float("%.6f" % msg.sublevel)
            idx_count += 1
            newline[ idx_count] = float("%.6f" % msg.last_seq)
            idx_count += 1

            for index in range(0,2):
                newline[ idx_count] = float("%.6f" % msg.type[index])
                idx_count += 1

            for index in range(0,6):
                newline[ idx_count] = float("%.6f" % msg.pos[index])
                idx_count += 1

            for index in range(0,18):
                newline[ idx_count] = float("%.6f" % msg.ori[index])
                idx_count += 1

            for index in range(0,18):
                newline[ idx_count] = float("%.6f" % msg.ori_d[index])
                idx_count += 1

            for index in range(0,6):
                newline[ idx_count] = float("%.6f" % msg.pos_d[index])
                idx_count += 1

            # newline[0, idx_count] = (msg.dt)
            # idx_count += 1

            for index in range(0,16):
                newline[ idx_count] = float("%.6f" % msg.encVals[index])
                idx_count += 1

            for index in range(0,16):
                newline[ idx_count] = float("%.6f" % msg.dac_val[index])
                idx_count += 1

            for index in range(0,16):
                newline[ idx_count] = float("%.6f" % msg.tau[index])
                idx_count += 1

            for index in range(0,16):
                newline[ idx_count] = float("%.6f" % msg.mpos[index])
                idx_count += 1

            for index in range(0,16):
                newline[ idx_count] = float("%.6f" % msg.mvel[index])
                idx_count += 1

            for index in range(0,16):
                newline[ idx_count] = float("%.6f" % msg.jvel[index])
                idx_count += 1

            for index in range(0,16):
                newline[ idx_count] = float("%.6f" % msg.mpos_d[index])
                idx_count += 1

            for index in range(0,16):
                newline[ idx_count] = float("%.6f" % msg.jpos_d[index])
                idx_count += 1

            for index in range(0,2):
                newline[ idx_count] = float("%.6f" % msg.grasp_d[index])
                idx_count += 1

            for index in range(0,16):
                newline[ idx_count] = float("%.6f" % msg.encoffsets[index])
                idx_count += 1

            for index in range(0,12):
                newline[ idx_count] = float("%.6f" % msg.jac_vel[index])
                idx_count += 1

            for index in range(0,12):
                newline[ idx_count] = float("%.6f" % msg.jac_f[index])
                idx_count += 1

            ravenstate_.append(newline)
            counter +=1
            print(counter)
        # create the index name
        info_name = ['time_stamp_raven_state',
                     'joint_position',
                     'run_level',
                     'sub_level',
                     'last_sequency',
                     'arm_type',
                     'end_effector_position',
                     'end_effector_orientation',
                     'end_effector_orientation_desired',
                     'end_effector_position_desired',
                     'encoder_value',
                     'dac_value',
                     'torque',
                     'motor_position',
                     'motor_velocity',
                     'joint_velocity',
                     'motor_position_desired',
                     'joint_position_desired',
                     'grasper_desired',
                     'encoder_offset',
                     'jacobian_velocity',
                     'jacobian_force']
        
        info_index = np.array([[0, 1], # time_stamp
                               [1, 17], # joint_position
                               [17, 18], # run_level
                               [18, 19], # sub_level
                               [19, 20], # last_sequency
                               [20, 22], # arm_type
                               [22, 28], # end_effector_position
                               [28, 46], # end_effector_orientation
                               [46, 64], # end_effector_orientation_desired
                               [64, 70], # end_effector_position_desired
                               [70, 86], # encoder_value
                               [86, 102], # dac_value
                               [102, 118], # torque
                               [118, 134], # motor_position
                               [134, 150], # motor_velocity
                               [150, 166], # joint_velocity
                               [166, 182], # motor_position_desired
                               [182, 198], # joint_position_desired
                               [198, 200], # grasper_desired
                               [200, 216], # encoder_offset
                               [216, 228], # jacobian_velocity
                               [228, 240]]) # jacobian_force

        ravenstate_array = np.array(ravenstate_[1:][:])
        if save_inter_rst:
            np.savetxt(output_name, ravenstate_array, delimiter=",")
        print('ravenstate bag converted to csv, shape: ' + str(ravenstate_array.shape))

    return ravenstate_array, [info_name, info_index]

def read_bag_CRTK_measured_js(bag_name, save_inter_rst = False, output_name = None):
    with rosbag.Bag(bag_name , 'r') as bag:

        result_ = [[0.0]*8]
        counter = 0
        for topic,msg,t in bag.read_messages():

            timestr = float("%.6f" %  msg.header.stamp.to_sec())
            newline = [0.0]*8
            newline[0] = timestr
            idx_count = 1

            for index in range(0,7):
                newline[idx_count] = float("%.6f" % msg.position[index])
                idx_count += 1

            result_.append(newline)
            counter += 1
            print(counter)

        result_array = np.array(result_[1:][:])

        info_name = ['time_stamp_CRTK',
                     'CRTK_measured_js']
        info_index = np.array([[0, 1],
                               [1, 8]])
        
        if save_inter_rst:
            np.savetxt(output_name, result_array, delimiter=",")
        print('CRTK_measured_js bag converted to csv, shape: ' + str(result_array.shape))

    return result_array, [info_name, info_index]

def read_bag_ext_joint_encoder(bag_name, save_inter_rst = False, output_name = None):
    with rosbag.Bag(bag_name , 'r') as bag:

        result_ = [[0.0]*4]
        counter = 0
        for topic,msg,t in bag.read_messages():

            timestr = float("%.6f" %  msg.header.stamp.to_sec())
            newline = [0.0]*4
            newline[0] = timestr
            idx_count = 1

            for index in range(0,3):
                newline[idx_count] = float("%.6f" % msg.position[index])
                idx_count += 1

            result_.append(newline)
            counter += 1
            print(counter)

        result_array = np.array(result_[1:][:])

        info_name = ['time_stamp_ext_joint_enc',
                     'ext_joint_position']
        info_index = np.array([[0, 1],
                               [1, 4]])
        
        if save_inter_rst:
            np.savetxt(output_name, result_array, delimiter=",")
        print('External joint encoder bag converted to csv, shape: ' + str(result_array.shape))

    return result_array, [info_name, info_index]


# this is the same as reading external joint encoders
def read_bag_force(bag_name, save_inter_rst = False, output_name = None):
    with rosbag.Bag(bag_name , 'r') as bag:

        result_ = [[0.0]*4]
        counter = 0
        for topic,msg,t in bag.read_messages():

            timestr = float("%.6f" %  msg.header.stamp.to_sec())
            newline = [0.0]*4
            newline[0] = timestr
            idx_count = 1

            for index in range(0,3):
                newline[idx_count] = float("%.6f" % msg.position[index])
                idx_count += 1

            result_.append(newline)
            counter += 1
            print(counter)

        result_array = np.array(result_[1:][:])

        info_name = ['time_stamp_force',
                     'ext_end_effector_force']
        info_index = np.array([[0, 1],
                               [1, 4]])
        
        if save_inter_rst:
            np.savetxt(output_name, result_array, delimiter=",")
        print('Force bag converted to csv, shape: ' + str(result_array.shape))

    return result_array, [info_name, info_index]



