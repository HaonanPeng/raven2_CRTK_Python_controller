import func_raven_bag_reader as reader
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from utils_bag_reader import rms_error

record_name = ''   # the record name is the common name at the end of the recorded ROS bags
train_data_name = '_test' # the training data name is the name that will be given at the end of the processed csv training data file

bag_path =           'bag_files/force_estimation_ws'
result_path =        'result_files/force_estimation_ws'
data_folder_path =   result_path
result_folder_path = 'training_data/force_estimation_ws'

bag_name = 'record_ext_jenc' + record_name + '.bag'
result_name = bag_name[:-3] + 'csv'
data_ext_jpos, index_ext_jpos = reader.read_bag_ext_joint_encoder(bag_name = bag_path + '/' + bag_name, save_inter_rst = False, output_name = None)

# --------------------------------------------------------------------------

bag_name = 'record_CRTKmeasurejs' + record_name + '.bag'
result_name = bag_name[:-3] + 'csv'
data_CRTK_measuredjs, index_CRTK_measuredjs = reader.read_bag_CRTK_measured_js(bag_name = bag_path + '/' + bag_name, save_inter_rst = False, output_name = None)

# --------------------------------------------------------------------------

bag_name = 'record_ravenstate' + record_name + '.bag'
result_name = bag_name[:-3] + 'csv'
data_raven_state, index_raven_state = reader.read_bag_ravenstate(bag_name = bag_path + '/' + bag_name, save_inter_rst = False, output_name = None)

# --------------------------------------------------------------------------

bag_name = 'record_force' + record_name + '.bag'
result_name = bag_name[:-3] + 'csv'
data_force, index_force = reader.read_bag_force(bag_name = bag_path + '/' + bag_name, save_inter_rst = False, output_name = None)

rad2deg = 180.0 / np.pi
deg2rad = np.pi / 180.0

# file_name_ravenstate = 'record_ravenstate' + record_name + '.csv'
# file_name_CRTKmeasuredjs = 'record_CRTKmeasurejs' + record_name + '.csv'
# file_name_ext_jpos = 'record_force' + record_name + '.csv'  # [IMPT] this is replaced by force, originally external joint encoder

# raven_state = np.loadtxt(data_folder_path+'/'+file_name_ravenstate, delimiter=',')
# CRTK_measuredjs = np.loadtxt(data_folder_path+'/'+file_name_CRTKmeasuredjs, delimiter=',')
# ext_jpos = np.loadtxt(data_folder_path+'/'+file_name_ext_jpos, delimiter=',')

print('ROSbag ravenstate size', data_raven_state.shape)
print('ROSbag CRTK measured js size', data_CRTK_measuredjs.shape)
print('ROSbag ext_jpos size', data_ext_jpos.shape)
print('ROSbag force size', data_force.shape)

# combine and pair the three measurements according to time stamp
data_dim = data_raven_state.shape[1] + data_CRTK_measuredjs.shape[1] + data_ext_jpos.shape[1] + data_force.shape[1]
data_comb = np.zeros((data_raven_state.shape[0], data_dim))
print(data_comb.shape)


#line_idx = 0
#for line in ext_jpos:
#    new_line = np.zeros((1,252))
#    idx_raven_state = np.argmin(np.abs(raven_state[:,0] - line[0]))
#    idx_CRTK_measuredjs = np.argmin(np.abs(CRTK_measuredjs[:,0] - line[0]))
#    
#    new_line[0, 0:4] = line
#    new_line[0, 4:12] = CRTK_measuredjs[idx_CRTK_measuredjs, :]
#    new_line[0, 12:] = raven_state[idx_raven_state, :]
#    data_comb[line_idx, :] = new_line[0,:]
#    
#    line_idx += 1

line_idx = 0
start_time = data_raven_state[0,0]

col_idx_ext_jpos = (0, 0+data_ext_jpos.shape[1])
col_idx_CRTK_measuredjs = (col_idx_ext_jpos[1], col_idx_ext_jpos[1]+data_CRTK_measuredjs.shape[1])
col_idx_raven_state = (col_idx_CRTK_measuredjs[1], col_idx_CRTK_measuredjs[1]+data_raven_state.shape[1])
col_idx_force = (col_idx_raven_state[1], col_idx_raven_state[1]+data_force.shape[1])


for line_raven_state in data_raven_state:
    new_line = np.zeros((1,data_dim))
    idx_ext_jpos = np.argmin(np.abs(data_ext_jpos[:,0] - line_raven_state[0]))
    idx_CRTK_measuredjs = np.argmin(np.abs(data_CRTK_measuredjs[:,0] - line_raven_state[0]))
    idx_force = np.argmin(np.abs(data_force[:,0] - line_raven_state[0]))
    
    new_line[0, col_idx_ext_jpos[0]:col_idx_ext_jpos[1]] = data_ext_jpos[idx_ext_jpos, :]
    new_line[0, col_idx_CRTK_measuredjs[0]:col_idx_CRTK_measuredjs[1]] = data_CRTK_measuredjs[idx_CRTK_measuredjs, :]
    new_line[0, col_idx_raven_state[0]:col_idx_raven_state[1]] = line_raven_state
    new_line[0, col_idx_force[0]:col_idx_force[1]] = data_force[idx_force, :]

    new_line[0,0] = new_line[0,0] - start_time
    data_comb[line_idx, :] = new_line[0,:]
    
    line_idx += 1
    print(line_idx)
    
np.savetxt(result_folder_path + '/data_record' + train_data_name + '.csv', data_comb, delimiter=',')

# write the index file
f = open(result_folder_path + '/data_index' + train_data_name + '.txt', "w")
start_idx = 0
# external joint position
for i in range(len(index_ext_jpos[0])):
    f.write(str(index_ext_jpos[1][i] + start_idx) + '  ' + index_ext_jpos[0][i] + '\n')
start_idx +=  index_ext_jpos[1][-1,1]

print('----------------------')
print(start_idx)
print(index_ext_jpos[1])
print(index_ext_jpos[1][-1:1])
print(index_CRTK_measuredjs[1][0] + start_idx)
# CRTK measured joint position
for i in range(len(index_CRTK_measuredjs[0])):
    f.write(str(index_CRTK_measuredjs[1][i] + start_idx) + '  ' + index_CRTK_measuredjs[0][i] + '\n')
start_idx +=  index_CRTK_measuredjs[1][-1,1]
# external joint position
for i in range(len(index_raven_state[0])):
    f.write(str(index_raven_state[1][i] + start_idx) + '  ' + index_raven_state[0][i] + '\n')
start_idx +=  index_raven_state[1][-1,1]
# external joint position
for i in range(len(index_force[0])):
    f.write(str(index_force[1][i] + start_idx) + '  ' + index_force[0][i] + '\n')
start_idx +=  index_force[1][-1,1]
f.close()

# # 3D traj plots --------------------------------------------------------------
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.plot3D(CRTK_measuredjs[:,1] * rad2deg, CRTK_measuredjs[:,2] * rad2deg, CRTK_measuredjs[:,3], 'b-', label = 'CRTK_ravenstate')
# ax.plot3D(ext_jpos[:,1] * rad2deg, ext_jpos[:,2] * rad2deg, ext_jpos[:,3], 'y--', label = 'external joint encoder')
# ax.set_xlabel('joint 1 (deg)')
# ax.set_ylabel('joint 2 (deg)')
# ax.set_zlabel('joint 3 (m)')
# plt.legend()
# plt.title('3D Traj')

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# CRTK_measuredjs_linear_moved =  CRTK_measuredjs
# CRTK_measuredjs_linear_moved[:,1] = CRTK_measuredjs[:,1] - (np.mean(CRTK_measuredjs[:,1]) - np.mean(ext_jpos[:,1]))
# CRTK_measuredjs_linear_moved[:,2] = CRTK_measuredjs[:,2] - (np.mean(CRTK_measuredjs[:,2]) - np.mean(ext_jpos[:,2]))
# CRTK_measuredjs_linear_moved[:,3] = CRTK_measuredjs[:,3] - (np.mean(CRTK_measuredjs[:,3]) - np.mean(ext_jpos[:,3]))

# ax.plot3D(CRTK_measuredjs_linear_moved[:,1] * rad2deg, CRTK_measuredjs_linear_moved[:,2] * rad2deg, CRTK_measuredjs_linear_moved[:,3], 'b-', label = 'CRTK_ravenstate')
# ax.plot3D(ext_jpos[:,1] * rad2deg, ext_jpos[:,2] * rad2deg, ext_jpos[:,3], 'y--', label = 'external joint encoder')
# ax.set_xlabel('joint 1 (deg)')
# ax.set_ylabel('joint 2 (deg)')
# ax.set_zlabel('joint 3 (m)')
# plt.legend()
# plt.title('3D Traj - linear corrected')

# # 2D plots -----------------------------------------------------------------------
# time_line = data_comb[:,0]
# jpos_1_ext = data_comb[:,1]
# jpos_1_CRTK_lin_corr = data_comb[:,5] - (np.mean(data_comb[:,5]) - np.mean(data_comb[:,1]))

# rms_joint1 = rms_error(jpos_1_ext, jpos_1_CRTK_lin_corr)

# print('joint 1 RMS (deg): ' + str(rms_joint1 * rad2deg))

# fig = plt.figure(figsize=(17,5))
# plt.subplot(131)
# plt.plot(time_line, jpos_1_CRTK_lin_corr * rad2deg, 'b-', label = 'CRTK_ravenstate')
# plt.plot(time_line, jpos_1_ext * rad2deg, 'y--', label = 'external joint encoder')
# plt.xlabel('time')
# plt.ylabel('degree')
# #plt.legend()
# plt.title('joint 1 - linear corrected')


# time_line = data_comb[:,0]
# jpos_2_ext = data_comb[:,2]
# jpos_2_CRTK_lin_corr = data_comb[:,6] - (np.mean(data_comb[:,6]) - np.mean(data_comb[:,2]))

# rms_joint2 = rms_error(jpos_2_ext, jpos_2_CRTK_lin_corr)

# print('joint 2 RMS (deg): ' + str(rms_joint2 * rad2deg))

# plt.subplot(132)
# plt.plot(time_line, jpos_2_CRTK_lin_corr * rad2deg, 'b-', label = 'CRTK_ravenstate')
# plt.plot(time_line, jpos_2_ext * rad2deg, 'y--', label = 'external joint encoder')
# plt.xlabel('time')
# plt.ylabel('degree')
# #plt.legend()
# plt.title('joint 2 - linear corrected')


# time_line = data_comb[:,0]
# jpos_3_ext = data_comb[:,3]
# jpos_3_CRTK_lin_corr = data_comb[:,7] - (np.mean(data_comb[:,7]) - np.mean(data_comb[:,3]))

# rms_joint3 = rms_error(jpos_3_ext, jpos_3_CRTK_lin_corr)

# print('joint 3 RMS (deg): ' + str(rms_joint3 ))

# plt.subplot(133)
# plt.plot(time_line, jpos_3_CRTK_lin_corr , 'b-', label = 'CRTK_ravenstate')
# plt.plot(time_line, jpos_3_ext , 'y--', label = 'external joint encoder')
# plt.xlabel('time')
# plt.ylabel('m')
# plt.legend()
# plt.title('joint 3 - linear corrected')


# plt.show()
