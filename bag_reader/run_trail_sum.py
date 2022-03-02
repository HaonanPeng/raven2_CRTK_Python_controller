import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from utils_bag_reader import rms_error

rad2deg = 180.0 / np.pi
deg2rad = np.pi / 180.0

data_folder_path = 'result_files'

file_name_ravenstate = 'recorder_ravenstate.csv'
file_name_CRTKmeasuredjs = 'recorder_CRTKmeasurejs.csv'
file_name_ext_jpos = 'recorder_ext_jenc.csv'

raven_state = np.loadtxt(data_folder_path+'/'+file_name_ravenstate, delimiter=',')
CRTK_measuredjs = np.loadtxt(data_folder_path+'/'+file_name_CRTKmeasuredjs, delimiter=',')
ext_jpos = np.loadtxt(data_folder_path+'/'+file_name_ext_jpos, delimiter=',')

print(raven_state.shape)
print(CRTK_measuredjs.shape)
print(ext_jpos.shape)

# combine and pair the three measurements according to time stamp
data_comb = np.zeros((ext_jpos.shape[0],252))
print(data_comb.shape)

line_idx = 0
for line in ext_jpos:
    new_line = np.zeros((1,252))
    idx_raven_state = np.argmin(np.abs(raven_state[:,0] - line[0]))
    idx_CRTK_measuredjs = np.argmin(np.abs(CRTK_measuredjs[:,0] - line[0]))
    
    new_line[0, 0:4] = line
    new_line[0, 4:12] = CRTK_measuredjs[idx_CRTK_measuredjs, :]
    new_line[0, 12:] = raven_state[idx_raven_state, :]
    data_comb[line_idx, :] = new_line[0,:]
    
    line_idx += 1
    
    

# 3D traj plots --------------------------------------------------------------
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(CRTK_measuredjs[:,1] * rad2deg, CRTK_measuredjs[:,2] * rad2deg, CRTK_measuredjs[:,3], 'b-', label = 'CRTK_ravenstate')
ax.plot3D(ext_jpos[:,1] * rad2deg, ext_jpos[:,2] * rad2deg, ext_jpos[:,3], 'y--', label = 'external joint encoder')
ax.set_xlabel('joint 1 (deg)')
ax.set_ylabel('joint 2 (deg)')
ax.set_zlabel('joint 3 (m)')
plt.legend()
plt.title('3D Traj')

fig = plt.figure()
ax = plt.axes(projection='3d')
CRTK_measuredjs_linear_moved =  CRTK_measuredjs
CRTK_measuredjs_linear_moved[:,1] = CRTK_measuredjs[:,1] - (np.mean(CRTK_measuredjs[:,1]) - np.mean(ext_jpos[:,1]))
CRTK_measuredjs_linear_moved[:,2] = CRTK_measuredjs[:,2] - (np.mean(CRTK_measuredjs[:,2]) - np.mean(ext_jpos[:,2]))
CRTK_measuredjs_linear_moved[:,3] = CRTK_measuredjs[:,3] - (np.mean(CRTK_measuredjs[:,3]) - np.mean(ext_jpos[:,3]))

ax.plot3D(CRTK_measuredjs_linear_moved[:,1] * rad2deg, CRTK_measuredjs_linear_moved[:,2] * rad2deg, CRTK_measuredjs_linear_moved[:,3], 'b-', label = 'CRTK_ravenstate')
ax.plot3D(ext_jpos[:,1] * rad2deg, ext_jpos[:,2] * rad2deg, ext_jpos[:,3], 'y--', label = 'external joint encoder')
ax.set_xlabel('joint 1 (deg)')
ax.set_ylabel('joint 2 (deg)')
ax.set_zlabel('joint 3 (m)')
plt.legend()
plt.title('3D Traj - linear corrected')

# 2D plots -----------------------------------------------------------------------
time_line = data_comb[:,0]
jpos_1_ext = data_comb[:,1]
jpos_1_CRTK_lin_corr = data_comb[:,5] - (np.mean(data_comb[:,5]) - np.mean(data_comb[:,1]))

rms_joint1 = rms_error(jpos_1_ext, jpos_1_CRTK_lin_corr)

print('joint 1 RMS (deg): ' + str(rms_joint1 * rad2deg))

fig = plt.figure()
plt.plot(time_line, jpos_1_CRTK_lin_corr * rad2deg, 'b-', label = 'CRTK_ravenstate')
plt.plot(time_line, jpos_1_ext * rad2deg, 'y--', label = 'external joint encoder')
plt.xlabel('time')
plt.ylabel('degree')
plt.legend()
plt.title('joint 1 - linear corrected')


time_line = data_comb[:,0]
jpos_2_ext = data_comb[:,2]
jpos_2_CRTK_lin_corr = data_comb[:,6] - (np.mean(data_comb[:,6]) - np.mean(data_comb[:,2]))

rms_joint2 = rms_error(jpos_2_ext, jpos_2_CRTK_lin_corr)

print('joint 2 RMS (deg): ' + str(rms_joint2 * rad2deg))

fig = plt.figure()
plt.plot(time_line, jpos_2_CRTK_lin_corr * rad2deg, 'b-', label = 'CRTK_ravenstate')
plt.plot(time_line, jpos_2_ext * rad2deg, 'y--', label = 'external joint encoder')
plt.xlabel('time')
plt.ylabel('degree')
plt.legend()
plt.title('joint 2 - linear corrected')


time_line = data_comb[:,0]
jpos_3_ext = data_comb[:,3]
jpos_3_CRTK_lin_corr = data_comb[:,7] - (np.mean(data_comb[:,7]) - np.mean(data_comb[:,3]))

rms_joint3 = rms_error(jpos_3_ext, jpos_3_CRTK_lin_corr)

print('joint 3 RMS (deg): ' + str(rms_joint3 ))

fig = plt.figure()
plt.plot(time_line, jpos_3_CRTK_lin_corr , 'b-', label = 'CRTK_ravenstate')
plt.plot(time_line, jpos_3_ext , 'y--', label = 'external joint encoder')
plt.xlabel('time')
plt.ylabel('m')
plt.legend()
plt.title('joint 3 - linear corrected')




plt.show()
