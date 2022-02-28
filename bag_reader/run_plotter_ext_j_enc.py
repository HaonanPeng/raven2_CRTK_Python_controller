import numpy as np
import matplotlib.pyplot as plt

file_name = 'recorder_ext_jenc1.csv'
data_path = 'result_files'
raven_state1 = np.loadtxt(data_path+'/'+file_name, delimiter=',')

file_name = 'recorder_ext_jenc2.csv'
data_path = 'result_files'
raven_state2 = np.loadtxt(data_path+'/'+file_name, delimiter=',')

file_name = 'recorder_ext_jenc3.csv'
data_path = 'result_files'
raven_state3 = np.loadtxt(data_path+'/'+file_name, delimiter=',')

raven_state = np.vstack((raven_state1, raven_state2, raven_state3))




rad2deg = 180/np.pi

plt.figure()
plt.plot( raven_state[:,1])
plt.title('1 ext_js')


plt.figure()
plt.plot( raven_state[:,2])
plt.title('2 ext_js')

plt.figure()
plt.plot( raven_state[:,3])
plt.title('3 ext_js')

plt.show()


# --------------------------------------------------
# plt.figure()
# plt.plot(raven_state[:,0], raven_state[:,17])
# plt.title('1 m pos')


# plt.figure()
# plt.plot(raven_state[:,0], raven_state[:,18])
# plt.title('2 m pos')


# plt.figure()
# plt.plot(raven_state[:,0], raven_state[:,19])
# plt.title('3 m pos')



# #-------------------------------------------------
# plt.figure()
# plt.plot(raven_state[:,0], raven_state[:,33])
# plt.title('1 jt pos')


# plt.figure()
# plt.plot(raven_state[:,0], raven_state[:,34])
# plt.title('2 jt pos')


# plt.figure()
# plt.plot(raven_state[:,0], raven_state[:,35])
# plt.title('3 jt pos')
# plt.show()


