import numpy as np
import matplotlib.pyplot as plt

file_name = 'recorder_ravenstate3.csv'
data_path = 'result_files'

raven_state = np.loadtxt(data_path+'/'+file_name, delimiter=',')

print(np.max(raven_state[:,1]))

plt.figure()
plt.plot(raven_state[:,0], raven_state[:,1])
plt.title('1 encVal')


plt.figure()
plt.plot(raven_state[:,0], raven_state[:,2])
plt.title('2 encVal')

plt.figure()
plt.plot(raven_state[:,0], raven_state[:,3])
plt.title('3 encVal')


# --------------------------------------------------
plt.figure()
plt.plot(raven_state[:,0], raven_state[:,17])
plt.title('1 m pos')


plt.figure()
plt.plot(raven_state[:,0], raven_state[:,18])
plt.title('2 m pos')


plt.figure()
plt.plot(raven_state[:,0], raven_state[:,19])
plt.title('3 m pos')



#-------------------------------------------------
plt.figure()
plt.plot(raven_state[:,0], raven_state[:,33])
plt.title('1 jt pos')


plt.figure()
plt.plot(raven_state[:,0], raven_state[:,34])
plt.title('2 jt pos')


plt.figure()
plt.plot(raven_state[:,0], raven_state[:,35])
plt.title('3 jt pos')
plt.show()


