# Reads the output.csv file for simple pendulum and plots the q vs time plot
import numpy as np
import matplotlib.pyplot as plt
#data = np.genfromtxt('/home/skumar/src/rbdl/examples/double-pendulum/build/output.csv', delimiter=',', skip_header=0, skip_footer=0, names=['t', 'q1', 'q2'])

data = np.genfromtxt('/local/skumar/software/rbdlpy/unittest/test.csv', delimiter=',', skip_header=0, skip_footer=0, names=['t', 'q1', 'q2'])

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Time evolution of Joint trajectory")    
ax1.set_xlabel('Time(s)')
ax1.set_ylabel('Joint position(rad)')

ax1.plot(data['t'], data['q1'], color='r', label='Joint trajectory')
ax1.plot(data['t'], data['q2'], color='b', label='Joint trajectory')
plt.show()
