#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

from smoothing_core_v2 import *

npz = np.load('/home/joel/catkin_ws/src/ur5e_natural_motion/natural_motion/jerk_profiles/test2.npz')  # type: np.lib.npyio.NpzFile
jerk = npz['jerk']
time = npz['time']
initial_vel = npz['vel']
initial_accel = npz['accel']
initial_pos = npz['pos'][:,0]
final_pos = npz['pos'][:,-1]

# time = integrate([x + 0.001 for x in np.diff(time)], 0)

# print("Total time: " + str(time[-1]))

# print("")

# print(len(jerk))
# print(len(accelerations))
# print(len(velocities))
# print(len(positions))

# print("")
# print(positions[0])
# print(positions[-1])
# print(initial_pos)
# print(final_pos)


for joint in range(len(jerk)):
	accelerations = integrate_time(jerk[joint], np.diff(time), initial_accel[joint])
	velocities = integrate_time(accelerations, np.diff(time), initial_vel[joint])
	positions = integrate_time(velocities, np.diff(time), initial_pos[joint])

	print("Joint " + str(joint))
	print("Original position: " + str(initial_pos[joint]) + " --> " + str(final_pos[joint]))
	print("Integrated position: " + str(positions[0]) + " --> " + str(positions[-1]))

	fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex=True)
	fig.suptitle("Joint " + str(joint))

	ax1.plot(time, positions)
	ax1.set_xlabel('time (s)')
	ax1.set_ylabel('rad')
	ax1.set_title('Position')

	ax2.plot(shift_times(time), velocities, c='orange')
	ax2.set_xlabel('time (s)')
	ax2.set_ylabel('rad/s')
	ax2.set_title('Velocity')

	ax3.plot(shift_times(time, 2), accelerations, c='r')
	ax3.set_xlabel('time (s)')
	ax3.set_ylabel('rad/s^2')
	ax3.set_title('Acceleration')

	ax4.plot(shift_times(time, 3), jerk[joint], c='g')
	ax4.set_xlabel('time (s)')
	ax4.set_ylabel('rad/s^3')
	ax4.set_title('Jerk')

plt.show()
