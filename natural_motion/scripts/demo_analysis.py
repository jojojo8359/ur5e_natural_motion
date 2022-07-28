#! /usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from sklearn.metrics import r2_score

def main():
	filename = '/home/joel/catkin_ws/src/ur5e_natural_motion/natural_motion/scripts/recorded_demo 2022-07-18 13_20_48.h5'  # 13_16_55.h5 / 13_20_48.h5
	hf = h5py.File(filename, 'r')
	js = hf.get('joint_state_info')
	joint_pos = np.array(js.get('joint_positions'))
	joint_time = np.array(js.get('joint_time'))

	time_min = 2614  # 2602
	time_max = 2713  # 2726
	joint_index = 1

	positions = joint_pos[time_min:time_max,joint_index]
	full_joint_time = joint_time[:, 0] + joint_time[:, 1] * (10.0**-9)
	# time = full_joint_time[time_min:time_max]
	time = np.array([x - full_joint_time[0] for x in full_joint_time[0:time_max-time_min]])
	print(np.diff(time).tolist())

	print("Position: " + str(positions[0]) + " -> " + str(positions[-1]))
	time_step = time[1] - time[0]
	print("Time step: " + str(time_step))
	orig_duration = time[-1] - time[0]
	print("Time: " + str(time[0]) + " -> " + str(time[-1]) + " = " + str(orig_duration))

	fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex=True)
	# fig.suptitle("")

	# Plot position
	ax1.plot(time, positions)
	ax1.set_xlabel('time (s)')
	ax1.set_ylabel('rad/s')
	ax1.set_title('Position')

	# Calculate and plot velocity
	velocities = np.diff(positions)/np.diff(time)
	time = time[0:-1]
	ax2.plot(time, velocities, c='orange')
	ax2.set_xlabel('time (s)')
	ax2.set_ylabel('rad/s')
	ax2.set_title('Velocity')

	v_hat = savgol_filter(velocities, 51, 5)
	ax2.plot(time, v_hat, c='black')

	# Calculate and plot acceleration
	# accelerations = np.diff(velocities)/np.diff(time)
	accelerations = np.diff(v_hat)/np.diff(time)
	time = time[0:-1]
	ax3.plot(time, accelerations, c='r')
	ax3.set_xlabel('time (s)')
	ax3.set_ylabel('rad/s')
	ax3.set_title('Acceleration')

	a_hat = savgol_filter(accelerations, 51, 5)
	ax3.plot(time, a_hat, c='black')

	# Calculate and plot jerk
	# jerk = np.diff(accelerations)/np.diff(time)
	jerk = np.diff(a_hat)/np.diff(time)
	time = time[0:-1]
	ax4.plot(time, jerk, c='g')
	ax4.set_xlabel('time (s)')
	ax4.set_ylabel('rad/s')
	ax4.set_title('Jerk')

	j_hat = savgol_filter(jerk, 51, 5)
	ax4.plot(time, j_hat, c='black')

	jerk_duration = time[-1] - time[0]
	duration_difference = orig_duration - jerk_duration
	print("Time after jerk: " + str(time[0]) + " -> " + str(time[-1]) + " = " + str(jerk_duration))
	print("Time difference: " + str(duration_difference) + " / " + str(duration_difference / time_step))

	plt.show()

if __name__ == '__main__':
	main()
