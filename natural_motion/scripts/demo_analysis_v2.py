#! /usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt
import copy
from scipy.signal import savgol_filter
from sklearn.metrics import r2_score
import os
from smoothing_core_v2 import *

def load_joint_state_data(filename):
	# type: (str) -> h5py._hl.group.Group
	hf = h5py.File(filename, 'r')
	js = hf.get('joint_state_info')
	return js

def plot_joint_position_data(js, time_min, time_max, joint_index=None, smooth=False):
	# type: (h5py._hl.group.Group, int, int, int, bool) -> tuple[np.ndarray]
	joint_pos = np.array(js.get('joint_positions'))
	joint_time = np.array(js.get('joint_time'))

	full_joint_time = joint_time[:, 0] + joint_time[:, 1] * (10.0**-9)
	time = np.array([x - full_joint_time[0] for x in full_joint_time[0:time_max-time_min]])

	if joint_index is not None:
		joints = [joint_index]
	else:
		joints = range(joint_pos.shape[1])

	p = []
	v = []
	a = []
	j = []

	for joint in joints:
		positions = joint_pos[time_min:time_max,joint]

		fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex=True)
		fig.suptitle("Joint " + str(joint))
		print("Joint " + str(joint))
		print("---")

		ax1.plot(time, positions)
		ax1.set_xlabel('time (s)')
		ax1.set_ylabel('rad')
		ax1.set_title('Position')
		if smooth:
			positions = savgol_filter(positions, 51, 5, mode='nearest')
			ax1.plot(time, positions, c='black')

		velocities = deriv(time, positions)
		ax2.plot(shift_times(time), velocities, c='orange')
		ax2.set_xlabel('time (s)')
		ax2.set_ylabel('rad/s')
		ax2.set_title('Velocity')
		if smooth:
			velocities = savgol_filter(velocities, 51, 5)
			ax2.plot(shift_times(time), velocities, c='black')

		accelerations = deriv(shift_times(time), velocities)
		ax3.plot(shift_times(time, 2), accelerations, c='r')
		ax3.set_xlabel('time (s)')
		ax3.set_ylabel('rad/s^2')
		ax3.set_title('Acceleration')
		if smooth:
			accelerations = savgol_filter(accelerations, 51, 5)
			ax3.plot(shift_times(time, 2), accelerations, c='black')

		jerk = deriv(shift_times(time, 2), accelerations)
		ax4.plot(shift_times(time, 3), jerk, c='g')
		ax4.set_xlabel('time (s)')
		ax4.set_ylabel('rad/s^3')
		ax4.set_title('Jerk')
		if smooth:
			jerk = savgol_filter(jerk, 51, 5)
			ax4.plot(shift_times(time, 3), jerk, c='black')

		if smooth:
			print("[Smoothing ON]")
		else:
			print("[Smoothing OFF]")
		
		print("---")

		print("List Lengths")
		print("Time:         [ " + str(len(time)) + " ]")
		print("Position:     [ " + str(len(positions)) + " ]")
		print("Velocity:     [ " + str(len(velocities)) + " ]")
		print("Acceleration: [ " + str(len(accelerations)) + " ]")
		print("Jerk:         [ " + str(len(jerk)) + " ]")

		print("---")

		print("Beginning/End Points")
		print("Time:         " + str(time[0]) + " --> " + str(time[-1]))
		print("Position:     " + str(positions[0]) + " --> " + str(positions[-1]))
		print("Velocity:     " + str(velocities[0]) + " --> " + str(velocities[-1]))
		print("Acceleration: " + str(accelerations[0]) + " --> " + str(accelerations[-1]))
		print("Jerk:         " + str(jerk[0]) + " --> " + str(jerk[-1]))

		max_vel = np.argmax(velocities)

		print("Max velocity: index " + str(max_vel) + " time=" + str(shift_times(time)[max_vel]) + " vel=" + str(velocities[max_vel]))

		print("")

		p.append(positions)
		v.append(velocities)
		a.append(accelerations)
		j.append(jerk)

	p = np.array(p)
	v = np.array(v)
	a = np.array(a)
	j = np.array(j)

	return (time, p, v, a, j)

def save_data(filename, data):
	# type: (str, tuple[np.ndarray]) -> None
	# time = data[0], pos = data[1], vel = data[2], accel = data[3], jerk = data[4]


	pos = [[x[0], x[-1]] for x in data[1]]
	vel = [[x[0]] for x in data[2]]
	accel = [[x[0]] for x in data[3]]


	np.savez(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, 'jerk_profiles', filename)),
		time=data[0], jerk=data[4], pos=pos, vel=vel, accel=accel)

def deriv(x, y):
	# type: (np.ndarray, np.ndarray) -> np.ndarray
	return np.diff(y)/np.diff(x)

def main():
	h5_filename = '/home/joel/catkin_ws/src/ur5e_natural_motion/natural_motion/scripts/recorded_demo 2022-07-18 13_20_48.h5'  # 13_16_55.h5 / 13_20_48.h5
	time_min = 2614  # 2602
	time_max = 2715  # 2713
	joint_index = 1
	smooth = True

	save = True
	npz_filename = 'test4.npz'

	js_data = load_joint_state_data(h5_filename)

	data = plot_joint_position_data(js_data, time_min, time_max, smooth=smooth)

	if save:
		save_data(npz_filename, data)

	plt.show()

if __name__ == '__main__':
	main()
