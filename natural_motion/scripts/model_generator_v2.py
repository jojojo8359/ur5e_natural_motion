#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score
from smoothing_core_v2 import *

def generate_jerk_scale_model(jerk_profile):
	jerk = jerk_profile['jerk']
	time = jerk_profile['time']
	initial_vel = jerk_profile['vel']
	initial_accel = jerk_profile['accel']
	initial_pos = jerk_profile['pos'][:,0]
	final_pos = jerk_profile['pos'][:,-1]

	jerk_scale_min = 0.1
	jerk_scale_max = 2.5
	jerk_scale_step = 0.1

	jerk_scales = np.arange(jerk_scale_min, jerk_scale_max, jerk_scale_step)

	models = []

	for joint in range(len(jerk)):
		delta_pos = []

		for scale in jerk_scales:
			scaled_jerk = [x * scale for x in jerk[joint]]

			accelerations = integrate_time(scaled_jerk, np.diff(time), initial_accel[joint])
			velocities = integrate_time(accelerations, np.diff(time), initial_vel[joint])
			positions = integrate_time(velocities, np.diff(time), initial_pos[joint])

			delta_pos.append(positions[-1] - positions[0])

		print("---")
		scale_model = np.polyfit(delta_pos, jerk_scales, 1)  # type: np.ndarray
		scale_predict = np.poly1d(scale_model)
		scale_r2 = r2_score(jerk_scales, scale_predict(delta_pos))
		print("Jerk Scale Model:")
		print(scale_model)
		print("r2: " + str(scale_r2))

		delta_pos_step = (max(delta_pos) - min(delta_pos)) / (len(delta_pos))
		x_lin_reg = np.arange(min(delta_pos), max(delta_pos) + delta_pos_step, delta_pos_step)
		y_lin_reg = scale_predict(x_lin_reg)

		fig, (ax1) = plt.subplots(1, 1)
		fig.suptitle("Jerk Scale Model (Joint " + str(joint) + ")")
		ax1.set_xlabel("Position Delta")
		ax1.set_ylabel("Jerk Scale")
		ax1.set_title("Position Delta vs. Jerk Scale")

		ax1.scatter(delta_pos, jerk_scales)
		ax1.plot(x_lin_reg, y_lin_reg, c='r')

		models.append(scale_model)

	return models

def load_npz(filename):
	return np.load(filename)

def save_npz(filename, data):
	np.savez(filename, **data)

def main():
	filename = '/home/joel/catkin_ws/src/ur5e_natural_motion/natural_motion/jerk_profiles/test3.npz'
	profile = load_npz(filename)
	profile_data = dict(profile)

	jerk_scale_model = generate_jerk_scale_model(profile)
	profile_data['jerk_scale_model'] = jerk_scale_model

	save_npz(filename, profile_data)
	print("---")
	print("File \"" + filename + "\" updated with models.")

	plt.show()

if __name__ == "__main__":
	main()
