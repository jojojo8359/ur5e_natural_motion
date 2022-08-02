#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score
from smoothing_core_v2 import *

def generate_jerk_scale_model(jerk_profile):
	jerk = jerk_profile['jerk']
	time = jerk_profile['time']
	initial_vel = jerk_profile['vel'][0]
	initial_accel = jerk_profile['accel'][0]
	initial_pos = jerk_profile['pos'][0]
	final_pos = jerk_profile['pos'][-1]

	jerk_scale_min = 0.1
	jerk_scale_max = 2.5
	jerk_scale_step = 0.1

	jerk_scales = np.arange(jerk_scale_min, jerk_scale_max, jerk_scale_step)
	delta_pos = []

	for scale in jerk_scales:
		scaled_jerk = [x * scale for x in jerk]

		accelerations = integrate_time(scaled_jerk, np.diff(time), initial_accel)
		velocities = integrate_time(accelerations, np.diff(time), initial_vel)
		positions = integrate_time(velocities, np.diff(time), initial_pos)

		delta_pos.append(positions[-1] - positions[0])

	print("---")
	scale_model = np.polyfit(delta_pos, jerk_scales, 1)  # type: np.ndarray
	scale_predict = np.poly1d(scale_model)
	scale_r2 = r2_score(jerk_scales, scale_predict(delta_pos))
	print("Jerk Scale Model:")
	print(scale_model)
	print("r2: " + str(scale_r2))

	x_lin_reg = np.arange(0, max(delta_pos) + 0.25, 0.25)
	y_lin_reg = scale_predict(x_lin_reg)

	fig, (ax1) = plt.subplots(1, 1)
	fig.suptitle("Jerk Scale Model")
	ax1.set_xlabel("Position Delta")
	ax1.set_ylabel("Jerk Scale")
	ax1.set_title("Position Delta vs. Jerk Scale")

	ax1.scatter(delta_pos, jerk_scales)
	ax1.plot(x_lin_reg, y_lin_reg, c='r')

	return scale_predict

def generate_time_delta_pos_model(jerk_profile):
	jerk = jerk_profile['jerk']
	time = jerk_profile['time']
	initial_vel = jerk_profile['vel'][0]
	initial_accel = jerk_profile['accel'][0]
	initial_pos = jerk_profile['pos'][0]
	final_pos = jerk_profile['pos'][-1]
	original_delta_pos = final_pos - initial_pos

	scale_min = 0.1
	scale_max = 2.5
	scale_step = 0.1

	time_scales = np.arange(scale_min, scale_max, scale_step)
	delta_pos_scales = []

	for time_scale in time_scales:
		time_diff = np.diff(time)
		time_scaled = integrate([x * time_scale for x in time_diff], time[0])

		accelerations = integrate_time(jerk, np.diff(time_scaled), initial_accel)
		velocities = integrate_time(accelerations, np.diff(time_scaled), initial_vel)
		positions = integrate_time(velocities, np.diff(time_scaled), initial_pos)

		delta_pos_scales.append((positions[-1] - positions[0]) / original_delta_pos)
	
	print("---")
	model = np.polyfit(time_scales, delta_pos_scales, 3)  # type: np.ndarray
	predict = np.poly1d(model)
	r2 = r2_score(delta_pos_scales, predict(time_scales))
	print("Time Delta Position Model:")
	print(model)
	print("r2: " + str(r2))

	x_lin_reg = np.arange(0, max(time_scales) + 0.1, 0.1)
	y_lin_reg = predict(x_lin_reg)

	fig, (ax1) = plt.subplots(1, 1)
	fig.suptitle("Time Delta Position Model")
	ax1.set_xlabel("Time Scale")
	ax1.set_ylabel("Delta Position Scale")
	ax1.set_title("Time Scale vs. Delta Position Scale")

	ax1.scatter(time_scales, delta_pos_scales)
	ax1.plot(x_lin_reg, y_lin_reg, c='r')

	return predict

def load_npz(filename):
	return np.load(filename)

def save_npz(filename, data):
	np.savez(filename, **data)

def main():
	filename = '/home/joel/catkin_ws/src/ur5e_natural_motion/natural_motion/jerk_profiles/test2.npz'
	profile = load_npz(filename)
	profile_data = dict(profile)

	time_delta_pos_model = generate_time_delta_pos_model(profile)
	profile_data['time_delta_pos_model'] = time_delta_pos_model

	jerk_scale_model = generate_jerk_scale_model(profile)
	profile_data['jerk_scale_model'] = jerk_scale_model

	save_npz(filename, profile_data)
	print("---")
	print("File \"" + filename + "\" updated with models.")

	plt.show()

if __name__ == "__main__":
	main()
