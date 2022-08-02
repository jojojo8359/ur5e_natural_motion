#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from smoothing_core_v2 import *

def load_npz(filename):
	return np.load(filename)

# TODO: add options for manipulation, merge into smoothing_core_v2
# TODO: add option to plot against original jerk profile models
# TODO: add suptitles
# TODO: convert position + time into JointTrajectory
# TODO: test with robot simulation
# TODO: work with more joints and figure out how to store multiple joints in a single jerk profile

def main():
	filename = '/home/joel/catkin_ws/src/ur5e_natural_motion/natural_motion/jerk_profiles/test2.npz'
	profile = load_npz(filename)

	jerk = profile['jerk']
	time = profile['time']
	initial_vel = profile['vel'][0]
	initial_accel = profile['accel'][0]
	initial_pos = profile['pos'][0]
	final_pos = profile['pos'][-1]
	original_delta_pos = final_pos - initial_pos

	jerk_scale_model = profile['jerk_scale_model']
	time_delta_pos_model = profile['time_delta_pos_model']
	jerk_scale_predict = np.poly1d(jerk_scale_model)
	time_delta_pos_predict = np.poly1d(time_delta_pos_model)

	new_duration = 0.8
	new_initial_pos = 0.3
	new_final_pos = 1.5
	new_delta_pos = new_final_pos - new_initial_pos
	new_initial_accel = 0.0

	time_scale = new_duration / (time[-1] - time[0])
	time_diff = np.diff(time)
	time_scaled = integrate([x * time_scale for x in time_diff], time[0])
	delta_pos_scale = time_delta_pos_predict(time_scale)

	print("Original:")
	print("  start:         " + str(initial_pos))
	print("  end:           " + str(final_pos))
	print("  -> delta pos:  " + str(original_delta_pos))
	print("  duration:      " + str(time[-1] - time[0]))
	print("  avg time step: " + str((time[-1] - time[0]) / len(time)))
	print("---")
	print("Desired:")
	print("  start:         " + str(new_initial_pos))
	print("  end:           " + str(new_final_pos))
	print("  -> delta pos:  " + str(new_delta_pos))
	print("  duration:      " + str(new_duration))
	print("  avg time step: " + str(new_duration / len(time)))
	print("---")
	print("delta pos scale as a result of time scale:")
	print("  " + str(delta_pos_scale))
	print("original delta pos with above scale:")
	print("  " + str(delta_pos_scale * original_delta_pos))
	print("desired delta pos with above scale:")
	print("  " + str(delta_pos_scale * (new_delta_pos)))

	scaled_original_delta_pos = delta_pos_scale * original_delta_pos
	scaled_desired_delta_pos = delta_pos_scale * new_delta_pos

	jerk_scale = jerk_scale_predict(new_delta_pos / delta_pos_scale)

	scaled_jerk = [x * jerk_scale for x in jerk]

	accelerations = integrate_time(scaled_jerk, np.diff(time_scaled), new_initial_accel)
	velocities = integrate_time(accelerations, np.diff(time_scaled), initial_vel)
	positions = integrate_time(velocities, np.diff(time_scaled), new_initial_pos)

	print("---")
	print("Actual:")
	print("  start:         " + str(positions[0]))
	print("  end:           " + str(positions[-1]))
	print("  -> delta pos:  " + str(positions[-1] - positions[0]))
	print("  duration:      " + str(time_scaled[-1] - time_scaled[0]))
	print("  avg time step: " + str((time_scaled[-1] - time_scaled[0]) / len(time_scaled)))

	fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex=True)

	ax1.plot(time_scaled, positions)
	ax1.set_xlabel('time (s)')
	ax1.set_ylabel('rad')
	ax1.set_title('Position')

	ax2.plot(shift_times(time_scaled), velocities, c='orange')
	ax2.set_xlabel('time (s)')
	ax2.set_ylabel('rad/s')
	ax2.set_title('Velocity')

	ax3.plot(shift_times(time_scaled, 2), accelerations, c='r')
	ax3.set_xlabel('time (s)')
	ax3.set_ylabel('rad/s^2')
	ax3.set_title('Acceleration')

	ax4.plot(shift_times(time_scaled, 3), scaled_jerk, c='g')
	ax4.set_xlabel('time (s)')
	ax4.set_ylabel('rad/s^3')
	ax4.set_title('Jerk')

	plt.show()



if __name__ == "__main__":
	main()
