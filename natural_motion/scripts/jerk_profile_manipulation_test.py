#! /usr/bin/env python

try:
	from typing import List, Union, Tuple, Optional, Type
except ImportError:
	pass  # Trick VSC into "importing" typing for type hints, but don't actually import it at runtime (still using Python 2.7.17)
import numpy as np
import matplotlib.pyplot as plt
from smoothing_core_v2 import *
import timeit

def load_npz(filename):
	return np.load(filename)

# TODO: add options for manipulation, merge into smoothing_core_v2
# TODO: add option to plot against original jerk profile models
# TODO: add suptitles
# TODO: convert position + time into JointTrajectory
# TODO: test with robot simulation
# TODO: work with more joints and figure out how to store multiple joints in a single jerk profile

def manipulate_profile(profile,                 # type: np.lib.npyio.NpzFile
					   mapping,                 # type: List[int]
					   new_duration=None,       # type: Optional[float]
					   new_initial_pos=None,    # type: Optional[List[Union[float, None]]]
					   new_final_pos=None,      # type: Optional[List[Union[float, None]]]
					   new_initial_vel=None,    # type: Optional[List[Union[float, None]]]
					   new_initial_accel=None,  # type: Optional[List[Union[float, None]]]
					   show_graphs=False        # type: bool
					   ):
	# type: (...) -> Tuple[Union[np.ndarray, List[np.ndarray]]]

	jerk = profile['jerk']
	time = profile['time']
	initial_vel = profile['vel']
	initial_accel = profile['accel']
	initial_pos = profile['pos'][:,0]
	final_pos = profile['pos'][:,-1]
	original_delta_pos = final_pos - initial_pos

	# profile joints 0 1 2 3 4 5
	# initial pos x y z
	# mapping 1 1 2

	if new_initial_pos is not None and len(new_initial_pos) != len(mapping):
		raise ValueError("Mapping size (" + len(mapping) + ") does not match size of new initial position values (" + len(new_initial_pos) + ")")
	elif new_initial_pos is None:
		new_initial_pos = [None] * len(mapping)
	
	if new_final_pos is not None and len(new_final_pos) != len(mapping):
		raise ValueError("Mapping size (" + len(mapping) + ") does not match size of new final position values (" + len(new_final_pos) + ")")
	elif new_final_pos is None:
		new_final_pos = [None] * len(mapping)
	
	if new_initial_vel is not None and len(new_initial_vel) != len(mapping):
		raise ValueError("Mapping size (" + len(mapping) + ") does not match size of new initial velocity values (" + len(new_initial_vel) + ")")
	elif new_initial_vel is None:
		new_initial_vel = [None] * len(mapping)
	
	if new_initial_accel is not None and len(new_initial_accel) != len(mapping):
		raise ValueError("Mapping size (" + len(mapping) + ") does not match size of new initial acceleration values (" + len(new_initial_accel) + ")")
	elif new_initial_accel is None:
		new_initial_accel = [None] * len(mapping)
	
	if new_duration is None:
		new_duration = time[-1] - time[0]
		time_scaled = time
	else:
		time_scale = new_duration / (time[-1] - time[0])
		time_diff = np.diff(time)
		time_scaled = integrate([x * time_scale for x in time_diff], time[0])
	
	mapped_positions = []

	for index, joint in enumerate(mapping):
		jerk_scale_model = profile['jerk_scale_model'][joint]
		jerk_scale_predict = np.poly1d(jerk_scale_model)

		if new_initial_pos[index] is None:
			new_initial_pos[index] = initial_pos[joint]
		if new_final_pos[index] is None:
			new_final_pos[index] = final_pos[joint]
		if new_initial_vel[index] is None:
			new_initial_vel[index] = initial_vel[joint]
		if new_initial_accel[index] is None:
			new_initial_accel[index] = initial_accel[joint]

		new_delta_pos = new_final_pos[index] - new_initial_pos[index]

		jerk_scale = jerk_scale_predict(new_delta_pos)

		scaled_jerk = [x * jerk_scale for x in jerk[joint]]

		accelerations = integrate_time(scaled_jerk, np.diff(time), new_initial_accel[index])
		velocities = integrate_time(accelerations, np.diff(time), new_initial_vel[index])
		positions = integrate_time(velocities, np.diff(time), new_initial_pos[index])

		mapped_positions.append(positions)

		print("Max velocity (joint " + str(joint) + "): " + str(max(velocities)))

		if show_graphs:
			print("Mapping index " + str(index) + " (joint " + str(joint) + ")")
			print("Original:")
			print("  start:         " + str(initial_pos[joint]))
			print("  end:           " + str(final_pos[joint]))
			print("  -> delta pos:  " + str(original_delta_pos[joint]))
			print("  duration:      " + str(time[-1] - time[0]))
			print("  avg time step: " + str((time[-1] - time[0]) / len(time)))
			print("---")
			print("Desired:")
			print("  start:         " + str(new_initial_pos[index]))
			print("  end:           " + str(new_final_pos[index]))
			print("  -> delta pos:  " + str(new_delta_pos))
			print("  duration:      " + str(new_duration))
			print("  avg time step: " + str(new_duration / len(time)))
			print("---")
			print("Actual:")
			print("  start:         " + str(positions[0]))
			print("  end:           " + str(positions[-1]))
			print("  -> delta pos:  " + str(positions[-1] - positions[0]))
			print("  duration:      " + str(time_scaled[-1] - time_scaled[0]))
			print("  avg time step: " + str((time_scaled[-1] - time_scaled[0]) / len(time_scaled)))
			print("")
			print("")
		
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

	return (time_scaled, mapped_positions)

def points_to_trajectory(time,                 # type: np.ndarray
						 mapped_positions,     # type: List[np.ndarray]
						 mapping,              # type: List[List[int]]
						 frame_id,             # type: str
						 joint_names,          # type: List[str]
						 current_joint_values  # type: List[float]
						 ):
	# type: (...) -> moveit_msgs.msg.RobotTrajectory
	result = moveit_msgs.msg.RobotTrajectory()

	result.joint_trajectory.header.frame_id = frame_id
	result.joint_trajectory.joint_names = joint_names

	for i in range(len(time)):
		result.joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint(time_from_start=Duration.from_sec(time[i])))

	# Fill all points with robot's current positions
	for joint_index in range(len(joint_names)):
		for time_index in range(len(time)):
			result.joint_trajectory.points[time_index].positions.append(current_joint_values[joint_index])

	for mapping_index in range(len(mapping)):
		for joint_index in mapping[mapping_index]:
			for time_index in range(len(time)):
				result.joint_trajectory.points[time_index].positions[joint_index] = mapped_positions[mapping_index][time_index]
	
	return result

def main():
	filename = '/home/joel/catkin_ws/src/ur5e_natural_motion/natural_motion/jerk_profiles/test3.npz'
	profile = load_npz(filename)

	show_graphs = False

	joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
	
	# jerk_mapping refers to the mapping between new_initial_pos, new_final_pos, new_initial_vel, and new_initial_accel.
	# It must be the same length as these variables, if they have values (None will fill them with the profile's original values)
	# Each number in jerk_mapping will correspond to a joint index to apply a jerk profile from.
	# Ex. jerk_mapping=[1, 2, 3] will produce three sets of position data, with jerk profiles defined from joints 1, 2, and 3 respectively
	#     This profile can be modified by including values in the "new" variables, so new_initial_pos = [0.0, 0.0, 0.0] will make all sets of position data start at position 0.0
	#     Including None in these variables will not change the value at that index, so new_final_pos = [1.0, None, 1.0] will make position sets 1 + 3 end at position 1.0, but position set 2 will end at the position defined by the jerk profile


	# jerk_mapping = [0, 1, 2, 3, 4, 5]

	# jerk_mapping = [1]
	# new_duration = 0.7
	# new_initial_pos = [0.0]
	# new_final_pos = [1.2]
	# new_initial_vel = None
	# new_initial_accel = None

	jerk_mapping = [0, 1, 2, 3, 4, 5]
	new_duration = None
	new_initial_pos = None
	new_final_pos = None
	new_initial_vel = None
	new_initial_accel = None

	start = timeit.default_timer()

	(time, pos) = manipulate_profile(profile, jerk_mapping, new_duration=new_duration, new_initial_pos=new_initial_pos, new_final_pos=new_final_pos, new_initial_vel=new_initial_vel, new_initial_accel=new_initial_accel, show_graphs=show_graphs)

	# Once the position data sets are created (pos), a position mapping is needed to tell the trajectory which position sets are to be applied to each joint.
	# Ex. position_mapping=[[1, 2], [3], [4]] will apply position set index 0 to joints 1 and 2, index 1 to joint 3, and index 2 to joint 4
	# Any joint that is not referenced in this list will have the robot's current position value applied for all time values, keeping that joint stationary for the duration of the trajectory.
	# Each joint may only be referenced once

	mid = timeit.default_timer()

	position_mapping = [[1]]

	traj = points_to_trajectory(time, pos, position_mapping, "base_link", joint_names, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

	# print(traj)

	end = timeit.default_timer()
	
	print("Manipulate time: " + str(mid - start))
	print("Translate time: " + str(end - mid))
	print("Total time: " + str(end - start))

	if show_graphs:
		plt.show()

if __name__ == "__main__":
	main()
