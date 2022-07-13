#! /usr/bin/env python

import sys
import copy
from genpy import Duration
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import numpy as np
import math

# TODO: Finish documenting

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_e_arm")
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

group_variable_values = group.get_current_joint_values()

home_pose = [0, -1.5447, 1.5447, -1.5794, -1.5794, 0]
default_pose = [0, 0, 0, 0, 0, 0]

throw_back_pose = [0, 0, math.radians(-150), math.radians(-90), math.radians(-90), 0]
throw_forward_pose = [0, math.radians(-40), math.radians(-20), math.radians(-127), math.radians(-90), 0]

group.set_joint_value_target(throw_back_pose)

# group_variable_values[1] = -1.5447
# group_variable_values[4] = 3.0
# group.set_joint_value_target(group_variable_values)

plan1 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory

# Jerk profile types
UDDU = lambda min, max: [max, 0, min, 0, min, 0, max]
DUUD = lambda min, max: [min, 0, max, 0, max, 0, min]
UDUD = lambda min, max: [max, 0, min, 0, max, 0, min]
DUDU = lambda min, max: [min, 0, max, 0, min, 0, max]

def generate_smooth_trajectory(input_plan, jerk_profile_type):
	# type: (moveit_msgs.msg.RobotTrajectory, function) -> moveit_msgs.msg.RobotTrajectory
	
	# Return an empty trajectory if there is no data that can be used
	if len(input_plan.joint_trajectory.joint_names) == 0:
		return moveit_msgs.msg.RobotTrajectory()

	initial_state = input_plan.joint_trajectory.points[0]  # type: trajectory_msgs.msg.JointTrajectoryPoint
	final_state = input_plan.joint_trajectory.points[-1]  # type: trajectory_msgs.msg.JointTrajectoryPoint
	
	# Create an array of timestamps to create 7 time intervals
	# t holds the 8 timestamps, while t_step is the step between each interval
	times = np.linspace(initial_state.time_from_start.to_sec(), final_state.time_from_start.to_sec(), 8, retstep=True)
	t = times[0]
	t_step = float(times[1])

	# Copy the original trajectory to retain header data
	result = copy.deepcopy(input_plan)
	# Clears the trajectory points, leaving an identical trajectory with no points
	result.joint_trajectory.points[:] = []

	# Create new empty points for the new generated timestamps
	for i in range(len(t)):
		result.joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint(time_from_start=Duration.from_sec(t[i])))

	for joint_index in range(len(input_plan.joint_trajectory.joint_names)):
		# Generate the jerk limits based on the DoF's final position and the trajectory's total time
		max_jerk = find_jerk_limit(final_state.positions[joint_index] - initial_state.positions[joint_index], final_state.time_from_start.to_sec(), jerk_profile_type)
		min_jerk = -max_jerk

		# Initialize the jerk profile with the generated max/min jerk values
		jerk_profile = jerk_profile_type(min_jerk, max_jerk)  # type: list

		for k in range(len(t)):
			current_jerk = jerk_profile[k-1]
			if k == 0:
				# Use initial position data as the first point to integrate from
				# Initialize acceleration with 0 (to avoid non-zeroed acceleration curves)
				result.joint_trajectory.points[k].accelerations.append(0)
				result.joint_trajectory.points[k].velocities.append(initial_state.velocities[joint_index])
				result.joint_trajectory.points[k].positions.append(initial_state.positions[joint_index])
			else:
				# Use integration to calculate the rest of the profile
				result.joint_trajectory.points[k].accelerations.append(gen_next_a(result.joint_trajectory.points[k-1].accelerations[joint_index], current_jerk, t_step))
				result.joint_trajectory.points[k].velocities.append(gen_next_v(result.joint_trajectory.points[k-1].accelerations[joint_index], result.joint_trajectory.points[k-1].velocities[joint_index], current_jerk, t_step))
				result.joint_trajectory.points[k].positions.append(gen_next_p(result.joint_trajectory.points[k-1].accelerations[joint_index], result.joint_trajectory.points[k-1].velocities[joint_index], result.joint_trajectory.points[k-1].positions[joint_index], current_jerk, t_step))
	
	return result

	
def gen_next_a(current_accel, jerk_with_sign, time_step):
	# type: (float, float, float) -> float

	# a_k+1 = a_k + s_k*j_k*t_k
	return current_accel + (jerk_with_sign * time_step)

def gen_next_v(current_accel, current_vel, jerk_with_sign, time_step):
	# type: (float, float, float, float) -> float

	# v_k+1 = v_k + a_k*t_k + (s_k*j_k / 2)t^2_k
	return current_vel + (current_accel*time_step) + ((jerk_with_sign / 2) * (time_step ** 2))

def gen_next_p(current_accel, current_vel, current_pos, jerk_with_sign, time_step):
	# type: (float, float, float, float, float) -> float

	# p_k+1 = p_k + v_k*t_k + (a_k / 2)t^2_k + (s_k*j_k / 6)t^3_k
	return current_pos + (current_vel * time_step) + ((current_accel / 2) * (time_step ** 2)) + ((jerk_with_sign / 6) * (time_step ** 3))

def find_jerk_limit(position_delta, total_duration, jerk_profile_type):
	# type: (float, float, function) -> float
	jerk_slope = time_to_jerk_slope(total_duration, jerk_profile_type)
	# When finding jerk limits, treat UDDU the same as DUUD and UDUD the same as DUDU by flipping the equation
	if jerk_profile_type in (DUUD, DUDU):
		coeff = -1
	else:
		coeff = 1
	return coeff * (position_delta / jerk_slope)

def time_to_jerk_slope(duration, jerk_profile_type):
	# type: (float, function) -> float
	# Each profile used in smoothing needs its own equation similar to these, which relate duration to position/jerk slope
	if jerk_profile_type in (UDDU, DUUD):
		return -0.00058 + (0.00138 * duration) - (0.000936 * (duration ** 2)) + (0.0235 * (duration ** 3))
	elif jerk_profile_type in (UDUD, DUDU):
		return -0.00328 + (0.00764 * duration) - (0.00522 * (duration ** 2)) + (0.0419 * (duration ** 3))

def publish_trajectory(publisher, trajectory, robot):
	# type: (rospy.Publisher, moveit_msgs.msg.RobotTrajectory, moveit_commander.RobotCommander) -> None
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(trajectory)
	publisher.publish(display_trajectory)


traj = generate_smooth_trajectory(plan1, DUDU)
print("Total time: " + str(plan1.joint_trajectory.points[-1].time_from_start.to_sec()))
print("MoveIt Final Position:")
print(plan1.joint_trajectory.points[-1].positions)
print("Smoothed Final Position:")
print(traj.joint_trajectory.points[-1].positions)

# Send the new trajectory to the /move_group/display_planned_path topic to display in RViz/rqt
publish_trajectory(display_trajectory_publisher, traj, robot)
group.execute(traj, wait=True)
# publish_trajectory(display_trajectory_publisher, plan1, robot)
# group.execute(plan1, wait=True)

rospy.sleep(1)

moveit_commander.roscpp_shutdown()
