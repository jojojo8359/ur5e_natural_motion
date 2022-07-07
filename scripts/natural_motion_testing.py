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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_e_arm")
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# Jerk profile types
UDDU = lambda min, max: [max, 0, min, 0, min, 0, max]
DUUD = lambda min, max: [min, 0, max, 0, max, 0, min]
UDUD = lambda min, max: [max, 0, min, 0, max, 0, min]
DUDU = lambda min, max: [min, 0, max, 0, min, 0, max]

# TODO: Implement min/max velocity/acceleration
def generate_smooth_trajectory(input_plan, jerk_profile_type, vel_min, vel_max, accel_min, accel_max, jerk_min=-1.0, jerk_max=None):
	# type: (moveit_msgs.msg.RobotTrajectory, function, float, float, float, float, float, float) -> moveit_msgs.msg.RobotTrajectory
	
	# num_of_dofs = len(input_plan.joint_trajectory.joint_names)
	# if num_of_dofs == 0:
	# 	return  # TODO: Return empty trajectory, cannot be optimized because it was never created

	# Problem Statement
	if jerk_max is None:
		jerk_max = -jerk_min

	jerk_profile = jerk_profile_type(jerk_min, jerk_max)  # Initialize the jerk profile with the provided max/min jerk values

	initial_state = input_plan.joint_trajectory.points[0]  # type: trajectory_msgs.msg.JointTrajectoryPoint
	final_state = input_plan.joint_trajectory.points[-1]  # type: trajectory_msgs.msg.JointTrajectoryPoint
	
	times = np.linspace(initial_state.time_from_start.to_sec(), final_state.time_from_start.to_sec(), 8, retstep=True)  # Create an array of timestamps to create 7 time intervals
	# print(times)
	t = times[0]
	t_step = float(times[1])

	result = copy.deepcopy(input_plan)
	result.joint_trajectory.points[:] = []  # Clears the trajectory points, leaving an identical trajectory with no points

	for k, ts in enumerate(t):  # For each point ts (index k) in all times t
		# print("Current time: ts=" + str(ts) + "  k=" + str(k))
		current_jerk = jerk_profile[k-1]
		# print("    jerk=" + str(current_jerk))
		new_point = trajectory_msgs.msg.JointTrajectoryPoint()
		new_point.time_from_start = Duration.from_sec(ts)
		if k == 0:  # First timestamp, so use initial pos, vel, and accel to start deriving
			new_point.accelerations = initial_state.accelerations
			new_point.velocities = initial_state.velocities
			new_point.positions = initial_state.positions
		else:  # Generate new values based on previous ones
			new_point.accelerations = [gen_next_a(x, current_jerk, t_step) for x in result.joint_trajectory.points[-1].accelerations]
			new_point.velocities = [gen_next_v(result.joint_trajectory.points[-1].accelerations[x], result.joint_trajectory.points[-1].velocities[x], current_jerk, t_step) for x in range(len(result.joint_trajectory.joint_names))]
			new_point.positions = [gen_next_p(result.joint_trajectory.points[-1].accelerations[x], result.joint_trajectory.points[-1].velocities[x], result.joint_trajectory.points[-1].positions[x], current_jerk, t_step) for x in range(len(result.joint_trajectory.joint_names))]
		result.joint_trajectory.points.append(new_point)
	
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


original_joint_positions = group.get_current_joint_values()
profile_type = UDUD
joint_index = 4

for pos in [1.75, 2.0, 2.25]:
	original_joint_positions[joint_index] = pos
	group.set_joint_value_target(original_joint_positions)

	plan1 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory
	print("Position: " + str(pos) + " Time: " + str(plan1.joint_trajectory.points[-1].time_from_start.to_sec()))

	prev_pos = 0.0
	prev_jerk = 0.0

	for i in np.arange(0.0, 50.0, 0.25):
		# final_pos = generate_smooth_trajectory(plan1, profile_type, 0.0, 0.0, 0.0, 0.0, -i).joint_trajectory.points[-1].positions[joint_index]
		# if final_pos < pos:
		# 	prev_pos = final_pos
		# 	prev_jerk = i
		# else:
		# 	print(str(prev_pos) + "->" + str(final_pos) + "===" + str(prev_jerk) + "->" + str(i))
		# 	break
		print(str(generate_smooth_trajectory(plan1, profile_type, 0.0, 0.0, 0.0, 0.0, -i).joint_trajectory.points[-1].positions[joint_index]))

	# for max_jerk in [0.25, 0.5, 1, 2, 3, 4, 5, 10, 15, 20, 25, 30]:
		# print("    Jerk: " + str(max_jerk) + " Final pos: " + str(generate_smooth_trajectory(plan1, profile_type, 0.0, 0.0, 0.0, 0.0, -max_jerk).joint_trajectory.points[-1].positions[joint_index]))
		# print(str(generate_smooth_trajectory(plan1, profile_type, 0.0, 0.0, 0.0, 0.0, -max_jerk).joint_trajectory.points[-1].positions[joint_index]))


# print(plan1)
# traj = generate_smooth_trajectory(plan1, UDUD, 0.0, 0.0, 0.0, 0.0, -3.1)
# Could the max/min jerk be directly related to the final position of the arm or the total duration of the trajectory?
# UDUD: move to 0.5 -> -12.3 jerk, but with move to 1.0 final position jumps to 4.0+
# print(traj)


rospy.sleep(1)

moveit_commander.roscpp_shutdown()
