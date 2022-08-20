#! /usr/bin/env python

try:
	from typing import List, Union, Tuple, Optional, Type
except ImportError:
	pass  # Trick VSC into "importing" typing for type hints, but don't actually import it at runtime (still using Python 2.7.17)

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from smoothing_core_v2 import add_table, add_wall, publish_trajectory
import math
import timeit
import trajectory_msgs.msg
from genpy import Duration
from jerk_profile_tests import compute_trajectory, get_durations


# initialize moveit_commander + node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group", anonymous=True)

# initialize moveit
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_e_arm")
# set trajectory display topic
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# add table and wall to planning scene with delays
rospy.sleep(0.5)
add_table(robot, scene)
rospy.sleep(0.5)
add_wall(robot, scene)

# custom poses go here
default_pose = [0, 0, 0, 0, 0, 0]
home_pose = [0, -1.5447, 1.5447, -1.5794, -1.5794, 0]
throw_back_pose = [0, 0, math.radians(-150), math.radians(-90), math.radians(-90), 0]
throw_forward_pose = [0, math.radians(-40), math.radians(-20), math.radians(-127), math.radians(-90), 0]

# [-0.31698329765878197, -0.6699532061591016, -2.5155680976576495, -1.7168752531681915, -0.6437471961150043, -0.1428391287193319]
# [-3.2120455594713477, -3.330059936185687, 5.712068645431851, 5.884150717822338, 3.5914288726074415, 3.5908095067098613]

new_throw_back_pose = [-0.31698329765878197, -2.5155680976576495, -0.6437471961150043, -3.2120455594713477, 5.712068645431851, 3.5914288726074415]
new_throw_forward_pose = [-0.6699532061591016, -1.7168752531681915, -0.1428391287193319, -3.330059936185687, 5.884150717822338, 3.5908095067098613]


def plan_is_invalid(plan):
	# type: (moveit_msgs.msg.RobotTrajectory) -> bool
	return len(plan.joint_trajectory.joint_names) == 0 or len(plan.joint_trajectory.points) == 0 or plan.joint_trajectory.header.frame_id == ''

def points_to_trajectory(time,                 # type: List[float]
						 positions,            # type: List[List[float]]
						 velocities,           # type: List[List[float]]
						 accelerations,        # type: List[List[float]]
						 frame_id,             # type: str
						 joint_names,          # type: List[str]
						 current_joint_values, # type: List[float]
						 joint_focus_index           # type: int
						 ):
	# type: (...) -> moveit_msgs.msg.RobotTrajectory
	result = moveit_msgs.msg.RobotTrajectory()

	result.joint_trajectory.header.frame_id = frame_id
	result.joint_trajectory.joint_names = joint_names

	for i in range(len(time)):
		result.joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint(time_from_start=Duration.from_sec(time[i])))
	
	for joint_index in range(len(joint_names)):
		for time_index in range(len(time)):
			result.joint_trajectory.points[time_index].positions.append(current_joint_values[joint_index])
			result.joint_trajectory.points[time_index].velocities.append(0.0)
			result.joint_trajectory.points[time_index].accelerations.append(0.0)

	# Fill all points with robot's current positions
	# for joint_index in range(len(joint_names)):
	for time_index in range(len(time)):
		result.joint_trajectory.points[time_index].positions[joint_focus_index] = positions[time_index]
		result.joint_trajectory.points[time_index].velocities[joint_focus_index] = velocities[time_index]
		result.joint_trajectory.points[time_index].accelerations[joint_focus_index] = accelerations[time_index]

	return result

group.set_joint_value_target(home_pose)

plan1 = group.plan()

group.execute(plan1, wait=True)

focus_joint = 2
current_position = group.get_current_joint_values()
# target position sign should match velocity waypoint sign
target_pos = math.radians(25.0)

pos = [current_position[focus_joint], target_pos]
# velocity waypoint sign should match target position sign
v_waypoints = [1.5]
waypoint_percentages = [0.7]

(time_points, jerk, positions, velocities, accelerations) = compute_trajectory(pos, v_waypoints, waypoint_percentages, duration_override=None)

traj = points_to_trajectory(time_points, positions, velocities, accelerations, 'base_link', ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], current_position, focus_joint)

publish_trajectory(display_trajectory_publisher, traj, robot)

group.execute(traj, wait=True)

# gracefully shut down moveit commander
moveit_commander.roscpp_shutdown()
