#! /usr/bin/env python

try:
	from typing import List, Union, Tuple, Optional, Type
except ImportError:
	pass  # Trick VSC into "importing" typing for type hints, but don't actually import it at runtime (still using Python 2.7.17)

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from jerk_profile_manipulation_test import manipulate_profile, points_to_trajectory, load_npz
from smoothing_core_v2 import add_table, add_wall, publish_trajectory
import math
import timeit

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
home_pose = [0, 0, 0, 0, 0, 0]
throw_back_pose = [0, 0, math.radians(-150), math.radians(-90), math.radians(-90), 0]
throw_forward_pose = [0, math.radians(-40), math.radians(-20), math.radians(-127), math.radians(-90), 0]

# [-0.31698329765878197, -0.6699532061591016, -2.5155680976576495, -1.7168752531681915, -0.6437471961150043, -0.1428391287193319]
# [-3.2120455594713477, -3.330059936185687, 5.712068645431851, 5.884150717822338, 3.5914288726074415, 3.5908095067098613]

new_throw_back_pose = [-0.31698329765878197, -2.5155680976576495, -0.6437471961150043, -3.2120455594713477, 5.712068645431851, 3.5914288726074415]
new_throw_forward_pose = [-0.6699532061591016, -1.7168752531681915, -0.1428391287193319, -3.330059936185687, 5.884150717822338, 3.5908095067098613]

# vflip = lambda angle: math.radians(360) - angle
# invert_pose = lambda x: [-x[0], x[1] - (2 * (x[1] - math.radians(-90))), -x[2], 

invalid_pose = [math.radians(-333), 0, 0, 0, 0, 0]

def plan_is_invalid(plan):
	# type: (moveit_msgs.msg.RobotTrajectory) -> bool
	return len(plan.joint_trajectory.joint_names) == 0 or len(plan.joint_trajectory.points) == 0 or plan.joint_trajectory.header.frame_id == ''

def throw(group, jerk_profile_filename, back_pose, forward_pose, new_duration=None, new_initial_vel=None, new_initial_accel=None, use_jerk_joint=None):
	# type: (moveit_commander.MoveGroupCommander, str, List[float], List[float], float, List[float], List[float], int) -> None
	group.set_joint_value_target(back_pose)
	group.go(wait=True)

	group.set_joint_value_target(new_throw_forward_pose)
	plan1 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory

	if plan_is_invalid(plan1):
		print("No plan was generated, no jerk profile will be applied")

	profile = load_npz(jerk_profile_filename)

	new_initial_pos = back_pose
	new_final_pos = forward_pose

	if use_jerk_joint is not None:
		jerk_mapping = [use_jerk_joint] * len(back_pose)
	else:
		jerk_mapping = range(len(back_pose))

	start = timeit.default_timer()

	(time, pos) = manipulate_profile(profile, jerk_mapping, new_duration=new_duration, new_initial_pos=new_initial_pos, new_final_pos=new_final_pos, new_initial_vel=new_initial_vel, new_initial_accel=new_initial_accel, show_graphs=False)

	mid = timeit.default_timer()

	position_mapping = [[0], [1], [2], [3], [4], [5]]

	traj = points_to_trajectory(time, pos, position_mapping, plan1.joint_trajectory.header.frame_id, plan1.joint_trajectory.joint_names, plan1.joint_trajectory.points[0].positions)

	end = timeit.default_timer()

	print("Manipulate time: " + str(mid - start))
	print("Translate time: " + str(end - mid))
	print("Total time: " + str(end - start))

	publish_trajectory(display_trajectory_publisher, traj, robot)

	group.execute(traj, wait=True)


filename = '/home/joel/catkin_ws/src/ur5e_natural_motion/natural_motion/jerk_profiles/test3.npz'
throw(group, filename, throw_back_pose, throw_forward_pose, new_duration=1.2, new_initial_vel=None, new_initial_accel=None, use_jerk_joint=1)
# throw(group, filename, new_throw_back_pose, new_throw_forward_pose, new_duration=1.2, new_initial_vel=None, new_initial_accel=None, use_jerk_joint=None)

# gracefully shut down moveit commander
moveit_commander.roscpp_shutdown()
