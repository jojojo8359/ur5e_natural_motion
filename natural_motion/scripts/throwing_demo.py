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


def plan_is_invalid(plan):
	# type: (moveit_msgs.msg.RobotTrajectory) -> bool
	return len(plan.joint_trajectory.joint_names) == 0 or len(plan.joint_trajectory.points) == 0 or plan.joint_trajectory.header.frame_id == ''

def points_to_trajectory(time,                 # type: List[float]
						 positions,            # type: List[List[float]]
						 velocities,           # type: List[List[float]]
						 accelerations,        # type: List[List[float]]
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

	return result
