#! /usr/bin/env python

import sys
import copy
from genpy import Duration
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from jerk_profiles import *
from smoothing_core import *
import trajectory_msgs.msg
import numpy as np
import math

def wait_for_state_update(box_name, box_is_known=False, box_is_attached=False, timeout=4):
	start = rospy.get_time()
	seconds = rospy.get_time()
	while (seconds - start < timeout) and not rospy.is_shutdown():
		attached_objects = scene.get_attached_objects([box_name])
		print(attached_objects)
		print(str(type(attached_objects)))
		print(scene.get_known_object_names())
		print(str(type(scene.get_known_object_names())))
		is_attached = len(attached_objects.keys()) > 0
		is_known = box_name in scene.get_known_object_names()
		if (box_is_attached == is_attached) and (box_is_known == is_known):
			return True
		rospy.sleep(0.1)
		seconds = rospy.get_time()
	return False

def add_table(timeout=4):
	# type: (int) -> bool
	box_pose = geometry_msgs.msg.PoseStamped()
	box_pose.header.frame_id = robot.get_planning_frame()
	box_pose.pose.orientation.w = 1.0
	box_pose.pose.position.z = -0.07
	table_name = "table"
	scene.add_box(table_name, box_pose, size=(10, 10, 0.1))
	return wait_for_state_update(table_name, box_is_known=True, timeout=timeout)

def add_wall(timeout=4):
	# type: (int) -> bool
	box_pose = geometry_msgs.msg.PoseStamped()
	box_pose.header.frame_id = robot.get_planning_frame()
	box_pose.pose.orientation.w = 1.0
	box_pose.pose.position.y = -0.15
	wall_name = "wall"
	scene.add_box(wall_name, box_pose, size=(10, 0.02, 10))
	return wait_for_state_update(wall_name, box_is_known=True, timeout=timeout)


# TODO: Finish documenting

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_e_arm")
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

rospy.sleep(0.5)
add_table()
rospy.sleep(0.5)
add_wall()

group_variable_values = group.get_current_joint_values()

home_pose = [0, -1.5447, 1.5447, -1.5794, -1.5794, 0]
default_pose = [0, 0, 0, 0, 0, 0]

throw_back_pose = [0, 0, math.radians(-150), math.radians(-90), math.radians(-90), 0]
throw_forward_pose = [0, math.radians(-40), math.radians(-20), math.radians(-127), math.radians(-90), 0]

group.set_joint_value_target(home_pose)

# group_variable_values[1] = -1.5447
# group_variable_values[4] = 3.0
# group.set_joint_value_target(group_variable_values)

plan1 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory

def publish_trajectory(publisher, trajectory, robot):
	# type: (rospy.Publisher, moveit_msgs.msg.RobotTrajectory, moveit_commander.RobotCommander) -> None
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(trajectory)
	publisher.publish(display_trajectory)


traj = generate_smooth_trajectory(plan1, DDDU)
print("Total time: " + str(plan1.joint_trajectory.points[-1].time_from_start.to_sec()))
print("MoveIt Final Position:")
print(plan1.joint_trajectory.points[-1].positions)
print("Smoothed Final Position:")
print(traj.joint_trajectory.points[-1].positions)

# Send the new trajectory to the /move_group/display_planned_path topic to display in RViz/rqt
publish_trajectory(display_trajectory_publisher, traj, robot)
print(str(type(group.get_current_pose().pose)))
print(group.get_current_pose().pose)
# group.execute(traj, wait=True)
# publish_trajectory(display_trajectory_publisher, plan1, robot)
# group.execute(plan1, wait=True)

rospy.sleep(1)

moveit_commander.roscpp_shutdown()
