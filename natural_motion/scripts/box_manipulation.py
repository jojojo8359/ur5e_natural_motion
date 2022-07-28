#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
# from jerk_profiles import *
# from smoothing_core import *

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_e_arm")
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# planning_frame = group.get_planning_frame()
# print("Planning frame: " + str(planning_frame))

# eef_link = group.get_end_effector_link()
# print("End effector link: " + str(eef_link))

# group_names = robot.get_group_names()
# print("Available Planning Groups: " + str(group_names))

box_name = "box"

def wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4):
	start = rospy.get_time()
	seconds = rospy.get_time()
	while (seconds - start < timeout) and not rospy.is_shutdown():
		attached_objects = scene.get_attached_objects([box_name])
		is_attached = len(attached_objects.keys()) > 0

		is_known = box_name in scene.get_known_object_names()

		if (box_is_attached == is_attached) and (box_is_known == is_known):
			return True
		
		rospy.sleep(0.1)
		seconds = rospy.get_time()

	return False

rospy.sleep(1)

box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "flange"
# box_pose.pose.position.x = 0.81
# box_pose.pose.position.y = 0.27
# box_pose.pose.position.z = 0.05
box_pose.pose.position.x = 0.04
scene.add_box(box_name, box_pose, size=(0.07, 0.07, 0.07))

print(wait_for_state_update(box_is_known=True))

grasping_group = "ur5_e_arm"
touch_links = robot.get_link_names(group=grasping_group)
scene.attach_box(group.get_end_effector_link(), box_name, touch_links=touch_links)

print(wait_for_state_update(box_is_attached=True, box_is_known=False))


rospy.sleep(1)

moveit_commander.roscpp_shutdown()

# Look into using gazebo for box simulation/physics
# Find a way to "release" a gripper during trajectory execution
