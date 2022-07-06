#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_e_arm")
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

group_variable_values = group.get_current_joint_values()

group_variable_values[1] = -1.5
group.set_joint_value_target(group_variable_values)

plan1 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory

rospy.sleep(1)

moveit_commander.roscpp_shutdown()
