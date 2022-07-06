#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_e_arm")
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.22
pose_target.position.y = 0.44
pose_target.position.z = 0.54
group.set_pose_target(pose_target)

plan1 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory

rospy.sleep(1)

moveit_commander.roscpp_shutdown()
