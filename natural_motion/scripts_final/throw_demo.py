#! /usr/bin/env python

import sys
from natural_motion import trajectory_to_target, publish_trajectory, add_table, add_wall
import moveit_msgs.msg
import rospy
import moveit_commander
import math

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


# set a target pose, where None means don't move the current joint index
target = [math.radians(90), math.radians(-90), math.radians(90), None, None, None]
# target = [None, 0.0, 0.0, None, None, None]

# configure other parameters
focus_joint = 0
focus_vel_waypoint = 1.0
focus_vel_percentage = 0.5

alternate_profile = True

# get info from the robot
current = group.get_current_joint_values()
joint_names = group.get_active_joints()
frame_id = group.get_pose_reference_frame()

# generate the natural motion trajectory
traj = trajectory_to_target(target, focus_joint, focus_vel_waypoint, focus_vel_percentage, current, joint_names, frame_id, alternate_profile)
# print(traj)

# display generated trajectory in visualizers to make sure it is safe to run
publish_trajectory(display_trajectory_publisher, traj, robot)

print("To Target")
print("Press [Enter]")
raw_input("")

# execute the trajectory and safely stop
group.execute(traj, wait=True)
group.stop()

# gracefully shut down moveit commander
moveit_commander.roscpp_shutdown()
