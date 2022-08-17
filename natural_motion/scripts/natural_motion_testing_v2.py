#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from smoothing_core_v3 import *
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
home_pose = [0, -1.5447, 1.5447, -1.5794, -1.5794, 0]
default_pose = [0, 0, 0, 0, 0, 0]

throw_back_pose = [0, 0, math.radians(-150), math.radians(-90), math.radians(-90), 0]
throw_forward_pose = [0, math.radians(-40), math.radians(-20), math.radians(-127), math.radians(-90), 0]

# set the arm's goal to a pose and create a trajectory
group.set_joint_value_target(home_pose)
plan1 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory

UDDU = lambda min, max: [max, 0, min, 0, min, 0, max]

# smooth trajectory
start = timeit.default_timer()
duration = plan1.joint_trajectory.points[-1].time_from_start.to_sec() - plan1.joint_trajectory.points[0].time_from_start.to_sec()
traj = generate_smooth_trajectory_with_limits(plan1, UDDU, duration, jerk_min=-2.0)
end = timeit.default_timer()
print("Smoothing time: " + str(end-start))
print("Total time: " + str(plan1.joint_trajectory.points[-1].time_from_start.to_sec()))
print("MoveIt Final Position:")
print(plan1.joint_trajectory.points[-1].positions)
print("Smoothed Final Position:")
print(traj.joint_trajectory.points[-1].positions)

# send the new trajectory to the /move_group/display_planned_path topic to display in RViz/rqt
publish_trajectory(display_trajectory_publisher, traj, robot)
# group.execute(traj, wait=True)

rospy.sleep(1)

# # send another pose goal
# group.set_joint_value_target(home_pose)
# plan2 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory

# # smooth trajectory
# start = timeit.default_timer()
# traj2 = generate_smooth_trajectory(plan2, jerk_profiles.UDDU)  # different jerk profiles can be used from jerk_profiles.py
# end = timeit.default_timer()
# print("Smoothing time: " + str(end-start))
# print("Total time: " + str(plan2.joint_trajectory.points[-1].time_from_start.to_sec()))
# print("MoveIt Final Position:")
# print(plan2.joint_trajectory.points[-1].positions)
# print("Smoothed Final Position:")
# print(traj2.joint_trajectory.points[-1].positions)

# # display trajectory
# publish_trajectory(display_trajectory_publisher, traj2, robot)
# # group.execute(traj2, wait=True)

# rospy.sleep(1)

# gracefully shut down moveit commander
moveit_commander.roscpp_shutdown()
