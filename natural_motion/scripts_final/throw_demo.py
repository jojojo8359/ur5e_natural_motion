#! /usr/bin/env python

import sys
import jerk_profile
import natural_motion
import moveit_msgs.msg
import trajectory_msgs.msg
from genpy import Duration
import rospy
import moveit_commander
import math
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 16})

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
natural_motion.add_table(robot, scene)
rospy.sleep(0.5)
natural_motion.add_wall(robot, scene)

home = [0, 0, 0, 0, 0, 0]

# target = [0, -1.5, 0, 0, 0, 0]

target = [3.0, None, None, None, None, None]
# target = [math.radians(90), math.radians(-90), math.radians(90), None, None, None]
# target = [None, 0.0, 0.0, None, None, None]

focus_joint = 0
focus_vel_waypoint = 1.0
focus_vel_percentage = 0.5
# focus_vel_percentage = None

alternate_profile = True

current = group.get_current_joint_values()
# current = [0, 0, 0, 0, 0, 0]

joint_names = group.get_active_joints()
frame_id = group.get_pose_reference_frame()

def trajectory_to_target(target, focus_joint_index, focus_velocity_waypoint, focus_waypoint_percentage, current_joint_values, joint_names, frame_id, use_alternate_profile=False):
	(focus_time, focus_jerk, focus_pos, focus_vel, focus_accel) = natural_motion.create_trajectory(current_joint_values[focus_joint_index], target[focus_joint_index], focus_velocity_waypoint, focus_waypoint_percentage, duration=None, use_alternate_profile=use_alternate_profile)
	natural_motion.plot_trajectory(focus_time, focus_jerk, focus_pos, focus_vel, focus_accel)
	focus_duration = focus_time[-1] - focus_time[0]

	result = moveit_msgs.msg.RobotTrajectory()

	result.joint_trajectory.header.frame_id = frame_id
	result.joint_trajectory.joint_names = joint_names

	for i in range(len(focus_time)):
		result.joint_trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint(time_from_start=Duration.from_sec(focus_time[i])))

	for joint_index, joint_target in enumerate(target):
		if joint_target is None:
			# keep constant
			for point in result.joint_trajectory.points:
				point.positions.append(current_joint_values[joint_index])
				point.velocities.append(0.0)
				point.accelerations.append(0.0)
		else:
			# create custom trajectory to position
			if joint_index == focus_joint_index:
				# use focus trajectory
				for i in range(len(result.joint_trajectory.points)):
					result.joint_trajectory.points[i].positions.append(focus_pos[i])
					result.joint_trajectory.points[i].velocities.append(focus_vel[i])
					result.joint_trajectory.points[i].accelerations.append(focus_accel[i])
			else:
				# generate a trajectory
				(t, j, pos, vel, accel) = natural_motion.create_trajectory(current_joint_values[joint_index], target[joint_index], waypoint_percent=focus_waypoint_percentage, duration=focus_duration, use_alternate_profile=use_alternate_profile)
				for i in range(len(focus_time)):
					result.joint_trajectory.points[i].positions.append(pos[i])
					result.joint_trajectory.points[i].velocities.append(vel[i])
					result.joint_trajectory.points[i].accelerations.append(accel[i])
				# num_to_skip = (len(focus_time) - 1) / (len(t) - 1)
				# for i in range(0, len(focus_time)):
				# 	if i % num_to_skip == 0:
				# 		result.joint_trajectory.points[i].positions.append(pos[i/num_to_skip])
				# 		result.joint_trajectory.points[i].velocities.append(vel[i/num_to_skip])
				# 		result.joint_trajectory.points[i].accelerations.append(accel[i/num_to_skip])
				# 	else:
				# 		prev = (i + num_to_skip - (i % num_to_skip)) / num_to_skip
				# 		nxt = (i - (i % num_to_skip)) / num_to_skip
				# 		result.joint_trajectory.points[i].positions.append(((pos[nxt] - pos[prev]) / 2.0) + pos[prev])
				# 		result.joint_trajectory.points[i].velocities.append(((vel[nxt] - vel[prev]) / 2.0) + vel[prev])
				# 		result.joint_trajectory.points[i].accelerations.append(((accel[nxt] - accel[prev]) / 2.0) + accel[prev])
	
	return result

traj = trajectory_to_target(target, focus_joint, focus_vel_waypoint, focus_vel_percentage, current, joint_names, frame_id, alternate_profile)
# print(traj)

natural_motion.publish_trajectory(display_trajectory_publisher, traj, robot)

plt.show()

print("To Target")
print("Press [Enter]")
raw_input("")

group.execute(traj, wait=True)
group.stop()

# gracefully shut down moveit commander
moveit_commander.roscpp_shutdown()
