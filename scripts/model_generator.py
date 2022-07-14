#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score
from jerk_profiles import *
from smoothing_core import *

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("ur5_e_arm")
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

original_joint_positions = group.get_current_joint_values()

# Can be a lambda function like the following:
# UDDU = lambda min, max: [max, 0, min, 0, min, 0, max]
# or a static method of a jerk profile class (JerkProfile.generate_steps)
# profile_type = DDUU.generate_steps  # type: function
profile_type = lambda min, max: [min, min, min, min, min, 0, max]  # type: function

# The index of arm joint to use in tests
joint_index = 4

# Parameters for testing various positions
position_min = 0.25
position_max = 2.5
position_step = 0.25

# Parameters for testing various jerk limits
jerk_min = 0.0
jerk_max = 50.0
jerk_step = 0.25

positions = np.arange(position_min, position_max, position_step)
times = []
slopes = []

for pos in positions:
	original_joint_positions[joint_index] = pos
	group.set_joint_value_target(original_joint_positions)

	plan1 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory
	initial_position = plan1.joint_trajectory.points[0].positions[joint_index]
	duration = plan1.joint_trajectory.points[-1].time_from_start.to_sec()
	print("Position: " + str(pos) + " Time: " + str(duration) + " Start Position: " + str(initial_position))
	times.append(duration)

	jerks = np.arange(jerk_min, jerk_max, jerk_step)
	positions = []
	for i in jerks:
		traj = generate_smooth_trajectory_with_limits(plan1, profile_type, -i)
		final_position = traj.joint_trajectory.points[-1].positions[joint_index]
		positions.append(final_position - initial_position)

	jerk_model = np.polyfit(jerks, positions, 1)
	jerk_predict = np.poly1d(jerk_model)
	jerk_r2 = r2_score(positions, jerk_predict(jerks))
	print(" model: " + str(jerk_model))
	print(" r2: " + str(jerk_r2))

	slopes.append(jerk_model[0])

	# Shows the plot for each position vs jerk graph generated
	# x_lin_reg = range(0, 51)
	# y_lin_reg = jerk_predict(x_lin_reg)
	# plt.scatter(jerks, positions)
	# plt.plot(x_lin_reg, y_lin_reg, c='r')
	# plt.show()

print("---")
slope_model = np.polyfit(times, slopes, 3)  # type: np.ndarray
slope_predict = np.poly1d(slope_model)
slope_r2 = r2_score(slopes, slope_predict(times))
print("Final model (copy this to create new jerk profile):")
print(slope_model.tolist())
print("r2: " + str(slope_r2))

x_lin_reg = np.arange(0, max(times) + 0.25, 0.25)
y_lin_reg = slope_predict(x_lin_reg)

plt.scatter(times, slopes)
plt.plot(x_lin_reg, y_lin_reg, c='r')

plt.show()

rospy.sleep(1)

moveit_commander.roscpp_shutdown()
