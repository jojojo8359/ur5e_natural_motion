#! /usr/bin/env python

try:
	from typing import List, Union, Tuple, Optional, Type
except ImportError:
	pass  # Trick VSC into "importing" typing for type hints, but don't actually import it at runtime (still using Python 2.7.17)

import matplotlib.pyplot as plt
from smoothing_core_v3 import *

def create_time_points(time_steps):
	time_points = [0.0]
	for time_step in time_steps:
		if time_step == 0.0:
			continue
		time_points.append(time_points[-1] + time_step)
	return time_points

def integrate(initial_pos, initial_vel, initial_accel, jerk_profile, time_steps):
	p = [initial_pos]
	v = [initial_vel]
	a = [initial_accel]

	for i in range(1, 8):
		# print("i=" + str(i))
		# print(" t: " + str(time_steps))
		# print(" p: " + str(p))
		# print(" v: " + str(v))
		# print(" a: " + str(a))
		# print(" j: " + str(jerk_profile))
		a.append(gen_next_a(a[i-1], jerk_profile[i-1], time_steps[i-1]))
		v.append(gen_next_v(a[i-1], v[i-1], jerk_profile[i-1], time_steps[i-1]))
		p.append(gen_next_p(a[i-1], v[i-1], p[i-1], jerk_profile[i-1], time_steps[i-1]))
	
	return (p, v, a)

def plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations):
	fig, (ax1) = plt.subplots(1, 1)
	ax1.plot(time_points, positions, c='blue', marker='o')
	ax1.plot(time_points, velocities, c='orange', marker='o')
	ax1.plot(time_points, accelerations, c='red', marker='o')
	ax1.hlines(jerk_profile, [time_points[x] for x in range(len(time_points)-1)], [time_points[x] for x in range(1, len(time_points))], colors='green')

	ymin = [0]
	for i in range(1, len(time_points) - 1):
		ymin.append(min(jerk_profile[i-1], jerk_profile[i]))
	ymin.append(0)

	ymax = [jerk_profile[0]]
	for i in range(1, len(time_points) - 1):
		ymax.append(max(jerk_profile[i-1], jerk_profile[i]))
	ymax.append(jerk_profile[-1])

	ax1.vlines([time_points[x] for x in range(len(time_points))], ymin, ymax, colors='green')

def check_vel_limit(vel, v_min, v_max):
	# for v in vel:
	# 	if v >= v_max or v <= v_min:
	# 		pass
	return vel[3] >= v_max or vel[3] <= v_min

def check_acc0_limit(acc, a_min, a_max):
	return acc[1] >= a_max or acc[1] <= a_min

def check_acc1_limit(acc, a_min, a_max):
	return acc[5] >= a_max or acc[5] <= a_min




# p = [0.0, None, 1.0]
# v = [0.0, 1.0, 0.0]
# t = [0.0, 0.7 * 2.0, 2.0]
# percentage = 0.7

# deltas
# p: 0->4  v: 4    t: 2.0
# p: 0->2  v: 2    t: 2.0
# p: 0->2  v: 1    t: 4.0   v/2 -> t*2
# p: 0->3  v: 1    t: 6.0   p*1.5 -> t*1.5
# p: 0->3  v: 1.5  t: 4.0   v/0.666 -> t * 0.666

# p: 0->1  v: 1    t: 2.0


# pos = [0.0, 1.0]
# v_waypoints = [1.0]
# waypoint_percentages = [0.3]

# 1 waypoint + 1 percent = UDUD/UOOD with waypoints and percentage
# 1 waypoint + 0 percent = UDDU with waypoint velocity in middle
# 0 waypoint + 0 percent = UDDU w/duration override

def something(pos, v_wp, percents, duration_override=None):
	# type: (List[float], List[Optional[float]], List[Optional[float]], Optional[float]) -> None

	# It's assumed that for any trajectory, initial/final velocity/acceleration will remain 0

	UDDU = lambda min, max: [max, 0, min, 0, min, 0, max]
	UDUD = lambda min, max: [max, 0, min, 0, max, 0, min]
	UOOD = lambda min, max: [max, 0, 0, 0, 0, 0, min]

	if len(pos) != 2:
		raise ValueError("position list must have exactly two positions")
	if len(v_wp) < 1 and not duration_override:
		raise ValueError("there must be at least one velocity waypoint without a duration override")
	if len(v_wp) >= 1 and len(percents) >= 1 and duration_override:
		print("*** Duration override is currently set to %d but offset waypoints were specified, ignoring... ***" % duration_override)

	# position delta
	dp = pos[1] - pos[0]

	# currently for one waypoint
	if len(v_wp) == 0 and len(percents) == 0 and duration_override:
		# use UDDU with duration override
		total_duration = duration_override
		time_stamps = [0.0, total_duration]

		time_step = (time_stamps[1] - time_stamps[0]) / 7.0
		time_steps = [time_step] * 7

		time_scale_factor = (time_step / 0.2) ** 3

		jerk_scale_factor = (pos[-1] - pos[0]) / (time_scale_factor * 0.64)  # UDDU - pos

		jerk_profile = UDDU(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

		(positions, velocities, accelerations) = integrate(pos[0], 0.0, 0.0, jerk_profile, time_steps)

		time_points = create_time_points(time_steps)
		
		print("T: " + str(time_points))
		print("Pos: " + str(positions))
		print("Vel: " + str(velocities))
		print("Accel: " + str(accelerations))

		plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations)


	elif len(v_wp) == 1 and len(percents) == 0:
		# use UDDU with velocity waypoint

		# original parameters for determining duration (UDDU)
		orig_dp = 1.0
		orig_vel = 1.0
		orig_time = 1.75

		total_duration = (dp / orig_dp) * (orig_vel / v_wp[0]) * orig_time
		time_stamps = [0.0, total_duration]

		time_step = (time_stamps[1] - time_stamps[0]) / 7.0
		time_steps = [time_step] * 7

		# TODO: isolate factor generation to its own function, maybe associate with revamped jerk profile class?
		time_scale_factor = (time_step / 0.2) ** 2

		jerk_scale_factor = (v_wp[0] - 0.0) / (time_scale_factor * 0.8)  # UDDU - vel

		jerk_profile = UDDU(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

		(positions, velocities, accelerations) = integrate(pos[0], 0.0, 0.0, jerk_profile, time_steps)

		time_points = create_time_points(time_steps)
		
		print("T: " + str(time_points))
		print("Pos: " + str(positions))
		print("Vel: " + str(velocities))
		print("Accel: " + str(accelerations))

		plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations)

	elif len(v_wp) == 1 and len(percents) == 1:
		# UDUD/UOOD

		# jerk_profile_type = UDUD
		jerk_profile_type = UOOD

		# original parameters for determining duration (UDUD)
		orig_dp = 1.0
		orig_vel = 1.0
		orig_time = 2.0

		total_duration = (dp / orig_dp) * (orig_vel / v_wp[0]) * orig_time
		time_stamps = [0.0, percents[0] * total_duration, total_duration]

		# first block
		time_step_1 = (time_stamps[1] - time_stamps[0]) / 7.0
		time_steps_1 = [time_step_1] * 7

		time_scale_factor = (time_step_1 / 0.2) ** 2

		if jerk_profile_type == UDUD:
			jerk_scale_factor = (v_wp[0] - 0.0) / (time_scale_factor * 1.6)  # UDUD - vel
		elif jerk_profile_type == UOOD:
			jerk_scale_factor = (v_wp[0] - 0.0) / (time_scale_factor * 2.4)  # UOOD - vel
		else:
			raise ValueError("jerk profile type does not exist!")

		jerk_profile = jerk_profile_type(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

		(p1, v1, a1) = integrate(pos[0], 0.0, 0.0, jerk_profile, time_steps_1)

		# second block
		time_step_2 = (time_stamps[2] - time_stamps[1]) / 7.0
		time_steps_2 = [time_step_2] * 7

		time_scale_factor_2 = (time_step_2 / 0.2) ** 2

		if jerk_profile_type == UDUD:
			jerk_scale_factor = (0.0 - v_wp[0]) / (time_scale_factor_2 * 1.6)  # UDUD - vel
		elif jerk_profile_type == UOOD:
			jerk_scale_factor = (0.0 - v_wp[0]) / (time_scale_factor_2 * 2.4)  # UOOD - vel
		else:
			raise ValueError("jerk profile type does not exist!")

		jerk_profile_2 = jerk_profile_type(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

		(p2, v2, a2) = integrate(p1[-1], v1[-1], 0.0, jerk_profile_2, time_steps_2)


		# plot both blocks

		time_points = create_time_points(time_steps_1 + time_steps_2)

		print("T: " + str(time_points))
		print("Pos: " + str(p1 + p2[1:]))
		print("Vel: " + str(v1 + v2[1:]))
		print("Accel: " + str(a1 + a2[1:]))

		plot_trajectory(time_points, jerk_profile + jerk_profile_2, p1 + p2[1:], v1 + v2[1:], a1 + a2[1:])
	
	else:
		raise ValueError("case couldn't be matched")

	
	
	# if len(pos) == 2 and len(vel) == 2:
	# 	# use UDDU
	# 	UDDU = lambda min, max: [max, 0, min, 0, min, 0, max]

	# 	time_step = (time[1] - time[0]) / 7.0
	# 	time_steps = [time_step] * 7

	# 	time_scale_factor = (time_step / 0.2) ** 3

	# 	jerk_scale_factor = (pos[-1] - pos[0]) / (time_scale_factor * 0.64)  # UDDU - pos

	# 	jerk_profile = UDDU(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

	# 	(positions, velocities, accelerations) = integrate(pos[0], vel[0], 0.0, jerk_profile, time_steps)

	# 	time_points = create_time_points(time_steps)

	# 	print("T: " + str(time_points))
	# 	print("Pos: " + str(positions))
	# 	print("Vel: " + str(velocities))
	# 	print("Accel: " + str(accelerations))

	# 	plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations)


	# elif len(pos) == 3 and len(vel) == 3:
	# 	# use UDUD/UOOD
	# 	UDUD = lambda min, max: [max, 0, min, 0, max, 0, min]
	# 	UOOD = lambda min, max: [max, 0, 0, 0, 0, 0, min]

	# 	jerk_profile_type = UOOD

	# 	time_step = (time[1] - time[0]) / 7.0
	# 	time_steps = [time_step] * 7

	# 	time_scale_factor = (time_step / 0.2) ** 2

	# 	# jerk_scale_factor = (vel[1] - vel[0]) / (time_scale_factor * 1.6)  # UDUD - vel
	# 	jerk_scale_factor = (vel[1] - vel[0]) / (time_scale_factor * 2.4)  # UOOD - vel
	# 	# jerk_scale_factor = (pos[-1] - pos[0]) / (time_scale_factor * 1.68)  # UOOD - pos

	# 	jerk_profile = jerk_profile_type(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

	# 	(positions, velocities, accelerations) = integrate(pos[0], vel[0], 0.0, jerk_profile, time_steps)

	# 	time_step_2 = (time[2] - time[1]) / 7.0
	# 	time_steps_2 = [time_step_2] * 7

	# 	time_scale_factor_2 = (time_step_2 / 0.2) ** 2

	# 	# jerk_scale_factor_2 = (pos[-1] - positions[-1]) / (time_scale_factor_2 * 1.12)  # UDUD - pos
	# 	# jerk_scale_factor_2 = (vel[-1] - vel[1]) / (time_scale_factor_2 * 1.6)  # UDUD - vel
	# 	jerk_scale_factor_2 = (vel[-1] - vel[1]) / (time_scale_factor_2 * 2.4)  # UOOD - vel

	# 	jerk_profile_2 = jerk_profile_type(-10.0 * jerk_scale_factor_2, 10.0 * jerk_scale_factor_2)
	# 	# jerk_profile_2 = [-x for x in jerk_profile_2]

	# 	(positions2, velocities2, accelerations2) = integrate(positions[-1], velocities[-1], 0.0, jerk_profile_2, time_steps_2)

	# 	time_points = create_time_points(time_steps + time_steps_2)

	# 	print("T: " + str(time_points))
	# 	print("Pos: " + str(positions + positions2[1:]))
	# 	print("Vel: " + str(velocities + velocities2[1:]))
	# 	print("Accel: " + str(accelerations + accelerations2[1:]))

	# 	plot_trajectory(time_points, jerk_profile + jerk_profile_2, positions + positions2[1:], velocities + velocities2[1:], accelerations + accelerations2[1:])


	# elif len(pos) == 5 and len(vel) == 5:
	# 	pass  # use UOD/UDU

pos = [0.0, 1.0]
v_waypoints = [3.0]
waypoint_percentages = [0.1]

something(pos, v_waypoints, waypoint_percentages, duration_override=None)



initial_pos = 0.0
final_pos = 1.0

initial_vel = 0.0
final_vel = 1.0

initial_accel = 0.0
final_accel = 0.0

accel_min = -1.0
accel_max = 1.0

vel_min = -2.0
vel_max = 2.0

total_duration = 1.75
# time_step = 0.2
time_step = total_duration / 7.0

# target_max_vel = 2.5
# target_max_accel = 3.0

# time_step = (target_max_vel - initial_vel) / (initial_accel + (2 * target_max_accel))

# jerk_max = target_max_accel / time_step


# This correction process uses the fact that a timestep of 0.2 seconds and jerk min/max of +/- 10.0 gives a final delta position of 1.12
# If this has to be tuned, perform a manual trial and substitute those numbers

# # Correct for time
# # time_scale_factor = (time_step / 0.2) ** 3  # for pos
# time_scale_factor = (time_step / 0.2) ** 2  # for vel

# # Correct for position
# # jerk_scale_factor = (final_pos - initial_pos) / (time_scale_factor * 0.64)  # UDDU
# # jerk_scale_factor = (final_pos - initial_pos) / (time_scale_factor * 1.12)  # UDUD
# # jerk_scale_factor = (final_pos - initial_pos) / (time_scale_factor * 1.68)  # UOOD

# # Correct for velocity
# # jerk_scale_factor = (final_vel - initial_vel) / (time_scale_factor * 1.6)  # UDUD
# # jerk_scale_factor = (final_vel - initial_vel) / (time_scale_factor * 2.4)  # UOOD
# jerk_scale_factor = (final_vel - initial_vel) / (time_scale_factor * 0.8)  # UDDU - corresponds to the max vel value, not the final one (Vi=Vf)


# UDDU = lambda min, max: [max, 0, min, 0, min, 0, max]
# UDUD = lambda min, max: [max, 0, min, 0, max, 0, min]
# UOOD = lambda min, max: [max, 0, 0, 0, 0, 0, min]

# # jerk_profile = UDDU(jerk_min, jerk_max)
# # jerk_profile = UDUD(jerk_min, jerk_max)
# # jerk_profile = UDUD(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)
# # jerk_profile = UDUD(-10.0, 10.0)
# # jerk_profile = UOOD(-10.0, 10.0)
# # jerk_profile = UDDU(-10.0, 10.0)
# # jerk_profile = UOOD(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)
# jerk_profile = UDDU(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

# # jerk_profile = UDDU(-jerk_max, jerk_max)

# time_steps = [time_step] * 7

# (positions, velocities, accelerations) = integrate(initial_pos, initial_vel, initial_accel, jerk_profile, time_steps)

# time_points = create_time_points(time_steps)

# print("T: " + str(time_points))

# print("Pos: " + str(positions))
# print("Vel: " + str(velocities))
# print("Accel: " + str(accelerations))


# plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations)




# initial_pos = positions[-1]
# initial_vel = velocities[-1]
# initial_accel = accelerations[-1]

# jerk_profile = [-x for x in jerk_profile]

# (positions, velocities, accelerations) = integrate(initial_pos, initial_vel, initial_accel, jerk_profile, time_steps)

# time_points = create_time_points(time_steps)

# print("T: " + str(time_points))

# print("Pos: " + str(positions))
# print("Vel: " + str(velocities))
# print("Accel: " + str(accelerations))


# plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations)




# time_step = 0.2

# jerk_max = 4.0
# jerk_min = -4.0
# jerk_profile = UDUD(jerk_min, jerk_max)

# time_steps = [time_step] * 7

# (positions, velocities, accelerations) = integrate(initial_pos, initial_vel, initial_accel, jerk_profile, time_steps)

# time_points = create_time_points(time_steps)

# print("T: " + str(time_points))

# print("Pos: " + str(positions))
# print("Vel: " + str(velocities))
# print("Accel: " + str(accelerations))


# plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations)





# # ACC0 VEL ACC1
# traj_limits = [check_acc0_limit(accelerations, accel_min, accel_max), check_vel_limit(velocities, vel_min, vel_max), check_acc1_limit(accelerations, accel_min, accel_max)]

# # if traj_limits == [False, False, False]:
# # time_steps[1] = 0.0
# # time_steps[3] = 0.0
# # time_steps[4] = 0.0
# # time_steps[5] = 0.0
# time_steps[1] = 0.0
# time_steps[3] = 0.0
# time_steps[5] = 0.0
# time_steps[6] = 0.0

# time_points = create_time_points(time_steps)

# (positions, velocities, accelerations) = integrate(initial_pos, initial_vel, initial_accel, jerk_profile, time_steps)
# for l in [positions, velocities, accelerations, jerk_profile]:
# 	# l.pop(5)
# 	# l.pop(4)
# 	# l.pop(3)
# 	# l.pop(1)
# 	l.pop(6)
# 	l.pop(5)
# 	l.pop(3)
# 	l.pop(1)
	

# print("After limits applied:")

# # print(str(traj_limits))

# print("T: " + str(time_points))

# print("Pos: " + str(positions))
# print("Vel: " + str(velocities))
# print("Accel: " + str(accelerations))

# plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations)


plt.show()
