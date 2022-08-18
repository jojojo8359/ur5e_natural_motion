#! /usr/bin/env python

try:
	from typing import List, Union, Tuple, Optional, Type
except ImportError:
	pass  # Trick VSC into "importing" typing for type hints, but don't actually import it at runtime (still using Python 2.7.17)

import matplotlib.pyplot as plt
from smoothing_core_v3 import gen_next_a, gen_next_v, gen_next_p
from jerk_profile_v2 import *

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

	for i in range(1, len(jerk_profile) + 1):
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

def get_durations(pos_ex, v_wp_ex, percents_ex, duration_override=None):
	durations = []
	for joint in range(len(pos_ex)):
		dp = pos_ex[joint][-1] - pos_ex[joint][0]
		if len(v_wp_ex[joint]) == 0 and len(percents_ex[joint]) == 0 and duration_override:
			durations.append(duration_override)
		elif len(v_wp_ex[joint]) == 1 and len(percents_ex[joint]) == 0:
			profile_type = UDDU
			durations.append((dp / profile_type.orig_dp) * (profile_type.orig_vel / v_wp_ex[joint][0]) * profile_type.orig_duration)
		elif len(v_wp_ex[joint]) == 1 and len(percents_ex[joint]) == 1:
			# profile_type = UDUD
			profile_type = UOOD
			durations.append((dp / profile_type.orig_dp) * (profile_type.orig_vel / v_wp_ex[joint][0]) * profile_type.orig_duration)
	return durations

def compute_trajectory(pos, v_wp, percents, duration_override=None):
	# type: (List[float], List[Optional[float]], List[Optional[float]], Optional[float]) -> Tuple[List[float], List[float], List[float], List[float], List[float]]

	# It's assumed that for any trajectory, initial/final velocity/acceleration will remain 0

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
		profile_type = UDDU

		total_duration = duration_override
		time_stamps = [0.0, total_duration]

		time_step = (time_stamps[1] - time_stamps[0]) / float(profile_type.size)
		time_steps = [time_step] * profile_type.size

		jerk_scale_factor = profile_type.jerk_scale(pos[0], pos[-1], time_step, MODE_POS)

		jerk_profile = profile_type(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

		(positions, velocities, accelerations) = integrate(pos[0], 0.0, 0.0, jerk_profile.jerk_values, time_steps)

		time_points = create_time_points(time_steps)
		
		print("T: " + str(time_points))
		print("Pos: " + str(positions))
		print("Vel: " + str(velocities))
		print("Accel: " + str(accelerations))

		# plot_trajectory(time_points, jerk_profile.jerk_values, positions, velocities, accelerations)
		return (time_points, jerk_profile.jerk_values, positions, velocities, accelerations)

	elif len(v_wp) == 1 and len(percents) == 0:
		# use UDDU with velocity waypoint

		profile_type = UDDU

		total_duration = (dp / profile_type.orig_dp) * (profile_type.orig_vel / v_wp[0]) * profile_type.orig_duration
		time_stamps = [0.0, total_duration]

		time_step = (time_stamps[1] - time_stamps[0]) / float(profile_type.size)
		time_steps = [time_step] * profile_type.size

		jerk_scale_factor = profile_type.jerk_scale(0.0, v_wp[0], time_step, MODE_VEL)

		jerk_profile = UDDU(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

		(positions, velocities, accelerations) = integrate(pos[0], 0.0, 0.0, jerk_profile.jerk_values, time_steps)

		time_points = create_time_points(time_steps)
		
		print("T: " + str(time_points))
		print("Pos: " + str(positions))
		print("Vel: " + str(velocities))
		print("Accel: " + str(accelerations))

		# plot_trajectory(time_points, jerk_profile.jerk_values, positions, velocities, accelerations)
		return (time_points, jerk_profile.jerk_values, positions, velocities, accelerations)

	elif len(v_wp) == 1 and len(percents) == 1:
		# UDUD/UOOD

		# profile_type = UDUD
		profile_type = UOOD

		total_duration = (dp / profile_type.orig_dp) * (profile_type.orig_vel / v_wp[0]) * profile_type.orig_duration
		time_stamps = [0.0, percents[0] * total_duration, total_duration]

		# first block
		time_step_1 = (time_stamps[1] - time_stamps[0]) / float(profile_type.size)
		time_steps_1 = [time_step_1] * profile_type.size

		jerk_scale_factor = profile_type.jerk_scale(0.0, v_wp[0], time_step_1, MODE_VEL)

		jerk_profile = profile_type(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

		(p1, v1, a1) = integrate(pos[0], 0.0, 0.0, jerk_profile.jerk_values, time_steps_1)

		# second block
		time_step_2 = (time_stamps[2] - time_stamps[1]) / float(profile_type.size)
		time_steps_2 = [time_step_2] * profile_type.size

		jerk_scale_factor = profile_type.jerk_scale(v_wp[0], 0.0, time_step_2, MODE_VEL)

		jerk_profile_2 = profile_type(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

		(p2, v2, a2) = integrate(p1[-1], v1[-1], 0.0, jerk_profile_2.jerk_values, time_steps_2)


		# plot both blocks

		time_points = create_time_points(time_steps_1 + time_steps_2)

		print("T: " + str(time_points))
		print("Pos: " + str(p1 + p2[1:]))
		print("Vel: " + str(v1 + v2[1:]))
		print("Accel: " + str(a1 + a2[1:]))

		# plot_trajectory(time_points, jerk_profile.jerk_values + jerk_profile_2.jerk_values, p1 + p2[1:], v1 + v2[1:], a1 + a2[1:])
		return (time_points, jerk_profile.jerk_values + jerk_profile_2.jerk_values, p1 + p2[1:], v1 + v2[1:], a1 + a2[1:])
	
	else:
		# UD

		# profile_type = UD
		raise ValueError("case not found")



	
	
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

# pos = [0.0, 1.0]
# v_waypoints = [2.0]
# waypoint_percentages = [0.7]
pos = [[0.0, 1.0], [-1.5, 0.5]]
v_waypoints = [[2.0], [1.0]]
waypoint_percentages = [[0.7], []]

# (time_points, jerk, positions, velocities, accelerations) = compute_trajectory(pos, v_waypoints, waypoint_percentages, duration_override=None)
# print(get_durations([pos], [v_waypoints], [waypoint_percentages], duration_override=None))
print(get_durations(pos, v_waypoints, waypoint_percentages, duration_override=None))

# dp = pos[1] - pos[0]


# # UDUD/UOOD

# # profile_type = UDUD
# # profile_type = UOOD
# profile_type = UD

# total_duration = (dp / profile_type.orig_dp) * profile_type.orig_duration
# vel_sum = 0.0
# print("duration: " + str(total_duration))
# for wp in v_waypoints:
# 	total_duration *= (profile_type.orig_vel / wp)
# 	# vel_sum += (profile_type.orig_vel / wp)
# 	print("duration: " + str(total_duration))
# # total_duration *= vel_sum

# total_duration = 0.57121

# print("duration: " + str(total_duration))
# time_stamps = [0.0]
# for percent in waypoint_percentages:
# 	time_stamps.append(percent * total_duration)
# # time_stamps.append(waypoint_percentages[0] * total_duration)
# time_stamps.append(total_duration)
# print("time stamps: " + str(time_stamps))

# time_step = []
# time_steps = []

# jerk = []  # type: List[JerkProfile]
# p = []
# v = []
# a = []

# for i in range(len(v_waypoints) + 1):
# 	print("iteration " + str(i))
# 	time_step.append((time_stamps[i+1] - time_stamps[i]) / float(profile_type.size))
# 	print(" time stamp = " + str(time_step[i]))
# 	time_steps.append([time_step[i]] * profile_type.size)

# 	if i == 0:
# 		jerk_scale_factor = profile_type.jerk_scale(0.0, v_waypoints[i], time_step[i], MODE_VEL)
# 	elif i == len(v_waypoints):
# 		jerk_scale_factor = profile_type.jerk_scale(v_waypoints[i-1], 0.0, time_step[i], MODE_VEL)
# 	else:
# 		jerk_scale_factor = profile_type.jerk_scale(v_waypoints[i-1], v_waypoints[i], time_step[i], MODE_VEL)
	
# 	jerk.append(profile_type(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor))
# 	print(" j: " + str(jerk[i]))

# 	if i == 0:
# 		(p_out, v_out, a_out) = integrate(pos[0], 0.0, 0.0, jerk[i].jerk_values, time_steps[i])
# 	else:
# 		(p_out, v_out, a_out) = integrate(p[i-1][-1], v[i-1][-1], 0.0, jerk[i].jerk_values, time_steps[i])
	
# 	p.append(p_out)
# 	v.append(v_out)
# 	a.append(a_out)

# 	print(" p: " + str(p[i]))
# 	print(" v: " + str(v[i]))
# 	print(" a: " + str(a[i]))

# time_points = create_time_points([time_step for time_step_list in time_steps for time_step in time_step_list])

# jerk_flat = [jerk_value for jerk_profile in jerk for jerk_value in jerk_profile.jerk_values]

# p_flat = p[0]
# for i in range(1, len(p)):
# 	for position in p[i][1:]:
# 		p_flat.append(position)

# v_flat = v[0]
# for i in range(1, len(v)):
# 	for velocity in v[i][1:]:
# 		v_flat.append(velocity)

# a_flat = a[0]
# for i in range(1, len(a)):
# 	for accel in a[i][1:]:
# 		a_flat.append(accel)

# print("T: " + str(time_points))
# print("Pos: " + str(p_flat))
# print("Vel: " + str(v_flat))
# print("Accel: " + str(a_flat))

# plot_trajectory(time_points, jerk_flat, p_flat, v_flat, a_flat)



# # first block
# time_step_1 = (time_stamps[1] - time_stamps[0]) / float(profile_type.size)
# time_steps_1 = [time_step_1] * profile_type.size

# jerk_scale_factor = profile_type.jerk_scale(0.0, v_waypoints[0], time_step_1, MODE_VEL)

# jerk_profile = profile_type(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

# (p1, v1, a1) = integrate(pos[0], 0.0, 0.0, jerk_profile.jerk_values, time_steps_1)

# # second block
# time_step_2 = (time_stamps[2] - time_stamps[1]) / float(profile_type.size)
# time_steps_2 = [time_step_2] * profile_type.size

# jerk_scale_factor = profile_type.jerk_scale(v_waypoints[0], 0.0, time_step_2, MODE_VEL)

# jerk_profile_2 = profile_type(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

# (p2, v2, a2) = integrate(p1[-1], v1[-1], 0.0, jerk_profile_2.jerk_values, time_steps_2)


# # plot both blocks

# time_points = create_time_points(time_steps_1 + time_steps_2)

# print("T: " + str(time_points))
# print("Pos: " + str(p1 + p2[1:]))
# print("Vel: " + str(v1 + v2[1:]))
# print("Accel: " + str(a1 + a2[1:]))

# plot_trajectory(time_points, jerk_profile.jerk_values + jerk_profile_2.jerk_values, p1 + p2[1:], v1 + v2[1:], a1 + a2[1:])




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

total_duration = 2.0
# time_step = 0.2
time_step = total_duration / 3.0

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
# # jerk_scale_factor = (final_pos - initial_pos) / (time_scale_factor * 0.24)  # UD

# # Correct for velocity
# # jerk_scale_factor = (final_vel - initial_vel) / (time_scale_factor * 1.6)  # UDUD
# # jerk_scale_factor = (final_vel - initial_vel) / (time_scale_factor * 2.4)  # UOOD
# # jerk_scale_factor = (final_vel - initial_vel) / (time_scale_factor * 0.8)  # UDDU - corresponds to the max vel value, not the final one (Vi=Vf)
# jerk_scale_factor = (final_vel - initial_vel) / (time_scale_factor * 0.8)  # UD


# # UDDU = lambda min, max: [max, 0, min, 0, min, 0, max]
# # UDUD = lambda min, max: [max, 0, min, 0, max, 0, min]
# # UOOD = lambda min, max: [max, 0, 0, 0, 0, 0, min]
# # UD = lambda min, max: [max, 0, min]

# # jerk_profile = UDDU(jerk_min, jerk_max)
# # jerk_profile = UDUD(jerk_min, jerk_max)
# # jerk_profile = UDUD(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)
# # jerk_profile = UDUD(-10.0, 10.0)
# # jerk_profile = UOOD(-10.0, 10.0)
# # jerk_profile = UDDU(-10.0, 10.0)
# # jerk_profile = UD(-10.0, 10.0)
# # jerk_profile = UOOD(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)
# # jerk_profile = UDDU(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)
# jerk_profile = UD(-10.0 * jerk_scale_factor, 10.0 * jerk_scale_factor)

# # jerk_profile = UDDU(-jerk_max, jerk_max)

# # time_steps = [time_step] * 7
# time_steps = [time_step] * 3

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
