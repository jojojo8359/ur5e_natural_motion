#! /usr/bin/env python

try:
	from typing import List, Union, Tuple, Optional, Type
except ImportError:
	pass  # Trick VSC into "importing" typing for type hints, but don't actually import it at runtime (still using Python 2.7.17)

import matplotlib.pyplot as plt
from natural_motion import integrate, plot_trajectory, create_time_points, print_trajectory

plt.rcParams.update({'font.size': 16})

initial_pos = 0.0
final_pos = 2.0

initial_vel = 0.0
final_vel = 1.0

initial_accel = 0.0

jerk_min = -10.0
jerk_max = 10.0


UDDU = lambda min, max: [max, 0, min, 0, min, 0, max]
UDUD = lambda min, max: [max, 0, min, 0, max, 0, min]
UOOD = lambda min, max: [max, 0, 0, 0, 0, 0, min]
UD = lambda min, max: [max, 0, min]

profile = UOOD

total_duration = 2.0

time_step = 0.2



jerk_profile = profile(jerk_min, jerk_max)

time_steps = [time_step] * len(jerk_profile)

(positions, velocities, accelerations) = integrate(initial_pos, initial_vel, initial_accel, jerk_profile, time_steps)

time_points = create_time_points(time_steps)

# print_trajectory(time_points, jerk_profile, positions, velocities, accelerations)

print("time_scale_factor: " + str(time_step))
print("pos_scale_factor: " + str(positions[-1]))
vel_scale_factor = max(velocities)
print("vel_scale_factor: " + str(vel_scale_factor))
print("orig_jerk_min: " + str(min(jerk_profile)))
print("orig_jerk_max: " + str(max(jerk_profile)))

plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations)

time_step = total_duration / float(len(jerk_profile))

pos_time_scale_factor = (time_step / 0.2) ** 3  # for pos
pos_jerk_scale_factor = (final_pos - initial_pos) / (pos_time_scale_factor * positions[-1])

jerk_profile = profile(jerk_min * pos_jerk_scale_factor, jerk_max * pos_jerk_scale_factor)

time_steps = [time_step] * len(jerk_profile)

(positions, velocities, accelerations) = integrate(initial_pos, initial_vel, initial_accel, jerk_profile, time_steps)

time_points = create_time_points(time_steps)

# print_trajectory(time_points, jerk_profile, positions, velocities, accelerations)
print("orig_dp: " + str(final_pos - initial_pos))
print("orig_vel: " + str(final_vel - initial_vel))
new_duration = total_duration / ((final_vel - initial_vel) / max(velocities))
print("orig_duration: " + str(new_duration))
plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations)


time_step = new_duration / float(len(jerk_profile))

vel_time_scale_factor = (time_step / 0.2) ** 2  # for vel
vel_jerk_scale_factor = (final_vel - initial_vel) / (vel_time_scale_factor * vel_scale_factor)

jerk_profile = profile(jerk_min * vel_jerk_scale_factor, jerk_max * vel_jerk_scale_factor)

time_steps = [time_step] * len(jerk_profile)

(positions, velocities, accelerations) = integrate(initial_pos, initial_vel, initial_accel, jerk_profile, time_steps)

time_points = create_time_points(time_steps)

# print_trajectory(time_points, jerk_profile, positions, velocities, accelerations)
plot_trajectory(time_points, jerk_profile, positions, velocities, accelerations)

plt.show()
