#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

from smoothing_core_v2 import *

npz = np.load('/home/joel/catkin_ws/src/ur5e_natural_motion/natural_motion/jerk_profiles/test.npz')  # type: np.lib.npyio.NpzFile
jerk = [x * 1.24833985247 for x in npz['jerk']]
time = npz['time']
initial_vel = npz['vel'][0]
initial_accel = npz['accel'][0]
initial_pos = npz['pos'][0]
final_pos = npz['pos'][-1]

# accelerations = []
# velocities = []
# positions = []

# time = integrate([x + 0.001 for x in np.diff(time)], 0)

accelerations = integrate_time(jerk, np.diff(time), initial_accel)
velocities = integrate_time(accelerations, np.diff(time), initial_vel)
positions = integrate_time(velocities, np.diff(time), initial_pos)

# print(accelerations)

# t = [0, 1, 3, 4]
# p = [5, 4, 3, 2]
# v = [-1, -0.5, -1]
# a = [0.5, 0.5]
# j = [0]

# accel = integrate(j, np.diff(t, 3), 0.5)
# vel = integrate(accel, np.diff(t, 2), -1)
# pos = integrate(vel, np.diff(t), 5)

# print("jerk: " + str(j))
# print("accel: " + str(accel))
# print("vel: " + str(vel))
# print("pos: " + str(pos))




# for k in range(len(time)):
# 	time_step = time[k] - time[k-1]
# 	current_jerk = jerk[k]
# 	if k == 0:
# 		accelerations.append(initial_accel)
# 		velocities.append(initial_vel)
# 		positions.append(initial_pos)
# 	else:
# 		accelerations.append(gen_next_a(accelerations[k-1], current_jerk, time_step))
# 		velocities.append(gen_next_v(accelerations[k-1], velocities[k-1], current_jerk, time_step))
# 		positions.append(gen_next_p(accelerations[k-1], velocities[k-1], positions[k-1], current_jerk, time_step))


print("Total time: " + str(time[-1]))

print("")

print(len(jerk))
print(len(accelerations))
print(len(velocities))
print(len(positions))

print("")
print(positions[0])
print(positions[-1])
print(initial_pos)
print(final_pos)

fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharex=True)

ax1.plot(time, positions)
ax1.set_xlabel('time (s)')
ax1.set_ylabel('rad')
ax1.set_title('Position')

ax2.plot(shift_times(time), velocities, c='orange')
ax2.set_xlabel('time (s)')
ax2.set_ylabel('rad/s')
ax2.set_title('Velocity')

ax3.plot(shift_times(time, 2), accelerations, c='r')
ax3.set_xlabel('time (s)')
ax3.set_ylabel('rad/s^2')
ax3.set_title('Acceleration')

ax4.plot(shift_times(time, 3), jerk, c='g')
ax4.set_xlabel('time (s)')
ax4.set_ylabel('rad/s^3')
ax4.set_title('Jerk')

plt.show()
