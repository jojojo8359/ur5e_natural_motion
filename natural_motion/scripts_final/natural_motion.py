try:
	from typing import List, Union, Tuple, Optional, Type
except ImportError:
	pass  # Trick VSC into "importing" typing for type hints, but don't actually import it at runtime (still using Python 2.7.17)

from jerk_profile import JerkProfile, UDDU, UDUD, UOOD, MODE_POS, MODE_VEL
import rospy
import moveit_msgs.msg
import moveit_commander
import matplotlib.pyplot as plt
import geometry_msgs.msg

def gen_next_a(current_accel, jerk_with_sign, time_step):
	# type: (float, float, float) -> float
	"""Generates a smooth trajectory's next acceleration value with numerical integration

	Follows the equation:

	:math:`a_{k+1}=a_{k}+s_{k}j_{k}t_{k}`
	
	:param current_accel: the trajectory's current acceleration value
	:param jerk_with_sign: the trajectory's current signed jerk value
	:param time_step: the duration between the current timestamp and the next timestamp
	:type current_accel: float
	:type jerk_with_sign: float
	:type time_step: float
	:returns: the next acceleration value
	:rtype: float
	"""

	return current_accel + (jerk_with_sign * time_step)


def gen_next_v(current_accel, current_vel, jerk_with_sign, time_step):
	# type: (float, float, float, float) -> float
	"""Generates a smooth trajectory's next velocity value with numerical integration

	Follows the equation:

	:math:`v_{k+1}=v_{k}+a_{k}t_{k}+\\frac{s_{k}j_{k}}{2}t^{2}_{k}`

	:param current_accel: the trajectory's current acceleration value
	:param current_vel: the trajectory's current velocity value
	:param jerk_with_sign: the trajectory's current signed jerk value
	:param time_step: the duration between the current timestamp and the next timestamp
	:type current_accel: float
	:type current_vel: float
	:type jerk_with_sign: float
	:type time_step: float
	:returns: the next velocity value
	:rtype: float
	"""

	return current_vel + (current_accel*time_step) + ((jerk_with_sign / 2) * (time_step ** 2))


def gen_next_p(current_accel, current_vel, current_pos, jerk_with_sign, time_step):
	# type: (float, float, float, float, float) -> float
	"""Generates a smooth trajectory's next position value with numerical integration

	Follows the equation:

	:math:`p_{k+1}=p_{k}+v_{k}t_{k}+\\frac{a_{k}}{2}t^{2}_{k}+\\frac{s_{k}j_{k}}{6}t^{3}_{k}`
	
	:param current_accel: the trajectory's current acceleration value
	:param current_vel: the trajectory's current velocity value
	:param current_pos: the trajectory's current position value
	:param jerk_with_sign: the trajectory's current signed jerk value
	:param time_step: the duration between the current timestamp and the next timestamp
	:type current_accel: float
	:type current_vel: float
	:type current_pos: float
	:type jerk_with_sign: float
	:type time_step: float
	:returns: the next position value
	:rtype: float
	"""

	return current_pos + (current_vel * time_step) + ((current_accel / 2) * (time_step ** 2)) + ((jerk_with_sign / 6) * (time_step ** 3))


def publish_trajectory(publisher, trajectory, robot):
	# type: (rospy.Publisher, moveit_msgs.msg.RobotTrajectory, moveit_commander.RobotCommander) -> None
	"""Sends a generated trajectory to a ROS publisher as a DisplayTrajectory msg
	
	Uses a RobotCommander instance to get the robot's current position

	:param publisher: the rospy Publisher to advertise on
	:param trajectory: the RobotTrajectory to display
	:param robot: the RobotCommander of the robot to display with
	:type publisher: rospy.Publisher
	:type trajectory: moveit_msgs.msg.RobotTrajectory
	:type robot: moveit_commander.RobotCommander
	:returns: None
	:rtype: None
	"""
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(trajectory)
	publisher.publish(display_trajectory)


def integrate(initial_pos, initial_vel, initial_accel, jerk_profile, time_steps):
	# type: (float, float, float, List[float], List[float]) -> Tuple[List[float], List[float], List[float]]
	"""Numerically integrates the provided jerk profile to accelerations, velocities, and positions

	:param initial_pos: a float representing the trajectory's starting position
	:param initial_vel: a float representing the trajectory's starting velocity
	:param initial_accel: a float representing the trajectory's starting acceleration
	:param jerk_profile: a list of jerk points to integrate from, with size n
	:param time_steps: a list of time durations between a trajectory's points, with size n
	:type initial_pos: float
	:type initial_vel: float
	:type initial_accel: float
	:type jerk_profile: List[float]
	:type time_steps: List[float]
	:returns: a tuple with position (size n+1), velocity (size n+1), and acceleration (size n+1) output, respectively
	:rtype: Tuple[List[float], List[float], List[float]]
	"""
	p = [initial_pos]
	v = [initial_vel]
	a = [initial_accel]

	for i in range(1, len(jerk_profile) + 1):
		a.append(gen_next_a(a[i-1], jerk_profile[i-1], time_steps[i-1]))
		v.append(gen_next_v(a[i-1], v[i-1], jerk_profile[i-1], time_steps[i-1]))
		p.append(gen_next_p(a[i-1], v[i-1], p[i-1], jerk_profile[i-1], time_steps[i-1]))
	
	return (p, v, a)


def populate_jerk_profile(jerk_profile_type, delta_val, time_step, scale_mode):
	# type: (JerkProfile, float, float, int) -> JerkProfile
	"""A wrapper for populating a jerk profile with scaled values

	:param jerk_profile_type: the profile type to populate (class type, not instance)
	:param delta_val: the position/velocity delta to create a scale with; type is specified by scale_mode
	:param time_step: the desired time step to scale to
	:param scale_mode: the mode to scale the jerk profile by; designates what delta_val refers to (0 = position, 1 = velocity; can use MODE_POS/MODE_VEL)
	:type jerk_profile_type: JerkProfile
	:type delta_val: float
	:type time_step: float
	:type scale_mode: int
	:returns: the populated jerk profile class
	:rtype: JerkProfile
	"""

	jerk_scale_factor = jerk_profile_type.jerk_scale(delta_val, time_step, scale_mode)

	return jerk_profile_type(jerk_profile_type.orig_jerk_min * jerk_scale_factor, jerk_profile_type.orig_jerk_max * jerk_scale_factor)


def create_time_points(time_steps):
	# type: (List[float]) -> List[float]
	"""Expands a list of durations into a list of chronological points

	ex: [0.2, 0.2] -> [0.0, 0.2, 0.4]

	:param time_steps: a list of durations (size n)
	:type time_steps: List[float]
	:returns: the expanded list of time points (size n+1)
	:rtype: List[float]
	"""

	time_points = [0.0]
	for time_step in time_steps:
		if time_step == 0.0:
			continue
		time_points.append(time_points[-1] + time_step)
	return time_points


def scale_duration(delta_pos, vel_waypoint, jerk_profile_type):
	# type: (float, float, JerkProfile) -> float
	return abs((delta_pos / jerk_profile_type.orig_dp) * (jerk_profile_type.orig_vel / vel_waypoint) * jerk_profile_type.orig_duration)


def create_trajectory(initial_pos, final_pos, velocity_waypoint=None, waypoint_percent=None, duration=None, use_alternate_profile=False):
	# type: (float, float, Optional[float], Optional[float], Optional[float], bool) -> Tuple[List[float], List[float], List[float], List[float], List[float]]
	"""Creates a joint trajectory with an optional velocity waypoint value or a desired duration

	:param initial_pos: the initial position of the joint
	:param final_pos: the desired final position of the joint
	:param velocity_waypoint: the target velocity value, optional
	:param waypoint_percent: the target waypoint's percentage position within the trajectory, optional
	:param duration: the trajectory's target duration, optional
	:param use_alternate_profile: if True, use UDUD profile over UOOD; if False, default to UOOD
	:param initial_pos: float
	:param final_pos: float
	:param velocity_waypoint: Optional[float]
	:param waypoint_percent: Optional[float]
	:param duration: Optional[float]
	:param use_alternate_profile: bool
	:returns: a tuple containing time stamps, jerk values, positions, velocities, and accelerations, respectively
	:rtype: Tuple[List[float], List[float], List[float], List[float], List[float]]
	"""

	delta_pos = final_pos - initial_pos
	if velocity_waypoint is not None and ((delta_pos < 0 and velocity_waypoint > 0) or (delta_pos > 0 and velocity_waypoint < 0)):
		raise ValueError("Position delta %d and velocity waypoint %d do not have the same sign!" % (delta_pos, velocity_waypoint))
	if velocity_waypoint is not None and waypoint_percent is not None and duration is not None:
		print("*** Duration override is currently set to %d but offset waypoints were specified, ignoring... ***" % duration)
	
	if velocity_waypoint is None and waypoint_percent is None and duration is not None:
		# use UDDU with a fixed duration
		profile_type = UDDU

		# duration should always be positive
		total_duration = abs(duration)
		time_stamps = [0.0, total_duration]

		time_step = (time_stamps[1] - time_stamps[0]) / float(profile_type.size)
		time_steps = [time_step] * profile_type.size

		jerk_profile = populate_jerk_profile(profile_type, delta_pos, time_step, MODE_POS)
		
		(positions, velocities, accelerations) = integrate(initial_pos, 0.0, 0.0, jerk_profile.jerk_values, time_steps)

		time_points = create_time_points(time_steps)
		jerk = jerk_profile.jerk_values
	elif velocity_waypoint is not None and waypoint_percent is None:
		# use UDDU with a velocity waypoint
		profile_type = UDDU

		total_duration = scale_duration(delta_pos, velocity_waypoint, profile_type)
		time_stamps = [0.0, total_duration]

		time_step = (time_stamps[1] - time_stamps[0]) / float(profile_type.size)
		time_steps = [time_step] * profile_type.size

		delta_vel = velocity_waypoint - 0.0

		jerk_profile = populate_jerk_profile(profile_type, delta_vel, time_step, MODE_VEL)

		(positions, velocities, accelerations) = integrate(initial_pos, 0.0, 0.0, jerk_profile.jerk_values, time_steps)

		time_points = create_time_points(time_steps)
		jerk = jerk_profile.jerk_values
	elif (velocity_waypoint is not None and waypoint_percent is not None) or (velocity_waypoint is None and waypoint_percent is not None and duration is not None):
		# use UDUD/UOOD in two segments

		if use_alternate_profile:
			profile_type = UDUD
		else:
			profile_type = UOOD
		
		if duration:
			total_duration = duration
		else:
			total_duration = scale_duration(delta_pos, velocity_waypoint, profile_type)
		time_stamps = [0.0, waypoint_percent * total_duration, total_duration]

		# FIRST BLOCK

		time_step_1 = (time_stamps[1] - time_stamps[0]) / float(profile_type.size)
		time_steps_1 = [time_step_1] * profile_type.size

		if duration:
			delta_pos_1 = delta_pos * waypoint_percent
			jerk_profile = populate_jerk_profile(profile_type, delta_pos_1, time_step_1, MODE_POS)
		else:
			delta_vel = velocity_waypoint - 0.0
			jerk_profile = populate_jerk_profile(profile_type, delta_vel, time_step_1, MODE_VEL)

		(p1, v1, a1) = integrate(initial_pos, 0.0, 0.0, jerk_profile.jerk_values, time_steps_1)

		# SECOND BLOCK

		time_step_2 = (time_stamps[2] - time_stamps[1]) / float(profile_type.size)
		time_steps_2 = [time_step_2] * profile_type.size

		if duration:
			delta_pos_2 = delta_pos_1 - delta_pos
			jerk_profile_2 = populate_jerk_profile(profile_type, delta_pos_2, time_step_2, MODE_POS)
		else:
			delta_vel = 0.0 - velocity_waypoint
			jerk_profile_2 = populate_jerk_profile(profile_type, delta_vel, time_step_2, MODE_VEL)

		(p2, v2, a2) = integrate(p1[-1], v1[-1], 0.0, jerk_profile_2.jerk_values, time_steps_2)

		# combine all data

		time_points = create_time_points(time_steps_1 + time_steps_2)
		jerk = jerk_profile.jerk_values + jerk_profile_2.jerk_values
		positions = p1 + p2[1:]
		velocities = v1 + v2[1:]
		accelerations = a1 + a2[1:]
	
	return (time_points, jerk, positions, velocities, accelerations)


def print_trajectory(time, jerk, pos, vel, accel):
	# type: (List[float], List[float], List[float], List[float], List[float]) -> None
	print("T: " + str(time))
	print("Pos: " + str(pos))
	print("Vel: " + str(vel))
	print("Accel: " + str(accel))
	print("Jerk: " + str(jerk))


def plot_trajectory(time, jerk, positions, velocities, accelerations):
	# type: (List[float], List[float], List[float], List[float], List[float]) -> None
	"""Plots a given trajectory using matplotlib

	:param time: time values in point form (as opposed to durations) (size n)
	:param jerk: jerk values of the trajectory (size n-1)
	:param positions: position values of the trajectory (size n)
	:param velocities: velocity values of the trajectory (size n)
	:param accelerations: acceleration values of the trajectory (size n)
	:type time: List[float]
	:type jerk: List[float]
	:type positions: List[float]
	:type velocities: List[float]
	:type accelerations: List[float]
	"""
	titlefont = {'size': 85}
	labelfont = {'size': 14}

	fig, (ax1) = plt.subplots(1, 1)

	# ax1.set_title("UDDU Jerk Profile", fontdict=titlefont)
	plt.suptitle("UDUD Jerk Profile", fontsize='x-large')
	
	ax1.plot(time, positions, c='blue', marker='o', linewidth=3, markersize=10, label='Position')
	ax1.plot(time, velocities, c='orange', marker='o', linewidth=3, markersize=10, label='Velocity')
	ax1.plot(time, accelerations, c='red', marker='o', linewidth=3, markersize=10, label='Acceleration')
	ax1.hlines(jerk, [time[x] for x in range(len(time)-1)], [time[x] for x in range(1, len(time))], colors='green', linewidth=3)

	ymin = [0]
	for i in range(1, len(time) - 1):
		ymin.append(min(jerk[i-1], jerk[i]))
	ymin.append(0)

	ymax = [jerk[0]]
	for i in range(1, len(time) - 1):
		ymax.append(max(jerk[i-1], jerk[i]))
	ymax.append(jerk[-1])

	ax1.vlines([time[x] for x in range(len(time))], ymin, ymax, colors='green', linewidth=3, label='Jerk')

	ax1.legend(loc='upper left', fontsize=14)
	ax1.set_ylabel("rad, rad/s, rad/s^2, rad/s^3", fontdict=labelfont)
	ax1.set_xlabel("time (s)", fontdict=labelfont)

	fig.set_size_inches(14.5, 10.5, forward=True)

	# plt.show()


def wait_for_state_update(box_name, scene, box_is_known=False, box_is_attached=False, timeout=4):
	# type: (str, moveit_commander.PlanningSceneInterface, bool, bool, float) -> bool
	"""Waits for the state of the planning scene to be updated to what is expected

	:param box_name: the name of a planning scene box object that should be added to the scene
	:param scene: the planning scene to check
	:param box_is_known: whether box_name should be known within the planning scene
	:param box_is_attached: whether box_name should be attached within the planning scene
	:param timeout: the amount of time to wait for an update before giving up
	:type box_name: str
	:type scene: moveit_commander.PlanningSceneInterface
	:type box_is_known: bool
	:type box_is_attached: bool
	:type timeout: float
	:returns: False if a timeout occurs, True if the object is found within the planning scene
	:rtype: bool
	"""
	start = rospy.get_time()
	seconds = rospy.get_time()
	while (seconds - start < timeout) and not rospy.is_shutdown():
		attached_objects = scene.get_attached_objects([box_name])  # type: dict[str, moveit_msgs.msg.AttachedCollisionObject]
		is_attached = len(attached_objects.keys()) > 0
		known_objects = scene.get_known_object_names()  # type: list[str]
		is_known = box_name in known_objects
		if (box_is_attached == is_attached) and (box_is_known == is_known):
			return True
		rospy.sleep(0.1)
		seconds = rospy.get_time()
	return False


def add_table(robot, scene, timeout=4):
	# type: (moveit_commander.RobotCommander, moveit_commander.PlanningSceneInterface, float) -> bool
	"""Adds a box object to a planning scene that represents a table the robot sits on

	:param robot: the RobotCommander object representing the robot
	:param scene: the planning scene to add the table to
	:param timeout: the amount of time to wait for a planning scene state update before giving up (passed on to `wait_for_state_update`)
	:type robot: moveit_commander.RobotCommander
	:type scene: moveit_commander.PlanningSceneInterface
	:type timeout: float
	:returns: True if the table was successfully added to the planning scene, False otherwise
	:rtype: bool
	"""
	box_pose = geometry_msgs.msg.PoseStamped()
	robot_planning_frame = robot.get_planning_frame()  # type: str
	box_pose.header.frame_id = robot_planning_frame
	box_pose.pose.orientation.w = 1.0
	box_pose.pose.position.z = -0.07
	table_name = "table"
	scene.add_box(table_name, box_pose, size=(10, 10, 0.1))
	return wait_for_state_update(table_name, scene, box_is_known=True, timeout=timeout)


def add_wall(robot, scene, timeout=4):
	# type: (moveit_commander.RobotCommander, moveit_commander.PlanningSceneInterface, float) -> bool
	"""Adds a box object to a planning scene that represents a wall the robot sits in front of

	:param robot: the RobotCommander object representing the robot
	:param scene: the planning scene to add the table to
	:param timeout: the amount of time to wait for a planning scene state update before giving up (passed on to `wait_for_state_update`)
	:type robot: moveit_commander.RobotCommander
	:type scene: moveit_commander.PlanningSceneInterface
	:type timeout: float
	:returns: True if the wall was successfully added to the planning scene, False otherwise
	:rtype: bool
	"""
	box_pose = geometry_msgs.msg.PoseStamped()
	robot_planning_frame = robot.get_planning_frame()  # type: str
	box_pose.header.frame_id = robot_planning_frame
	box_pose.pose.orientation.w = 1.0
	box_pose.pose.position.y = -0.15
	wall_name = "wall"
	scene.add_box(wall_name, box_pose, size=(10, 0.02, 10))
	return wait_for_state_update(wall_name, scene, box_is_known=True, timeout=timeout)
