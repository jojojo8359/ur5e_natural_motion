import moveit_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
import numpy as np
import copy
import rospy
import moveit_commander
from genpy import Duration
from jerk_profile import JerkProfile


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
