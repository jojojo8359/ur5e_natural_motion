# ur5e_natural_motion

## Requirements
- [moveit](https://github.com/ros-planning/moveit)
- [universal_robot](https://github.com/ros-industrial/universal_robot/tree/melodic-devel-staging)

## Optional Packages
- python3-catkin-tools (used `catkin build` for building)
- [rqt_joint_trajectory_plot](https://github.com/tork-a/rqt_joint_trajectory_plot) (used for plotting test trajectories)

## Package Contents
### ur5e_moveit_config
- This package contains the MoveIt! configuration files for a simulated UR5e robot
### natural_motion
- This package houses code and tests that deal with creating natural, velocity-controlled movements for a simulated UR5e robot

## Running
Custom launch files were created for starting the simulated UR5e:
- `launch/ur5e_gazebo_bringup.launch` starts a Gazebo instance that loads the robot model and a model for its table and a wall
	- `roslaunch natural_motion ur5e_gazebo_bringup.launch`
- `launch/ur5e_planning_execution.launch` starts an instance of RViz with the MoveIt! planning plugin and links it to a Gazebo-simulated arm
	- `roslaunch natural_motion ur5e_planning_execution.launch`

Alternatively, `roslaunch ur_gazebo ur5e_bringup.launch` can be used to start a Gazebo instance with no other custom models in the scene.
