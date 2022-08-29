# natural_motion

This package houses code and tests that deal with creating natural, velocity-controlled movements for a simulated UR5e robot.

## File Overview

- scripts_final/
	- jerk_profile.py: Provides a wrapper class for a jerk profile type to provide profile population and different scaling methods
	- baseline_generator.py: Generates baseline and scaling values for creating a custom jerk profile class for a new jerk profile pattern
	- natural_motion.py: A library for functions relating to generating natural motion (integrating, scaling, creating trajectories, and interacting with simulation)
	- throw_demo.py: A demonstration of how the codebase can be used to generate natural motion