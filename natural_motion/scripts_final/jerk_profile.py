try:
	from typing import List, Union, Tuple, Optional, Type
except ImportError:
	pass  # Trick VSC into "importing" typing for type hints, but don't actually import it at runtime (still using Python 2.7.17)

MODE_POS = 0
MODE_VEL = 1

class JerkProfile(object):
	pattern = lambda self, min, max: [0, 0, 0, 0, 0, 0, 0]
	size = 7
	time_scale_factor = 0.0
	pos_scale_factor = 0.0
	vel_scale_factor = 0.0
	orig_dp = 0.0
	orig_vel = 0.0
	orig_duration = 0.0
	orig_jerk_min = 0.0
	orig_jerk_max = 0.0

	def __init__(self, jerk_min, jerk_max):
		# type: (float, float) -> None
		self.jerk_values = self.populate(jerk_min, jerk_max)
	
	def populate(self, min, max):
		# type: (float, float) -> List[float]
		"""Populates the current profile's pattern with min/max values

		:param min: the minimum jerk value
		:param max: the maximum jerk value
		:type min: float
		:type max: float
		:returns: the profile's populated pattern
		:rtype: List[float]
		"""
		return self.pattern(min, max)

	@classmethod
	def time_scale(cls, time_step, mode):
		# type: (float, int) -> float
		# TODO: Update documentation description with base jerk value
		"""Produces the factor to scale a new trajectory's time by based on a pre-defined base trajectory's time component

		The base trajectory's constraint (for this method) is time_scale_factor, which was found by using orig_dp, orig_vel, and orig_jerk_min/max

		A ValueError will be raised if the provided mode is not 0 or 1

		:param time_step: the new time step to scale for
		:param mode: the mode to scale for (0 = position, 1 = velocity); the MODE_POS and MODE_VEL constants can be used for this parameter
		:type time_step: float
		:type mode: int
		:returns: the time scale factor for the specified jerk profile
		:rtype: float
		"""

		if mode == 0:
			return (time_step / cls.time_scale_factor) ** 3
		elif mode == 1:
			return (time_step / cls.time_scale_factor) ** 2
		else:
			raise ValueError("mode %d is not a valid mode" % mode)
	
	@classmethod
	def jerk_scale(cls, delta_val, time_step, mode):
		# type: (float, float, int) -> float
		# TODO: Update documentation description with base jerk value
		"""Produces the factor to scale a new trajectory's jerk by based on a pre-defined base trajectory's position or velocity component
		
		The base trajectory's constraints (for this method) are pos_scale_factor and vel_scale_factor, which were both found by using orig_dp, orig_vel, and orig_jerk_min/max

		:param delta_val: the position/velocity delta to use in the new trajectory
		:param time_step: the new time step to scale for
		:param mode: the mode to scale for (0 = position, 1 = velocity); the MODE_POS and MODE_VEL constant can be used for this parameter
		:type delta_val: float
		:type time_step: float
		:type mode: int
		:returns: the jerk scale factor for the specified jerk profile
		:rtype: float
		"""
		
		if mode == 0:
			return (delta_val) / (cls.time_scale(time_step, mode) * cls.pos_scale_factor)
		elif mode == 1:
			return (delta_val) / (cls.time_scale(time_step, mode) * cls.vel_scale_factor)
		else:
			raise ValueError("mode %d is not a valid mode" % mode)


class UDDU(JerkProfile):
	pattern = lambda self, min, max: [max, 0, min, 0, min, 0, max]
	size = 7
	time_scale_factor = 0.2
	pos_scale_factor = 0.64
	vel_scale_factor = 0.8  # vel scale for UDDU is for max vel, not final vel
	orig_dp = 1.0
	orig_vel = 1.0
	orig_duration = 1.75
	orig_jerk_min = -10.0
	orig_jerk_max = 10.0

	def __init__(self, jerk_min, jerk_max):
		super(UDDU, self).__init__(jerk_min, jerk_max)


class UDUD(JerkProfile):
	pattern = lambda self, min, max: [max, 0, min, 0, max, 0, min]
	size = 7
	time_scale_factor = 0.2
	pos_scale_factor = 1.12
	vel_scale_factor = 1.6
	orig_dp = 1.0
	orig_vel = 1.0
	orig_duration = 2.0
	orig_jerk_min = -10.0
	orig_jerk_max = 10.0

	def __init__(self, jerk_min, jerk_max):
		super(UDUD, self).__init__(jerk_min, jerk_max)


class UOOD(JerkProfile):
	pattern = lambda self, min, max: [max, 0, 0, 0, 0, 0, min]
	size = 7
	time_scale_factor = 0.2
	pos_scale_factor = 1.68
	vel_scale_factor = 2.4
	orig_dp = 1.0
	orig_vel = 1.0
	orig_duration = 2.0
	orig_jerk_min = -10.0
	orig_jerk_max = 10.0

	def __init__(self, jerk_min, jerk_max):
		super(UOOD, self).__init__(jerk_min, jerk_max)

# TODO: consider removing or leave in as another option for 1-waypoint profiles
class UD(JerkProfile):
	pattern = lambda self, min, max: [max, 0, min]
	size = 3
	time_scale_factor = 0.2
	pos_scale_factor = 0.24
	vel_scale_factor = 0.8
	orig_dp = 1.0
	orig_vel = 1.0
	orig_duration = 2.0
	orig_jerk_min = -10.0
	orig_jerk_max = 10.0

	def __init__(self, jerk_min, jerk_max):
		super(UD, self).__init__(jerk_min, jerk_max)
