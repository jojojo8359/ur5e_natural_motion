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

	def __init__(self, jerk_min, jerk_max):
		self.jerk_values = self.populate(jerk_min, jerk_max)
	
	def populate(self, min, max):
		return self.pattern(min, max)

	@classmethod
	def time_scale(cls, time_step, mode):
		# type: (float, int) -> float
		"""Description

		:param time_step: the new time step to scale for
		:param mode: the mode to scale for (0 = position, 1 = velocity)
		:type time_step: float
		:type mode: int
		"""

		if mode == 0:
			return (time_step / cls.time_scale_factor) ** 3
		elif mode == 1:
			return (time_step / cls.time_scale_factor) ** 2
		else:
			raise ValueError("mode %d is not a valid mode" % mode)
	
	@classmethod
	def jerk_scale(cls, initial_val, final_val, time_step, mode):
		# type: (float, float, float, int) -> float
		
		if mode == 0:
			return (final_val - initial_val) / (cls.time_scale(time_step, mode) * cls.pos_scale_factor)
		elif mode == 1:
			return (final_val - initial_val) / (cls.time_scale(time_step, mode) * cls.vel_scale_factor)
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

	def __init__(self, jerk_min, jerk_max):
		super(UOOD, self).__init__(jerk_min, jerk_max)


class UD(JerkProfile):
	pattern = lambda self, min, max: [max, 0, min]
	size = 3
	time_scale_factor = 0.2
	pos_scale_factor = 0.24
	vel_scale_factor = 0.8
	orig_dp = 1.0
	orig_vel = 1.0
	orig_duration = 2.0

	def __init__(self, jerk_min, jerk_max):
		super(UD, self).__init__(jerk_min, jerk_max)
