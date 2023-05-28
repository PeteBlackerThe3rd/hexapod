"""
JointTrajectory class used to load, store, and eacess joint trajectories.

This class provides interpolated access to any time point, including supporting
cyclic trajectories.
"""
import numpy as np
import scipy.interpolate.interp1d as interp1d


class JointTrajectoryLoadException(Exception):
  pass


class JointTrajectory:

  JOINT_COUNT = 18

  def __init__(self, filename=None):

    self.sample_times = np.zeros(1,)
    self.samples = np.zeros(1, self.JOINT_COUNT)
    self._is_cyclic = False
    self._is_setup = False
    self._samples_lookup = False

    if filename is not None:
      self.load_trajectory(filename)

  def load_trajectory(self, filename):
    csv_data = np.genfromtxt(filename, delimiter=',')
    if csv_data.shape[0] != self.JOINT_COUNT + 1:
      raise JointTrajectoryLoadException("Failed to load trajectory CSV file contains fewer than %d columns " %
                                         (self.JOINT_COUNT + 1))
    self.set_trajectory(csv_data[0][:], csv_data[1:-1][:])

  def set_trajectory(self, sample_times, samples):
    self.sample_times = sample_times
    self.samples = samples
    self.setup()

  def get_cyclic(self):
    if not self._is_setup:
      self.setup()
    return self._is_cyclic

  def setup(self):
    self._is_cyclic = (self.samples[0] - self.samples[-1]) == np.zeros(self.JOINT_COUNT,)
    self._samples_lookup = interp1d(self.sample_times, self.samples)
    self._is_setup = True
    pass

  def get_duration(self):
    return self.sample_times[-1]

  def __call__(self, sample_time):
    if not self._is_setup:
      self.setup()
    return self._samples_lookup(sample_time)

