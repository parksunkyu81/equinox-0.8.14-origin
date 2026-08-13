import unittest
from types import SimpleNamespace

import numpy as np

from selfdrive.controls.lib.model_data_validation import as_finite_vector, validated_model_trajectory


def trajectory_message(size=33):
  t = np.linspace(0.0, 10.0, size).tolist()
  return SimpleNamespace(
    position=SimpleNamespace(x=t, y=[0.0] * size, z=[0.0] * size, t=t),
    velocity=SimpleNamespace(x=[10.0] * size, y=[0.0] * size, z=[0.0] * size),
    orientation=SimpleNamespace(z=[0.0] * size),
    orientationRate=SimpleNamespace(z=[0.0] * size),
  )


class TestModelDataValidation(unittest.TestCase):
  def test_finite_vector(self):
    self.assertIsNotNone(as_finite_vector([0.0, 1.0], expected_size=2))
    self.assertIsNone(as_finite_vector([0.0, np.nan], expected_size=2))
    self.assertIsNone(as_finite_vector([0.0], expected_size=2))

  def test_complete_trajectory(self):
    trajectory = validated_model_trajectory(trajectory_message(), 33)
    self.assertIsNotNone(trajectory)
    self.assertEqual(trajectory[0].shape, (33, 3))

  def test_rejects_partial_and_non_monotonic_trajectory(self):
    partial = trajectory_message()
    partial.velocity.z = partial.velocity.z[:-1]
    self.assertIsNone(validated_model_trajectory(partial, 33))

    non_monotonic = trajectory_message()
    non_monotonic.position.t[10] = non_monotonic.position.t[9]
    self.assertIsNone(validated_model_trajectory(non_monotonic, 33))


if __name__ == "__main__":
  unittest.main()
