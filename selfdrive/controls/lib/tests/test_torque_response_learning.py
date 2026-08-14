import math
import unittest

from selfdrive.controls.lib.torque_authority import LateralResponseCompensator


class TestTorqueResponseLearning(unittest.TestCase):
  def test_realistic_stable_curve_learns_in_each_speed_bin(self):
    learner = LateralResponseCompensator()
    speeds = [15.0, 25.0, 35.0, 50.0, 70.0]
    for expected_bin, speed in enumerate(speeds):
      before = learner.update_count
      for frame in range(80):
        desired = 0.24 + 0.006 * math.sin(frame * 0.15)
        learner.update(speed, desired, desired * 0.90, dt=0.01)
      self.assertGreater(learner.update_count, before)
      self.assertGreater(learner.counts[expected_bin], 0)

  def test_safety_conditions_still_block_learning(self):
    blockers = [
      {"steering_pressed": True},
      {"steer_limited": True},
      {"rate_limited": True},
      {"reversal_active": True},
      {"path_unstable": True},
    ]
    for blocker in blockers:
      learner = LateralResponseCompensator()
      for _ in range(100):
        learner.update(35.0, 0.25, 0.22, dt=0.01, **blocker)
      self.assertEqual(learner.update_count, 0)

  def test_speed_and_direction_gates_remain_safe(self):
    for speed, desired, actual in [
      (9.9, 0.25, 0.22),
      (80.1, 0.25, 0.22),
      (35.0, 0.25, -0.22),
      (35.0, 0.10, 0.09),
    ]:
      learner = LateralResponseCompensator()
      for _ in range(100):
        learner.update(speed, desired, actual, dt=0.01)
      self.assertEqual(learner.update_count, 0)


if __name__ == '__main__':
  unittest.main()
