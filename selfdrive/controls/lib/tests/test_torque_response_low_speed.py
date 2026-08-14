import unittest

from selfdrive.controls.lib.torque_authority import LateralResponseCompensator


class TestLowSpeedTorqueResponseLearning(unittest.TestCase):
  @staticmethod
  def run_samples(compensator, speed, desired, actual, count=80, **safety_state):
    for _ in range(count):
      compensator.update(speed, desired, actual, **safety_state)

  def test_10_to_20_kph_learns_valid_corner(self):
    learner = LateralResponseCompensator()
    self.run_samples(learner, 15.0, 0.050, 0.045)
    self.assertGreater(learner.counts[0], 0)
    self.assertEqual(sum(learner.counts[1:]), 0)

  def test_20_to_30_kph_learns_valid_corner(self):
    learner = LateralResponseCompensator()
    self.run_samples(learner, 25.0, 0.075, 0.068)
    self.assertGreater(learner.counts[1], 0)
    self.assertEqual(learner.counts[0], 0)

  def test_low_speed_near_straight_does_not_learn(self):
    learner = LateralResponseCompensator()
    self.run_samples(learner, 15.0, 0.025, 0.023)
    self.assertEqual(learner.update_count, 0)

  def test_direction_mismatch_does_not_learn(self):
    learner = LateralResponseCompensator()
    self.run_samples(learner, 15.0, 0.050, -0.045)
    self.assertEqual(learner.update_count, 0)

  def test_safety_gates_still_block_low_speed_learning(self):
    for gate in ('steering_pressed', 'steer_limited', 'rate_limited',
                 'reversal_active', 'path_unstable'):
      with self.subTest(gate=gate):
        learner = LateralResponseCompensator()
        self.run_samples(learner, 15.0, 0.050, 0.045, **{gate: True})
        self.assertEqual(learner.update_count, 0)


if __name__ == '__main__':
  unittest.main()
