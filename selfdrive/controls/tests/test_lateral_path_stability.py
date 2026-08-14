import unittest

from selfdrive.controls.lib.lateral_path_stability import LaneCenterCorrection, PathStabilityMonitor


class TestPathStabilityMonitor(unittest.TestCase):
  def test_monotonic_curve_is_not_wobble(self):
    monitor = PathStabilityMonitor()
    for i in range(40):
      monitor.update(i * 0.04)
    self.assertGreater(monitor.range_m, 1.0)
    self.assertEqual(monitor.flips, 0)
    self.assertFalse(monitor.active)

  def test_alternating_path_triggers_and_holds(self):
    monitor = PathStabilityMonitor(hold_frames=12)
    for value in [0.0, 0.18, -0.18, 0.20, -0.20]:
      monitor.update(value)
    self.assertTrue(monitor.active)
    self.assertGreaterEqual(monitor.flips, 3)
    for _ in range(5):
      monitor.update(-0.20)
    self.assertTrue(monitor.active)


class TestLaneCenterCorrection(unittest.TestCase):
  def test_requires_persistent_confident_straight(self):
    correction = LaneCenterCorrection()
    for _ in range(159):
      correction.update(0.10, True, 0.05)
    self.assertEqual(correction.correction_m, 0.0)
    correction.update(0.10, True, 0.05)
    correction.update(0.10, True, 0.05)
    self.assertGreater(correction.correction_m, 0.0)
    self.assertLessEqual(correction.correction_m, 0.0005)

  def test_ineligible_state_decays_correction(self):
    correction = LaneCenterCorrection(confirm_seconds=0.0)
    for _ in range(100):
      correction.update(0.20, True, 0.05)
    before = correction.correction_m
    for _ in range(10):
      correction.update(0.0, False, 0.05)
    self.assertGreater(before, correction.correction_m)
    self.assertGreaterEqual(correction.correction_m, 0.0)


if __name__ == '__main__':
  unittest.main()
