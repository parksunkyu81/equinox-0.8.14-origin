import unittest

from common.conversions import Conversions as CV
from selfdrive.car.gm.carcontroller import PEDAL_COMMAND_MAX, compute_comma_pedal


class TestEquinoxPedalMap(unittest.TestCase):
  def test_non_positive_accel_releases_pedal(self):
    for accel in (-2.0, -0.1, 0.0):
      self.assertEqual(compute_comma_pedal(accel, 60.0 * CV.KPH_TO_MS), 0.0)

  def test_speed_breakpoints(self):
    self.assertAlmostEqual(compute_comma_pedal(1.0, 0.0), 0.132)
    self.assertAlmostEqual(compute_comma_pedal(1.0, 30.0 * CV.KPH_TO_MS), 0.185)
    self.assertAlmostEqual(compute_comma_pedal(1.0, 100.0 * CV.KPH_TO_MS), 0.188)

  def test_command_is_capped(self):
    self.assertEqual(compute_comma_pedal(100.0, 100.0 * CV.KPH_TO_MS), PEDAL_COMMAND_MAX)


if __name__ == "__main__":
  unittest.main()
