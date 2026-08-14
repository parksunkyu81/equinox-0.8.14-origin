import unittest

from selfdrive.controls.lib.stop_accel_boost import apply_stop_accel_boost


class TestStopAccelBoostFactor(unittest.TestCase):
  def test_positive_launch_request_is_boosted_by_40_percent(self):
    boosted = apply_stop_accel_boost(
      requested_accel=0.5,
      v_ego=10.0 / 3.6,
      boost_active=True,
      accel_limits=(-3.5, 2.0),
    )
    self.assertAlmostEqual(boosted, 0.7)

  def test_boost_is_not_applied_outside_launch_speed_range(self):
    unboosted = apply_stop_accel_boost(
      requested_accel=0.5,
      v_ego=30.0 / 3.6,
      boost_active=True,
      accel_limits=(-3.5, 2.0),
    )
    self.assertAlmostEqual(unboosted, 0.5)


if __name__ == '__main__':
  unittest.main()
