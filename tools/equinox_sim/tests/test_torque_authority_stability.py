import unittest

from selfdrive.controls.lib.torque_authority import (
  DynamicTorqueAuthorityScheduler, authority_ceiling,
  effective_torque_params,
)


class TestTorqueAuthorityStability(unittest.TestCase):
  BASE_LAT = 1.948
  BASE_FRIC = 0.168
  TOTAL_POINTS = 12003

  def test_maximum_low_speed_authority_is_bounded(self):
    scheduler = DynamicTorqueAuthorityScheduler()
    for _ in range(100):
      state = scheduler.update(25.0, 0.0020, 0.90)
    self.assertAlmostEqual(state['authorityRequest'], 0.65)
    self.assertAlmostEqual(state['authorityCeiling'], 0.65)

    lat, friction, blend = effective_torque_params(
      self.BASE_LAT, self.BASE_FRIC, 25.0,
      state['authorityRequest'], self.TOTAL_POINTS)

    self.assertAlmostEqual(blend, 0.65)
    self.assertGreaterEqual(lat, 1.87)
    self.assertLessEqual(friction, 0.177)
    self.assertLess((self.BASE_LAT / lat) - 1.0, 0.05)

  def test_mild_corner_does_not_force_full_boost(self):
    scheduler = DynamicTorqueAuthorityScheduler()
    for _ in range(100):
      state = scheduler.update(25.0, 0.00032, 0.05)

    self.assertLess(state['cornerStrength'], 0.02)
    self.assertLess(state['authorityRequest'], 0.02)

  def test_corner_hold_decays_quickly(self):
    scheduler = DynamicTorqueAuthorityScheduler()
    for _ in range(50):
      scheduler.update(25.0, 0.0020, 0.90)

    for _ in range(40):
      state = scheduler.update(25.0, 0.0, 0.0)

    self.assertEqual(state['holdFrames'], 0)
    self.assertAlmostEqual(state['authorityRequest'], 0.0)

  def test_direction_reversal_immediately_damps_authority(self):
    scheduler = DynamicTorqueAuthorityScheduler()
    for _ in range(50):
      scheduler.update(25.0, 0.0020, 0.90)

    state = scheduler.update(25.0, -0.0020, -0.90)
    self.assertTrue(state['directionReversal'])
    self.assertTrue(state['directionDamping'])
    self.assertLessEqual(state['authorityRequest'], 0.10)
    self.assertEqual(state['holdFrames'], 0)

  def test_opposite_corner_after_straight_is_not_a_reversal(self):
    scheduler = DynamicTorqueAuthorityScheduler()
    for _ in range(50):
      scheduler.update(25.0, 0.0020, 0.90)
    for _ in range(60):
      scheduler.update(25.0, 0.0, 0.0)

    state = scheduler.update(25.0, -0.0020, -0.90)
    self.assertFalse(state['directionReversal'])
    self.assertFalse(state['directionDamping'])

  def test_driver_override_removes_transient_authority(self):
    scheduler = DynamicTorqueAuthorityScheduler()
    for _ in range(50):
      scheduler.update(25.0, 0.0020, 0.90)

    for _ in range(20):
      state = scheduler.update(
        25.0, 0.0020, 0.90,
        steering_pressed=True, strong_driver_override=True)

    self.assertAlmostEqual(state['authorityRequest'], 0.0)
    self.assertEqual(state['holdFrames'], 0)

  def test_highway_profile_adds_no_torque_authority(self):
    self.assertEqual(authority_ceiling(80.0), 0.0)
    lat, friction, blend = effective_torque_params(
      self.BASE_LAT, self.BASE_FRIC, 100.0, 1.0, self.TOTAL_POINTS)
    self.assertEqual(blend, 0.0)
    self.assertAlmostEqual(lat, self.BASE_LAT)
    self.assertAlmostEqual(friction, self.BASE_FRIC)


if __name__ == '__main__':
  unittest.main()
