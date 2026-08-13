import unittest

from selfdrive.controls.lib.torque_authority import DynamicTorqueAuthorityScheduler
from selfdrive.controls.lib.v0813_lateral_compat import (
  V0813CurvatureGuard,
  compensated_steer_delay,
)


class TestV0813LateralCompatibility(unittest.TestCase):
  def test_official_delay_matches_live_torque_alignment(self):
    self.assertAlmostEqual(compensated_steer_delay(0.10), 0.30)
    self.assertAlmostEqual(compensated_steer_delay(0.05), 0.25)
    self.assertAlmostEqual(compensated_steer_delay(float("nan")), 0.20)
    self.assertAlmostEqual(compensated_steer_delay(2.0), 1.00)

  def test_low_speed_corner_is_not_weakened(self):
    guard = V0813CurvatureGuard()
    curvature, rate, active = guard.update(40.0, 0.012, 0.03)
    self.assertFalse(active)
    self.assertAlmostEqual(curvature, 0.012)
    self.assertAlmostEqual(rate, 0.03)

  def test_steady_high_speed_corner_reaches_full_demand(self):
    guard = V0813CurvatureGuard()
    for _ in range(300):
      curvature, _rate, active = guard.update(100.0, 0.003, 0.0)
    self.assertTrue(active)
    self.assertAlmostEqual(curvature, 0.003, places=6)

  def test_real_curve_is_not_double_limited(self):
    guard = V0813CurvatureGuard()
    outputs = [guard.update(90.0, 0.003, 0.0)[0] for _ in range(5)]
    # One 20 Hz model interval should pass a useful fraction of a legitimate
    # curve while still smoothing the step.
    self.assertGreater(outputs[-1], 0.0012)
    self.assertLess(outputs[-1], 0.0020)

  def test_one_model_frame_direction_twitch_does_not_reverse_command(self):
    guard = V0813CurvatureGuard()
    for _ in range(200):
      curvature, _rate, _active = guard.update(100.0, 0.002, 0.0)
    self.assertGreater(curvature, 0.0019)

    twitch_outputs = []
    for _ in range(5):  # one 20 Hz model plan repeated by 100 Hz controls
      curvature, _rate, _active = guard.update(100.0, -0.002, 0.0)
      twitch_outputs.append(curvature)
    self.assertTrue(guard.diagnostics()["modelCurvatureDirectionReversal"])
    self.assertGreater(min(twitch_outputs), 0.0)

  def test_limit_hold_reduces_transient_step(self):
    normal = V0813CurvatureGuard()
    limited = V0813CurvatureGuard()
    normal_curvature, _rate, _active = normal.update(90.0, 0.002, 0.0)
    limited_curvature, _rate, _active = limited.update(
      90.0, 0.002, 0.0, limited_hold=True)
    self.assertLess(limited_curvature, normal_curvature)

  def test_alternating_model_noise_does_not_request_torque_boost(self):
    guard = V0813CurvatureGuard()
    scheduler = DynamicTorqueAuthorityScheduler()
    state = None
    speed_ms = 100.0 / 3.6
    for i in range(100):
      raw_curvature = 0.001 if i % 2 == 0 else -0.001
      curvature, _rate, _active = guard.update(100.0, raw_curvature, 0.0)
      state = scheduler.update(
        100.0, curvature, curvature * speed_ms * speed_ms)
    self.assertIsNotNone(state)
    self.assertAlmostEqual(state["authorityRequest"], 0.0)


if __name__ == "__main__":
  unittest.main()
