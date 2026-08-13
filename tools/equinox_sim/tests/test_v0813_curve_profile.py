import math
import unittest

from selfdrive.controls.lib.curve_speed_limiter import (
  CURVE_SPEED_DISABLED,
  build_model_curve_profile,
  build_v0813_model_curve_profile,
  calculate_curve_speed_details,
)
from selfdrive.modeld.constants import T_IDXS


def model_trajectory(curvature=0.0, speed=15.0):
  times = list(T_IDXS)
  if abs(curvature) < 1e-9:
    xs = [speed * t for t in times]
    ys = [0.0 for _ in times]
    headings = [0.0 for _ in times]
  else:
    radius = 1.0 / curvature
    headings = [speed * t * curvature for t in times]
    xs = [radius * math.sin(heading) for heading in headings]
    ys = [radius * (1.0 - math.cos(heading)) for heading in headings]

  vxs = [speed * math.cos(heading) for heading in headings]
  vys = [speed * math.sin(heading) for heading in headings]
  vzs = [0.0 for _ in times]
  yaw_rates = [speed * curvature for _ in times]
  zs = [0.0 for _ in times]
  return times, yaw_rates, vxs, vys, vzs, xs, ys, zs


class TestV0813CurveProfile(unittest.TestCase):
  def test_consistent_curve_uses_path_and_yaw(self):
    trajectory = model_trajectory(curvature=1.0 / 80.0)
    curvatures, times, distances, valid, diag = build_v0813_model_curve_profile(
      *trajectory, measured_curvature=0.0)

    self.assertTrue(valid)
    self.assertEqual(len(curvatures), len(times))
    self.assertEqual(len(curvatures), len(distances))
    self.assertGreaterEqual(diag["model_agree_points"], 7)
    self.assertEqual(diag["model_disagree_points"], 0)
    self.assertAlmostEqual(max(curvatures[1:]), 1.0 / 80.0, places=4)
    curve_speed, values_valid, _speed_diag = calculate_curve_speed_details(
      curvatures, v_ego=22.0, cruise_speed=27.0, min_curve_speed=11.1,
      curvature_factor=0.85, time_idxs=times, distances=distances)
    self.assertTrue(values_valid)
    self.assertEqual(curve_speed, 11.1)

  def test_path_twitch_is_capped_when_yaw_stays_straight(self):
    trajectory = list(model_trajectory())
    trajectory[6][15] = 2.0
    curvatures, _times, _distances, valid, diag = build_v0813_model_curve_profile(
      *trajectory, measured_curvature=0.0)

    self.assertTrue(valid)
    self.assertGreater(diag["model_disagree_points"], 0)
    self.assertLessEqual(max(curvatures), 0.0025 + 1e-9)
    curve_speed, values_valid, _speed_diag = calculate_curve_speed_details(
      curvatures, v_ego=22.0, cruise_speed=27.0, min_curve_speed=11.1,
      curvature_factor=0.85, time_idxs=_times, distances=_distances)
    self.assertTrue(values_valid)
    self.assertEqual(curve_speed, CURVE_SPEED_DISABLED)

  def test_rejects_non_monotonic_model_time(self):
    trajectory = list(model_trajectory(curvature=1.0 / 100.0))
    trajectory[0][10] = trajectory[0][9]
    curvatures, times, distances, valid, diag = build_v0813_model_curve_profile(
      *trajectory, measured_curvature=0.0)

    self.assertFalse(valid)
    self.assertEqual(curvatures, [])
    self.assertEqual(times, [])
    self.assertEqual(distances, [])
    self.assertFalse(diag["model_profile_valid"])

  def test_legacy_wrapper_keeps_four_value_contract(self):
    result = build_model_curve_profile(*model_trajectory(), measured_curvature=0.0)
    self.assertEqual(len(result), 4)
    self.assertTrue(result[3])


if __name__ == "__main__":
  unittest.main()
