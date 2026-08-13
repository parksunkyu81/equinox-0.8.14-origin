import unittest

from selfdrive.controls.lib.predictive_coasting import PredictiveCoastingCoordinator


class TestPredictiveCoastingStallRelease(unittest.TestCase):
  def setUp(self):
    self.coast = PredictiveCoastingCoordinator(dt=0.01)
    self.coast.phase = "hysteresis_hold"
    self.coast.pedal_scale = 0.0
    self.coast.intervening = True
    self.coast.clear_elapsed = 0.0

  def update(self, *, requested_accel=0.70, v_rel=0.30,
             margin=1.20, a_ego=-0.50):
    v_ego = 9.5
    effective_tr = 1.2
    return self.coast.update(
      enabled=True, control_active=True, requested_accel=requested_accel,
      v_ego=v_ego, a_ego=a_ego, brake_pressed=False, gas_pressed=False,
      lead_valid=True,
      lead_distance=4.5 + v_ego * effective_tr + margin,
      lead_rel_speed=v_rel, lead_accel=0.0, lead_model_prob=0.99,
      effective_tr=effective_tr, fcw=False, radar_valid=True,
      can_valid=True, curve_active=False, curve_target_speed=v_ego,
      speed_limit_active=False)

  def test_sustained_positive_demand_releases_stale_zero(self):
    for _ in range(60):
      scale = self.update()

    self.assertTrue(self.coast.positive_demand_release_active)
    self.assertEqual(self.coast.phase, "recover")
    self.assertGreater(scale, 0.0)

  def test_subthreshold_demand_keeps_near_gap_hold(self):
    for _ in range(100):
      scale = self.update(requested_accel=0.49)

    self.assertFalse(self.coast.positive_demand_release_active)
    self.assertEqual(self.coast.phase, "hysteresis_hold")
    self.assertEqual(scale, 0.0)

  def test_closing_lead_cancels_positive_demand_release(self):
    for _ in range(40):
      self.update()
    for _ in range(30):
      scale = self.update(v_rel=-0.40, a_ego=-0.80)

    self.assertFalse(self.coast.positive_demand_release_active)
    self.assertEqual(self.coast.phase, "hysteresis_hold")
    self.assertEqual(scale, 0.0)


if __name__ == "__main__":
  unittest.main()
