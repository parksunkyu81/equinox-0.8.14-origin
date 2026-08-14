import json
import unittest

from selfdrive.controls.lib.torque_authority import (DynamicTorqueAuthorityScheduler,
                                                     LateralResponseCompensator)


class TestLateralResponseCompensator(unittest.TestCase):
  def test_stable_corner_learns_independent_speed_bin(self):
    comp = LateralResponseCompensator()
    initial_other_bin = comp.responses[1]
    for _ in range(100):
      scale = comp.update(50.0, 0.50, 0.40, dt=0.01)

    self.assertTrue(comp.stable)
    self.assertGreater(comp.update_count, 0)
    self.assertGreater(scale, 1.0)
    self.assertLess(comp.responses[3], 0.843)
    self.assertEqual(comp.responses[1], initial_other_bin)
    self.assertLessEqual(scale, 1.12)

  def test_unsafe_conditions_freeze_and_remove_extra_authority(self):
    comp = LateralResponseCompensator()
    for _ in range(60):
      comp.update(50.0, 0.50, 0.40, dt=0.01)
    self.assertGreater(comp.scale, 1.0)

    count = comp.update_count
    for _ in range(10):
      comp.update(50.0, 0.50, 0.40, dt=0.01, path_unstable=True)
    self.assertTrue(comp.frozen)
    self.assertEqual(comp.update_count, count)
    self.assertAlmostEqual(comp.scale, 1.0, places=6)

  def test_state_round_trip_and_bounds(self):
    comp = LateralResponseCompensator(json.dumps({
      'responses': [0.1, 0.8, 0.9, 1.5, 1.0],
      'counts': [1, 2, 3, 4, 5],
      'updateCount': 21,
    }))
    self.assertEqual(comp.responses[0], 0.72)
    self.assertEqual(comp.responses[3], 1.08)
    restored = LateralResponseCompensator(comp.serialize())
    self.assertEqual(restored.counts, comp.counts)
    self.assertEqual(restored.update_count, 21)
    self.assertEqual(restored.responses, comp.responses)

  def test_speed_bin_caps_match_vehicle_profile(self):
    for speed, expected_cap in ((15.0, 1.06), (25.0, 1.04), (35.0, 1.09),
                                (50.0, 1.12), (70.0, 1.06)):
      comp = LateralResponseCompensator()
      for _ in range(100):
        scale = comp.update(speed, 0.50, 0.30, dt=0.01)
      self.assertLessEqual(scale, expected_cap)
      self.assertAlmostEqual(scale, expected_cap, places=6)

  def test_path_instability_drops_scheduler_hold(self):
    scheduler = DynamicTorqueAuthorityScheduler()
    for _ in range(30):
      state = scheduler.update(30.0, 0.002, 0.6)
    self.assertGreater(state['authorityRequest'], 0.1)
    state = scheduler.update(30.0, 0.002, 0.6, path_unstable=True)
    self.assertTrue(state['pathUnstable'])
    self.assertEqual(state['holdFrames'], 0)
    self.assertLessEqual(state['authorityRequest'], 0.04 + 1e-9)


if __name__ == '__main__':
  unittest.main()
