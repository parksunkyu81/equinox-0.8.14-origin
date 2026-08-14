#!/usr/bin/env python3
import unittest

from selfdrive.controls.lib.pedal_force_recovery import (
  LEAD_ASSIST_CANCEL_GAP_RECOVERED,
  LEAD_ASSIST_CANCEL_LEAD_JUMP,
  LEAD_ASSIST_CANCEL_SAFETY,
  MOVING_GAP_ACCEL_RISE_MS3,
  MovingGapCatchupAssist,
  moving_gap_accel_cap,
)


class TestMovingGapCatchupAssist(unittest.TestCase):
  DT = 0.01

  def setUp(self):
    self.assist = MovingGapCatchupAssist(self.DT)

  def update(self, **overrides):
    values = {
      "base_safe": True,
      "lead_valid": True,
      "lead_v_rel": 0.30,
      "lead_distance": 30.0,
      "lead_model_prob": 0.95,
      "v_ego": 40.0 / 3.6,
      "desired_tr": 1.2,
      "cruise_speed_error": 2.0,
      "requested_accel": 0.02,
      "lead_measurement_updated": True,
    }
    values.update(overrides)
    return self.assist.update(**values)

  def run_frames(self, frames, **overrides):
    output = 0.0
    for _ in range(frames):
      output = self.update(**overrides)
    return output

  def test_target_distance_is_not_shortened(self):
    self.update(v_ego=10.0, desired_tr=1.25, lead_distance=25.0)
    self.assertAlmostEqual(self.assist.desired_gap_m, 17.0)
    self.assertAlmostEqual(self.assist.distance_margin_m, 8.0)

  def test_stable_opening_lead_activates_bounded_floor(self):
    output = self.run_frames(100)
    self.assertTrue(self.assist.active)
    self.assertEqual(self.assist.activation_count, 1)
    self.assertGreater(output, self.assist.raw_accel)
    self.assertLessEqual(self.assist.target_accel,
                         moving_gap_accel_cap(40.0 / 3.6) + 1e-9)
    self.assertLessEqual(self.assist.assist_accel,
                         self.assist.duration * MOVING_GAP_ACCEL_RISE_MS3 + 1e-9)

  def test_never_overrides_meaningful_negative_request(self):
    output = self.run_frames(150, requested_accel=-0.10)
    self.assertFalse(self.assist.active)
    self.assertAlmostEqual(output, -0.10)

  def test_closing_lead_does_not_activate(self):
    output = self.run_frames(150, lead_v_rel=-0.30)
    self.assertFalse(self.assist.active)
    self.assertAlmostEqual(output, 0.02)

  def test_speed_and_model_probability_gates(self):
    self.run_frames(150, v_ego=19.0 / 3.6)
    self.assertFalse(self.assist.active)
    self.assist.reset()
    self.run_frames(150, lead_model_prob=0.79)
    self.assertFalse(self.assist.active)

  def test_gap_recovery_cancels(self):
    self.run_frames(100)
    self.assertTrue(self.assist.active)
    desired_gap = 4.5 + (40.0 / 3.6) * 1.2
    distance = 30.0
    output = 0.0
    while distance > desired_gap + 0.5:
      distance = max(desired_gap + 0.5, distance - 0.1)
      output = self.update(lead_distance=distance, lead_v_rel=-0.10)
    self.assertFalse(self.assist.active)
    self.assertEqual(self.assist.cancel_reason, LEAD_ASSIST_CANCEL_GAP_RECOVERED)
    self.assertAlmostEqual(output, 0.02)

  def test_lead_jump_cancels_and_requires_restabilization(self):
    self.run_frames(100)
    self.assertTrue(self.assist.active)
    output = self.update(lead_distance=15.0)
    self.assertFalse(self.assist.active)
    self.assertTrue(self.assist.lead_jump_detected)
    self.assertEqual(self.assist.cancel_reason, LEAD_ASSIST_CANCEL_LEAD_JUMP)
    self.assertAlmostEqual(output, 0.02)
    self.run_frames(30, lead_distance=15.0, lead_measurement_updated=False)
    self.assertFalse(self.assist.active)

  def test_safety_gate_cancels_immediately(self):
    self.run_frames(100)
    self.assertTrue(self.assist.active)
    output = self.update(base_safe=False)
    self.assertFalse(self.assist.active)
    self.assertEqual(self.assist.cancel_reason, LEAD_ASSIST_CANCEL_SAFETY)
    self.assertAlmostEqual(output, 0.02)


if __name__ == "__main__":
  unittest.main()
