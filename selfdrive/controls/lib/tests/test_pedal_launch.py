#!/usr/bin/env python3
import unittest
from types import SimpleNamespace

from selfdrive.controls.lib.pedal_launch import (
  PEDAL_LAUNCH_STATE_ACTIVE,
  PEDAL_LAUNCH_STATE_BLOCKED_BRAKING,
  PEDAL_LAUNCH_STATE_BLOCKED_CLOSING,
  PEDAL_LAUNCH_STATE_DRIVER_OVERRIDE,
  PEDAL_LAUNCH_STATE_NO_LEAD,
  PEDAL_LAUNCH_STATE_SPEED,
  PedalLaunchBoostController,
  pedal_launch_accel_floor,
)


def lead(d_rel=6.0, v_rel=0.0, v_lead=0.0, a_lead=0.0, status=True):
  return SimpleNamespace(status=status, dRel=d_rel, vRel=v_rel,
                         vLead=v_lead, vLeadK=v_lead, aLeadK=a_lead)


class TestPedalLaunchBoostController(unittest.TestCase):
  def setUp(self):
    self.controller = PedalLaunchBoostController()

  def update(self, now, lead_data=None, radar_updated=True, radar_age=0.0,
             v_ego=0.0, requested_accel=0.0, brake=False, gas=False,
             standstill=True, force_decel=False):
    return self.controller.update(
      enabled=True,
      brake_pressed=brake,
      gas_pressed=gas,
      standstill=standstill,
      v_ego=v_ego,
      lead=lead_data,
      radar_updated=radar_updated,
      radar_age=radar_age,
      requested_accel=requested_accel,
      accel_limit_max=1.0,
      force_decel=force_decel,
      now=now,
    )

  def arm(self):
    self.update(0.0, lead())
    self.update(0.5, lead())

  def depart(self, brake=False):
    moving = lead(d_rel=6.25, v_rel=0.35, v_lead=0.35)
    self.update(0.60, moving, brake=brake)
    return self.update(0.67, moving, brake=brake)

  def test_repeated_control_frames_do_not_confirm_departure(self):
    self.arm()
    moving = lead(d_rel=6.25, v_rel=0.35, v_lead=0.35)
    self.update(0.60, moving, radar_updated=True)
    for index in range(10):
      self.update(0.61 + index * 0.01, moving, radar_updated=False)
    self.assertFalse(self.controller.active)

  def test_two_new_radar_samples_activate_launch(self):
    self.arm()
    output = self.depart()
    self.assertTrue(self.controller.active)
    self.assertEqual(self.controller.state, PEDAL_LAUNCH_STATE_ACTIVE)
    self.assertAlmostEqual(output, 0.70)

  def test_brake_holds_departure_latch_until_release(self):
    self.arm()
    self.depart(brake=True)
    self.assertFalse(self.controller.active)
    moving = lead(d_rel=6.30, v_rel=0.35, v_lead=0.35)
    output = self.update(0.75, moving, radar_updated=False, brake=False)
    self.assertTrue(self.controller.active)
    self.assertGreater(output, 0.0)

  def test_driver_override_cancels_same_frame(self):
    self.arm()
    self.depart()
    output = self.update(0.70, lead(d_rel=6.3, v_rel=0.3, v_lead=0.4), gas=True)
    self.assertEqual(output, 0.0)
    self.assertFalse(self.controller.active)
    self.assertEqual(self.controller.state, PEDAL_LAUNCH_STATE_DRIVER_OVERRIDE)

  def test_brake_cancels_active_launch_same_frame(self):
    self.arm()
    self.depart()
    output = self.update(0.70, lead(d_rel=6.3, v_rel=0.3, v_lead=0.4), brake=True)
    self.assertEqual(output, 0.0)
    self.assertFalse(self.controller.active)
    self.assertEqual(self.controller.state, PEDAL_LAUNCH_STATE_DRIVER_OVERRIDE)

  def test_normal_low_speed_request_is_unchanged_without_launch_context(self):
    output = self.update(0.0, None, v_ego=10.0 / 3.6,
                         standstill=False, requested_accel=0.4)
    self.assertAlmostEqual(output, 0.4)
    self.assertFalse(self.controller.active)

  def test_armed_launch_is_cancelled_if_ego_rolls_first(self):
    self.arm()
    output = self.update(0.55, lead(), v_ego=1.1 / 3.6,
                         standstill=False, requested_accel=0.2)
    self.assertAlmostEqual(output, 0.2)
    self.assertFalse(self.controller.active)
    self.assertEqual(self.controller.state, PEDAL_LAUNCH_STATE_SPEED)

  def test_closing_or_braking_lead_cancels(self):
    self.arm()
    self.depart()
    output = self.update(0.75, lead(d_rel=6.0, v_rel=-0.3, v_lead=0.2))
    self.assertEqual(output, 0.0)
    self.assertEqual(self.controller.state, PEDAL_LAUNCH_STATE_BLOCKED_CLOSING)

    self.controller = PedalLaunchBoostController()
    self.arm()
    self.depart()
    output = self.update(0.75, lead(d_rel=6.4, v_rel=0.2, v_lead=0.4, a_lead=-0.4))
    self.assertEqual(output, 0.0)
    self.assertEqual(self.controller.state, PEDAL_LAUNCH_STATE_BLOCKED_BRAKING)

  def test_stale_radar_cancels(self):
    self.arm()
    self.depart()
    output = self.update(0.90, lead(d_rel=6.4, v_rel=0.2, v_lead=0.4), radar_age=0.21)
    self.assertEqual(output, 0.0)
    self.assertEqual(self.controller.state, PEDAL_LAUNCH_STATE_NO_LEAD)

  def test_boost_tapers_and_exits_at_25_kph(self):
    self.arm()
    self.depart()
    moving = lead(d_rel=10.0, v_rel=0.4, v_lead=7.0)
    output_20 = self.update(1.0, moving, v_ego=20.0 / 3.6,
                            standstill=False, requested_accel=0.2)
    self.assertAlmostEqual(output_20, pedal_launch_accel_floor(20.0 / 3.6))

    output_25 = self.update(1.1, moving, v_ego=25.0 / 3.6,
                            standstill=False, requested_accel=0.2)
    self.assertAlmostEqual(output_25, 0.2)
    self.assertFalse(self.controller.active)
    self.assertEqual(self.controller.state, PEDAL_LAUNCH_STATE_SPEED)


if __name__ == "__main__":
  unittest.main()
