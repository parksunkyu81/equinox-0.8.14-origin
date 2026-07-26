import math
import sys
import unittest
from pathlib import Path


GM_DIR = Path(__file__).resolve().parents[1]
if str(GM_DIR) not in sys.path:
  sys.path.insert(0, str(GM_DIR))

from pedal_command import (calculate_gm_pedal_command,  # noqa: E402
                           gm_pedal_control_allowed, limit_gm_pedal_accel)


class TestGMPedalCommand(unittest.TestCase):
  def test_driver_and_control_safety_gates_are_immediate(self):
    self.assertTrue(gm_pedal_control_allowed(True, True, False, False, True))
    self.assertFalse(gm_pedal_control_allowed(False, True, False, False, True))
    self.assertFalse(gm_pedal_control_allowed(True, False, False, False, True))
    self.assertFalse(gm_pedal_control_allowed(True, True, True, False, True))
    self.assertFalse(gm_pedal_control_allowed(True, True, False, True, True))
    self.assertFalse(gm_pedal_control_allowed(True, True, False, False, False))

  def test_comfort_cap_is_applied_before_pedal_conversion(self):
    limited_accel = limit_gm_pedal_accel(2.0, 0.8, blocked=False)
    self.assertEqual(limited_accel, 0.8)
    self.assertAlmostEqual(calculate_gm_pedal_command(limited_accel, 0.18), 0.144)
    self.assertLess(calculate_gm_pedal_command(limited_accel, 0.18),
                    calculate_gm_pedal_command(2.0, 0.18))

  def test_brake_or_standstill_block_never_keeps_positive_accel(self):
    self.assertEqual(limit_gm_pedal_accel(1.0, 0.8, blocked=True), 0.0)
    self.assertEqual(limit_gm_pedal_accel(-0.5, 0.8, blocked=True), -0.5)

  def test_invalid_accel_fails_closed(self):
    self.assertEqual(limit_gm_pedal_accel(math.nan, 1.0, blocked=False), 0.0)
    self.assertEqual(calculate_gm_pedal_command(math.inf, 0.18), 0.0)

  def test_controller_does_not_invent_throttle_for_a_zero_request(self):
    self.assertEqual(calculate_gm_pedal_command(0.0, 0.18), 0.0)


if __name__ == "__main__":
  unittest.main()
