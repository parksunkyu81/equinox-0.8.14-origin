import unittest

from selfdrive.controls.lib.comma_pedal_profile import (
  CommaPedalProfileController, combine_comma_pedal_gain,
  comma_pedal_profile_gain, normalize_comma_pedal_profile,
)


class TestCommaPedalProfile(unittest.TestCase):
  def test_normalize_defaults_to_mid(self):
    self.assertEqual(normalize_comma_pedal_profile(b'HIGH'), 'high')
    self.assertEqual(normalize_comma_pedal_profile(' Low '), 'low')
    self.assertEqual(normalize_comma_pedal_profile('invalid'), 'mid')
    self.assertEqual(normalize_comma_pedal_profile(None), 'mid')

  def test_profile_gains_are_distinct_and_speed_bounded(self):
    self.assertAlmostEqual(comma_pedal_profile_gain('low', 30.0 / 3.6), 0.85)
    self.assertAlmostEqual(comma_pedal_profile_gain('mid', 30.0 / 3.6), 1.00)
    self.assertAlmostEqual(comma_pedal_profile_gain('high', 30.0 / 3.6), 1.18)
    self.assertAlmostEqual(comma_pedal_profile_gain('low', 100.0 / 3.6), 0.88)
    self.assertAlmostEqual(comma_pedal_profile_gain('high', 100.0 / 3.6), 1.12)
    self.assertAlmostEqual(comma_pedal_profile_gain('low', 150.0 / 3.6), 0.92)
    self.assertAlmostEqual(comma_pedal_profile_gain('high', 150.0 / 3.6), 1.08)

  def test_learning_is_a_fine_offset_and_launch_is_neutral(self):
    self.assertAlmostEqual(combine_comma_pedal_gain(0.85, 1.04), 0.89)
    self.assertAlmostEqual(combine_comma_pedal_gain(1.00, 1.04), 1.04)
    self.assertAlmostEqual(combine_comma_pedal_gain(1.18, 1.04), 1.22)
    self.assertAlmostEqual(combine_comma_pedal_gain(0.85, 0.90), 0.82)
    self.assertAlmostEqual(combine_comma_pedal_gain(1.18, 1.08), 1.22)
    self.assertAlmostEqual(combine_comma_pedal_gain(1.18, 1.08, True), 1.00)

  def test_live_change_slews_only_while_pedal_is_active(self):
    controller = CommaPedalProfileController('mid')
    self.assertAlmostEqual(controller.update('mid', 30.0 / 3.6, False, 0.01), 1.00)
    self.assertAlmostEqual(controller.update('high', 30.0 / 3.6, True, 1.0), 1.16)
    self.assertTrue(controller.changing)
    self.assertAlmostEqual(controller.update('high', 30.0 / 3.6, True, 0.2), 1.18)
    self.assertFalse(controller.changing)
    self.assertAlmostEqual(controller.update('low', 30.0 / 3.6, False, 0.01), 0.85)
    self.assertFalse(controller.changing)

  def test_speed_attenuation_does_not_block_learning(self):
    controller = CommaPedalProfileController('high')
    controller.update('high', 30.0 / 3.6, True, 0.01)
    self.assertAlmostEqual(controller.update('high', 100.0 / 3.6, True, 0.01), 1.12)
    self.assertFalse(controller.changing)


if __name__ == '__main__':
  unittest.main()
