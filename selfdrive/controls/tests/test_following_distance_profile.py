import unittest

from selfdrive.controls.lib.following_distance_profile import (FollowingDistanceProfileController,
                                                               combine_following_tr,
                                                               normalize_following_distance_profile)


class TestFollowingDistanceProfile(unittest.TestCase):
  def test_invalid_and_empty_values_default_to_mid(self):
    self.assertEqual(normalize_following_distance_profile(''), 'mid')
    self.assertEqual(normalize_following_distance_profile('unknown'), 'mid')
    self.assertEqual(normalize_following_distance_profile(b'SHORT'), 'short')

  def test_no_lead_applies_selection_immediately(self):
    controller = FollowingDistanceProfileController('mid')
    self.assertEqual(controller.update('short', False, 0.05), -0.25)
    self.assertEqual(controller.update('long', False, 0.05), 0.35)

  def test_tracked_lead_slews_without_jump(self):
    controller = FollowingDistanceProfileController('mid')
    for _ in range(20):
      offset = controller.update('short', True, 0.05)
    self.assertTrue(controller.changing)
    self.assertAlmostEqual(offset, -0.10, places=6)
    for _ in range(30):
      offset = controller.update('short', True, 0.05)
    self.assertAlmostEqual(offset, -0.25, places=6)
    self.assertFalse(controller.changing)

    controller = FollowingDistanceProfileController('mid')
    for _ in range(70):
      offset = controller.update('long', True, 0.05)
    self.assertAlmostEqual(offset, 0.35, places=6)
    self.assertFalse(controller.changing)

  def test_profile_and_learning_offsets_are_added(self):
    raw_tr = 1.30
    learned = -0.09
    self.assertAlmostEqual(combine_following_tr(raw_tr, -0.25, learned, 0.90), 0.96)
    self.assertAlmostEqual(combine_following_tr(raw_tr, 0.00, learned, 0.90), 1.21)
    self.assertAlmostEqual(combine_following_tr(raw_tr, 0.35, learned, 0.90), 1.56)

  def test_safety_floor_has_final_authority(self):
    self.assertEqual(combine_following_tr(0.90, -0.25, -0.10, 1.10), 1.10)


if __name__ == '__main__':
  unittest.main()
