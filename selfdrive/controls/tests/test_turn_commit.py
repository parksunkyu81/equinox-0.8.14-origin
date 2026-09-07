#!/usr/bin/env python3
"""Turn-commit state machine.

The trigger and the release conditions are the whole feature -- the steering
itself is untouched -- so this covers the gesture being recognised, every
release path, and the two ways a naive implementation goes wrong: releasing on
the first frame because the wheel is still straight, and reading one ordinary
blinker as a double tap.
"""
import unittest

from selfdrive.controls.lib.turn_commit import (TurnCommit, DOUBLE_TAP_GAP_S,
                                                MAX_SPEED_KPH,
                                                MIN_FIRST_SIGNAL_S, TIMEOUT_S,
                                                TURN_STARTED_DEG)

DT = 0.01
KPH = 1.0 / 3.6
# A GM stalk tapped lightly runs three blinks and gives up on its own; the
# measured cluster of those is 2.0-2.2 s.
LIGHT_TAP_S = 2.1
# The shortest gap ordinary signalling produced across 60 segments.
SHORTEST_REAL_GAP_S = 1.65


class TestTurnCommit(unittest.TestCase):
  def setUp(self):
    self.tc = TurnCommit(dt=DT)

  def run_for(self, seconds, engaged=True, kph=15.0, left=False, right=False,
              angle=0.0, pressed=False):
    """Hold one input for a while; returns the final active state."""
    for _ in range(int(round(seconds / DT))):
      self.tc.update(engaged, kph * KPH, left, right, angle, pressed)
    return self.tc.active

  def double_tap(self, kph=15.0, left=True, engaged=True, gap_s=0.4,
                 first_signal_s=MIN_FIRST_SIGNAL_S + 0.5):
    """Signal, off, signal again -- the gesture, ending with the stalk on."""
    right = not left
    self.run_for(first_signal_s, engaged=engaged, kph=kph, left=left, right=right)
    self.run_for(gap_s, engaged=engaged, kph=kph)
    # The second rising edge arms it, and the stalk stays on from here.
    self.tc.update(engaged, kph * KPH, left, right, 0.0, False)

  def test_double_tap_arms(self):
    self.double_tap()
    self.assertTrue(self.tc.active)
    self.assertEqual(self.tc.direction, 'left')

  def test_right_direction(self):
    self.double_tap(left=False)
    self.assertTrue(self.tc.active)
    self.assertEqual(self.tc.direction, 'right')

  def test_single_signal_does_not_arm(self):
    # One ordinary signal is one rising edge, however long it is held.
    self.assertFalse(self.run_for(6.0, left=True))

  def test_light_tap_does_not_open_the_gesture(self):
    # The stalk's lane-change tap ends itself at about 2.1 s. Repeating it must
    # do nothing at all, or the tap a driver already uses to change lanes would
    # commit a corner.
    self.double_tap(first_signal_s=LIGHT_TAP_S)
    self.assertFalse(self.tc.active)

  def test_short_flick_does_not_open_the_gesture(self):
    # Same rule from the other side: anything shorter than a held signal is not
    # the driver signalling, so it cannot be the first half either.
    self.double_tap(first_signal_s=0.3)
    self.assertFalse(self.tc.active)

  def test_two_ordinary_signals_do_not_arm(self):
    # The shortest gap real signalling produced has to stay outside the rule,
    # or a driver cancelling and re-signalling would commit a corner by
    # accident.
    self.double_tap(gap_s=SHORTEST_REAL_GAP_S)
    self.assertFalse(self.tc.active)

  def test_gap_too_long_does_not_arm(self):
    self.double_tap(gap_s=DOUBLE_TAP_GAP_S + 0.3)
    self.assertFalse(self.tc.active)

  def test_gap_is_measured_from_the_signal_going_off(self):
    # However long the first signal was held, only the gap after it counts.
    self.double_tap(first_signal_s=12.0, gap_s=0.3)
    self.assertTrue(self.tc.active)

  def test_opposite_signals_do_not_pair(self):
    # A held left signal followed straight away by a right one is a driver
    # changing their mind, not the second half of a left gesture.
    self.run_for(MIN_FIRST_SIGNAL_S + 0.5, left=True)
    self.run_for(0.3)
    self.run_for(0.3, right=True)
    self.assertFalse(self.tc.active)

  def test_does_not_arm_above_speed(self):
    self.double_tap(kph=MAX_SPEED_KPH + 1.0)
    self.assertFalse(self.tc.active)

  def test_does_not_arm_disengaged(self):
    self.double_tap(engaged=False)
    self.assertFalse(self.tc.active)

  def test_survives_straight_wheel_at_entry(self):
    # The release check on a returning wheel must not fire before the corner
    # has begun, or the mode dies on the frame it arms.
    self.double_tap()
    self.assertTrue(self.run_for(1.0, left=True, angle=0.0))

  def test_releases_when_turn_completes(self):
    self.double_tap()
    self.assertTrue(self.run_for(1.0, left=True, angle=TURN_STARTED_DEG + 40.0))
    self.assertFalse(self.run_for(0.1, left=True, angle=TURN_STARTED_DEG - 5.0))
    self.assertEqual(self.tc.release_reason, 'turn complete')

  def test_releases_on_driver_torque(self):
    self.double_tap()
    self.assertFalse(self.run_for(0.05, left=True, angle=60.0, pressed=True))
    self.assertEqual(self.tc.release_reason, 'driver')

  def test_releases_on_blinker_off(self):
    self.double_tap()
    self.assertFalse(self.run_for(0.05, left=False, angle=60.0))
    self.assertEqual(self.tc.release_reason, 'blinker off')

  def test_releases_over_speed(self):
    self.double_tap()
    self.assertFalse(self.run_for(0.05, left=True, angle=60.0,
                                  kph=MAX_SPEED_KPH + 2.0))
    self.assertEqual(self.tc.release_reason, 'over speed')

  def test_releases_on_disengage(self):
    self.double_tap()
    self.assertFalse(self.run_for(0.05, engaged=False, left=True, angle=60.0))
    self.assertEqual(self.tc.release_reason, 'disengaged')

  def test_releases_on_timeout(self):
    self.double_tap()
    # Held past the timeout at an angle that never returns through the start
    # threshold, so only the clock can end it.
    self.assertFalse(self.run_for(TIMEOUT_S + 0.5, left=True, angle=80.0))
    self.assertEqual(self.tc.release_reason, 'timeout')

  def test_release_reports_direction_once(self):
    self.double_tap()
    # just_released is the edge a caller acts on, so it has to be true on the
    # release frame and false on the next one -- otherwise one corner would be
    # logged repeatedly.
    self.tc.update(True, 15.0 * KPH, True, False, 60.0, True)
    self.assertTrue(self.tc.just_released)
    self.assertEqual(self.tc.release_direction, 'left')
    self.tc.update(True, 15.0 * KPH, False, False, 0.0, False)
    self.assertFalse(self.tc.just_released)
    # The description outlives the edge, so a caller logging a frame later
    # still knows what happened.
    self.assertEqual(self.tc.release_reason, 'driver')

  def test_can_rearm_after_release(self):
    self.double_tap()
    self.run_for(0.05, left=True, angle=60.0, pressed=True)
    self.assertFalse(self.tc.active)
    self.run_for(0.5)
    self.double_tap()
    self.assertTrue(self.tc.active)


if __name__ == '__main__':
  unittest.main()
