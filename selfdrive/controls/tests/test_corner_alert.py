#!/usr/bin/env python3
"""Corner-entry prompt.

The prompt exists to say "brake for this one", so the tests are about the two
gates that decide that -- how deep the corner is, and how hard the driver has
to brake to meet it -- plus the cases that made the old prompt useless: a lane
change, a gentle bend, and a deep corner already being taken slowly.
"""
import unittest

from selfdrive.controls.lib.curve_speed_limiter import (
  corner_alert_lookahead, CornerAlert, CORNER_ALERT_HOLD_S,
  CORNER_ALERT_MAX_LEAD_S, CORNER_ALERT_MIN_SPEED_KPH, CURVE_PLAN_DT)

KPH = 1.0 / 3.6
N = 33


def profile(curvature, at_distance, v_ego, spread=3):
  """A flat path with one corner at a distance, plus its distance/time axes.

  The corner spans a few points because the smoothing step takes the median of
  each triple: a single-point spike is what it is built to delete, which is the
  lane-change case below.
  """
  step = max(at_distance, 1.0) / 10.0
  dists = [i * step for i in range(N)]
  ks = [0.0] * N
  idx = min(range(N), key=lambda i: abs(dists[i] - at_distance))
  for j in range(idx, min(idx + spread, N)):
    ks[j] = curvature
  times = [d / max(v_ego, 1.0) for d in dists]
  return ks, dists, times


class TestCornerAlertLookahead(unittest.TestCase):
  def look(self, curvature, at_distance, kph, **kw):
    v = kph * KPH
    ks, dists, times = profile(curvature, at_distance, v, **kw)
    return corner_alert_lookahead(ks, v, distances=dists, time_idxs=times)

  def test_deep_corner_taken_fast_fires(self):
    # 0.02 1/m is a ~50 m radius; at 60 kph that is 5.6 m/s2 of lateral demand
    # and needs real braking from 30 m out.
    fired, lat, req, lead = self.look(0.02, 30.0, 60.0)
    self.assertTrue(fired)
    self.assertGreater(lat, 2.0)
    self.assertGreater(req, 0.5)
    self.assertLessEqual(lead, CORNER_ALERT_MAX_LEAD_S)

  def test_same_corner_taken_slowly_does_not_fire(self):
    # The corner is just as deep, but entered at a speed that already suits it
    # there is nothing to brake for. This is the case a lateral-only gate got
    # wrong, and why it fired more often than what it replaced.
    fired, _, req, _ = self.look(0.02, 30.0, 32.0)
    self.assertFalse(fired)
    self.assertLess(req, 0.5)

  def test_gentle_bend_does_not_fire(self):
    # 0.003 1/m at 50 kph is 0.6 m/s2 -- below the worst false trigger in the
    # drive-log audit, let alone a corner.
    self.assertFalse(self.look(0.003, 30.0, 50.0)[0])

  def test_near_straight_does_not_fire(self):
    # 0.0023 1/m, the 435 m radius that fired on the real drive.
    self.assertFalse(self.look(0.0023, 30.0, 40.0)[0])

  def test_lane_change_spike_does_not_fire(self):
    # A lane change is a narrow spike in the predicted path; the median filter
    # removes it before anything can read a corner into it.
    self.assertFalse(self.look(0.05, 30.0, 50.0, spread=1)[0])

  def test_below_speed_floor_does_not_fire(self):
    self.assertFalse(self.look(0.02, 30.0, CORNER_ALERT_MIN_SPEED_KPH - 5.0)[0])

  def test_corner_beyond_the_window_does_not_fire(self):
    beyond = (60.0 * KPH) * (CORNER_ALERT_MAX_LEAD_S + 3.0)
    self.assertFalse(self.look(0.02, beyond, 60.0)[0])

  def test_broken_input(self):
    self.assertFalse(corner_alert_lookahead([], 15.0)[0])
    self.assertFalse(corner_alert_lookahead(None, 15.0)[0])
    self.assertFalse(corner_alert_lookahead([0.01] * N, None)[0])
    self.assertFalse(corner_alert_lookahead([0.01] * N, float('nan'))[0])


class TestCornerAlertHold(unittest.TestCase):
  def setUp(self):
    self.ca = CornerAlert()

  def feed(self, frames, curvature, at_distance, kph, **kw):
    v = kph * KPH
    ks, dists, times = profile(curvature, at_distance, v, **kw)
    for _ in range(frames):
      self.ca.update(ks, v, distances=dists, time_idxs=times)
    return self.ca.active

  def quiet(self, frames):
    flat = [0.0] * N
    for _ in range(frames):
      self.ca.update(flat, 60.0 * KPH, distances=[i * 5.0 for i in range(N)],
                     time_idxs=[i * 0.3 for i in range(N)])
    return self.ca.active

  def test_needs_two_frames(self):
    self.assertFalse(self.feed(1, 0.02, 30.0, 60.0))
    self.assertTrue(self.feed(1, 0.02, 30.0, 60.0))

  def test_holds_after_the_corner_leaves_the_window(self):
    self.feed(2, 0.02, 30.0, 60.0)
    # Replayed episodes were a tenth of a second long without this.
    self.assertTrue(self.quiet(int(0.5 / CURVE_PLAN_DT)))

  def test_clears_after_the_hold(self):
    self.feed(2, 0.02, 30.0, 60.0)
    self.assertFalse(self.quiet(int((CORNER_ALERT_HOLD_S + 0.5) / CURVE_PLAN_DT)))
    self.assertEqual(self.ca.lead_s, None)

  def test_reset_clears_everything(self):
    self.feed(2, 0.02, 30.0, 60.0)
    self.ca.reset()
    self.assertFalse(self.ca.active)
    self.assertEqual(self.ca.req_decel, 0.0)


if __name__ == '__main__':
  unittest.main()
