"""Is the car in a traffic jam? Decides which driving the learners may learn from.

In a jam the driver presses the gas to keep cut-ins out and not feel stuck,
and brakes because the queue stops. Neither says anything about the gap or
the pedal strength wanted in flowing traffic, yet on the 11-18 drive (109 of
150 engaged minutes in jams) those presses pinned the follow gap_scale at its
minimum and pushed pedal AUTO toward HIGH -- which then accelerated hard and
ended in a brake press in flowing traffic (44 of 68 brakes there followed an
accel command above 0.5 m/s^2).

Jam: the mean speed over the last JAM_WINDOW_S is below JAM_ENTER_KPH; it
ends above JAM_LEAVE_KPH. One sample a second, so it costs nothing.
"""
from collections import deque

JAM_WINDOW_S = 60
JAM_MIN_SAMPLES = 20          # decide nothing in the first 20 s
JAM_ENTER_KPH = 25.0
JAM_LEAVE_KPH = 30.0


class JamDetector:
  def __init__(self):
    self.samples = deque(maxlen=JAM_WINDOW_S)
    self._acc_t = 0.0
    self.jam = False
    self.mean_kph = 0.0

  def update(self, v_ego, dt):
    self._acc_t += dt
    if self._acc_t >= 1.0:
      self._acc_t -= 1.0
      self.samples.append(max(0.0, float(v_ego)) * 3.6)
      if len(self.samples) >= JAM_MIN_SAMPLES:
        self.mean_kph = sum(self.samples) / len(self.samples)
        if self.jam:
          self.jam = self.mean_kph < JAM_LEAVE_KPH
        else:
          self.jam = self.mean_kph < JAM_ENTER_KPH
    return self.jam
