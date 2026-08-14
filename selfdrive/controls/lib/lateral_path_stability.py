"""Small, deterministic helpers for model-path stability and lane centering."""

from collections import deque

from common.numpy_fast import clip


class PathStabilityMonitor:
  """Detect alternating 20 m path motion without suppressing a real curve entry."""

  def __init__(self, window_frames=40, hold_frames=12):
    self.samples = deque(maxlen=int(window_frames))
    self.hold_frames_max = int(hold_frames)
    self.hold_frames = 0
    self.active = False
    self.range_m = 0.0
    self.flips = 0

  def reset(self):
    self.samples.clear()
    self.hold_frames = 0
    self.active = False
    self.range_m = 0.0
    self.flips = 0

  def update(self, y_20m):
    self.samples.append(float(y_20m))
    values = list(self.samples)
    self.range_m = max(values) - min(values) if len(values) >= 2 else 0.0

    last_direction = 0
    flips = 0
    for previous, current in zip(values[:-1], values[1:]):
      delta = current - previous
      direction = 1 if delta > 0.015 else (-1 if delta < -0.015 else 0)
      if direction and last_direction and direction != last_direction:
        flips += 1
      if direction:
        last_direction = direction
    self.flips = flips

    # A monotonic curve can have a large range. Require alternating movement
    # before declaring instability, with a stricter range for only two flips.
    triggered = ((flips >= 3 and self.range_m >= 0.12) or
                 (flips >= 2 and self.range_m >= 0.30))
    if triggered:
      self.hold_frames = self.hold_frames_max
    elif self.hold_frames > 0:
      self.hold_frames -= 1
    self.active = bool(triggered or self.hold_frames > 0)
    return self.active

  def diagnostics(self):
    return {
      'active': bool(self.active),
      'rangeM': float(self.range_m),
      'flips': int(self.flips),
    }


class LaneCenterCorrection:
  """Slowly correct a persistent model/lane-center residual on long straights."""

  def __init__(self, confirm_seconds=8.0, max_correction=0.15, slew_mps=0.005):
    self.confirm_seconds = float(confirm_seconds)
    self.max_correction = float(max_correction)
    self.slew_mps = float(slew_mps)
    self.correction_m = 0.0
    self.confirmed_seconds = 0.0
    self.last_sign = 0
    self.active = False

  def update(self, residual_m, eligible, dt):
    residual = float(residual_m)
    dt = max(0.0, float(dt))
    sign = 1 if residual > 0.03 else (-1 if residual < -0.03 else 0)
    if eligible and sign:
      if sign == self.last_sign:
        self.confirmed_seconds += dt
      else:
        self.confirmed_seconds = dt
      self.last_sign = sign
    else:
      self.confirmed_seconds = 0.0
      self.last_sign = 0

    confirmed = bool(eligible and sign and self.confirmed_seconds >= self.confirm_seconds)
    target = float(clip(residual, -self.max_correction, self.max_correction)) if confirmed else 0.0
    step = self.slew_mps * dt
    self.correction_m += float(clip(target - self.correction_m, -step, step))
    if abs(self.correction_m) < 1e-6:
      self.correction_m = 0.0
    self.active = bool(confirmed or abs(self.correction_m) > 0.002)
    return float(self.correction_m)
