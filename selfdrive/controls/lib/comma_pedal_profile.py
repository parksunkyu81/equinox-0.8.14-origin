"""User-selectable comma-pedal response with driver-style fine tuning."""

import math


DEFAULT_COMMA_PEDAL_PROFILE = 'mid'
COMMA_PEDAL_PROFILE_SPEED_BP_KPH = (0.0, 60.0, 100.0, 130.0)
# Shaped by speed from the 11-18 rlogs (181 segments, all on LOW): while
# openpilot was accelerating in flowing traffic the driver pressed the gas 5
# times and tapped the brake 0 times below 30 km/h, 8 vs 10 at 30-50, and 0 vs
# 6 above 50 (launch presses within 3 s of a brake release left out). So LOW
# was too weak at low speed, right in the middle and a little strong at speed.
COMMA_PEDAL_PROFILE_GAINS = {
  'low': (0.95, 0.85, 0.80, 0.80),
  'mid': (1.10, 1.00, 0.95, 0.95),
  'high': (1.25, 1.15, 1.08, 1.05),
}
COMMA_PEDAL_PROFILE_SLEW_PER_S = 0.16

# How fast the pedal command may grow, relative to the rise limiter's measured
# rates. The gains above only move where the pedal ends up; the driver feels the
# ramp getting there, and that ramp is owned by the rise limiter. Without this
# the profile changes the plateau by 18% and leaves the onset identical, which
# is why the setting read as doing nothing.
#
# Raised after the 11-18/19 rlogs (364 segments, LOW one day, mostly HIGH the
# next): below 30 km/h every level drew more gas presses than brake taps while
# openpilot accelerated, and the delivered acceleration (p75 0.53-0.56) stayed
# well under the ceiling (~1.0), so the ceiling was not what held it back.
# Above 30 km/h the delivered acceleration was the same on every level.
COMMA_PEDAL_PROFILE_RISE_SCALE = {
  'low': 0.90,
  'mid': 1.15,
  'high': 1.40,
}


def _finite_float(value, default):
  try:
    value = float(value)
    return value if math.isfinite(value) else float(default)
  except (TypeError, ValueError):
    return float(default)


def normalize_comma_pedal_profile(profile):
  try:
    if isinstance(profile, bytes):
      profile = profile.decode('utf8')
    normalized = str(profile).strip().lower()
  except Exception:
    normalized = DEFAULT_COMMA_PEDAL_PROFILE
  return (normalized if normalized in COMMA_PEDAL_PROFILE_GAINS
          else DEFAULT_COMMA_PEDAL_PROFILE)


def comma_pedal_profile_gain(profile, v_ego):
  profile = normalize_comma_pedal_profile(profile)
  speed_kph = max(0.0, _finite_float(v_ego, 0.0) * 3.6)
  gains = COMMA_PEDAL_PROFILE_GAINS[profile]

  if speed_kph <= COMMA_PEDAL_PROFILE_SPEED_BP_KPH[0]:
    return gains[0]
  for index in range(1, len(COMMA_PEDAL_PROFILE_SPEED_BP_KPH)):
    upper_speed = COMMA_PEDAL_PROFILE_SPEED_BP_KPH[index]
    if speed_kph <= upper_speed:
      lower_speed = COMMA_PEDAL_PROFILE_SPEED_BP_KPH[index - 1]
      ratio = (speed_kph - lower_speed) / (upper_speed - lower_speed)
      return gains[index - 1] + ratio * (gains[index] - gains[index - 1])
  return gains[-1]


def comma_pedal_profile_rise_scale(profile):
  return COMMA_PEDAL_PROFILE_RISE_SCALE[normalize_comma_pedal_profile(profile)]


class CommaPedalProfileController:
  """Slew only live profile changes; ordinary speed interpolation stays direct."""

  def __init__(self, profile=DEFAULT_COMMA_PEDAL_PROFILE):
    self.profile = normalize_comma_pedal_profile(profile)
    self.gain = comma_pedal_profile_gain(self.profile, 0.0)
    self.changing = False
    self._initialized = False

  def update(self, profile, v_ego, pedal_active, dt):
    requested_profile = normalize_comma_pedal_profile(profile)
    target_gain = comma_pedal_profile_gain(requested_profile, v_ego)

    if not self._initialized:
      self.profile = requested_profile
      self.gain = target_gain
      self.changing = False
      self._initialized = True
      return self.gain

    profile_changed = requested_profile != self.profile
    self.profile = requested_profile
    if profile_changed:
      self.changing = bool(pedal_active)

    if not bool(pedal_active):
      self.gain = target_gain
      self.changing = False
      return self.gain

    if not self.changing:
      self.gain = target_gain
      return self.gain

    step = COMMA_PEDAL_PROFILE_SLEW_PER_S * max(0.0, _finite_float(dt, 0.0))
    difference = target_gain - self.gain
    if difference > step:
      difference = step
    elif difference < -step:
      difference = -step
    self.gain += difference
    self.changing = abs(target_gain - self.gain) > 1e-6
    return self.gain
