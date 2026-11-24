"""User-selectable comma-pedal response with driver-style fine tuning."""

import math


DEFAULT_COMMA_PEDAL_PROFILE = 'mid'
COMMA_PEDAL_PROFILE_SPEED_BP_KPH = (0.0, 20.0, 30.0, 60.0, 80.0, 100.0, 130.0)
# Gains on the planner's cruise acceleration ceiling, whose base table
# (longitudinal_planner._A_CRUISE_MAX_V_FOLLOWING) IS the MID profile. So MID
# is 1.0 and the other levels read directly as a share of MID. The ceilings
# they give (m/s^2 of real acceleration, the plan's accel):
#
#   km/h     0     20    30    60    80    100   130
#   LOW    1.15  0.98  0.73  0.61  0.50  0.38  0.32
#   MID    1.32  1.14  0.85  0.72  0.65  0.50  0.42
#   HIGH   1.55  1.36  1.30  1.05  0.85  0.69  0.55
#
# 2026-11-23: MID and LOW lowered at 30-60 km/h (MID 1.05 / 0.80 -> 0.85 /
# 0.72; LOW keeps its share of MID). On the 11-22 drives (98% MID) brake taps
# right after openpilot accelerated in flowing traffic were 0.78/min against
# 0.22 on 11-21, with 0.80 m/s^2 just before the tap (0.58 on 11-21); most
# came while it pulled from 25-45 up to 50-60 km/h with the lead 60-100 m
# away or none. The pedal command at a given speed was the same both days:
# the free road at 30-50 km/h was the difference (283 s of it on 11-22, 29 s
# on 11-21), where MID asked ~1.0 and delivered p90 1.1 m/s^2. 0-20 km/h is
# unchanged (launches braked within 6 s fell to 11% on 11-22), and HIGH keeps
# the same absolute ceilings, so its gain rises where MID's base fell.
#
# Redesigned from the 2026-11-21 evening rlogs. Below 60 km/h LOW and MID are
# what they were (the AUTO votes there were balanced on LOW and leaned "less"
# on MID). Above 60 km/h every level used to deliver the same: the plan did ask
# for its ceiling (feedforward 0.52-0.71 = ceiling x gain), but the speed PID
# sat at its old 2.0 limit and the 1.2 follow cap (aEgo ~0.4 on all levels).
# 61e6b7eb lifted those, so the plan's ceiling is now what binds on the
# highway, and it was low: 0.49 at 80 and 0.38 at 100 km/h on MID, a 25 s run
# from 60 to 100. The levels are spread further apart there and HIGH sits
# 1.24-1.4x MID from 30 km/h up, where it used to be 1.13x and read as MID.
#
# The profile only ever moves this plan ceiling and the rise rate below. It
# never scales the speed PID's limit or the speed-hold level the follow cap
# adds (61e6b7eb), so LOW holds speed up a hill exactly like HIGH; it only
# asks for less acceleration on top of holding.
COMMA_PEDAL_PROFILE_GAINS = {
  'low': (0.87, 0.86, 0.86, 0.85, 0.77, 0.76, 0.76),
  'mid': (1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00),
  'high': (1.17, 1.19, 1.53, 1.46, 1.31, 1.38, 1.31),
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
  # Already a profile name: controlsd passes the same normalized string three
  # times a frame (controller, gain, rise scale), and the decode/strip/lower
  # below would return it unchanged.
  if type(profile) is str and profile in COMMA_PEDAL_PROFILE_GAINS:
    return profile
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
