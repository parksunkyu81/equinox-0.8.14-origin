"""User-selectable coarse following distance adjustment."""


DEFAULT_FOLLOWING_DISTANCE_PROFILE = 'mid'
FOLLOWING_DISTANCE_PROFILE_OFFSETS = {
  'short': -0.25,
  'mid': 0.0,
  'long': 0.35,
}
FOLLOWING_DISTANCE_OFFSET_SLEW_PER_S = 0.10


def normalize_following_distance_profile(profile):
  try:
    if isinstance(profile, bytes):
      profile = profile.decode('utf8')
    normalized = str(profile).strip().lower()
  except Exception:
    normalized = DEFAULT_FOLLOWING_DISTANCE_PROFILE
  return (normalized if normalized in FOLLOWING_DISTANCE_PROFILE_OFFSETS
          else DEFAULT_FOLLOWING_DISTANCE_PROFILE)


def combine_following_tr(raw_tr, profile_offset_s, learned_offset_s,
                         safety_floor, upper_bound=2.7):
  lower = max(0.0, float(safety_floor))
  selected = float(raw_tr) + float(profile_offset_s) + float(learned_offset_s)
  return max(lower, min(float(upper_bound), selected))


class FollowingDistanceProfileController:
  """Smooth profile changes while a lead is actively being followed."""

  def __init__(self, profile=DEFAULT_FOLLOWING_DISTANCE_PROFILE):
    self.profile = normalize_following_distance_profile(profile)
    self.target_offset_s = FOLLOWING_DISTANCE_PROFILE_OFFSETS[self.profile]
    self.offset_s = self.target_offset_s
    self.changing = False

  def update(self, profile, lead_present, dt):
    self.profile = normalize_following_distance_profile(profile)
    self.target_offset_s = FOLLOWING_DISTANCE_PROFILE_OFFSETS[self.profile]
    if not bool(lead_present):
      self.offset_s = self.target_offset_s
      self.changing = False
      return self.offset_s

    step = FOLLOWING_DISTANCE_OFFSET_SLEW_PER_S * max(0.0, float(dt))
    difference = self.target_offset_s - self.offset_s
    if difference > step:
      difference = step
    elif difference < -step:
      difference = -step
    self.offset_s += difference
    self.changing = abs(self.target_offset_s - self.offset_s) > 1e-6
    return self.offset_s
