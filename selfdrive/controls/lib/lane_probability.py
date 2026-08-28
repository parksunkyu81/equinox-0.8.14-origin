LANE_PROB_BOOST_START = 0.65
LANE_PROB_BOOST_END = 0.95

# The boost is withheld when the model's lane-line head and its end-to-end path
# head contradict each other. Measured at 30 m on 2026-08-26--22-41-50 (night)
# and 2026-08-27--17-05-22 (day): across 4444 frames where combined lane
# confidence was >= 0.90, the two heads never sat more than 0.996 m apart, so a
# gap past 1.0 m is the model disagreeing with itself rather than ordinary lane
# following. In the 0.65-0.90 band -- exactly the range this boost amplifies --
# the same measurement reaches 4.17 m on curves.
LANE_PATH_DISAGREE_X_M = 30.0
LANE_PATH_DISAGREE_M = 1.0
# Gap at which lane reliance is pulled all the way down to the floor. The same
# measurement reached 4.17 m (night) and 4.91 m (day) in the 0.65-0.90 band, so
# 2.5 m is deep inside "the model is contradicting itself" territory.
LANE_PATH_DISAGREE_FULL_M = 2.5
# Reliance floor. The lane path still anchors absolute position in the lane --
# the end-to-end path head is trained on human driving and carries no notion of
# staying centred -- so this never falls to laneless.
LANE_PATH_MIN_TRUST = 0.45

# Maximum permitted increase in lane-path reliance per second. At highway
# speed, a one-frame confidence spike is deliberately admitted more slowly.
LANE_PROB_RISE_RATE_LOW_SPEED = 1.50
LANE_PROB_RISE_RATE_HIGH_SPEED = 0.60
LANE_PROB_RISE_RATE_HIGH_SPEED_MS = 30.0


def _clip(value, lower, upper):
  return max(lower, min(upper, float(value)))


def combined_lane_probability(l_prob, r_prob):
  """Probability that at least one of the two lane lines is usable."""
  left = _clip(l_prob, 0.0, 1.0)
  right = _clip(r_prob, 0.0, 1.0)
  return left + right - left * right


def enhance_lane_probability(raw_prob, enabled=True):
  """Continuously strengthen only high-confidence lane-path probability."""
  raw_prob = _clip(raw_prob, 0.0, 1.0)
  if not enabled or raw_prob <= LANE_PROB_BOOST_START:
    return raw_prob
  if raw_prob >= LANE_PROB_BOOST_END:
    return 1.0

  # Smoothstep has zero slope at both boundaries, avoiding a path-weight jump
  # when confidence crosses the enhancement threshold.
  progress = (raw_prob - LANE_PROB_BOOST_START) / \
             (LANE_PROB_BOOST_END - LANE_PROB_BOOST_START)
  smooth_progress = progress * progress * (3.0 - 2.0 * progress)
  return raw_prob + (1.0 - raw_prob) * smooth_progress


def lane_head_trust_cap(gap_m):
  """Ceiling on lane-path reliance once the two model heads contradict.

  Returns 1.0 (no ceiling) up to LANE_PATH_DISAGREE_M and ramps to
  LANE_PATH_MIN_TRUST by LANE_PATH_DISAGREE_FULL_M.
  """
  span = max(1e-3, LANE_PATH_DISAGREE_FULL_M - LANE_PATH_DISAGREE_M)
  strength = _clip((float(gap_m) - LANE_PATH_DISAGREE_M) / span, 0.0, 1.0)
  return 1.0 + strength * (LANE_PATH_MIN_TRUST - 1.0)


def lane_probability_rise_rate(v_ego):
  speed_ratio = _clip(v_ego, 0.0, LANE_PROB_RISE_RATE_HIGH_SPEED_MS) / \
                LANE_PROB_RISE_RATE_HIGH_SPEED_MS
  return LANE_PROB_RISE_RATE_LOW_SPEED + speed_ratio * \
         (LANE_PROB_RISE_RATE_HIGH_SPEED - LANE_PROB_RISE_RATE_LOW_SPEED)


def limit_lane_probability_rise(previous_prob, target_prob, v_ego, dt):
  """Rate-limit confidence increases; unsafe confidence losses pass at once."""
  previous_prob = _clip(previous_prob, 0.0, 1.0)
  target_prob = _clip(target_prob, 0.0, 1.0)
  if target_prob <= previous_prob:
    return target_prob

  max_rise = lane_probability_rise_rate(v_ego) * max(0.0, float(dt))
  return min(target_prob, previous_prob + max_rise)
