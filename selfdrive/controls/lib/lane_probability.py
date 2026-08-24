LANE_PROB_BOOST_START = 0.65
LANE_PROB_BOOST_END = 0.95

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
