import math


def _finite(value):
  try:
    return math.isfinite(float(value))
  except (TypeError, ValueError):
    return False


def _clip(value, lower, upper):
  return max(float(lower), min(float(upper), float(value)))


def _scaled_moment(moment, target_n):
  n = float(moment["n"])
  if n <= 0.0 or not _finite(n):
    raise ValueError("empty moment")
  scale = float(target_n) / n
  return {
    "n": n * scale,
    "sx": float(moment["sx"]) * scale,
    "sy": float(moment["sy"]) * scale,
    "sxx": float(moment["sxx"]) * scale,
    "syy": float(moment["syy"]) * scale,
    "sxy": float(moment["sxy"]) * scale,
  }


def _model_mse(rows, factor, friction, offset):
  factor = float(factor)
  friction = float(friction)
  offset = float(offset)
  sse = 0.0
  total_n = 0.0
  for sign, moment in rows:
    n = moment["n"]
    sx = moment["sx"]
    sy = moment["sy"]
    sxx = moment["sxx"]
    syy = moment["syy"]
    sxy = moment["sxy"]
    intercept = offset - factor * friction * sign
    sse += (syy + factor * factor * sxx + intercept * intercept * n +
            2.0 * factor * intercept * sx - 2.0 * factor * sxy - 2.0 * intercept * sy)
    total_n += n
  return max(0.0, sse / max(total_n, 1.0))


def fit_balanced_common_torque_model(pairs, current_factor, current_friction, current_offset,
                                     factor_bounds, friction_bounds):
  """Fit one bounded torque model to magnitude-matched left/right moments.

  Each side of a steering-magnitude pair receives the smaller side's effective
  point count. This removes route-direction frequency bias without discarding
  the accumulated moment statistics. The center offset is held at the currently
  applied value because Equinox learns that parameter independently on straights.

  The physical model is:
    lateral_accel = factor * steer + offset - factor * friction * sign(steer)

  The constrained quadratic optimum is found analytically by checking the
  unconstrained solution and every factor/friction boundary. No iterative or
  grid optimizer is used in the real-time process.
  """
  diag = {
    "valid": False,
    "reason": "invalid_input",
    "candidate_mse": None,
    "current_mse": None,
    "error_improvement": None,
    "effective_points": 0.0,
    "factor_at_bound": False,
    "friction_at_bound": False,
  }

  try:
    factor_min, factor_max = sorted(float(v) for v in factor_bounds)
    friction_min, friction_max = sorted(float(v) for v in friction_bounds)
    current_factor = float(current_factor)
    current_friction = float(current_friction)
    current_offset = float(current_offset)
    if not all(_finite(v) for v in (factor_min, factor_max, friction_min, friction_max,
                                    current_factor, current_friction, current_offset)):
      return math.nan, math.nan, current_offset, diag
    if factor_min <= 0.0 or friction_min < 0.0 or factor_max < factor_min or friction_max < friction_min:
      return math.nan, math.nan, current_offset, diag

    rows = []
    paired_points = 0.0
    for pair in pairs:
      weight = float(pair["weight"])
      if weight <= 0.0 or not _finite(weight):
        continue
      rows.append((-1.0, _scaled_moment(pair["neg"], weight)))
      rows.append((1.0, _scaled_moment(pair["pos"], weight)))
      paired_points += weight

    if len(rows) < 2 or paired_points <= 0.0:
      diag["reason"] = "no_pairs"
      return math.nan, math.nan, current_offset, diag

    diag["effective_points"] = float(paired_points)
    candidates = []

    def add_candidate(factor, friction):
      if not (_finite(factor) and _finite(friction)):
        return
      factor = float(factor)
      friction = float(friction)
      eps = 1e-9
      if not (factor_min - eps <= factor <= factor_max + eps and
              friction_min - eps <= friction <= friction_max + eps):
        return
      factor = _clip(factor, factor_min, factor_max)
      friction = _clip(friction, friction_min, friction_max)
      candidates.append((_model_mse(rows, factor, friction, current_offset), factor, friction))

    # Unconstrained least squares in factor and q=(factor*friction), using
    # features [steer, -sign(steer)] and target (lateral_accel-offset).
    a11 = sum(moment["sxx"] for _, moment in rows)
    a12 = sum(-sign * moment["sx"] for sign, moment in rows)
    a22 = sum(moment["n"] for _, moment in rows)
    b1 = sum(moment["sxy"] - current_offset * moment["sx"] for _, moment in rows)
    b2 = sum(-sign * (moment["sy"] - current_offset * moment["n"])
             for sign, moment in rows)
    determinant = a11 * a22 - a12 * a12
    if determinant > 1e-12:
      factor = (b1 * a22 - b2 * a12) / determinant
      q_value = (a11 * b2 - a12 * b1) / determinant
      if factor > 1e-9:
        add_candidate(factor, q_value / factor)

    # Friction boundary: with r fixed, solve the one-dimensional factor fit for
    # z=(steer-r*sign), target=(lateral_accel-offset).
    for friction in (friction_min, friction_max):
      denominator = sum(
        moment["sxx"] - 2.0 * friction * sign * moment["sx"] +
        friction * friction * moment["n"] for sign, moment in rows)
      numerator = sum(
        moment["sxy"] - current_offset * moment["sx"] -
        friction * sign * (moment["sy"] - current_offset * moment["n"])
        for sign, moment in rows)
      if denominator > 1e-12:
        add_candidate(_clip(numerator / denominator, factor_min, factor_max), friction)

    # Factor boundary: with factor fixed, solve q=(factor*friction), then clamp
    # the corresponding friction to its safe interval.
    total_n = sum(moment["n"] for _, moment in rows)
    for factor in (factor_min, factor_max):
      signed_x = sum(sign * moment["sx"] for sign, moment in rows)
      signed_y_centered = sum(sign * (moment["sy"] - current_offset * moment["n"])
                              for sign, moment in rows)
      q_value = (factor * signed_x - signed_y_centered) / max(total_n, 1.0)
      add_candidate(factor, _clip(q_value / factor, friction_min, friction_max))

    for factor in (factor_min, factor_max):
      for friction in (friction_min, friction_max):
        add_candidate(factor, friction)

    if not candidates:
      diag["reason"] = "fit_invalid"
      return math.nan, math.nan, current_offset, diag

    candidate_mse, factor, friction = min(candidates, key=lambda value: value[0])
    current_mse = _model_mse(rows, current_factor, current_friction, current_offset)
    improvement = 0.0
    if _finite(current_mse) and current_mse > 1e-12:
      improvement = 1.0 - candidate_mse / current_mse

    bound_eps = 1e-6
    diag.update({
      "valid": True,
      "reason": "ok",
      "candidate_mse": float(candidate_mse),
      "current_mse": float(current_mse),
      "error_improvement": float(improvement),
      "factor_at_bound": bool(abs(factor - factor_min) <= bound_eps or abs(factor - factor_max) <= bound_eps),
      "friction_at_bound": bool(abs(friction - friction_min) <= bound_eps or
                                 abs(friction - friction_max) <= bound_eps),
    })
    return float(factor), float(friction), float(current_offset), diag
  except (KeyError, TypeError, ValueError, ZeroDivisionError, OverflowError):
    diag["reason"] = "exception"
    return math.nan, math.nan, _clip(current_offset if _finite(current_offset) else 0.0, -1.0, 1.0), diag

