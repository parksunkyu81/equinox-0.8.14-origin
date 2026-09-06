import math

from common.numpy_fast import clip
from selfdrive.modeld.constants import T_IDXS


CURVE_SPEED_DISABLED = 255.0
# A lower assumed deceleration starts reducing the speed target farther ahead
# of a curve. 0.8 m/s^2 keeps the approach comfortable while the confirmation
# and output filters below continue to reject abrupt model changes.
CURVE_DECEL_MPS2 = 0.8
CURVE_ACTIVATION_MARGIN_MS = 0.5
# This used to read "do not turn a modest cruise-speed reduction into the fixed
# curve target", and with a fixed target that was the right instinct: any curve
# that got past this skip was commanded all the way down to MIN_CURVE_SPEED, so
# only genuinely tight ones could be allowed through. The target is no longer
# fixed (see raw_speed at the end of calculate_curve_speed_details), so the
# reason for rejecting ordinary bends is gone with it.
#
# The skip stays as a floor, not a ceiling: a point is worth considering when it
# asks for less than the driver's own set speed. Whether that adds up to a real
# reduction is already decided once, on the aggregate, by the
# CURVE_ACTIVATION_MARGIN_MS check below -- this per-point test only has to stop
# the loop doing arithmetic on points that cannot matter.
CURVE_DEEP_SPEED_MARGIN_MS = 1.5
# modelV2 is evaluated at 20 Hz, so this requires roughly 0.3 seconds of
# consistent deep-curve evidence before CURV is allowed to engage.
CURVE_CONFIRM_FRAMES = 6
CURVE_INVALID_HOLD_FRAMES = 4
CURVE_TIGHTEN_RC = 0.20
CURVE_RELEASE_RC = 1.50
CURVATURE_FLOOR = 1e-4
CURVE_PLAN_DT = 0.05

MODEL_TRAJECTORY_SIZE = 33
MODEL_CURVE_MIN_TIME_S = 0.50
MODEL_CURVE_MAX_TIME_S = 5.00
MODEL_CURVE_CONTROL_MIN_SPEED_KPH = 10.0
MODEL_CURVE_CONTROL_RELEASE_SPEED_KPH = MODEL_CURVE_CONTROL_MIN_SPEED_KPH - 1.0
# The v0.8.13 model was trained and released with the single road camera. Its
# position path is stable enough for road geometry, but orientationRate can be
# noticeably noisier than the later dual-camera model around a corner entry.
# Use a broad path chord and require the two predictions to agree before a
# large yaw-rate curvature can tighten the speed target.
MODEL_CURVE_MAX_ABS = 0.20
MODEL_CURVE_AGREEMENT_ABS = 0.0025
MODEL_CURVE_AGREEMENT_RATIO = 2.5
MODEL_CURVE_GEOMETRY_WEIGHT = 0.75

# v0.8.13 predicts positions on a time grid. At low speed those points are much
# closer together than in later models, so a fixed index span and 0.75 m chord
# reject valid tight turns. Use a speed-dependent arc-length window and profile
# contract instead. Values are calibrated around the Equinox 10 km/h LKAS gate.
MODEL_CURVE_SPEED_TUNING = (
  # max kph, leg distance, min points, min horizon, confidence, confirm, invalid hold, max curvature
  (8.0,   0.30, 3,  1.5, 0.45, 3, 10, 0.20),
  (10.0,  0.35, 4,  2.5, 0.50, 3, 10, 0.20),
  (15.0,  0.40, 4,  3.0, 0.55, 3, 10, 0.20),
  (25.0,  0.55, 5,  5.0, 0.60, 4,  7, 0.20),
  (40.0,  0.75, 7,  8.0, 0.65, 4,  6, 0.20),
  (60.0,  1.00, 8, 12.0, 0.72, 6,  4, 0.20),
  (999.0, 1.25, 9, 18.0, 0.78, 6,  4, 0.20),
)


def _finite_sequence(values):
  try:
    converted = [float(v) for v in values]
  except (TypeError, ValueError):
    return None
  return converted if all(math.isfinite(v) for v in converted) else None


def _smoothed_abs_curvatures(curvatures):
  """Reject isolated spatial spikes, then smooth without cancelling curve sign changes."""
  values = [abs(float(v)) for v in curvatures]
  if len(values) < 3:
    return values

  despiked = values[:]
  for i in range(1, len(values) - 1):
    despiked[i] = sorted((values[i - 1], values[i], values[i + 1]))[1]
  # Index zero is the measured vehicle curvature and is protected temporally
  # by CURVE_CONFIRM_FRAMES. The far-horizon endpoint has no next neighbor, so
  # use the last three samples to prevent a lone final model point from braking.
  despiked[-1] = sorted(values[-3:])[1]

  smoothed = despiked[:]
  for i in range(1, len(despiked) - 1):
    smoothed[i] = 0.25 * despiked[i - 1] + 0.50 * despiked[i] + 0.25 * despiked[i + 1]
  return smoothed


def _model_curve_tuning(v_ego, control_min_speed_kph=MODEL_CURVE_CONTROL_MIN_SPEED_KPH):
  try:
    speed_kph = max(0.0, float(v_ego) * 3.6)
  except (TypeError, ValueError):
    speed_kph = 0.0
  for (max_kph, leg_distance, min_points, min_horizon, min_confidence,
       confirm_frames, invalid_hold_frames, max_curvature) in MODEL_CURVE_SPEED_TUNING:
    if speed_kph < max_kph:
      return {
        "speed_kph": speed_kph,
        "leg_distance_m": leg_distance,
        "min_points": min_points,
        "min_horizon_m": min_horizon,
        "min_confidence": min_confidence,
        "confirm_frames": confirm_frames,
        "invalid_hold_frames": invalid_hold_frames,
        "max_curvature": max_curvature,
        "control_allowed": speed_kph >= float(control_min_speed_kph),
      }
  raise RuntimeError("unreachable model curve speed tuning")


def _path_arc_distances(position_x, position_y, position_z):
  distances = [0.0]
  for i in range(1, len(position_x)):
    segment = math.sqrt(
      (position_x[i] - position_x[i - 1]) ** 2 +
      (position_y[i] - position_y[i - 1]) ** 2 +
      (position_z[i] - position_z[i - 1]) ** 2)
    distances.append(distances[-1] + segment)
  return distances


def _path_geometry_curvature(position_x, position_y, arc_distances, index,
                             min_leg_distance, max_curvature):
  """Return signed curvature using an adaptive arc-length path window."""
  left = index - 1
  while left > 0 and arc_distances[index] - arc_distances[left] < min_leg_distance:
    left -= 1
  right = index + 1
  while right < len(position_x) - 1 and arc_distances[right] - arc_distances[index] < min_leg_distance:
    right += 1

  left_arc = arc_distances[index] - arc_distances[left] if left >= 0 else 0.0
  right_arc = arc_distances[right] - arc_distances[index] if right < len(position_x) else 0.0
  if left < 0 or right >= len(position_x) or \
     left_arc < min_leg_distance or right_arc < min_leg_distance:
    return None, "short_horizon", 0

  ax, ay = position_x[left], position_y[left]
  bx, by = position_x[index], position_y[index]
  cx, cy = position_x[right], position_y[right]
  ab = math.hypot(bx - ax, by - ay)
  bc = math.hypot(cx - bx, cy - by)
  ac = math.hypot(cx - ax, cy - ay)
  # Arc length selects a stable window. Keep only a small horizontal chord
  # floor so steep grades or very tight turns are not mistaken for no motion.
  if min(ab, bc) < 0.15 or ac < 0.20:
    return None, "short_chord", right - left

  cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
  curvature = 2.0 * cross / max(ab * bc * ac, 1e-6)
  if not math.isfinite(curvature):
    return None, "nonfinite", right - left
  if abs(curvature) > max_curvature:
    return None, "curvature_cap", right - left
  return float(curvature), "ok", right - left


def build_v0813_model_curve_profile(position_t, orientation_rate_z,
                                    velocity_x, velocity_y, velocity_z,
                                    position_x, position_y, position_z,
                                    measured_curvature, v_ego=None,
                                    control_min_speed_kph=MODEL_CURVE_CONTROL_MIN_SPEED_KPH):
  """Build a v0.8.13-compatible curve profile and adapter diagnostics.

  Road-path geometry is the primary signal. orientationRate/velocity is used
  as an independent consistency check, preventing one noisy model head from
  turning a path twitch into a false tight curve.
  """
  if v_ego is None:
    try:
      v_ego = math.hypot(float(velocity_x[0]), float(velocity_y[0]))
    except (TypeError, ValueError, IndexError):
      v_ego = 0.0
  tuning = _model_curve_tuning(v_ego, control_min_speed_kph)
  diag = {
    "model_curve_adapter": "v0.8.13_adaptive_arc_fusion",
    "model_geometry_points": 0,
    "model_yaw_points": 0,
    "model_agree_points": 0,
    "model_disagree_points": 0,
    "model_sign_mismatch_points": 0,
    "model_geometry_only_points": 0,
    "model_short_horizon_points": 0,
    "model_short_chord_points": 0,
    "model_curvature_cap_points": 0,
    "model_adaptive_span_max": 0,
    "model_geometry_max_curvature": 0.0,
    "model_yaw_max_curvature": 0.0,
    "model_profile_horizon_m": 0.0,
    "model_profile_confidence": 0.0,
    "model_profile_min_points": tuning["min_points"],
    "model_profile_min_horizon_m": tuning["min_horizon_m"],
    "model_profile_min_confidence": tuning["min_confidence"],
    "model_profile_leg_distance_m": tuning["leg_distance_m"],
    "model_profile_control_allowed": tuning["control_allowed"],
    "model_confirm_frames": tuning["confirm_frames"],
    "model_invalid_hold_frames": tuning["invalid_hold_frames"],
    "model_profile_valid": False,
  }
  seqs = [_finite_sequence(v) for v in (
    position_t, orientation_rate_z, velocity_x, velocity_y, velocity_z,
    position_x, position_y, position_z,
  )]
  if any(v is None for v in seqs):
    return [], [], [], False, diag

  times, yaw_rates, vxs, vys, _vzs, pxs, pys, pzs = seqs
  lengths = [len(v) for v in seqs]
  if any(n != MODEL_TRAJECTORY_SIZE for n in lengths):
    return [], [], [], False, diag
  if any(times[i] <= times[i - 1] for i in range(1, len(times))):
    return [], [], [], False, diag

  try:
    measured_signed = float(measured_curvature)
  except (TypeError, ValueError):
    measured_signed = 0.0
  if not math.isfinite(measured_signed):
    measured_signed = 0.0
  measured = abs(measured_signed)

  curvatures = [measured]
  profile_times = [0.0]
  distances = [0.0]
  arc_distances = _path_arc_distances(pxs, pys, pzs)

  for i, t in enumerate(times):
    if t < MODEL_CURVE_MIN_TIME_S or t > MODEL_CURVE_MAX_TIME_S:
      continue

    distance = arc_distances[i]
    geometry_curvature, reject_reason, adaptive_span = _path_geometry_curvature(
      pxs, pys, arc_distances, i, tuning["leg_distance_m"], tuning["max_curvature"])
    diag["model_adaptive_span_max"] = max(diag["model_adaptive_span_max"], adaptive_span)
    if geometry_curvature is None:
      reject_key = {
        "short_horizon": "model_short_horizon_points",
        "short_chord": "model_short_chord_points",
        "curvature_cap": "model_curvature_cap_points",
      }.get(reject_reason)
      if reject_key is not None:
        diag[reject_key] += 1
      continue

    geometry_abs = abs(geometry_curvature)
    diag["model_geometry_points"] += 1
    diag["model_geometry_max_curvature"] = max(
      diag["model_geometry_max_curvature"], geometry_abs)

    horizontal_speed = math.hypot(vxs[i], vys[i])
    yaw_curvature = yaw_rates[i] / max(horizontal_speed, 1.0)
    yaw_valid = math.isfinite(yaw_curvature) and abs(yaw_curvature) <= tuning["max_curvature"]
    if yaw_valid:
      yaw_abs = abs(yaw_curvature)
      diag["model_yaw_points"] += 1
      diag["model_yaw_max_curvature"] = max(diag["model_yaw_max_curvature"], yaw_abs)
      smaller = min(geometry_abs, yaw_abs)
      agreement_limit = max(MODEL_CURVE_AGREEMENT_ABS,
                            smaller * (MODEL_CURVE_AGREEMENT_RATIO - 1.0))
      direction_reliable = min(geometry_abs, yaw_abs) > MODEL_CURVE_AGREEMENT_ABS
      same_direction = not direction_reliable or geometry_curvature * yaw_curvature >= 0.0
      agrees = same_direction and abs(geometry_abs - yaw_abs) <= agreement_limit
      if agrees:
        curvature = (MODEL_CURVE_GEOMETRY_WEIGHT * geometry_abs +
                     (1.0 - MODEL_CURVE_GEOMETRY_WEIGHT) * yaw_abs)
        diag["model_agree_points"] += 1
      else:
        # A disagreement is kept at the weaker prediction plus a small noise
        # allowance. Persistent real bends agree in both model heads; isolated
        # path or yaw-rate spikes therefore cannot request a deep-curve target.
        curvature = min(geometry_abs, yaw_abs) + MODEL_CURVE_AGREEMENT_ABS
        curvature = min(curvature, max(geometry_abs, yaw_abs))
        diag["model_disagree_points"] += 1
        if not same_direction:
          diag["model_sign_mismatch_points"] += 1
    else:
      # Geometry remains usable when the velocity head briefly degenerates,
      # but reduce its authority until yaw-rate corroboration returns.
      curvature = MODEL_CURVE_GEOMETRY_WEIGHT * geometry_abs
      diag["model_geometry_only_points"] += 1

    curvatures.append(float(curvature))
    profile_times.append(float(t))
    distances.append(max(0.0, float(distance)))

  geometry_points = diag["model_geometry_points"]
  evidence_points = max(1, geometry_points)
  horizon_m = max(distances, default=0.0)
  agreement_score = (
    diag["model_agree_points"] +
    0.35 * diag["model_disagree_points"] +
    0.15 * diag["model_geometry_only_points"]
  ) / evidence_points
  point_score = min(1.0, geometry_points / max(1.0, float(tuning["min_points"])))
  horizon_score = min(1.0, horizon_m / max(0.1, float(tuning["min_horizon_m"])))
  direction_score = max(
    0.0, 1.0 - diag["model_sign_mismatch_points"] / evidence_points)
  confidence = clip(
    0.40 * point_score + 0.20 * horizon_score +
    0.25 * agreement_score + 0.15 * direction_score,
    0.0, 1.0)

  # Above 40 km/h require geometry/yaw corroboration for at least half of the
  # minimum profile. A geometry-only twitch must never acquire high-speed CURV
  # authority even if it happens to span a long distance.
  high_speed_corroborated = bool(
    tuning["speed_kph"] < 40.0 or
    (diag["model_agree_points"] >= max(3, tuning["min_points"] // 2) and
     diag["model_sign_mismatch_points"] == 0))
  valid = bool(
    geometry_points >= tuning["min_points"] and
    horizon_m >= tuning["min_horizon_m"] and
    confidence >= tuning["min_confidence"] and
    high_speed_corroborated)
  diag["model_profile_horizon_m"] = float(horizon_m)
  diag["model_profile_confidence"] = float(confidence)
  diag["model_high_speed_corroborated"] = high_speed_corroborated
  diag["model_profile_valid"] = bool(valid)
  return curvatures, profile_times, distances, valid, diag


def build_model_curve_profile(position_t, orientation_rate_z,
                              velocity_x, velocity_y, velocity_z,
                              position_x, position_y, position_z,
                              measured_curvature, v_ego=None,
                              control_min_speed_kph=MODEL_CURVE_CONTROL_MIN_SPEED_KPH):
  """Compatibility wrapper retaining the original four-value API."""
  curvatures, times, distances, valid, _diag = build_v0813_model_curve_profile(
    position_t, orientation_rate_z, velocity_x, velocity_y, velocity_z,
    position_x, position_y, position_z, measured_curvature, v_ego=v_ego,
    control_min_speed_kph=control_min_speed_kph)
  return curvatures, times, distances, valid


def calculate_curve_speed_details(curvatures, v_ego, cruise_speed, min_curve_speed,
                                  curvature_factor, time_idxs=T_IDXS, distances=None):
  """Return a present-time speed ceiling and diagnostics for a curvature profile."""
  values = _finite_sequence(curvatures)
  times = _finite_sequence(time_idxs)
  dists = None if distances is None else _finite_sequence(distances)
  try:
    v_ego = float(v_ego)
    cruise_speed = float(cruise_speed)
    min_curve_speed = float(min_curve_speed)
    curvature_factor = float(curvature_factor)
  except (TypeError, ValueError):
    values = None

  diag = {
    "values_valid": False,
    "raw_speed_ms": CURVE_SPEED_DISABLED,
    "selected_index": -1,
    "selected_time_s": None,
    "selected_distance_m": None,
    "selected_curvature": 0.0,
    "max_curvature": 0.0,
    "deep_curve_points": 0,
    "deep_speed_threshold_ms": None,
  }
  if values is not None and times is not None and len(times) >= len(values):
    times = times[:len(values)]
  valid_lengths = bool(
    values is not None and times is not None and len(values) > 0 and len(values) == len(times) and
    (dists is None or len(dists) == len(values)))
  if (not valid_lengths or
      not all(math.isfinite(v) for v in (v_ego, cruise_speed, min_curve_speed, curvature_factor)) or
      v_ego < 0.0 or cruise_speed <= 0.0 or min_curve_speed <= 0.0 or curvature_factor <= 0.0):
    return CURVE_SPEED_DISABLED, False, diag

  a_y_max = clip(2.975 - v_ego * 0.0375, 1.85, 2.975)
  smoothed_curvatures = _smoothed_abs_curvatures(values)
  diag["max_curvature"] = float(max(smoothed_curvatures, default=0.0))
  deep_speed_threshold = max(min_curve_speed + CURVE_DEEP_SPEED_MARGIN_MS, cruise_speed)
  diag["deep_speed_threshold_ms"] = float(deep_speed_threshold)

  allowed_now = CURVE_SPEED_DISABLED
  for i, (curvature, t) in enumerate(zip(smoothed_curvatures, times)):
    calculated_curve_speed = math.sqrt(a_y_max / max(curvature, CURVATURE_FLOOR)) * curvature_factor
    if calculated_curve_speed > deep_speed_threshold:
      continue

    diag["deep_curve_points"] += 1
    curve_speed = max(calculated_curve_speed, min_curve_speed)
    distance = (max(v_ego, 1.0) * max(float(t), 0.0)
                if dists is None else max(float(dists[i]), 0.0))
    speed_now = math.sqrt(curve_speed ** 2 + 2.0 * CURVE_DECEL_MPS2 * distance)
    if speed_now < allowed_now:
      allowed_now = speed_now
      diag["selected_index"] = int(i)
      diag["selected_time_s"] = float(t)
      diag["selected_distance_m"] = float(distance)
      diag["selected_curvature"] = float(curvature)

  diag["values_valid"] = True
  if allowed_now >= cruise_speed - CURVE_ACTIVATION_MARGIN_MS:
    return CURVE_SPEED_DISABLED, True, diag

  # allowed_now is already the answer: the most limiting point's traversal speed
  # carried back to here through CURVE_DECEL_MPS2, i.e. the speed that may be
  # held now to arrive at that point at its safe speed. It used to be computed
  # and then thrown away in favour of the fixed MIN_CURVE_SPEED, which is why
  # every engagement in a drive log reported exactly 30.0 km/h whatever the
  # bend: the output had no gradation at all, so the only curve worth acting on
  # was one tight enough to justify 30, and everything else was skipped above.
  #
  # Command the graded speed instead. A gentle bend now asks for a gentle
  # reduction; a tight one still bottoms out at min_curve_speed, which is what
  # the floor is for. This Equinox drives a gas interceptor with no openpilot
  # brake actuation, so a lower target only releases throttle earlier -- the
  # driver still supplies any braking needed to reach it.
  raw_speed = max(allowed_now, min_curve_speed)
  diag["raw_speed_ms"] = float(raw_speed)
  return float(raw_speed), True, diag


def calculate_curve_speed(curvatures, v_ego, cruise_speed, min_curve_speed,
                          curvature_factor, time_idxs=T_IDXS):
  speed, valid, _ = calculate_curve_speed_details(
    curvatures, v_ego, cruise_speed, min_curve_speed, curvature_factor, time_idxs=time_idxs)
  return speed, valid


class CurveSpeedLimiter:
  """Stateful confirmation and asymmetric filtering for curve speed limits."""

  def __init__(self):
    self.reset()

  def reset(self):
    self.speed_ms = CURVE_SPEED_DISABLED
    self.curve_frames = 0
    self.invalid_frames = 0
    self.last_diag = {
      "source": "disabled",
      "values_valid": False,
      "raw_speed_ms": CURVE_SPEED_DISABLED,
      "filtered_speed_ms": CURVE_SPEED_DISABLED,
    }

  def update(self, curvatures, v_ego, cruise_speed, min_curve_speed,
             curvature_factor, plan_valid=True, time_idxs=T_IDXS,
             distances=None, source="lateralPlan", confirm_frames=None,
             invalid_hold_frames=None):
    raw_speed, values_valid, diag = calculate_curve_speed_details(
      curvatures, v_ego, cruise_speed, min_curve_speed, curvature_factor,
      time_idxs=time_idxs, distances=distances)
    values_valid = bool(plan_valid and values_valid)
    diag["source"] = str(source)
    diag["plan_valid"] = bool(plan_valid)
    confirm_frames = max(1, int(CURVE_CONFIRM_FRAMES if confirm_frames is None else confirm_frames))
    invalid_hold_frames = max(
      0, int(CURVE_INVALID_HOLD_FRAMES if invalid_hold_frames is None else invalid_hold_frames))
    diag["confirm_frames_required"] = confirm_frames
    diag["invalid_hold_frames_allowed"] = invalid_hold_frames

    if not values_valid:
      self.invalid_frames += 1
      self.curve_frames = 0
      if self.invalid_frames <= invalid_hold_frames:
        diag["filtered_speed_ms"] = float(self.speed_ms)
        diag["invalid_hold"] = True
        self.last_diag = diag
        return self.speed_ms
      raw_speed = CURVE_SPEED_DISABLED
    else:
      self.invalid_frames = 0

    curve_detected = raw_speed < CURVE_SPEED_DISABLED
    self.curve_frames = self.curve_frames + 1 if curve_detected else 0

    if self.speed_ms >= CURVE_SPEED_DISABLED:
      if self.curve_frames < confirm_frames:
        diag["filtered_speed_ms"] = CURVE_SPEED_DISABLED
        diag["confirmed"] = False
        self.last_diag = diag
        return CURVE_SPEED_DISABLED
      self.speed_ms = float(cruise_speed)

    target = raw_speed if curve_detected else float(cruise_speed)
    rc = CURVE_TIGHTEN_RC if target < self.speed_ms else CURVE_RELEASE_RC
    alpha = CURVE_PLAN_DT / (rc + CURVE_PLAN_DT)
    self.speed_ms += alpha * (target - self.speed_ms)
    self.speed_ms = max(float(min_curve_speed), min(float(cruise_speed), self.speed_ms))

    if not curve_detected and self.speed_ms >= float(cruise_speed) - CURVE_ACTIVATION_MARGIN_MS:
      self.speed_ms = CURVE_SPEED_DISABLED

    diag["raw_speed_ms"] = float(raw_speed)
    diag["filtered_speed_ms"] = float(self.speed_ms)
    diag["confirmed"] = bool(curve_detected and self.curve_frames >= confirm_frames)
    diag["invalid_hold"] = False
    self.last_diag = diag
    return self.speed_ms
