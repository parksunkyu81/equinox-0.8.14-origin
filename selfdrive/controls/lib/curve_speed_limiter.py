import math

from common.numpy_fast import clip
from selfdrive.modeld.constants import T_IDXS


CURVE_SPEED_DISABLED = 255.0
# A lower assumed deceleration starts reducing the speed target farther ahead
# of a curve. 0.8 m/s^2 keeps the approach comfortable while the confirmation
# and output filters below continue to reject abrupt model changes.
CURVE_DECEL_MPS2 = 0.8
CURVE_ACTIVATION_MARGIN_MS = 0.5
# Do not turn a modest cruise-speed reduction into the fixed 40 km/h curve
# target. A curve must first be tight enough that its calculated traversal
# speed is within about 5 km/h of the configured curve target.
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
# The v0.8.13 model was trained and released with the single road camera. Its
# position path is stable enough for road geometry, but orientationRate can be
# noticeably noisier than the later dual-camera model around a corner entry.
# Use a broad path chord and require the two predictions to agree before a
# large yaw-rate curvature can tighten the speed target.
MODEL_CURVE_PATH_INDEX_SPAN = 2
MODEL_CURVE_MIN_CHORD_M = 0.75
MODEL_CURVE_MAX_ABS = 0.20
MODEL_CURVE_AGREEMENT_ABS = 0.0025
MODEL_CURVE_AGREEMENT_RATIO = 2.5
MODEL_CURVE_GEOMETRY_WEIGHT = 0.75


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


def _path_geometry_curvature(position_x, position_y, index):
  """Return signed horizontal curvature from a wide three-point path chord."""
  left = max(0, index - MODEL_CURVE_PATH_INDEX_SPAN)
  right = min(len(position_x) - 1, index + MODEL_CURVE_PATH_INDEX_SPAN)
  if left == index or right == index:
    return None

  ax, ay = position_x[left], position_y[left]
  bx, by = position_x[index], position_y[index]
  cx, cy = position_x[right], position_y[right]
  ab = math.hypot(bx - ax, by - ay)
  bc = math.hypot(cx - bx, cy - by)
  ac = math.hypot(cx - ax, cy - ay)
  if min(ab, bc) < MODEL_CURVE_MIN_CHORD_M or ac < 2.0 * MODEL_CURVE_MIN_CHORD_M:
    return None

  cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
  curvature = 2.0 * cross / max(ab * bc * ac, 1e-6)
  if not math.isfinite(curvature) or abs(curvature) > MODEL_CURVE_MAX_ABS:
    return None
  return float(curvature)


def build_v0813_model_curve_profile(position_t, orientation_rate_z,
                                    velocity_x, velocity_y, velocity_z,
                                    position_x, position_y, position_z,
                                    measured_curvature):
  """Build a v0.8.13-compatible curve profile and adapter diagnostics.

  Road-path geometry is the primary signal. orientationRate/velocity is used
  as an independent consistency check, preventing one noisy model head from
  turning a path twitch into a false tight curve.
  """
  diag = {
    "model_curve_adapter": "v0.8.13_path_yaw_fusion",
    "model_geometry_points": 0,
    "model_yaw_points": 0,
    "model_agree_points": 0,
    "model_disagree_points": 0,
    "model_geometry_only_points": 0,
    "model_geometry_max_curvature": 0.0,
    "model_yaw_max_curvature": 0.0,
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
    measured = abs(float(measured_curvature))
  except (TypeError, ValueError):
    measured = 0.0
  if not math.isfinite(measured):
    measured = 0.0

  curvatures = [measured]
  profile_times = [0.0]
  distances = [0.0]
  p0x, p0y, p0z = pxs[0], pys[0], pzs[0]

  for i, t in enumerate(times):
    if t < MODEL_CURVE_MIN_TIME_S or t > MODEL_CURVE_MAX_TIME_S:
      continue

    distance = math.sqrt((pxs[i] - p0x) ** 2 + (pys[i] - p0y) ** 2 + (pzs[i] - p0z) ** 2)
    geometry_curvature = _path_geometry_curvature(pxs, pys, i)
    if geometry_curvature is None or not math.isfinite(distance):
      continue

    geometry_abs = abs(geometry_curvature)
    diag["model_geometry_points"] += 1
    diag["model_geometry_max_curvature"] = max(
      diag["model_geometry_max_curvature"], geometry_abs)

    horizontal_speed = math.hypot(vxs[i], vys[i])
    yaw_curvature = yaw_rates[i] / max(horizontal_speed, 1.0)
    yaw_valid = math.isfinite(yaw_curvature) and abs(yaw_curvature) <= MODEL_CURVE_MAX_ABS
    if yaw_valid:
      yaw_abs = abs(yaw_curvature)
      diag["model_yaw_points"] += 1
      diag["model_yaw_max_curvature"] = max(diag["model_yaw_max_curvature"], yaw_abs)
      smaller = min(geometry_abs, yaw_abs)
      agreement_limit = max(MODEL_CURVE_AGREEMENT_ABS,
                            smaller * (MODEL_CURVE_AGREEMENT_RATIO - 1.0))
      agrees = abs(geometry_abs - yaw_abs) <= agreement_limit
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
    else:
      # Geometry remains usable when the velocity head briefly degenerates,
      # but reduce its authority until yaw-rate corroboration returns.
      curvature = MODEL_CURVE_GEOMETRY_WEIGHT * geometry_abs
      diag["model_geometry_only_points"] += 1

    curvatures.append(float(curvature))
    profile_times.append(float(t))
    distances.append(max(0.0, float(distance)))

  # Keep the same minimum horizon contract as the original 33-point adapter.
  valid = len(curvatures) >= 8 and diag["model_geometry_points"] >= 7
  diag["model_profile_valid"] = bool(valid)
  return curvatures, profile_times, distances, valid, diag


def build_model_curve_profile(position_t, orientation_rate_z,
                              velocity_x, velocity_y, velocity_z,
                              position_x, position_y, position_z,
                              measured_curvature):
  """Compatibility wrapper retaining the original four-value API."""
  curvatures, times, distances, valid, _diag = build_v0813_model_curve_profile(
    position_t, orientation_rate_z, velocity_x, velocity_y, velocity_z,
    position_x, position_y, position_z, measured_curvature)
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
  deep_speed_threshold = min_curve_speed + CURVE_DEEP_SPEED_MARGIN_MS
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

  # This Equinox uses a gas interceptor without openpilot brake actuation.
  # Once a real curve has crossed the slowdown threshold, command the fixed
  # curve target so longitudinal control releases throttle early; the driver
  # remains responsible for any braking needed to reach that speed.
  raw_speed = min_curve_speed
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
             distances=None, source="lateralPlan"):
    raw_speed, values_valid, diag = calculate_curve_speed_details(
      curvatures, v_ego, cruise_speed, min_curve_speed, curvature_factor,
      time_idxs=time_idxs, distances=distances)
    values_valid = bool(plan_valid and values_valid)
    diag["source"] = str(source)
    diag["plan_valid"] = bool(plan_valid)

    if not values_valid:
      self.invalid_frames += 1
      self.curve_frames = 0
      if self.invalid_frames <= CURVE_INVALID_HOLD_FRAMES:
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
      if self.curve_frames < CURVE_CONFIRM_FRAMES:
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
    diag["confirmed"] = bool(curve_detected and self.curve_frames >= CURVE_CONFIRM_FRAMES)
    diag["invalid_hold"] = False
    self.last_diag = diag
    return self.speed_ms
