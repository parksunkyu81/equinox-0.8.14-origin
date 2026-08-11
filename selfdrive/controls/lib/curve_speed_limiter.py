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


def build_model_curve_profile(position_t, orientation_rate_z,
                              velocity_x, velocity_y, velocity_z,
                              position_x, position_y, position_z,
                              measured_curvature):
  """Build a present-plus-future curvature profile from the v0.8.16 model output."""
  seqs = [_finite_sequence(v) for v in (
    position_t, orientation_rate_z, velocity_x, velocity_y, velocity_z,
    position_x, position_y, position_z,
  )]
  if any(v is None for v in seqs):
    return [], [], [], False

  times, yaw_rates, vxs, vys, vzs, pxs, pys, pzs = seqs
  lengths = [len(v) for v in seqs]
  if any(n != MODEL_TRAJECTORY_SIZE for n in lengths):
    return [], [], [], False

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

  for t, yaw_rate, vx, vy, vz, px, py, pz in zip(
      times, yaw_rates, vxs, vys, vzs, pxs, pys, pzs):
    if t < MODEL_CURVE_MIN_TIME_S or t > MODEL_CURVE_MAX_TIME_S:
      continue
    speed = math.sqrt(vx * vx + vy * vy + vz * vz)
    curvature = yaw_rate / max(speed, 1.0)
    distance = math.sqrt((px - p0x) ** 2 + (py - p0y) ** 2 + (pz - p0z) ** 2)
    if not (math.isfinite(curvature) and math.isfinite(distance)):
      return [], [], [], False
    curvatures.append(float(curvature))
    profile_times.append(float(t))
    distances.append(max(0.0, float(distance)))

  # The official 33 point horizon has many samples in this interval. Requiring
  # several prevents a malformed partial model message from enabling slowdown.
  valid = len(curvatures) >= 8
  return curvatures, profile_times, distances, valid


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
