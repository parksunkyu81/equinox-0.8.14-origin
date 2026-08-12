import json
import math
import time
from dataclasses import dataclass


STATE_VERSION = 1
SPEED_BIN_CENTERS_KPH = (15.0, 30.0, 50.0, 70.0, 90.0, 110.0, 130.0)
SPEED_BIN_EDGES_KPH = (22.5, 40.0, 60.0, 80.0, 100.0, 120.0)
DEFAULT_DECEL_MS2 = (0.10, 0.13, 0.18, 0.23, 0.29, 0.35, 0.40)

MIN_LEARN_SPEED_KPH = 10.0
MAX_FLAT_ROAD_PITCH_DEG = 2.0
MAX_FALLBACK_ROAD_PITCH_DEG = 1.5
MAX_FALLBACK_DECEL_MS2 = 0.80
PEDAL_ZERO_THRESHOLD = 0.005
SETTLE_TIME_S = 0.60
SAVE_INTERVAL_S = 60.0
CONFIDENCE_DURATION_S = 45.0
CONFIDENCE_EVENTS = 6
READY_CONFIDENCE = 0.65


def _clip(value, lower, upper):
  return max(lower, min(upper, value))


def _finite(value, default=0.0):
  try:
    value = float(value)
  except (TypeError, ValueError):
    return float(default)
  return value if math.isfinite(value) else float(default)


def select_road_pitch(pitch_rad, *, llk_valid, orientation_valid,
                      inputs_ok, sensors_ok, calibration_ok):
  """Select GPS-backed pitch or a conservative inertial-only fallback.

  locationd marks the complete NED orientation invalid without external GPS,
  even though its gravity-observable roll/pitch values remain available. Only
  use the fallback pitch when localization inputs, IMU sensors, and camera
  calibration are all healthy. Yaw is intentionally never used here.
  """
  try:
    pitch_rad = float(pitch_rad)
  except (TypeError, ValueError):
    return 0.0, False, False, "invalid"
  if not math.isfinite(pitch_rad) or not llk_valid:
    return 0.0, False, False, "invalid"

  pitch_deg = math.degrees(pitch_rad)
  if orientation_valid:
    return pitch_deg, True, False, "gps_ned"
  if inputs_ok and sensors_ok and calibration_ok:
    return pitch_deg, True, True, "imu_pitch_fallback"
  return 0.0, False, False, "invalid"


@dataclass(frozen=True)
class NaturalDecelStatus:
  bin_index: int
  speed_kph: float
  decel_ms2: float
  confidence: float
  ready: bool
  eligible: bool
  settle_s: float
  duration_s: float
  events: int


class NaturalDecelLearner:
  """Learns zero-pedal Equinox deceleration without controlling the vehicle."""

  def __init__(self, params=None, writer=None, clock=None):
    if params is None:
      from common.params import Params
      params = Params()
    self.params = params
    self._writer = writer
    self._clock = clock or time.monotonic
    self.estimates = list(DEFAULT_DECEL_MS2)
    self.durations = [0.0] * len(SPEED_BIN_CENTERS_KPH)
    self.events = [0] * len(SPEED_BIN_CENTERS_KPH)
    self._settle_s = 0.0
    self._eligible = False
    self._segment_bins = set()
    self._dirty = False
    self._last_save = self._clock()
    self._load_state()

  @staticmethod
  def speed_bin(v_ego):
    speed_kph = max(0.0, _finite(v_ego)) * 3.6
    for index, edge in enumerate(SPEED_BIN_EDGES_KPH):
      if speed_kph < edge:
        return index
    return len(SPEED_BIN_CENTERS_KPH) - 1

  def _param_text(self, key, default=""):
    try:
      value = self.params.get(key, encoding="utf8")
    except TypeError:
      value = self.params.get(key)
      if isinstance(value, bytes):
        value = value.decode("utf8", errors="ignore")
    except Exception:
      return default
    return default if value is None else str(value)

  def _queue_put(self, key, value):
    value = str(value)
    if self._writer is not None:
      self._writer(key, value)
      return
    from common.params import put_nonblocking
    put_nonblocking(key, value)

  def _load_state(self):
    raw = self._param_text("NaturalDecelState")
    if not raw:
      return
    try:
      state = json.loads(raw)
      if int(state.get("version", 0)) != STATE_VERSION:
        return
      estimates = state.get("estimates", [])
      durations = state.get("durations", [])
      events = state.get("events", [])
      if len(estimates) == len(self.estimates):
        self.estimates = [_clip(_finite(value, default), 0.03, 1.20)
                          for value, default in zip(estimates, DEFAULT_DECEL_MS2)]
      if len(durations) == len(self.durations):
        self.durations = [max(0.0, _finite(value)) for value in durations]
      if len(events) == len(self.events):
        self.events = [max(0, int(value)) for value in events]
    except (TypeError, ValueError, json.JSONDecodeError):
      return

  def _state_dict(self):
    return {
      "version": STATE_VERSION,
      "centers_kph": list(SPEED_BIN_CENTERS_KPH),
      "estimates": [round(value, 6) for value in self.estimates],
      "durations": [round(value, 3) for value in self.durations],
      "events": list(self.events),
    }

  def _save(self, now, force=False):
    if not self._dirty or (not force and now - self._last_save < SAVE_INTERVAL_S):
      return
    self._queue_put("NaturalDecelState",
                    json.dumps(self._state_dict(), separators=(",", ":")))
    self._last_save = now
    self._dirty = False

  def confidence_for_bin(self, index):
    duration_part = min(1.0, self.durations[index] / CONFIDENCE_DURATION_S)
    event_part = min(1.0, self.events[index] / float(CONFIDENCE_EVENTS))
    # Both repeated independent coast events and enough accumulated time are
    # required. One unusually long downhill/downshift segment cannot become a
    # trusted vehicle model by itself.
    return _clip(min(duration_part, event_part), 0.0, 1.0)

  def status(self, v_ego):
    index = self.speed_bin(v_ego)
    confidence = self.confidence_for_bin(index)
    return NaturalDecelStatus(
      bin_index=index,
      speed_kph=max(0.0, _finite(v_ego)) * 3.6,
      decel_ms2=float(self.estimates[index]),
      confidence=float(confidence),
      ready=confidence >= READY_CONFIDENCE,
      eligible=bool(self._eligible and self._settle_s >= SETTLE_TIME_S),
      settle_s=float(self._settle_s),
      duration_s=float(self.durations[index]),
      events=int(self.events[index]),
    )

  def bins_snapshot(self):
    return [{
      "speed_kph": SPEED_BIN_CENTERS_KPH[index],
      "decel_ms2": round(self.estimates[index], 4),
      "confidence": round(self.confidence_for_bin(index), 4),
      "duration_s": round(self.durations[index], 1),
      "events": self.events[index],
    } for index in range(len(self.estimates))]

  def update(self, *, v_ego, a_ego, pedal_output, brake_pressed, gas_pressed,
             context_ok, pitch_deg=0.0, pitch_valid=False,
             pitch_fallback=False, dt=0.01, now=None):
    now = self._clock() if now is None else _finite(now, self._clock())
    dt = _clip(_finite(dt, 0.01), 0.001, 0.2)
    speed_kph = max(0.0, _finite(v_ego)) * 3.6
    pitch_limit = (MAX_FALLBACK_ROAD_PITCH_DEG
                   if pitch_fallback else MAX_FLAT_ROAD_PITCH_DEG)
    flat_road = bool(pitch_valid and abs(_finite(pitch_deg)) <= pitch_limit)
    eligible = bool(context_ok and flat_road and speed_kph >= MIN_LEARN_SPEED_KPH and
                    _finite(pedal_output) <= PEDAL_ZERO_THRESHOLD and
                    not brake_pressed and not gas_pressed)

    if not eligible:
      completed_segment = bool(self._eligible and self._settle_s >= SETTLE_TIME_S)
      self._settle_s = 0.0
      self._eligible = False
      self._segment_bins.clear()
      self._save(now, force=completed_segment)
      return self.status(v_ego)

    self._eligible = True
    self._settle_s += dt
    if self._settle_s < SETTLE_TIME_S:
      self._save(now)
      return self.status(v_ego)

    # Positive acceleration on a nominally flat road is not natural slowdown;
    # it is usually grade, wind, drivetrain creep, or a transition sample.
    measured_decel = -_finite(a_ego)
    max_decel = MAX_FALLBACK_DECEL_MS2 if pitch_fallback else 1.20
    if not 0.03 <= measured_decel <= max_decel:
      self._save(now)
      return self.status(v_ego)

    index = self.speed_bin(v_ego)
    if index not in self._segment_bins:
      self.events[index] += 1
      self._segment_bins.add(index)

    if self.durations[index] <= 0.0:
      self.estimates[index] = measured_decel
    else:
      # Bound each innovation so one downshift or noisy acceleration sample
      # cannot rewrite the vehicle model.
      innovation = _clip(measured_decel - self.estimates[index], -0.12, 0.12)
      alpha = _clip(dt / 30.0, 0.0001, 0.02)
      self.estimates[index] = _clip(
        self.estimates[index] + alpha * innovation, 0.03, 1.20)
    self.durations[index] += dt
    self._dirty = True
    self._save(now)
    return self.status(v_ego)
