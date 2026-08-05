"""Shared live-torque configuration and Equinox steering-rate profile.

This module is intentionally dependency-light so controlsd, torqued, and the
GM CarController all interpret the same Params values and safety bounds.
"""
from dataclasses import dataclass

from common.numpy_fast import clip, interp


DEFAULT_LAT_ACCEL_FACTOR = 2.05
DEFAULT_FRICTION = 0.230

# User supplied anchors are intentionally narrower than the controller-side
# absolute clamps. The live learner may move within its own confidence-gated
# envelope, but a malformed starting value must not make steering aggressive.
ANCHOR_LAT_ACCEL_MIN = 1.95
ANCHOR_LAT_ACCEL_MAX = 2.15
ANCHOR_FRICTION_MIN = 0.210
ANCHOR_FRICTION_MAX = 0.255

CONTROLLER_LAT_ACCEL_MIN = 1.75
CONTROLLER_LAT_ACCEL_MAX = 2.42
CONTROLLER_FRICTION_MIN = 0.165
CONTROLLER_FRICTION_MAX = 0.305

EQUINOX_TORQUE_FINGERPRINT = "CHEVROLET EQUINOX NO RADAR"

# Conservative vehicle-side rate profile. The requested map is further capped
# by DynamicSteerDeltaMaxUp/Down and remains subject to panda safety limits.
EQUINOX_DELTA_KPH_BP = [0.0, 10.0, 20.0, 30.0, 35.0, 45.0, 60.0, 80.0, 130.0]
EQUINOX_DELTA_UP_V = [7.0, 7.0, 9.0, 11.0, 12.0, 10.0, 8.0, 7.0, 7.0]
EQUINOX_DELTA_DOWN_V = [17.0, 17.0, 17.0, 17.0, 17.0, 16.0, 15.0, 14.0, 14.0]

DEFAULT_DYNAMIC_DELTA_ENABLED = False
# Keep the stock 7 limit by default until the installed panda safety firmware
# is verified. Raising this Param opts into the map without changing code.
DEFAULT_DYNAMIC_DELTA_MAX_UP = 7
DEFAULT_DYNAMIC_DELTA_MAX_DOWN = 17

PARAM_RELOAD_INTERVAL_S = 0.5


@dataclass(frozen=True)
class TorqueTuningConfig:
  enabled: bool = True
  lat_accel_anchor: float = DEFAULT_LAT_ACCEL_FACTOR
  friction_anchor: float = DEFAULT_FRICTION
  dynamic_delta_enabled: bool = DEFAULT_DYNAMIC_DELTA_ENABLED
  dynamic_delta_max_up: int = DEFAULT_DYNAMIC_DELTA_MAX_UP
  dynamic_delta_max_down: int = DEFAULT_DYNAMIC_DELTA_MAX_DOWN
  directional_comp_enabled: bool = False
  center_offset_enabled: bool = False


def _read_raw(params, name):
  try:
    return params.get(name)
  except Exception:
    return None


def _parse_bool(raw, default):
  if raw is None:
    return bool(default)
  try:
    val = raw.strip().lower()
    if val in (b"1", b"true", b"yes", b"on"):
      return True
    if val in (b"0", b"false", b"no", b"off"):
      return False
  except Exception:
    pass
  return bool(default)


def _parse_float(raw, default, legacy_kind=None):
  if raw is None:
    return float(default)
  try:
    text = raw.decode("utf-8", errors="ignore").strip()
    if not text:
      return float(default)
    val = float(text)
    if legacy_kind == "lat":
      # Legacy forks used 30 -> 3.0 and 205 -> 2.05.
      if abs(val) > 100.0:
        val *= 0.01
      elif abs(val) > 10.0:
        val *= 0.1
    elif legacy_kind == "friction" and abs(val) > 1.0:
      # Legacy forks stored 230 -> 0.230.
      val *= 0.001
    return float(val)
  except Exception:
    return float(default)


def _parse_int(raw, default, lo, hi):
  try:
    val = int(round(float(raw.decode("utf-8", errors="ignore").strip()))) if raw is not None else int(default)
  except Exception:
    val = int(default)
  return int(clip(val, lo, hi))


def read_torque_tuning_config(params, migrate=False):
  enabled = _parse_bool(_read_raw(params, "IsLiveTorque"), True)

  lat = _parse_float(_read_raw(params, "TorqueMaxLatAccel"), DEFAULT_LAT_ACCEL_FACTOR, "lat")
  fric = _parse_float(_read_raw(params, "TorqueFriction"), DEFAULT_FRICTION, "friction")
  if not (ANCHOR_LAT_ACCEL_MIN <= lat <= ANCHOR_LAT_ACCEL_MAX):
    lat = DEFAULT_LAT_ACCEL_FACTOR
  if not (ANCHOR_FRICTION_MIN <= fric <= ANCHOR_FRICTION_MAX):
    fric = DEFAULT_FRICTION

  dynamic_enabled = _parse_bool(_read_raw(params, "DynamicSteerDeltaEnabled"), DEFAULT_DYNAMIC_DELTA_ENABLED)
  max_up = _parse_int(_read_raw(params, "DynamicSteerDeltaMaxUp"), DEFAULT_DYNAMIC_DELTA_MAX_UP, 7, 12)
  max_down = _parse_int(_read_raw(params, "DynamicSteerDeltaMaxDown"), DEFAULT_DYNAMIC_DELTA_MAX_DOWN, 14, 20)
  directional = _parse_bool(_read_raw(params, "TorqueDirectionalCompEnabled"), False)
  center_offset = _parse_bool(_read_raw(params, "TorqueCenterOffsetEnabled"), False)

  cfg = TorqueTuningConfig(
    enabled=enabled,
    lat_accel_anchor=float(lat),
    friction_anchor=float(fric),
    dynamic_delta_enabled=dynamic_enabled,
    dynamic_delta_max_up=max_up,
    dynamic_delta_max_down=max_down,
    directional_comp_enabled=directional,
    center_offset_enabled=center_offset,
  )

  if migrate:
    # Normalize legacy encodings once so every process sees the same readable values.
    try:
      params.put("TorqueMaxLatAccel", ("%.3f" % cfg.lat_accel_anchor).encode("utf-8"))
      params.put("TorqueFriction", ("%.3f" % cfg.friction_anchor).encode("utf-8"))
      params.put("DynamicSteerDeltaEnabled", b"1" if cfg.dynamic_delta_enabled else b"0")
      params.put("DynamicSteerDeltaMaxUp", str(cfg.dynamic_delta_max_up).encode("utf-8"))
      params.put("DynamicSteerDeltaMaxDown", str(cfg.dynamic_delta_max_down).encode("utf-8"))
      params.put("TorqueDirectionalCompEnabled", b"1" if cfg.directional_comp_enabled else b"0")
      params.put("TorqueCenterOffsetEnabled", b"1" if cfg.center_offset_enabled else b"0")
    except Exception:
      pass

  return cfg


def equinox_steer_delta_profile(v_kph, config, steering_pressed=False,
                                 driver_torque=0.0, reversing=False):
  """Return vehicle-side delta-up/down limits for one steering command.

  Driver intervention and torque direction reversal always fall back to the
  stock 7/17 limits. This prevents a fast opposite-direction torque ramp.
  """
  if config is None or not config.dynamic_delta_enabled:
    return 7, 17

  if bool(steering_pressed) or abs(float(driver_torque)) >= 30.0 or bool(reversing):
    return 7, 17

  up = int(round(interp(float(v_kph), EQUINOX_DELTA_KPH_BP, EQUINOX_DELTA_UP_V)))
  down = int(round(interp(float(v_kph), EQUINOX_DELTA_KPH_BP, EQUINOX_DELTA_DOWN_V)))
  up = int(clip(up, 7, int(config.dynamic_delta_max_up)))
  down = int(clip(down, 14, int(config.dynamic_delta_max_down)))
  return up, down
