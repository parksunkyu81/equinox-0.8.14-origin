#!/usr/bin/env python3
"""Read-only live monitor for Equinox staged center-offset learning gates."""

import math
import time
from collections import deque

from cereal import car, messaging
from common.params import Params

BIN_RANGES = ((20.0, 40.0), (40.0, 60.0), (60.0, 100.0))
BIN_NAMES = ("20_40", "40_60", "60_100")
YAW_MAX = (0.008, 0.010, 0.012)
LATACC_MAX = (0.070, 0.085, 0.100)
STEER_MAX = (0.045, 0.055, 0.065)
BOOTSTRAP_MIN_OK = (350, 320, 350)
BOOTSTRAP_MIN_OBS_S = (20.0, 18.0, 20.0)
BOOTSTRAP_MIN_RATIO = 0.82
REFINE_MIN_OK = (700, 800, 1000)
REFINE_MIN_OBS_S = (40.0, 45.0, 60.0)
REFINE_MIN_RATIO = 0.90
ROLL_MAX = math.radians(1.0)
RATE_ERR_ALLOW = 0.02
STEER_MAX_DIAG = 300.0


def get_bin(v_kph):
  for i, (lo, hi) in enumerate(BIN_RANGES):
    if lo <= v_kph < hi or (i == 2 and v_kph == hi):
      return i
  return None


def param_text(params, key, default="-"):
  raw = params.get(key)
  return raw.decode("utf-8", errors="ignore") if raw is not None else default


def saved_count(params, idx):
  try:
    return max(0, int(float(param_text(params, f"TorqueCenterOffset{BIN_NAMES[idx]}Count", "0"))))
  except Exception:
    return 0


def requirements(idx, count):
  if count <= 0:
    return "bootstrap", BOOTSTRAP_MIN_OK[idx], BOOTSTRAP_MIN_OBS_S[idx], BOOTSTRAP_MIN_RATIO
  return "refine", REFINE_MIN_OK[idx], REFINE_MIN_OBS_S[idx], REFINE_MIN_RATIO


def main():
  params = Params()
  cp_raw = params.get("CarParams", block=True)
  cp = car.CarParams.from_bytes(cp_raw)

  print("carFingerprint:", cp.carFingerprint)
  print("IsLiveTorque:", param_text(params, "IsLiveTorque"))
  print("TorqueCenterOffsetEnabled:", param_text(params, "TorqueCenterOffsetEnabled"))
  print("Expected fingerprint: CHEVROLET EQUINOX NO RADAR")
  print("Count is an approved offset-save count, not a raw sample count.")
  print("Count 0: bootstrap 18~20 s; Count >= 1: fresh 40~60 s refinement.")
  print()

  sm = messaging.SubMaster(
    ["carControl", "carState", "liveLocationKalman"],
    poll=["liveLocationKalman"],
  )

  windows = [deque(), deque(), deque()]
  last_saved_counts = [saved_count(params, i) for i in range(3)]
  last_print = 0.0
  previous_applied = None

  while True:
    sm.update()

    now = time.monotonic()
    cc = sm["carControl"]
    cs = sm["carState"]
    llk = sm["liveLocationKalman"]

    try:
      v_ego = float(cs.vEgo)
      v_kph = v_ego * 3.6
      yaw_rate = float(llk.angularVelocityCalibrated.value[2])
      roll = float(llk.orientationNED.value[0])
      lateral_acc = (v_ego * yaw_rate) - (math.sin(roll) * 9.81)
      desired = float(cc.actuators.steer)
      applied = float(cc.actuatorsOutput.steer)
      lat_active = bool(cc.latActive)
      steering_pressed = bool(cs.steeringPressed)
    except Exception:
      continue

    # Mirror torqued.py's fresh-window behavior after every accepted save.
    for i in range(3):
      count_now = saved_count(params, i)
      if count_now != last_saved_counts[i]:
        windows[i].clear()
        last_saved_counts[i] = count_now

    idx = get_bin(v_kph)
    if idx is None:
      current_ok = False
      reasons = ["speed_bin"]
    else:
      des_abs = abs(desired)
      app_abs = abs(applied)
      clip_gap = des_abs - app_abs
      steer_clip = (des_abs >= 0.12) and (clip_gap > max(0.035, 0.18 * des_abs))

      rate_limited = False
      delta_err = abs(desired - applied)
      if previous_applied is not None:
        d = applied - previous_applied
        lim_up = 7.0 / STEER_MAX_DIAG
        lim_dn = 17.0 / STEER_MAX_DIAG
        lim = lim_up if abs(desired) > abs(applied) else lim_dn
        rate_limited = abs(d) > 1e-6 and delta_err > 0.005 and 0.85 * lim <= abs(d) <= 1.25 * lim
      mild_rate_ok = rate_limited and delta_err <= RATE_ERR_ALLOW

      checks = [
        ("fingerprint", str(cp.carFingerprint) == "CHEVROLET EQUINOX NO RADAR"),
        ("IsLiveTorque", param_text(params, "IsLiveTorque", "0") == "1"),
        ("CenterOffsetEnabled", param_text(params, "TorqueCenterOffsetEnabled", "0") == "1"),
        ("latActive", lat_active),
        ("driverSteer", not steering_pressed),
        ("roll", abs(roll) <= ROLL_MAX),
        ("yaw", abs(yaw_rate) <= YAW_MAX[idx]),
        ("latacc", abs(lateral_acc) <= LATACC_MAX[idx]),
        ("desiredSteer", abs(desired) <= STEER_MAX[idx]),
        ("appliedSteer", abs(applied) <= STEER_MAX[idx]),
        ("clip", not steer_clip),
        ("rate", (not rate_limited) or mild_rate_ok),
      ]
      reasons = [name for name, ok in checks if not ok]
      current_ok = not reasons

      windows[idx].append((now, current_ok))
      while windows[idx] and now - windows[idx][0][0] > 90.0:
        windows[idx].popleft()

    previous_applied = applied

    if now - last_print < 0.5:
      continue
    last_print = now

    if idx is None:
      print(f"{v_kph:6.1f} km/h | outside learning bins")
      continue

    total = len(windows[idx])
    ok_count = sum(1 for _, ok in windows[idx] if ok)
    ratio = (ok_count / total) if total else 0.0
    observation = (windows[idx][-1][0] - windows[idx][0][0]) if total >= 2 else 0.0
    count_now = saved_count(params, idx)
    phase, min_ok, min_obs, min_ratio = requirements(idx, count_now)
    effective_min_ok = max(min_ok, int(math.ceil(total * min_ratio)))
    reason_text = ",".join(reasons) if reasons else "PASS"

    print(
      f"{v_kph:6.1f} km/h bin={BIN_NAMES[idx]} phase={phase} "
      f"ok={ok_count}/{total} ratio={ratio:.2f}/{min_ratio:.2f} obs={observation:4.1f}/{min_obs:.0f}s "
      f"need_ok={effective_min_ok} savedCount={count_now} "
      f"roll={math.degrees(roll):+.2f}deg yaw={yaw_rate:+.4f} "
      f"latacc={lateral_acc:+.3f} steer={applied:+.3f} "
      f"block={reason_text}"
    )


if __name__ == "__main__":
  main()
