#!/usr/bin/env python3
"""Read-only live monitor for strict zero-only comma-pedal recovery."""

import time
from cereal import messaging
from common.params import Params


def param_text(params, key, default="-"):
  raw = params.get(key)
  return raw.decode("utf-8", errors="ignore") if raw is not None else default


def main():
  params = Params()
  print("PedalForceRecoveryEnabled:", param_text(params, "PedalForceRecoveryEnabled", "0"))
  print("0=완전 비활성, 1=엄격한 ACCEL=0 고착 복구")
  print("양수 페달 무효 자동 부스트는 제거되었습니다.")
  print()

  sm = messaging.SubMaster(["controlsState", "carState", "carControl"], poll=["controlsState"])
  last_print = 0.0
  while True:
    sm.update()
    now = time.monotonic()
    if now - last_print < 0.2:
      continue
    last_print = now

    cs = sm["controlsState"]
    car_state = sm["carState"]
    cc = sm["carControl"]
    try:
      raw = float(cs.pedalForceRecoveryRawAccel)
      output = float(cs.pedalForceRecoveryAccel)
      floor = float(cs.pedalForceRecoveryPedalFloor)
      active = bool(cs.pedalForceRecoveryActive)
      duration = float(cs.pedalForceRecoveryDuration)
      count = int(cs.pedalForceRecoveryCount)
      v_kph = float(car_state.vEgo) * 3.6
      a_ego = float(car_state.aEgo)
      gas = float(cc.actuatorsOutput.gas) * 100.0
    except Exception:
      continue

    enabled = param_text(params, "PedalForceRecoveryEnabled", "0")
    print(
      f"enabled={enabled} active={int(active)} count={count} duration={duration:.2f}s "
      f"v={v_kph:.1f}km/h aEgo={a_ego:+.3f} rawAccel={raw:+.3f} "
      f"outputAccel={output:+.3f} pedalFloor={floor*100:.1f}% gasOut={gas:.1f}%"
    )


if __name__ == "__main__":
  main()
