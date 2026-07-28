#!/usr/bin/env python3
import os
import time

from cereal import car
from common.params import Params
from common.realtime import set_core_affinity
from selfdrive.car.gm.steer_diagnostics import run_gm_lkas_can_blackbox


def main():
  # Keep diagnostic work away from controlsd's dedicated EON core 3 and the
  # model/camera core 2. This process is deliberately not real-time.
  set_core_affinity([0, 1])
  try:
    os.nice(10)
  except OSError:
    pass

  params = Params()
  while True:
    car_params = params.get("CarParams")
    if car_params:
      CP = car.CarParams.from_bytes(car_params)
      break
    time.sleep(0.1)

  # The manager starts this diagnostic process for every car. Stay idle on
  # non-GM vehicles instead of interpreting an unrelated CAN address 0x180.
  if CP.carName != "gm":
    while True:
      time.sleep(60.0)

  run_gm_lkas_can_blackbox()


if __name__ == "__main__":
  main()
