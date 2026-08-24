#!/usr/bin/env python3
from cereal import car
from common.params import Params
from common.realtime import Priority, config_realtime_process
from system.swaglog import cloudlog
from selfdrive.controls.lib.longitudinal_planner import Planner
from selfdrive.controls.lib.lateral_planner import LateralPlanner
from selfdrive.controls.lib.planner_fault_guard import run_planner_cycle
from selfdrive.hardware import TICI
import cereal.messaging as messaging


def plannerd_thread(sm=None, pm=None):
  config_realtime_process(5 if TICI else 2, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = car.CarParams.from_bytes(params.get("CarParams", block=True))
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  #use_lanelines = not params.get_bool('EndToEndToggle')
  #cloudlog.event("e2e mode", on=use_lanelines)

  longitudinal_planner = Planner(CP)
  lateral_planner = LateralPlanner(CP)
  last_exception_log = {"lateral": 0.0, "longitudinal": 0.0}

  if sm is None:
    sm = messaging.SubMaster(['carState', 'controlsState', 'radarState', 'modelV2', 'liveLocationKalman'],
                             poll=['radarState', 'modelV2'], ignore_avg_freq=['radarState'])

  if pm is None:
    pm = messaging.PubMaster(['longitudinalPlan', 'lateralPlan'])

  while True:
    sm.update()

    if sm.updated['modelV2']:
      last_exception_log["lateral"] = run_planner_cycle(
        "lateral", lateral_planner, sm, pm, last_exception_log["lateral"]
      )
      last_exception_log["longitudinal"] = run_planner_cycle(
        "longitudinal", longitudinal_planner, sm, pm, last_exception_log["longitudinal"]
      )


def main(sm=None, pm=None):
  plannerd_thread(sm, pm)


if __name__ == "__main__":
  main()
