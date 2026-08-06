import csv
from types import SimpleNamespace

from selfdrive.controls.lateral_center_logger import FIELDNAMES, CsvSink, build_row, should_record


class FakeLateralState:
  def __init__(self, torque_state):
    self.torqueState = torque_state

  @staticmethod
  def which():
    return "torqueState"


class FakeSubMaster(dict):
  def __init__(self, messages):
    super().__init__(messages)
    self.valid = {name: True for name in messages}
    self.alive = {name: True for name in messages}
    self.logMonoTime = {name: 9_900_000_000 for name in messages}


def ns(**kwargs):
  return SimpleNamespace(**kwargs)


def fake_sm():
  torque = ns(active=True, error=0.1, errorRate=0.2, p=0.3, i=0.4, d=0.5, f=0.6,
              output=0.7, saturated=False, actualLateralAccel=0.8, desiredLateralAccel=0.9)
  lane = lambda values: ns(y=values)
  model = ns(
    position=ns(x=[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0],
                y=[0.01] * 11),
    laneLines=[lane([0.0] * 11), lane([-1.7] * 11), lane([1.9] * 11), lane([0.0] * 11)],
  )
  plan = ns(
    dPathPoints=[0.02] * 11, useLaneLines=True, laneWidth=3.6, lProb=0.9, rProb=0.8,
    dProb=0.7, mpcSolutionValid=True, laneChangeState="off", laneChangeDirection="none",
    totalCameraOffset=-0.06,
  )
  live_params = ns(angleOffsetDeg=-1.8, angleOffsetAverageDeg=-1.9,
                   angleOffsetFastStd=0.01, angleOffsetAverageStd=0.02,
                   steerRatio=16.4, stiffnessFactor=1.0, roll=0.001)
  live_torque = ns(liveValid=True, latAccelFactorRaw=2.0, latAccelOffsetRaw=0.0,
                   frictionCoefficientRaw=0.25, latAccelFactorFiltered=2.01,
                   latAccelOffsetFiltered=0.0, frictionCoefficientFiltered=0.245,
                   totalBucketPoints=1000)
  controls = ns(active=True, curvature=0.001, angleSteers=-2.0,
                lateralControlState=FakeLateralState(torque))
  car_state = ns(vEgo=20.0, steeringAngleDeg=-2.1, steeringRateDeg=0.2,
                 steeringTorque=0.1, steeringTorqueEps=-0.2, steeringPressed=False)
  car_control = ns(latActive=True, actuators=ns(steer=-0.2), actuatorsOutput=ns(steer=-0.18))
  return FakeSubMaster({
    "modelV2": model, "lateralPlan": plan, "liveParameters": live_params,
    "liveTorqueParameters": live_torque, "controlsState": controls,
    "carState": car_state, "carControl": car_control,
  })


def test_records_only_active_lateral_control():
  sm = fake_sm()
  assert should_record(sm)
  sm["carControl"].latActive = False
  assert should_record(sm)
  sm["controlsState"].active = False
  assert not should_record(sm)


def test_row_contains_lane_center_and_applied_torque():
  row = build_row(fake_sm(), now_ns=10_000_000_000, wall_time=123.0)
  values = dict(zip(FIELDNAMES, row))
  assert len(row) == len(FIELDNAMES)
  assert abs(values["raw_lane_center_y_i0_m"] - 0.1) < 1e-9
  assert abs(values["corrected_lane_center_y_i0_m"] - 0.04) < 1e-9
  assert abs(values["requested_steer"] + 0.2) < 1e-9
  assert abs(values["applied_steer"] + 0.18) < 1e-9
  assert abs(values["angle_offset_average_deg"] + 1.9) < 1e-9
  assert abs(values["lane_width_i0_m"] - 3.6) < 1e-9


def test_csv_sink_writes_header_and_row(tmp_path):
  sink = CsvSink(directory=str(tmp_path))
  row = build_row(fake_sm(), now_ns=10_000_000_000, wall_time=123.0)
  sink.write(row)
  sink.close(sync=False)

  files = list(tmp_path.glob("lateral_center_*.csv"))
  assert len(files) == 1
  with files[0].open(newline="", encoding="utf-8") as stream:
    rows = list(csv.reader(stream))
  assert rows[0] == list(FIELDNAMES)
  assert len(rows[1]) == len(FIELDNAMES)
