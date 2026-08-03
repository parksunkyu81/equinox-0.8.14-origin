#!/usr/bin/env python3
import json
import os
import time

from cereal import car, log, messaging
from common.params import Params
from common.realtime import DT_CTRL, sec_since_boot
from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car import crc8_pedal
from selfdrive.car.gm.values import CAR, DBC, CanBus, CruiseButtons
from selfdrive.swaglog import cloudlog
from tools.equinox_sim.vehicle import EquinoxVehicle


SIM_ENV = "EQUINOX_SIMULATOR"
STATUS_DIR = "/tmp/equinox_sim"
STATUS_FILE = os.path.join(STATUS_DIR, "status.json")

PARAM_TARGET_SPEED = "EquinoxSimTargetSpeedKph"
PARAM_IGNITION = "EquinoxSimIgnition"
PARAM_ACCEL_ZERO = "EquinoxSimAccelZero"
PARAM_BRAKE = "EquinoxSimBrakePressed"
PARAM_GAS = "EquinoxSimGasPressed"
PARAM_RESET = "EquinoxSimReset"
PARAM_ENGAGE = "EquinoxSimEngage"


class EquinoxVirtualPanda:
  def __init__(self):
    if os.getenv(SIM_ENV) != "1" or os.getenv("SIMULATION") is None or os.getenv("NOBOARD") is None:
      raise RuntimeError(
        "Equinox simulator requires EQUINOX_SIMULATOR=1, SIMULATION=1 and NOBOARD=1"
      )

    self.params = Params()
    self._put_default(PARAM_TARGET_SPEED, "100")
    self._put_default(PARAM_IGNITION, "1")
    self._put_default(PARAM_ACCEL_ZERO, "0")
    self._put_default(PARAM_BRAKE, "0")
    self._put_default(PARAM_GAS, "0")
    self._put_default(PARAM_RESET, "0")
    self._put_default(PARAM_ENGAGE, "1")

    initial_speed_kph = float(os.getenv("EQUINOX_SIM_INITIAL_SPEED_KPH", "80"))
    self.vehicle = EquinoxVehicle(initial_speed_kph=initial_speed_kph)

    self.pm = messaging.PubMaster(["can", "pandaStates", "peripheralState"])
    self.sm = messaging.SubMaster(["carControl", "controlsState"])
    self.sendcan_sock = messaging.sub_sock("sendcan")

    dbc_name = DBC[CAR.EQUINOX_NR]["pt"]
    self.packer = CANPacker(dbc_name)
    self.output_parser = CANParser(
      dbc_name,
      [
        ("GAS_COMMAND", "GAS_COMMAND"),
        ("ENABLE", "GAS_COMMAND"),
        ("RollingCounter", "ASCMLKASteeringCmd"),
        ("LKASteeringCmd", "ASCMLKASteeringCmd"),
      ],
      [("GAS_COMMAND", 0), ("ASCMLKASteeringCmd", 0)],
      CanBus.POWERTRAIN,
    )

    self.frame = 0
    self.started_at = sec_since_boot()
    self.pedal_command = 0.0
    self.steer_command = 0.0
    self.loopback_counter = 0
    self.gas_sensor_counter = 0
    self.gas_can_frames = 0
    self.steer_can_frames = 0
    self.button_queue = []
    self.last_engage_attempt_frame = -1000
    self.last_resume_frame = -1000
    self.last_status_frame = -100
    self.last_status_time = self.started_at
    self.measured_loop_hz = 0.0
    self.last_vehicle_update_time = sec_since_boot()
    self.cached_commands = None
    self.returned_pedal_frames = []

  def _put_default(self, key, value):
    if self.params.get(key) is None:
      self.params.put(key, value)

  def _get_bool(self, key):
    return self.params.get_bool(key)

  def _get_float(self, key, default):
    raw = self.params.get(key, encoding="utf8")
    try:
      return float(raw) if raw is not None else float(default)
    except ValueError:
      return float(default)

  def _get_int(self, key, default):
    raw = self.params.get(key, encoding="utf8")
    try:
      return int(raw) if raw is not None else int(default)
    except ValueError:
      return int(default)

  def _read_commands(self):
    # Params are file-backed on EON. Polling seven keys at 100 Hz consumed a
    # substantial part of the CAN loop budget; 10 Hz still gives controls a
    # maximum command latency of only 100 ms.
    if self.cached_commands is not None and self.frame % 10 != 0:
      return self.cached_commands

    if self._get_bool(PARAM_RESET):
      self.vehicle.reset()
      self.pedal_command = 0.0
      self.steer_command = 0.0
      self.gas_sensor_counter = 0
      self.button_queue.clear()
      self.params.put_bool(PARAM_RESET, False)
      self.params.put_bool(PARAM_ENGAGE, True)

    if self._get_bool(PARAM_ENGAGE):
      self.params.put_bool(PARAM_ENGAGE, False)
      self.last_engage_attempt_frame = -1000

    fault_mode = min(3, max(0, self._get_int(PARAM_ACCEL_ZERO, 0)))
    self.cached_commands = {
      "target_speed_kph": min(145.0, max(20.0, self._get_float(PARAM_TARGET_SPEED, 100.0))),
      "ignition": self._get_bool(PARAM_IGNITION),
      "fault_mode": fault_mode,
      "fault_accel_zero": fault_mode in (1, 2, 3),
      "fault_recovery_enabled": fault_mode in (2, 3),
      "production_fidelity": fault_mode == 3,
      "brake_pressed": self._get_bool(PARAM_BRAKE),
      "gas_pressed": self._get_bool(PARAM_GAS),
    }
    return self.cached_commands

  def _consume_sendcan(self):
    for raw in messaging.drain_sock_raw(self.sendcan_sock):
      event = messaging.log_from_bytes(raw)
      for can_message in event.sendcan:
        if can_message.address == 512:
          # Match real boardd/Panda transmit receipts: a successful send is
          # returned on the CAN socket with the source bus plus 0x80.
          self.returned_pedal_frames.append([
            can_message.address, can_message.busTime, bytes(can_message.dat),
            int(can_message.src) + 0x80,
          ])
      updated = self.output_parser.update_string(raw, sendcan=True)
      if 512 in updated:
        gas_values = self.output_parser.vl["GAS_COMMAND"]
        self.pedal_command = max(0.0, float(gas_values["GAS_COMMAND"]) / 255.0) \
          if gas_values["ENABLE"] else 0.0
        self.gas_can_frames += 1
      if 384 in updated:
        steer_values = self.output_parser.vl["ASCMLKASteeringCmd"]
        self.loopback_counter = int(steer_values["RollingCounter"])
        self.steer_command = max(-1.0, min(1.0, float(steer_values["LKASteeringCmd"]) / 300.0))
        self.steer_can_frames += 1

  def _controls_enabled(self):
    return bool(self.sm["controlsState"].enabled) if self.sm.rcv_frame["controlsState"] > 0 else False

  def _update_buttons(self, target_speed_kph, brake_pressed):
    controls_enabled = self._controls_enabled()

    if not controls_enabled and not brake_pressed and not self.button_queue and \
       self.frame - self.last_engage_attempt_frame >= 100:
      self.button_queue.extend([CruiseButtons.DECEL_SET, CruiseButtons.DECEL_SET,
                                CruiseButtons.UNPRESS, CruiseButtons.UNPRESS])
      self.last_engage_attempt_frame = self.frame

    current_v_cruise = float(self.sm["controlsState"].vCruise) \
      if self.sm.rcv_frame["controlsState"] > 0 else 0.0
    if controls_enabled and current_v_cruise < target_speed_kph - 1.0 and \
       not self.button_queue and self.frame - self.last_resume_frame >= 30:
      self.button_queue.extend([CruiseButtons.RES_ACCEL, CruiseButtons.RES_ACCEL,
                                CruiseButtons.UNPRESS, CruiseButtons.UNPRESS])
      self.last_resume_frame = self.frame

    return self.button_queue.pop(0) if self.button_queue else CruiseButtons.UNPRESS

  def _build_can(self, button, brake_pressed, gas_pressed):
    speed_kph = self.vehicle.speed_mps * 3.6
    driver_gas = 0.12 if gas_pressed else 0.0
    steering_torque = self.steer_command * 2.0

    # Keep one CAN packet arriving every 10 ms so controlsd continues to run at
    # 100 Hz. Individual vehicle messages are distributed across their required
    # parser timeouts; packing all 15 messages at 100 Hz overloads EON hardware.
    frames = [
      self.packer.make_can_msg("ECMEngineStatus", CanBus.POWERTRAIN, {
        "CruiseMainOn": 0, "Brake_Pressed": int(brake_pressed),
        "Standstill": int(self.vehicle.speed_mps < 0.01), "EngineRPM": 1500,
      }),
      self.packer.make_can_msg("ASCMSteeringButton", CanBus.POWERTRAIN, {
        "ACCButtons": button, "DistanceButton": 0, "LKAButton": 0,
      }),
    ]
    if self.returned_pedal_frames:
      frames.extend(self.returned_pedal_frames)
      self.returned_pedal_frames = []

    if self.frame % 2 == 0:  # 50 Hz
      frames.extend([
        self.packer.make_can_msg("PSCMSteeringAngle", CanBus.POWERTRAIN, {
          "SteeringWheelAngle": self.vehicle.steering_angle_deg,
          "SteeringWheelRate": self.vehicle.steering_rate_deg_s,
        }),
        self.packer.make_can_msg("EBCMBrakePedalPosition", CanBus.POWERTRAIN, {
          "BrakePedalPosition": 80 if brake_pressed else 0,
        }),
        self._make_gas_sensor(driver_gas),
        self.packer.make_can_msg("ASCMLKASteeringCmd", CanBus.LOOPBACK, {
          "RollingCounter": self.loopback_counter,
          "LKASteeringCmd": int(round(self.steer_command * 300.0)),
          "LKASteeringCmdActive": int(self._controls_enabled()),
          "LKASteeringCmdChecksum": 0,
        }),
      ])

    if self.frame % 3 == 0:  # about 33 Hz
      frames.append(self.packer.make_can_msg("AcceleratorPedal2", CanBus.POWERTRAIN, {
        "CruiseState": 1, "AcceleratorPedal2": driver_gas * 254.0,
      }))

    if self.frame % 5 == 0:  # 20 Hz
      frames.extend([
        self.packer.make_can_msg("EBCMWheelSpdFront", CanBus.POWERTRAIN, {
          "FLWheelSpd": speed_kph, "FRWheelSpd": speed_kph,
        }),
        self.packer.make_can_msg("EBCMWheelSpdRear", CanBus.POWERTRAIN, {
          "RLWheelSpd": speed_kph, "RRWheelSpd": speed_kph,
        }),
        self.packer.make_can_msg("EPBStatus", CanBus.POWERTRAIN, {"EPBClosed": 0}),
      ])

    if self.frame % 10 == 0:  # 10 Hz
      frames.extend([
        self.packer.make_can_msg("ECMPRDNL2", CanBus.POWERTRAIN, {
          "PRNDL2": 4, "ManualMode": 0, "TransmissionState": 9,
        }),
        self.packer.make_can_msg("PSCMStatus", CanBus.POWERTRAIN, {
          "LKADriverAppldTrq": 0.0,
          "LKATorqueDelivered": steering_torque,
          "LKATorqueDeliveredStatus": 1,
        }),
        self.packer.make_can_msg("BCMDoorBeltStatus", CanBus.POWERTRAIN, {
          "FrontLeftDoor": 0, "FrontRightDoor": 0,
          "RearLeftDoor": 0, "RearRightDoor": 0,
          "LeftSeatBelt": 1, "RightSeatBelt": 1,
        }),
        self.packer.make_can_msg("ESPStatus", CanBus.POWERTRAIN, {"TractionControlOn": 1}),
      ])

    if self.frame % 100 == 0:  # 1 Hz
      frames.append(self.packer.make_can_msg(
        "BCMTurnSignals", CanBus.POWERTRAIN, {"TurnSignals": 0}
      ))

    return frames

  def _make_gas_sensor(self, driver_gas):
    values = {
      "INTERCEPTOR_GAS": driver_gas * 255.0,
      "INTERCEPTOR_GAS2": driver_gas * 255.0,
      "STATE": 0,
      "COUNTER_PEDAL": self.gas_sensor_counter,
      "CHECKSUM_PEDAL": 0,
    }

    # CHECKSUM_PEDAL is not auto-filled by this openpilot version's packer.
    # Match pedal_checksum() in opendbc: CRC-8 (poly 0xD5, init 0xFF)
    # over bytes 0..4, in reverse byte order.
    unsigned_message = self.packer.make_can_msg("GAS_SENSOR", CanBus.POWERTRAIN, values)
    values["CHECKSUM_PEDAL"] = crc8_pedal(unsigned_message[2][:-1])
    message = self.packer.make_can_msg("GAS_SENSOR", CanBus.POWERTRAIN, values)
    self.gas_sensor_counter = (self.gas_sensor_counter + 1) & 0x0F
    return message

  def _publish_panda(self, ignition):
    panda_msg = messaging.new_message("pandaStates", 1)
    panda = panda_msg.pandaStates[0]
    panda.ignitionLine = bool(ignition)
    panda.ignitionCan = bool(ignition)
    panda.controlsAllowed = True
    panda.gasInterceptorDetected = True
    panda.pandaType = log.PandaState.PandaType.uno
    panda.safetyModel = car.CarParams.SafetyModel.gm
    panda.safetyParam = 0
    panda.alternativeExperience = 1
    panda.harnessStatus = log.PandaState.HarnessStatus.normal
    panda.uptime = int(max(0.0, sec_since_boot() - self.started_at))
    self.pm.send("pandaStates", panda_msg)

    peripheral_msg = messaging.new_message("peripheralState")
    peripheral = peripheral_msg.peripheralState
    peripheral.pandaType = log.PandaState.PandaType.uno
    peripheral.voltage = 12500
    peripheral.current = 0
    peripheral.fanSpeedRpm = 3000
    peripheral.usbPowerMode = log.PeripheralState.UsbPowerMode.cdp
    self.pm.send("peripheralState", peripheral_msg)

  def _write_status(self, commands):
    controls = self.sm["controlsState"]
    controls_seen = self.sm.rcv_frame["controlsState"] > 0
    status = {
      "mode": "EQUINOX VIRTUAL PANDA",
      "speedKph": round(self.vehicle.speed_mps * 3.6, 2),
      "accelMps2": round(self.vehicle.accel_mps2, 3),
      "targetSpeedKph": round(commands["target_speed_kph"], 1),
      "pedalCommand": round(self.pedal_command, 4),
      "steerCommand": round(self.steer_command, 4),
      "controlsEnabled": bool(controls.enabled) if controls_seen else False,
      "vCruiseKph": round(float(controls.vCruise), 1) if controls_seen else 0.0,
      "faultAccelZero": commands["fault_accel_zero"],
      "faultRecoveryEnabled": commands["fault_recovery_enabled"],
      "faultMode": commands["fault_mode"],
      "productionFidelity": commands["production_fidelity"],
      "brakePressed": commands["brake_pressed"],
      "ignition": commands["ignition"],
      "recoveryActive": bool(controls.pedalForceRecoveryActive) if controls_seen else False,
      "recoveryDuration": round(float(controls.pedalForceRecoveryDuration), 2) if controls_seen else 0.0,
      "recoveryCount": int(controls.pedalForceRecoveryCount) if controls_seen else 0,
      "recoveryRawAccel": round(float(controls.pedalForceRecoveryRawAccel), 3) if controls_seen else 0.0,
      "recoveryForcedAccel": round(float(controls.pedalForceRecoveryAccel), 3) if controls_seen else 0.0,
      "targetErrorMps": round(commands["target_speed_kph"] / 3.6 - self.vehicle.speed_mps, 3),
      "gasCanFrames": self.gas_can_frames,
      "steerCanFrames": self.steer_can_frames,
      "distanceM": round(self.vehicle.distance_m, 1),
      "loopHz": round(self.measured_loop_hz, 1),
    }
    os.makedirs(STATUS_DIR, exist_ok=True)
    temp_file = STATUS_FILE + ".tmp"
    with open(temp_file, "w", encoding="utf8") as status_file:
      json.dump(status, status_file, ensure_ascii=False, indent=2)
    os.replace(temp_file, STATUS_FILE)
    print(
      "EquinoxSim "
      f"v={status['speedKph']:6.2f}km/h target={status['targetSpeedKph']:5.1f} "
      f"pedal={status['pedalCommand']:.4f} enabled={status['controlsEnabled']} "
      f"fault={status['faultAccelZero']} recovery={status['recoveryActive']} "
      f"loop={status['loopHz']:.1f}Hz"
    )

  def run(self):
    cloudlog.warning("Starting Equinox virtual Panda bench simulator (NOBOARD)")

    while True:
      loop_started_at = sec_since_boot()
      self.sm.update(0)
      commands = self._read_commands()
      self._consume_sendcan()

      if commands["ignition"]:
        now = sec_since_boot()
        vehicle_dt = max(0.001, min(0.05, now - self.last_vehicle_update_time))
        self.last_vehicle_update_time = now
        self.vehicle.step(
          vehicle_dt,
          self.pedal_command,
          self.steer_command,
          brake_pressed=commands["brake_pressed"],
          driver_gas_pressed=commands["gas_pressed"],
        )

      button = self._update_buttons(commands["target_speed_kph"], commands["brake_pressed"])
      frames = self._build_can(button, commands["brake_pressed"], commands["gas_pressed"])
      self.pm.send("can", can_list_to_can_capnp(frames))

      if self.frame % 50 == 0:
        self._publish_panda(commands["ignition"])

      if self.frame - self.last_status_frame >= 100:
        status_time = sec_since_boot()
        status_frames = self.frame - max(0, self.last_status_frame)
        status_elapsed = status_time - self.last_status_time
        if self.last_status_frame >= 0 and status_elapsed > 0.0:
          self.measured_loop_hz = status_frames / status_elapsed
        self._write_status(commands)
        self.last_status_frame = self.frame
        self.last_status_time = status_time

      self.frame += 1
      # Do not run missed frames faster than real time. Ratekeeper's cumulative
      # schedule catches up after startup stalls and can flood EON at 200-300
      # Hz, evicting other messaging readers. This limiter drops missed ticks.
      loop_elapsed = sec_since_boot() - loop_started_at
      if loop_elapsed < DT_CTRL:
        time.sleep(DT_CTRL - loop_elapsed)


def main():
  EquinoxVirtualPanda().run()


if __name__ == "__main__":
  main()
