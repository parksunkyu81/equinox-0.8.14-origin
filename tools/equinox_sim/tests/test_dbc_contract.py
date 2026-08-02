import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
DBC_PATH = REPO_ROOT / "opendbc" / "gm_global_a_powertrain_generated.dbc"


def dbc_messages():
  messages = {}
  current_message = None
  for line in DBC_PATH.read_text(encoding="utf8").splitlines():
    message_match = re.match(r"BO_\s+\d+\s+(\w+):", line)
    if message_match:
      current_message = message_match.group(1)
      messages[current_message] = set()
      continue
    signal_match = re.match(r"\s+SG_\s+(\w+)\s+:", line)
    if signal_match and current_message is not None:
      messages[current_message].add(signal_match.group(1))
  return messages


class TestEquinoxDbcContract(unittest.TestCase):
  def test_virtual_can_inputs_exist(self):
    expected = {
      "EBCMWheelSpdFront": {"FLWheelSpd", "FRWheelSpd"},
      "EBCMWheelSpdRear": {"RLWheelSpd", "RRWheelSpd"},
      "ECMPRDNL2": {"PRNDL2", "ManualMode", "TransmissionState"},
      "ECMEngineStatus": {"CruiseMainOn", "Brake_Pressed", "Standstill", "EngineRPM"},
      "AcceleratorPedal2": {"CruiseState", "AcceleratorPedal2"},
      "ASCMSteeringButton": {"ACCButtons", "DistanceButton", "LKAButton"},
      "PSCMSteeringAngle": {"SteeringWheelAngle", "SteeringWheelRate"},
      "PSCMStatus": {"LKADriverAppldTrq", "LKATorqueDelivered", "LKATorqueDeliveredStatus"},
      "BCMDoorBeltStatus": {"FrontLeftDoor", "FrontRightDoor", "RearLeftDoor", "RearRightDoor", "LeftSeatBelt"},
      "BCMTurnSignals": {"TurnSignals"},
      "ESPStatus": {"TractionControlOn"},
      "EBCMBrakePedalPosition": {"BrakePedalPosition"},
      "EPBStatus": {"EPBClosed"},
      "GAS_SENSOR": {"INTERCEPTOR_GAS", "INTERCEPTOR_GAS2", "STATE", "COUNTER_PEDAL", "CHECKSUM_PEDAL"},
    }
    actual = dbc_messages()
    for message, signals in expected.items():
      self.assertIn(message, actual)
      self.assertTrue(signals.issubset(actual[message]), f"{message}: {signals - actual[message]}")

  def test_sendcan_feedback_signals_exist(self):
    actual = dbc_messages()
    self.assertTrue({"GAS_COMMAND", "ENABLE"}.issubset(actual["GAS_COMMAND"]))
    self.assertTrue({"RollingCounter", "LKASteeringCmd"}.issubset(actual["ASCMLKASteeringCmd"]))


if __name__ == "__main__":
  unittest.main()

