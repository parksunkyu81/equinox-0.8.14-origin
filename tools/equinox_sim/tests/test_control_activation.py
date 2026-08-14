import unittest
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = Path(__file__).parents[3] / "selfdrive" / "controls" / "lib" / "control_activation.py"
SPEC = spec_from_file_location("control_activation", MODULE_PATH)
ACTIVATION_MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(ACTIVATION_MODULE)
apply_control_activation = ACTIVATION_MODULE.apply_control_activation


def control_message():
  return SimpleNamespace(latActive=False, longActive=False)


def car_state(**overrides):
  values = {
    "steerFaultTemporary": False,
    "steerFaultPermanent": False,
    "vEgo": 4.0,
    "standstill": False,
    "steeringAngleDeg": 25.0,
  }
  values.update(overrides)
  return SimpleNamespace(**values)


def car_params():
  return SimpleNamespace(
    minSteerSpeed=10.0 / 3.6,
    maxSteeringAngleDeg=1000.0,
    openpilotLongitudinalControl=True,
  )


class TestControlActivation(unittest.TestCase):
  def test_reconstructed_car_control_preserves_lateral_activation(self):
    state = car_state()
    params = car_params()

    state_control_message = control_message()
    published_control_message = control_message()
    apply_control_activation(state_control_message, True, state, params, False)
    apply_control_activation(published_control_message, True, state, params, False)

    self.assertTrue(state_control_message.latActive)
    self.assertTrue(published_control_message.latActive)
    self.assertTrue(published_control_message.longActive)

  def test_longitudinal_override_does_not_disable_lateral(self):
    control = control_message()
    apply_control_activation(control, True, car_state(), car_params(), True)

    self.assertTrue(control.latActive)
    self.assertFalse(control.longActive)

  def test_lateral_safety_gates_remain_enforced(self):
    for state in (
      car_state(vEgo=9.9 / 3.6),
      car_state(standstill=True),
      car_state(steerFaultTemporary=True),
      car_state(steerFaultPermanent=True),
      car_state(steeringAngleDeg=1000.0),
    ):
      control = control_message()
      apply_control_activation(control, True, state, car_params(), False)
      self.assertFalse(control.latActive)


if __name__ == "__main__":
  unittest.main()
