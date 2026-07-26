import ast
import os
import unittest


TORQUED_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "locationd", "torqued.py")


class TestTorquedCapnpFields(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    with open(TORQUED_PATH, encoding="utf-8") as source_file:
      cls.source = source_file.read()
    cls.tree = ast.parse(cls.source)

  def test_no_dynamic_capnp_field_probing(self):
    capnp_reader_names = {"CP", "msg", "ao", "tl", "lts"}
    unsafe_calls = []
    for node in ast.walk(self.tree):
      if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name) or node.func.id != "getattr":
        continue
      if node.args and isinstance(node.args[0], ast.Name) and node.args[0].id in capnp_reader_names:
        unsafe_calls.append((node.lineno, node.args[0].id))
    self.assertEqual(unsafe_calls, [])

  def test_only_confirmed_car_state_field_names_remain(self):
    for nonexistent in ("steeringTorqueDriver", "steeringTorqueEPS", "lkasEnabled",
                        "lateralTorqueState", "epsEvt", "epsDamp", "torqueLimits"):
      self.assertNotIn(nonexistent, self.source)

    direct_msg_fields = {
      node.attr for node in ast.walk(self.tree)
      if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "msg"
    }
    self.assertTrue({"steeringTorque", "steeringTorqueEps", "lkasEnable"}.issubset(direct_msg_fields))


if __name__ == "__main__":
  unittest.main()
