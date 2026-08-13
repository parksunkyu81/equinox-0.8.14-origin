import ast
import importlib.util
from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[3]


def import_source(module_path, symbol):
  tree = ast.parse((REPO_ROOT / module_path).read_text(encoding="utf-8"))
  for node in ast.walk(tree):
    if isinstance(node, ast.ImportFrom) and any(alias.name == symbol for alias in node.names):
      return node.module
  return None


def load_realtime_with_eon_hardware():
  clock = ModuleType("common.clock")
  clock.sec_since_boot = lambda: 0.0
  hardware = ModuleType("selfdrive.hardware")
  hardware.PC = False
  setproctitle = ModuleType("setproctitle")
  setproctitle.getproctitle = lambda: "test"
  spec = importlib.util.spec_from_file_location("eon_realtime_test", REPO_ROOT / "common/realtime.py")
  module = importlib.util.module_from_spec(spec)
  with patch.dict(sys.modules, {
    "common.clock": clock,
    "selfdrive.hardware": hardware,
    "setproctitle": setproctitle,
  }):
    spec.loader.exec_module(module)
  return module


class TestEonHardwareRouting(unittest.TestCase):
  def test_realtime_uses_eon_aware_hardware_detection(self):
    self.assertEqual(import_source("common/realtime.py", "PC"), "selfdrive.hardware")

  def test_system_swaglog_uses_eon_aware_hardware_detection(self):
    self.assertEqual(import_source("system/swaglog.py", "PC"), "selfdrive.hardware")

  def test_integer_core_affinity_is_supported(self):
    realtime = load_realtime_with_eon_hardware()
    with patch.object(realtime.os, "sched_setaffinity", create=True) as set_affinity:
      realtime.set_core_affinity(1)
    set_affinity.assert_called_once_with(0, [1])


if __name__ == "__main__":
  unittest.main()
