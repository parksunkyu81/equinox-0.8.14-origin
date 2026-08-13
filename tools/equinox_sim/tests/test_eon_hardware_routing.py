import ast
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[3]


def import_source(module_path, symbol):
  tree = ast.parse((REPO_ROOT / module_path).read_text(encoding="utf-8"))
  for node in ast.walk(tree):
    if isinstance(node, ast.ImportFrom) and any(alias.name == symbol for alias in node.names):
      return node.module
  return None


class TestEonHardwareRouting(unittest.TestCase):
  def test_realtime_uses_eon_aware_hardware_detection(self):
    self.assertEqual(import_source("common/realtime.py", "PC"), "selfdrive.hardware")

  def test_system_swaglog_uses_eon_aware_hardware_detection(self):
    self.assertEqual(import_source("system/swaglog.py", "PC"), "selfdrive.hardware")


if __name__ == "__main__":
  unittest.main()
