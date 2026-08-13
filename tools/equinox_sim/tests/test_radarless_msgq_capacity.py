import re
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[3]


class TestRadarlessMsgqCapacity(unittest.TestCase):
  def test_msgq_has_capacity_for_eon_diagnostics(self):
    header = (REPO_ROOT / "cereal/messaging/msgq.h").read_text(encoding="utf-8")
    match = re.search(r"^#define NUM_READERS (\d+)$", header, re.MULTILINE)
    self.assertIsNotNone(match)
    self.assertGreaterEqual(int(match.group(1)), 32)

  def test_no_ascm_gm_uses_vision_only_radar(self):
    interface = (REPO_ROOT / "selfdrive/car/gm/interface.py").read_text(encoding="utf-8")
    self.assertIn("ret.radarOffCan = candidate in NO_ASCM", interface)


if __name__ == "__main__":
  unittest.main()
