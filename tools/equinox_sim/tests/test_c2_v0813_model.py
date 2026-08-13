import hashlib
import tempfile
import unittest
from pathlib import Path

from tools.equinox_sim.c2_v0813_model import file_info, verify_source_stack


class TestC2V0813Model(unittest.TestCase):
  def test_file_info_hashes_content(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      path = Path(tmpdir) / "model.bin"
      path.write_bytes(b"official-model")
      info = file_info(path)
      self.assertTrue(info["present"])
      self.assertEqual(info["size"], 14)
      self.assertEqual(info["sha256"], hashlib.sha256(b"official-model").hexdigest())

  def test_source_stack_rejects_dual_camera_parser(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      root = Path(tmpdir)
      model_dir = root / "selfdrive/modeld/models"
      model_dir.mkdir(parents=True)
      (model_dir / "driving.h").write_text("ModelOutputStopLines wide_frame", encoding="utf-8")
      (model_dir / "driving.cc").write_text("s->m->addExtra", encoding="utf-8")
      (root / "selfdrive/modeld/modeld.cc").write_text("vipc_client_extra", encoding="utf-8")
      with self.assertRaises(RuntimeError):
        verify_source_stack(root)

  def test_source_stack_accepts_v0813_layout(self):
    with tempfile.TemporaryDirectory() as tmpdir:
      root = Path(tmpdir)
      model_dir = root / "selfdrive/modeld/models"
      model_dir.mkdir(parents=True)
      (model_dir / "driving.h").write_text("struct ModelOutput {};", encoding="utf-8")
      (model_dir / "driving.cc").write_text("USE_GPU_RUNTIME, false", encoding="utf-8")
      (root / "selfdrive/modeld/modeld.cc").write_text("vipc_client", encoding="utf-8")
      checks = verify_source_stack(root)
      self.assertTrue(all(checks.values()))


if __name__ == "__main__":
  unittest.main()
