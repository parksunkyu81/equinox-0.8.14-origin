import json
import os
import tempfile
import unittest

from selfdrive.crashlog import CRASH_LOG_NAME, append_crash_record, parse_android_tombstone, \
                               record_python_exception


TOMBSTONE = """*** *** *** *** *** *** *** *** *** *** *** *** *** *** *** ***
Build fingerprint: 'comma/eon/eon:8.1.0/test'
Revision: '0'
ABI: 'arm'
pid: 1410, tid: 1410, name: boardd  >>> ./boardd <<<
signal 6 (SIGABRT), code -6 (SI_TKILL), fault addr --------
Abort message: 'selfdrive/boardd/main.cc:16: assertion "err == 0" failed'
    r0 00000000

backtrace:
    #00 pc 0001a000  /system/lib/libc.so (abort+63)
    #01 pc 00001000  /data/openpilot/selfdrive/boardd/boardd (main+88)

stack:
    00000000  00000000
"""


class TestCrashLog(unittest.TestCase):
  def test_parse_android_tombstone(self):
    record = parse_android_tombstone(TOMBSTONE, "/data/tombstones/tombstone_00")
    self.assertEqual(record["process"], "boardd")
    self.assertEqual(record["executable"], "./boardd")
    self.assertEqual(record["pid"], 1410)
    self.assertEqual(record["signal"], "SIGABRT")
    self.assertEqual(record["signal_number"], 6)
    self.assertIn("assertion", record["message"])
    self.assertEqual(len(record["backtrace"]), 2)
    self.assertEqual(record["source_file"], "tombstone_00")

  def test_rotation_is_bounded(self):
    with tempfile.TemporaryDirectory() as directory:
      for sequence in range(20):
        append_crash_record({
          "source": "test",
          "process": "boardd",
          "signal": "SIGABRT",
          "message": "x" * 96,
          "sequence": sequence,
          "backtrace": ["#00 test"] * 4,
        }, directory=directory, max_size=512, max_files=3)

      logs = [name for name in os.listdir(directory) if name.startswith(CRASH_LOG_NAME)]
      self.assertLessEqual(len(logs), 3)
      for name in logs:
        path = os.path.join(directory, name)
        self.assertLessEqual(os.path.getsize(path), 512)
        with open(path, encoding="utf-8") as stream:
          for line in stream:
            json.loads(line)

  def test_python_exception_contains_process_and_traceback(self):
    with tempfile.TemporaryDirectory() as directory:
      try:
        raise RuntimeError("test crash")
      except RuntimeError:
        path = record_python_exception("torqued", directory=directory)

      with open(path, encoding="utf-8") as stream:
        record = json.loads(stream.readline())
      self.assertEqual(record["process"], "torqued")
      self.assertEqual(record["signal"], "PYTHON_EXCEPTION")
      self.assertIn("RuntimeError: test crash", record["message"])
      self.assertTrue(any("raise RuntimeError" in line for line in record["backtrace"]))


if __name__ == "__main__":
  unittest.main()
