#!/usr/bin/env python3
import csv
import os
import tempfile
import unittest

from selfdrive.controls.lib.pedal_tuning_logger import PedalTuningLogger


class TestPedalTuningLogger(unittest.TestCase):
  @staticmethod
  def log(logger, mono_time_s):
    return logger.log_sample(
      mono_time_s=mono_time_s,
      v_ego_mps=10.0,
      pid_accel_request_mps2=0.8,
      accel_limit_max_mps2=0.5,
      applied_accel_mps2=0.5,
      pedal_command=0.084,
      vehicle_accel_mps2=0.42,
      brake_pressed=False,
      gas_pressed=False,
      controls_active=True,
      adaptive_cruise=True,
    )

  def test_rate_limit_and_columns(self):
    with tempfile.TemporaryDirectory() as directory:
      logger = PedalTuningLogger(directory, sample_hz=10.0)
      self.assertTrue(self.log(logger, 1.00))
      self.assertFalse(self.log(logger, 1.05))
      self.assertTrue(self.log(logger, 1.10))
      logger.close()

      files = os.listdir(directory)
      self.assertEqual(len(files), 1)
      with open(os.path.join(directory, files[0]), newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

      self.assertEqual(len(rows), 2)
      self.assertEqual(tuple(rows[0].keys()), PedalTuningLogger.FIELDNAMES)
      self.assertAlmostEqual(float(rows[0]["v_ego_kph"]), 36.0)
      self.assertAlmostEqual(float(rows[0]["pid_accel_request_mps2"]), 0.8)
      self.assertAlmostEqual(float(rows[0]["accel_limit_max_mps2"]), 0.5)
      self.assertEqual(rows[0]["brake_pressed"], "0")

  def test_rotation_is_bounded(self):
    with tempfile.TemporaryDirectory() as directory:
      logger = PedalTuningLogger(
        directory, sample_hz=1000.0, max_file_bytes=300, max_files=2)
      for index in range(100):
        self.log(logger, index * 0.01)
      logger.close()

      files = [name for name in os.listdir(directory) if name.endswith(".csv")]
      self.assertEqual(len(files), 2)
      self.assertIsNone(logger.last_error)


if __name__ == "__main__":
  unittest.main()
