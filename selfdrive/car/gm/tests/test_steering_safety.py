import csv
import sys
import tempfile
import unittest
from pathlib import Path


GM_DIR = Path(__file__).resolve().parents[1]
if str(GM_DIR) not in sys.path:
  sys.path.insert(0, str(GM_DIR))

from gm_lkas_log_check import analyze_rows  # noqa: E402
from steer_diagnostics import GMSteeringDiagnosticLogger, gm_lkas_checksum  # noqa: E402
from steer_scheduler import GMSteeringCommandScheduler  # noqa: E402


class TestGMSteeringCommandScheduler(unittest.TestCase):
  def test_official_loopback_gate_and_counter_cycle(self):
    scheduler = GMSteeringCommandScheduler()

    sent, counter = scheduler.update(0.00, 0, 0)
    self.assertFalse(sent)
    self.assertIsNone(counter)
    self.assertEqual(scheduler.block_reason, "initial_sync")

    sent_counters = []
    loopback_counter = 0
    for frame in range(1, 11):
      if frame in (3, 5, 7, 9):
        loopback_counter = (loopback_counter + 1) % 4
      sent, counter = scheduler.update(frame * 0.01, frame, loopback_counter)
      if sent:
        sent_counters.append(counter)

    self.assertEqual(sent_counters, [1, 2, 3, 0, 1])
    self.assertAlmostEqual(scheduler.last_interval, 0.02, places=6)

  def test_ack_on_due_frame_blocks_same_cycle_send(self):
    scheduler = GMSteeringCommandScheduler()
    scheduler.update(0.00, 0, 0)
    scheduler.update(0.01, 1, 0)

    sent, counter = scheduler.update(0.02, 2, 0)
    self.assertTrue(sent)
    self.assertEqual(counter, 1)

    scheduler.update(0.03, 3, 0)
    sent, counter = scheduler.update(0.04, 4, 1)
    self.assertFalse(sent)
    self.assertIsNone(counter)
    self.assertEqual(scheduler.block_reason, "loopback_changed")

    scheduler.update(0.05, 5, 1)
    sent, counter = scheduler.update(0.06, 6, 1)
    self.assertTrue(sent)
    self.assertEqual(counter, 2)

  def test_missing_ack_never_retransmits_same_counter(self):
    scheduler = GMSteeringCommandScheduler()
    scheduler.update(0.00, 0, 0)
    scheduler.update(0.01, 1, 0)
    self.assertEqual(scheduler.update(0.02, 2, 0), (True, 1))
    scheduler.update(0.03, 3, 0)

    sent, counter = scheduler.update(0.04, 4, 0)
    self.assertFalse(sent)
    self.assertIsNone(counter)
    self.assertEqual(scheduler.block_reason, "unacked")
    self.assertTrue(scheduler.unacked_fault)

    scheduler.update(0.05, 5, 1)
    self.assertEqual(scheduler.update(0.06, 6, 1), (True, 2))

  def test_frame_gate_not_monotonic_deadline_controls_send(self):
    scheduler = GMSteeringCommandScheduler()
    scheduler.update(10.000, 0, 0)
    scheduler.update(10.200, 1, 0)
    self.assertEqual(scheduler.update(10.201, 2, 0), (True, 1))

  def test_catch_up_cannot_send_two_commands_inside_18ms(self):
    scheduler = GMSteeringCommandScheduler()
    scheduler.update(0.000, 0, 0)
    scheduler.update(0.010, 1, 0)
    self.assertEqual(scheduler.update(0.020, 2, 0), (True, 1))
    scheduler.update(0.021, 3, 1)

    sent, counter = scheduler.update(0.022, 4, 1)
    self.assertFalse(sent)
    self.assertIsNone(counter)
    self.assertEqual(scheduler.block_reason, "min_interval")

    scheduler.update(0.030, 5, 1)
    self.assertEqual(scheduler.update(0.040, 6, 1), (True, 2))


class TestGMSteeringDiagnostics(unittest.TestCase):
  @staticmethod
  def _base_row(index, mono_time):
    return {
      "_filename": "memory.csv",
      "_row_index": index + 2,
      "mono_time_s": "{:.3f}".format(mono_time),
      "frame": str(index),
      "command_due": "False",
      "command_sent": "False",
      "command_block_reason": "not_due",
      "command_counter": "",
      "command_torque": "",
      "command_active": "",
      "command_checksum": "",
      "loopback_count": "0",
      "loopback_counters": "",
      "loopback_torques": "",
      "loopback_actives": "",
      "loopback_checksums": "",
      "loopback_changed": "False",
      "gap_fault": "False",
      "unacked_fault": "False",
      "pscm_lkas_status": "0",
      "steer_fault_temporary": "False",
      "steer_fault_permanent": "False",
      "can_valid": "True",
      "queue_drops": "0",
    }

  @classmethod
  def _valid_rows(cls, command_count=20):
    rows = []
    row_index = 0
    for command_index in range(command_count):
      counter = (command_index + 1) % 4
      torque = 0
      active = False
      checksum = gm_lkas_checksum(active, torque, counter)

      command_time = 0.02 + command_index * 0.02
      command_row = cls._base_row(row_index, command_time)
      command_row.update({
        "command_due": "True",
        "command_sent": "True",
        "command_block_reason": "sent",
        "command_counter": str(counter),
        "command_torque": str(torque),
        "command_active": "False",
        "command_checksum": str(checksum),
      })
      rows.append(command_row)
      row_index += 1

      loopback_row = cls._base_row(row_index, command_time + 0.01)
      loopback_row.update({
        "loopback_count": "1",
        "loopback_counters": str(counter),
        "loopback_torques": str(torque),
        "loopback_actives": "0",
        "loopback_checksums": str(checksum),
        "loopback_changed": "True",
      })
      rows.append(loopback_row)
      row_index += 1
    return rows

  def test_checksum_matches_gm_12_bit_formula(self):
    for active in (False, True):
      for torque in (-300, -1, 0, 1, 300):
        for counter in range(4):
          expected = (0x1000 - (int(active) << 11) - (torque & 0x7ff) - counter) & 0xfff
          self.assertEqual(gm_lkas_checksum(active, torque, counter), expected)

  def test_stationary_log_passes_counter_checksum_pairing_and_pscm(self):
    result = analyze_rows(self._valid_rows())
    self.assertEqual(result["errors"], [])
    self.assertEqual(result["command_count"], 20)
    self.assertEqual(result["loopback_count"], 20)
    self.assertEqual(result["paired_loopback_count"], 20)
    self.assertEqual(result["pscm_status_counts"], {0: 40})

  def test_duplicate_unacknowledged_counter_fails(self):
    rows = self._valid_rows()
    second_command = rows[2]
    second_loopback = rows[3]
    repeated_counter = int(rows[0]["command_counter"])
    repeated_checksum = gm_lkas_checksum(False, 0, repeated_counter)
    second_command["command_counter"] = str(repeated_counter)
    second_command["command_checksum"] = str(repeated_checksum)
    second_loopback["loopback_counters"] = str(repeated_counter)
    second_loopback["loopback_checksums"] = str(repeated_checksum)

    result = analyze_rows(rows)
    self.assertTrue(any("command counter" in error for error in result["errors"]))
    self.assertTrue(any("loopback counter" in error for error in result["errors"]))

  def test_bad_checksum_and_pscm_fault_fail(self):
    rows = self._valid_rows()
    rows[4]["command_checksum"] = "123"
    rows[7]["pscm_lkas_status"] = "2"

    result = analyze_rows(rows)
    self.assertTrue(any("command checksum" in error for error in result["errors"]))
    self.assertTrue(any("PSCM temporary" in error for error in result["errors"]))

  def test_rotating_logger_writes_bounded_files_without_blocking_caller(self):
    with tempfile.TemporaryDirectory() as log_dir:
      logger = GMSteeringDiagnosticLogger(
        log_dir=log_dir, max_bytes=1024, max_files=2, queue_size=256)
      for index in range(80):
        counter = index % 4
        logger.log(
          mono_time_s=index * 0.01,
          frame=index,
          command_due=index % 2 == 0,
          command_sent=False,
          command_block_reason="not_due",
          loopbacks=[(counter, 0, False, gm_lkas_checksum(False, 0, counter))],
          pscm_lkas_status=0,
          pscm_torque_delivered=0.0,
          pscm_driver_torque=0.0,
          can_valid=True,
        )
      logger.close(timeout=5.0)

      self.assertFalse(logger.disabled, logger.last_error)
      files = sorted(Path(log_dir).glob("gm_lkas_*.csv"))
      self.assertGreaterEqual(len(files), 1)
      self.assertLessEqual(len(files), 2)
      with files[-1].open(newline="", encoding="utf-8") as log_file:
        rows = list(csv.DictReader(log_file))
      self.assertGreater(len(rows), 0)
      self.assertIn("pscm_torque_delivered", rows[0])
      self.assertEqual(rows[0]["loopback_checksum_valid"], "1")


if __name__ == "__main__":
  unittest.main()
