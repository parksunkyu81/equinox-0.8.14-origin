import unittest
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


MODULE_PATH = Path(__file__).parents[3] / "selfdrive" / "car" / "gm" / "steer_scheduler.py"
SPEC = spec_from_file_location("gm_steer_scheduler", MODULE_PATH)
SCHEDULER_MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(SCHEDULER_MODULE)
GMSteeringCommandScheduler = SCHEDULER_MODULE.GMSteeringCommandScheduler


class TestGMSteeringCommandScheduler(unittest.TestCase):
  def test_loopback_on_every_due_frame_does_not_starve_commands(self):
    scheduler = GMSteeringCommandScheduler()
    sent = []
    loopback = 0

    # Reproduce the device timing: loopback changes on every even 100 Hz frame,
    # exactly where the old frame-parity scheduler attempted to transmit.
    for frame in range(21):
      if frame and frame % 2 == 0:
        loopback = (loopback + 1) % 4
      command_sent, counter = scheduler.update(frame * 0.01, loopback)
      if command_sent:
        sent.append((frame, counter))

    self.assertEqual([frame for frame, _ in sent], list(range(0, 21, 2)))

  def test_missing_loopback_retries_counter_without_gap(self):
    scheduler = GMSteeringCommandScheduler()

    self.assertEqual(scheduler.update(0.00, 2), (True, 3))
    self.assertFalse(scheduler.update(0.01, 2)[0])
    self.assertEqual(scheduler.update(0.02, 2), (True, 3))
    self.assertFalse(scheduler.loopback_acked)

    self.assertEqual(scheduler.update(0.04, 3), (True, 0))
    self.assertTrue(scheduler.loopback_acked)

  def test_delayed_control_loop_does_not_emit_catchup_burst(self):
    scheduler = GMSteeringCommandScheduler()

    self.assertTrue(scheduler.update(1.00, 0)[0])
    self.assertTrue(scheduler.update(1.10, 1)[0])
    self.assertTrue(scheduler.gap_fault)
    self.assertFalse(scheduler.update(1.11, 2)[0])
    self.assertTrue(scheduler.update(1.12, 2)[0])


if __name__ == "__main__":
  unittest.main()
