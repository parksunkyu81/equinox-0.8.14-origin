import importlib.util
import unittest
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / "lib" / "pedal_force_recovery.py"
spec = importlib.util.spec_from_file_location("pedal_force_recovery", MODULE_PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class TestPedalForceRecoveryStrict(unittest.TestCase):
  def setUp(self):
    self.r = mod.PedalForceRecovery(dt=0.01)

  def feed_prior(self, frames=30, accel=0.15, v_ego=25.0, error=0.8):
    for _ in range(frames):
      out = self.r.update(True, accel, vehicle_accel=0.05,
                          speed_error=error, v_ego=v_ego)
      self.assertEqual(out, accel)

  def test_positive_ineffective_never_boosts(self):
    for i in range(400):
      raw = 0.05
      out = self.r.update(True, raw, vehicle_accel=-0.20,
                          speed_error=1.0 + i * 0.001, v_ego=25.0 - i * 0.001)
      self.assertEqual(out, raw)
      self.assertFalse(self.r.active)

  def test_normal_zero_coast_does_not_trigger(self):
    self.feed_prior()
    for i in range(150):
      out = self.r.update(True, 0.0, vehicle_accel=-0.02,
                          speed_error=0.8 + i * 0.0002, v_ego=25.0 - i * 0.0001)
      self.assertEqual(out, 0.0)
      self.assertFalse(self.r.active)

  def test_zero_with_noise_but_no_speed_loss_does_not_trigger(self):
    self.feed_prior()
    for i in range(120):
      out = self.r.update(True, 0.0, vehicle_accel=-0.10,
                          speed_error=0.8 + i * 0.0002, v_ego=25.0 - i * 0.0005)
      self.assertEqual(out, 0.0)
      self.assertFalse(self.r.active)

  def test_true_zero_stall_triggers_after_confirmation(self):
    self.feed_prior()
    triggered = False
    for i in range(100):
      v_ego = 25.0 - 0.0035 * (i + 1)
      error = 0.80 + 0.0030 * (i + 1)
      out = self.r.update(True, 0.0, vehicle_accel=-0.12,
                          speed_error=error, v_ego=v_ego)
      if out >= mod.PEDAL_FORCE_RECOVERY_ACCEL:
        triggered = True
        break
      self.assertEqual(out, 0.0)
    self.assertTrue(triggered)
    self.assertTrue(self.r.active)
    self.assertEqual(self.r.activation_count, 1)
    self.assertEqual(self.r.pedal_floor, mod.PEDAL_FORCE_RECOVERY_PEDAL_FLOOR)

  def test_candidate_expires_instead_of_accumulating_during_long_coast(self):
    self.feed_prior()
    # Enough negative aEgo, but deliberately insufficient speed loss/error growth.
    for i in range(self.r.zero_max_frames + 30):
      out = self.r.update(True, 0.0, vehicle_accel=-0.08,
                          speed_error=0.8 + i * 0.0001,
                          v_ego=25.0 - i * 0.0002)
      self.assertEqual(out, 0.0)
      self.assertFalse(self.r.active)
    self.assertEqual(self.r.zero_candidate_frames, 0)
    self.assertEqual(self.r.prior_positive_frames, 0)

  def test_no_prior_positive_no_trigger(self):
    for i in range(150):
      out = self.r.update(True, 0.0, vehicle_accel=-0.20,
                          speed_error=1.0 + i * 0.003, v_ego=25.0 - i * 0.004)
      self.assertEqual(out, 0.0)
      self.assertFalse(self.r.active)

  def test_safety_gate_cancels_active_same_frame(self):
    self.feed_prior()
    for i in range(100):
      out = self.r.update(True, 0.0, vehicle_accel=-0.12,
                          speed_error=0.8 + 0.003 * (i + 1),
                          v_ego=25.0 - 0.0035 * (i + 1))
      if self.r.active:
        break
    self.assertTrue(self.r.active)
    out = self.r.update(False, 0.0, vehicle_accel=-0.12,
                        speed_error=1.2, v_ego=24.6)
    self.assertEqual(out, 0.0)
    self.assertFalse(self.r.active)
    self.assertEqual(self.r.pedal_floor, 0.0)

  def test_recovery_is_bounded(self):
    self.feed_prior()
    for i in range(100):
      self.r.update(True, 0.0, vehicle_accel=-0.12,
                    speed_error=0.8 + 0.003 * (i + 1),
                    v_ego=25.0 - 0.0035 * (i + 1))
      if self.r.active:
        break
    self.assertTrue(self.r.active)
    for _ in range(self.r.max_force_frames):
      out = self.r.update(True, 0.0, vehicle_accel=-0.12,
                          speed_error=1.2, v_ego=24.5)
    self.assertFalse(self.r.active)
    self.assertEqual(out, 0.0)
    self.assertGreater(self.r.cooldown_frames, 0)


if __name__ == "__main__":
  unittest.main()
