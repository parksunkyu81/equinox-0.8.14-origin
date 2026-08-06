import importlib.util
import unittest
from pathlib import Path
from types import SimpleNamespace

import sys
import types


def _clip(x, lo, hi):
  return max(lo, min(hi, x))


def _interp(x, xp, fp):
  if x <= xp[0]:
    return fp[0]
  if x >= xp[-1]:
    return fp[-1]
  for i in range(1, len(xp)):
    if x <= xp[i]:
      ratio = (x - xp[i - 1]) / (xp[i] - xp[i - 1])
      return fp[i - 1] + ratio * (fp[i] - fp[i - 1])
  return fp[-1]


common_pkg = types.ModuleType("common")
conversions_mod = types.ModuleType("common.conversions")
numpy_fast_mod = types.ModuleType("common.numpy_fast")
params_mod = types.ModuleType("common.params")
realtime_mod = types.ModuleType("common.realtime")

class _Conversions:
  MS_TO_KPH = 3.6

class _Params:
  pass

conversions_mod.Conversions = _Conversions
numpy_fast_mod.clip = _clip
numpy_fast_mod.interp = _interp
params_mod.Params = _Params
realtime_mod.DT_CTRL = 0.01
sys.modules["common"] = common_pkg
sys.modules["common.conversions"] = conversions_mod
sys.modules["common.numpy_fast"] = numpy_fast_mod
sys.modules["common.params"] = params_mod
sys.modules["common.realtime"] = realtime_mod

MODULE_PATH = Path(__file__).resolve().parents[1] / "lib" / "manual_lead_catchup.py"
spec = importlib.util.spec_from_file_location("manual_lead_catchup", MODULE_PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


class FakeParams:
  def __init__(self, enabled=True, max_accel="0.70"):
    self.enabled = enabled
    self.max_accel = max_accel

  def get_bool(self, key):
    if key == "ManualLeadCatchupEnabled":
      return self.enabled
    return False

  def get(self, key):
    if key == "ManualLeadCatchupMaxAccel":
      return self.max_accel.encode()
    return None


def car_state(v_ego=0.4, gas=False, brake=False, standstill=False, a_ego=0.0):
  return SimpleNamespace(
    vEgo=v_ego,
    gasPressed=gas,
    brakePressed=brake,
    standstill=standstill,
    steeringAngleDeg=0.0,
    aEgo=a_ego,
  )


def lead_state(v_lead=0.0, v_rel=None, d_rel=10.0, status=True, a_lead=0.0):
  if v_rel is None:
    v_rel = v_lead - 0.4
  return SimpleNamespace(
    status=status,
    dRel=d_rel,
    vLead=v_lead,
    vRel=v_rel,
    aLeadK=a_lead,
  )


class TestManualLeadCatchupRestart(unittest.TestCase):
  def setUp(self):
    self.m = mod.ManualLeadCatchup(dt=0.01, params=FakeParams())
    self.frame = 0

  def step(self, cs, lead, plan_valid=True, fcw=False, curve=False):
    ready = self.m.pre_update(
      self.frame, True, True, cs, lead, 1.3,
      plan_valid, fcw, curve,
    )
    self.frame += 1
    return ready

  def arm_and_release(self, release_lead=None):
    stopped = lead_state(v_lead=0.0, v_rel=-0.4)
    self.assertFalse(self.step(car_state(gas=True), stopped))
    self.assertEqual(self.m.state, mod.MANUAL_CATCHUP_GAS_PRESSED)
    if release_lead is None:
      release_lead = stopped
    self.assertFalse(self.step(car_state(gas=False), release_lead))
    self.assertEqual(self.m.state, mod.MANUAL_CATCHUP_RELEASE_PENDING)

  def test_never_activates_without_explicit_driver_launch(self):
    moving = lead_state(v_lead=1.0, v_rel=0.6)
    for _ in range(100):
      self.assertFalse(self.step(car_state(gas=False), moving))
    self.assertEqual(self.m.state, mod.MANUAL_CATCHUP_IDLE)
    self.assertEqual(self.m.activation_count, 0)

  def test_driver_can_arm_while_lead_radar_still_reports_stopped(self):
    stopped = lead_state(v_lead=0.0, v_rel=-0.4)
    self.assertFalse(self.step(car_state(gas=True), stopped))
    self.assertEqual(self.m.state, mod.MANUAL_CATCHUP_GAS_PRESSED)

  def test_strong_lead_motion_on_release_handoffs_same_frame(self):
    stopped = lead_state(v_lead=0.0, v_rel=-0.4)
    self.assertFalse(self.step(car_state(gas=True), stopped))
    moving = lead_state(v_lead=1.0, v_rel=0.6)
    self.assertTrue(self.step(car_state(gas=False), moving))
    self.assertTrue(self.m.active)
    self.assertEqual(self.m.activation_count, 1)

  def test_release_waits_for_delayed_lead_motion_then_activates(self):
    self.arm_and_release()

    # Simulate several stale 15 Hz radar samples after the release edge.
    stopped = lead_state(v_lead=0.1, v_rel=-0.3)
    for _ in range(15):
      self.assertFalse(self.step(car_state(gas=False), stopped))
      self.assertEqual(self.m.state, mod.MANUAL_CATCHUP_RELEASE_PENDING)

    moving = lead_state(v_lead=0.7, v_rel=0.3)
    activated = False
    for _ in range(self.m.lead_start_confirm_frames + 2):
      if self.step(car_state(gas=False), moving):
        activated = True
        break
    self.assertTrue(activated)
    self.assertTrue(self.m.active)
    self.assertEqual(self.m.activation_count, 1)

  def test_release_pending_times_out_without_lead_departure(self):
    self.arm_and_release()
    stopped = lead_state(v_lead=0.0, v_rel=-0.4)
    for _ in range(self.m.release_pending_timeout_frames + 1):
      self.step(car_state(gas=False), stopped)
    self.assertFalse(self.m.active)
    self.assertFalse(self.m.pending)
    self.assertEqual(self.m.cancel_reason, "lead_start_timeout")

  def test_below_one_kph_never_handoffs(self):
    stopped = lead_state(v_lead=0.0, v_rel=-0.2)
    self.assertFalse(self.step(car_state(v_ego=0.2, gas=True), stopped))
    self.assertFalse(self.step(car_state(v_ego=0.2, gas=False), stopped))
    self.assertEqual(self.m.state, mod.MANUAL_CATCHUP_IDLE)

  def test_brake_cancels_pending_same_frame(self):
    self.arm_and_release()
    moving = lead_state(v_lead=0.7, v_rel=0.3)
    self.assertFalse(self.step(car_state(gas=False, brake=True), moving))
    self.assertEqual(self.m.state, mod.MANUAL_CATCHUP_IDLE)
    self.assertEqual(self.m.cancel_reason, "pending_context")

  def test_first_active_apply_has_bounded_nonzero_entry_floor(self):
    self.arm_and_release(lead_state(v_lead=0.7, v_rel=0.3))
    moving = lead_state(v_lead=0.7, v_rel=0.3, d_rel=10.0)
    for _ in range(self.m.lead_start_confirm_frames + 2):
      if self.step(car_state(gas=False), moving):
        break
    self.assertTrue(self.m.active)

    out = self.m.apply(
      0.0, (-3.5, 2.0), car_state(gas=False), moving,
      1.3, True, False, False, catchup_factor=1.0,
    )
    self.assertGreaterEqual(out, mod.BOOST_ENTRY_ACCEL)
    self.assertLessEqual(out, self.m.max_accel)


if __name__ == "__main__":
  unittest.main()
