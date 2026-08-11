import math

from common.numpy_fast import clip


LEAD_ACQUIRE_PROB = 0.55
LEAD_RELEASE_PROB = 0.35
LEAD_ACQUIRE_FRAMES = 3
LEAD_RELEASE_FRAMES = 6
LEAD_HOLD_SECONDS = 0.70
LEAD_REACQUIRE_BLEND_SECONDS = 0.50
LEAD_MATCH_DISTANCE_M = 8.0
LEAD_MATCH_DISTANCE_RATIO = 0.15
LEAD_MATCH_SPEED_MS = 5.0
LEAD_IMMEDIATE_DISTANCE_M = 40.0
LEAD_IMMEDIATE_TTC_S = 4.0


def _lead_copy(lead):
  return dict(lead) if lead is not None else {"status": False}


def _finite(value, default=0.0):
  try:
    value = float(value)
  except (TypeError, ValueError):
    return float(default)
  return value if math.isfinite(value) else float(default)


class LeadContinuityFilter:
  """Short, constraint-only continuity for intermittent vision/radar leads.

  The filter never invents a new lead. It only predicts a previously confirmed
  lead for a bounded interval, preventing a one-frame lead0 -> cruise jump.
  """

  def __init__(self, dt):
    self.dt = max(1e-3, float(dt))
    self.reset()

  def reset(self):
    self.tracked = False
    self.last_lead = {"status": False}
    self.acquire_frames = 0
    self.release_frames = 0
    self.hold_time = 0.0
    self.blend_time = 0.0
    self.blend_elapsed = 0.0

  @property
  def state(self):
    if not self.tracked:
      return "lost"
    if self.hold_time > 0.0:
      return "hold"
    if self.blend_time > 0.0:
      return "reacquire"
    return "tracked"

  def _predict(self, v_ego):
    lead = _lead_copy(self.last_lead)
    v_lead = max(0.0, _finite(lead.get("vLead", 0.0)))
    a_lead = clip(_finite(lead.get("aLeadK", 0.0)), -3.0, 1.5)
    # Do not extrapolate an uncertain positive acceleration aggressively.
    a_predict = min(float(a_lead), 0.0)
    v_lead = max(0.0, v_lead + a_predict * self.dt)
    v_rel = v_lead - float(v_ego)
    d_rel = max(0.0, _finite(lead.get("dRel", 0.0)) + v_rel * self.dt)

    lead.update({
      "status": True,
      "dRel": float(d_rel),
      "vRel": float(v_rel),
      "vLead": float(v_lead),
      "vLeadK": float(v_lead),
      "aLeadK": float(a_predict),
      "fcw": False,
      "modelProb": max(0.0, _finite(lead.get("modelProb", 0.0)) - self.dt / LEAD_HOLD_SECONDS),
    })
    self.last_lead = lead
    return _lead_copy(lead)

  def _same_lead(self, lead):
    predicted_d = _finite(self.last_lead.get("dRel", 0.0))
    predicted_v = _finite(self.last_lead.get("vLead", 0.0))
    distance_gate = max(LEAD_MATCH_DISTANCE_M, predicted_d * LEAD_MATCH_DISTANCE_RATIO)
    return (abs(_finite(lead.get("dRel", 0.0)) - predicted_d) <= distance_gate and
            abs(_finite(lead.get("vLead", 0.0)) - predicted_v) <= LEAD_MATCH_SPEED_MS)

  def _blend(self, lead):
    if self.blend_time <= 0.0:
      return _lead_copy(lead)
    self.blend_elapsed = min(LEAD_REACQUIRE_BLEND_SECONDS,
                             self.blend_elapsed + self.dt)
    alpha = clip(self.blend_elapsed / LEAD_REACQUIRE_BLEND_SECONDS, 0.0, 1.0)
    blended = _lead_copy(lead)
    for key in ("dRel", "yRel", "vRel", "vLead", "vLeadK", "aLeadK"):
      blended[key] = ((1.0 - alpha) * _finite(self.last_lead.get(key, lead.get(key, 0.0))) +
                      alpha * _finite(lead.get(key, 0.0)))
    blended["status"] = True
    blended["fcw"] = bool(lead.get("fcw", False))
    self.blend_time = max(0.0, self.blend_time - self.dt)
    return blended

  def update(self, raw_lead, model_prob, v_ego, enabled=True):
    if not enabled:
      self.reset()
      return {"status": False}

    lead = _lead_copy(raw_lead)
    valid = bool(lead.get("status", False))
    probability = _finite(model_prob, lead.get("modelProb", 0.0))

    if valid:
      immediate = (_finite(lead.get("dRel", 1e6), 1e6) <= LEAD_IMMEDIATE_DISTANCE_M or
                   (_finite(lead.get("vRel", 0.0)) < -0.3 and
                    _finite(lead.get("dRel", 1e6), 1e6) /
                    max(-_finite(lead.get("vRel", 0.0)), 0.1) <= LEAD_IMMEDIATE_TTC_S))
      if not self.tracked:
        self.acquire_frames = self.acquire_frames + 1 if probability >= LEAD_ACQUIRE_PROB else 0
        if not immediate and self.acquire_frames < LEAD_ACQUIRE_FRAMES:
          return {"status": False}
        self.tracked = True
        self.acquire_frames = 0
      elif self.hold_time > 0.0:
        if self._same_lead(lead):
          self.blend_time = LEAD_REACQUIRE_BLEND_SECONDS
          self.blend_elapsed = 0.0
        elif _finite(lead.get("dRel", 1e6), 1e6) >= _finite(self.last_lead.get("dRel", 0.0)):
          # A farther, inconsistent candidate must prove stable before replacing
          # the predicted lead. A closer candidate is accepted immediately.
          self.acquire_frames += 1
          if self.acquire_frames < LEAD_ACQUIRE_FRAMES:
            return self._predict(v_ego)

      output = self._blend(lead)
      self.last_lead = _lead_copy(output)
      self.release_frames = 0
      self.hold_time = 0.0
      self.acquire_frames = 0
      return output

    if not self.tracked:
      self.acquire_frames = 0
      return {"status": False}

    self.release_frames += 1
    self.hold_time += self.dt
    weak_evidence = probability >= LEAD_RELEASE_PROB
    release_confirmed = self.release_frames >= LEAD_RELEASE_FRAMES
    if self.hold_time <= LEAD_HOLD_SECONDS and (weak_evidence or not release_confirmed or self.last_lead.get("radar", False)):
      return self._predict(v_ego)

    self.reset()
    return {"status": False}
