"""Observe whether a deep curve is suitable for a future virtual-path experiment.

This module is diagnostic only. It deliberately never changes a trajectory,
speed request, steering torque, or safety limit.
"""

from collections import deque
import math


class CurveVirtualReadinessMonitor:
  """Measure curve stability, vehicle-motion agreement, and driver takeover."""

  MIN_SPEED_MS = 5.0
  MIN_CURVATURE = 0.0075
  MIN_SAMPLES = 8  # 0.40 s at the model rate
  MAX_SAMPLES = 80  # retain the most recent four seconds
  MIN_STABLE_RATIO = 0.80
  MIN_YAW_AGREEMENT_RATIO = 0.85
  MAX_DRIVER_INTERVENTION_RATIO = 0.02
  LOW_LANE_CONFIDENCE = 0.35

  def __init__(self):
    self.samples = deque(maxlen=self.MAX_SAMPLES)
    self.active = False
    self.current = self._empty_report()

  @staticmethod
  def _empty_report():
    return {
      'active': False,
      'sampleCount': 0,
      'curvatureMean': 0.0,
      'curvatureStableRatio': 0.0,
      'yawAgreementRatio': 0.0,
      'driverInterventionRatio': 0.0,
      'laneLossRatio': 0.0,
      'quality': 0.0,
      'eligible': False,
    }

  @staticmethod
  def _median(values):
    ordered = sorted(values)
    size = len(ordered)
    if size == 0:
      return 0.0
    midpoint = size // 2
    return (ordered[midpoint] if size % 2 else
            0.5 * (ordered[midpoint - 1] + ordered[midpoint]))

  def _report(self):
    if not self.samples:
      return self._empty_report()

    steering_curvatures = [sample['steeringCurvature'] for sample in self.samples]
    magnitudes = [abs(curvature) for curvature in steering_curvatures]
    curvature_mean = self._median(magnitudes)
    stable_count = 0
    for previous, current in zip(steering_curvatures[:-1], steering_curvatures[1:]):
      # Constant-radius and slowly tightening curves are candidates. The
      # tolerance scales with the observed bend to avoid rejecting gentle arcs.
      limit = max(0.0015, 0.25 * curvature_mean)
      stable_count += abs(current - previous) <= limit
    stable_ratio = (stable_count / max(1, len(steering_curvatures) - 1))

    yaw_agreement_ratio = (
      sum(sample['yawAgrees'] for sample in self.samples) / len(self.samples))
    driver_ratio = (
      sum(sample['driverIntervening'] for sample in self.samples) / len(self.samples))
    lane_loss_ratio = (
      sum(sample['laneWeak'] for sample in self.samples) / len(self.samples))

    quality = max(0.0, min(1.0,
      0.45 * stable_ratio +
      0.40 * yaw_agreement_ratio +
      0.15 * (1.0 - driver_ratio)))
    eligible = bool(
      len(self.samples) >= self.MIN_SAMPLES and
      stable_ratio >= self.MIN_STABLE_RATIO and
      yaw_agreement_ratio >= self.MIN_YAW_AGREEMENT_RATIO and
      driver_ratio <= self.MAX_DRIVER_INTERVENTION_RATIO)
    return {
      'active': bool(self.active),
      'sampleCount': len(self.samples),
      'curvatureMean': float(curvature_mean),
      'curvatureStableRatio': float(stable_ratio),
      'yawAgreementRatio': float(yaw_agreement_ratio),
      'driverInterventionRatio': float(driver_ratio),
      'laneLossRatio': float(lane_loss_ratio),
      'quality': float(quality),
      'eligible': eligible,
    }

  def update(self, v_ego, steering_curvature, yaw_rate, steering_pressed,
             lane_change_active, lane_confidence, yaw_valid=True):
    """Return (current report, completed report or None).

    yaw_rate comes from the device IMU (liveLocationKalman.angularVelocityCalibrated),
    not the car's CAN bus: this GM's EBCM does not transmit a usable yaw signal
    (its DBC-mapped field reads a constant 0 in every real drive log checked).
    yaw_valid should reflect that measurement's own per-sample validity
    (angularVelocityCalibrated.valid and inputsOK/sensorsOK/deviceStable), not
    the coarser liveLocationKalman.status, which stays 'uninitialized' on this
    device (no GPS lock, ever) even while the angular-velocity measurement
    itself is fine.
    """
    values = (v_ego, steering_curvature, yaw_rate, lane_confidence)
    if not all(math.isfinite(float(value)) for value in values):
      curve_active = False
    else:
      yaw_curvature = float(yaw_rate) / max(float(v_ego), 1.0)
      curve_active = bool(
        not lane_change_active and
        float(v_ego) >= self.MIN_SPEED_MS and
        max(abs(float(steering_curvature)), abs(yaw_curvature)) >=
        self.MIN_CURVATURE)

    if curve_active:
      steering_curvature = float(steering_curvature)
      yaw_curvature = float(yaw_rate) / max(float(v_ego), 1.0)
      agreement_limit = max(0.003, 0.50 * abs(steering_curvature))
      # An invalid yaw sample cannot confirm agreement. Treat it the same as a
      # real disagreement rather than skipping it -- this readiness monitor
      # exists to gate how much a fallback path is trusted, so an unmeasurable
      # frame should count against trust, not be silently dropped.
      yaw_agrees = bool(
        yaw_valid and
        steering_curvature * yaw_curvature > 0.0 and
        abs(steering_curvature - yaw_curvature) <= agreement_limit)
      self.active = True
      self.samples.append({
        'steeringCurvature': steering_curvature,
        'yawAgrees': yaw_agrees,
        'driverIntervening': bool(steering_pressed),
        'laneWeak': float(lane_confidence) < self.LOW_LANE_CONFIDENCE,
      })
      self.current = self._report()
      return self.current, None

    completed = None
    if self.active:
      completed = self._report()
      completed['active'] = False
      self.samples.clear()
      self.active = False
    self.current = self._empty_report()
    return self.current, completed
