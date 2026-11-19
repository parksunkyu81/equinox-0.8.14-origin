"""Corner preload: start steering into a corner the model can already see.

Measured on 2026-11-19--06-42-21 (61 min, 28 corners of 0.6 m/s^2 or more):

  * Above 30 km/h the model's path shows the corner 1.9 s (30-50 km/h) to
    3.1 s (50+ km/h) before the car turns in, but at only ~46% of the
    curvature the corner turns out to have (the known corner flattening).
  * The steering target handed to the torque controller leads the car's own
    turn-in by only ~0.1 s: the plan looks 0.25 s ahead
    (steerActuatorDelay 0.05 + 0.20) and is anchored on the current motion.
  * In the second before turn-in the controller uses ~70-140 of the 300
    torque units, so there is room to steer harder earlier. Under-turn in
    these corners is the target being too gentle, not the car failing to
    follow it (2-9% of corner frames).

So the target is raised toward what the model sees 1.0-1.5 s ahead, scaled up
to undo part of the flattening. The preload only ever adds turn in the
direction the target already points, and lets go as soon as the corner it saw
stops growing:

  * 30 km/h and up only. Intersection turns (radius 11-18 m) show up in the
    model 0.1-0.3 s before turn-in, so there is nothing to preview there.
  * Off while the driver holds the wheel, for a second after they let go,
    and during lane changes.
  * Both horizons must agree on the direction, so an S-bend never preloads
    against the bend the car is in.
  * The addition is capped at PRELOAD_MAX_EXTRA_LAT_ACCEL. The jerk limit
    and lateral-accel ceiling in drive_helpers still apply after this.

Replayed on the drive above, gain 2.0 turned the car ~1 s earlier with +143
torque units before turn-in, but asked 1.1 m/s^2 too much at p90 on corner
exit. 1.4 with the exit release and the cap is the compromise.

Replay of this module as committed (gain 1.4, release at 40% of the peak,
smoothed preview): it preloads 13 of the 21 corners at 30 km/h and up, adding
+14..+143 torque units in the second before turn-in (about +45 typical). The
other 8 are the driver already holding the wheel or a preview gentler than
0.3 m/s^2. It never added more than 0.05 m/s^2 on 16445 straight frames, and
the worst over-ask after a corner ended was 0.47 m/s^2. Releasing at 85% of the
peak instead preloaded almost nothing: the raw preview dips that far
frame to frame well before the corner arrives.
"""

from common.numpy_fast import interp
from selfdrive.modeld.constants import T_IDXS

PRELOAD_MIN_SPEED = 30.0 / 3.6     # full effect from here up
PRELOAD_FADE_SPEED = 25.0 / 3.6    # zero effect below this
PRELOAD_GAIN = 1.4
PRELOAD_HORIZONS_S = (1.0, 1.5)
PRELOAD_MAX_EXTRA_LAT_ACCEL = 1.0  # m/s^2 added on top of today's target
PRELOAD_MIN_PREVIEW_LAT_ACCEL = 0.3  # ignore previews gentler than this
PRELOAD_RELEASE_DROP = 0.4         # preview fell below this share of its peak -> exit
# The 20 Hz preview flickers by tens of percent frame to frame, so it is
# low-passed (~0.15 s) before the peak/release logic sees it.
PRELOAD_PREVIEW_FILTER_K = 0.25
PRELOAD_DRIVER_HOLDOFF_S = 1.0
PRELOAD_MODEL_TIMEOUT_S = 0.3
PRELOAD_RAMP_IN_S = 0.4
PRELOAD_RAMP_OUT_S = 0.25


def model_preview_curvature(model):
  """Curvature the model path holds 1.0-1.5 s ahead, or 0.0 if unclear.

  The model yaw rate over its forward speed is a curvature with the same sign
  as controlsState.curvature and the plan's curvatures.
  """
  yaw = model.orientationRate.z
  vx = model.velocity.x
  if len(yaw) != len(T_IDXS) or len(vx) != len(T_IDXS):
    return 0.0
  ks = []
  for h in PRELOAD_HORIZONS_S:
    v = max(interp(h, T_IDXS, vx), 1.0)
    ks.append(interp(h, T_IDXS, yaw) / v)
  k0, k1 = ks
  if k0 * k1 <= 0.0:
    return 0.0
  return 0.5 * (k0 + k1)


class CornerPreload:
  def __init__(self):
    self.reset()

  def reset(self):
    self.preview = 0.0
    self.preview_age = 1e3
    self.peak = 0.0
    self.released = False
    self.alpha = 0.0
    self.driver_timer = 0.0
    self.active = False
    self.extra_lat_accel = 0.0

  def update_model(self, model):
    raw = model_preview_curvature(model)
    if raw * self.preview <= 0.0:
      preview = raw   # direction changed: do not drag the old corner along
    else:
      preview = self.preview + PRELOAD_PREVIEW_FILTER_K * (raw - self.preview)
    if preview * self.preview <= 0.0 or abs(preview) < 1e-5:
      # New corner (or none): start tracking a fresh peak.
      self.peak = abs(preview)
      self.released = False
    elif abs(preview) > self.peak:
      self.peak = abs(preview)
      self.released = False
    elif abs(preview) < PRELOAD_RELEASE_DROP * self.peak:
      # The corner the model saw is no longer growing: we are at or past it.
      self.released = True
    self.preview = preview
    self.preview_age = 0.0

  def apply(self, desired_curvature, v_ego, dt, lat_active, steering_pressed, lane_change):
    self.preview_age += dt
    if steering_pressed:
      self.driver_timer = PRELOAD_DRIVER_HOLDOFF_S
    else:
      self.driver_timer = max(0.0, self.driver_timer - dt)

    v2 = max(v_ego, 0.1) ** 2
    preview = self.preview
    target = PRELOAD_GAIN * preview
    # A near-straight target that leans the other way (lane centring) does not
    # count as pointing against the corner.
    same_dir = target * desired_curvature >= 0.0 or abs(desired_curvature) * v2 < 0.2
    wanted = (lat_active and not lane_change and self.driver_timer <= 0.0 and
              self.preview_age < PRELOAD_MODEL_TIMEOUT_S and not self.released and
              v_ego > PRELOAD_FADE_SPEED and same_dir and
              abs(preview) * v2 >= PRELOAD_MIN_PREVIEW_LAT_ACCEL and
              abs(target) > abs(desired_curvature))

    if not lat_active:
      self.alpha = 0.0
    elif wanted:
      self.alpha = min(1.0, self.alpha + dt / PRELOAD_RAMP_IN_S)
    else:
      self.alpha = max(0.0, self.alpha - dt / PRELOAD_RAMP_OUT_S)

    speed_scale = interp(v_ego, [PRELOAD_FADE_SPEED, PRELOAD_MIN_SPEED], [0.0, 1.0])
    extra = 0.0
    if self.alpha > 0.0 and same_dir and abs(target) > abs(desired_curvature):
      max_extra = PRELOAD_MAX_EXTRA_LAT_ACCEL / v2
      extra = min(abs(target) - abs(desired_curvature), max_extra) * self.alpha * speed_scale
      extra = extra if target > 0.0 else -extra

    self.active = extra != 0.0
    self.extra_lat_accel = extra * v2
    return desired_curvature + extra
