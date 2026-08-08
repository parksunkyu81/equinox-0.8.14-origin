"""Bounded Equinox torque-authority scheduling.

The live-torque learner identifies the vehicle. This module decides how much
of the learned authority may be used for the current speed and corner. Keeping
those jobs separate prevents a temporary boost from contaminating persistence.
"""

from common.numpy_fast import clip, interp


AUTHORITY_SPEED_BP = [0.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0,
                      45.0, 60.0, 80.0, 100.0, 110.0, 130.0]

# A smaller factor produces more torque for the same lateral-accel request.
# Extra authority is concentrated below 60 km/h. Highway authority remains at
# the learned baseline and is governed by the curvature guard.
AUTHORITY_LAT_FACTOR_SCALE_V = [1.000, 0.960, 0.900, 0.860, 0.840, 0.840,
                                0.860, 0.900, 0.960, 1.000, 1.000, 1.000, 1.000]
AUTHORITY_FRICTION_SCALE_V = [1.000, 1.050, 1.120, 1.180, 1.200, 1.200,
                              1.180, 1.120, 1.060, 1.000, 1.000, 1.000, 1.000]

# These speed envelopes replace the old global +/-4% and +/-10% clamps that
# silently cancelled most of the requested low-speed profile.
AUTHORITY_LAT_FACTOR_DOWN_V = [0.000, 0.040, 0.100, 0.140, 0.160, 0.160,
                               0.140, 0.100, 0.050, 0.000, 0.000, 0.000, 0.000]
AUTHORITY_FRICTION_UP_V = [0.000, 0.050, 0.120, 0.180, 0.200, 0.200,
                           0.180, 0.120, 0.060, 0.020, 0.000, 0.000, 0.000]

LAT_FACTOR_ABS_MIN = 1.75
LAT_FACTOR_ABS_MAX = 2.42
FRICTION_ABS_MIN = 0.165
FRICTION_ABS_MAX = 0.305


def authority_confidence(total_points):
  """Return partial cold-start authority and progressively unlock the rest."""
  points = max(0.0, float(total_points))
  return float(interp(points, [0.0, 500.0, 2500.0], [0.35, 0.60, 1.00]))


def effective_torque_params(base_lat_factor, base_friction, v_kph, corner_blend,
                            total_points):
  """Return bounded effective parameters without modifying the learned base."""
  base_lat = float(clip(float(base_lat_factor), LAT_FACTOR_ABS_MIN, LAT_FACTOR_ABS_MAX))
  base_fric = float(clip(float(base_friction), FRICTION_ABS_MIN, FRICTION_ABS_MAX))
  speed = max(0.0, float(v_kph))
  blend = float(clip(float(corner_blend), 0.0, 1.0)) * authority_confidence(total_points)

  lat_scale = float(interp(speed, AUTHORITY_SPEED_BP, AUTHORITY_LAT_FACTOR_SCALE_V))
  friction_scale = float(interp(speed, AUTHORITY_SPEED_BP, AUTHORITY_FRICTION_SCALE_V))
  lat_down = float(interp(speed, AUTHORITY_SPEED_BP, AUTHORITY_LAT_FACTOR_DOWN_V))
  friction_up = float(interp(speed, AUTHORITY_SPEED_BP, AUTHORITY_FRICTION_UP_V))

  effective_lat = base_lat + (base_lat * lat_scale - base_lat) * blend
  effective_fric = base_fric + (base_fric * friction_scale - base_fric) * blend

  lat_min = max(LAT_FACTOR_ABS_MIN, base_lat * (1.0 - lat_down))
  friction_max = min(FRICTION_ABS_MAX, base_fric * (1.0 + friction_up))
  effective_lat = float(clip(effective_lat, lat_min, base_lat))
  effective_fric = float(clip(effective_fric, base_fric, friction_max))
  return effective_lat, effective_fric, blend
