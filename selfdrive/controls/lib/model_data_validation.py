import numpy as np


def as_finite_vector(values, expected_size=None, minimum_size=None):
  """Return a finite 1-D float array, or None when a model field is incomplete."""
  # A capnp list goes through list() first: ~41 us against ~51 for asarray on
  # the EON, and this runs ~27 times a model frame in plannerd.
  try:
    if isinstance(values, np.ndarray):
      vector = np.asarray(values, dtype=float)
    else:
      vector = np.array(list(values), dtype=float)
  except (TypeError, ValueError):
    return None

  if vector.ndim != 1:
    return None
  if expected_size is not None and vector.size != expected_size:
    return None
  if minimum_size is not None and vector.size < minimum_size:
    return None
  # The sum is finite exactly when every element is (a NaN or an inf, of
  # either sign, makes it non-finite); a 1-D float vector of model outputs
  # cannot overflow it. ~5 us against ~24 for isfinite().all().
  if not np.isfinite(vector.sum()):
    return None
  return vector


def validated_model_trajectory(md, trajectory_size):
  """Validate every model field consumed by the lateral planner."""
  fields = (
    as_finite_vector(md.position.x, expected_size=trajectory_size),
    as_finite_vector(md.position.y, expected_size=trajectory_size),
    as_finite_vector(md.position.z, expected_size=trajectory_size),
    as_finite_vector(md.velocity.x, expected_size=trajectory_size),
    as_finite_vector(md.velocity.y, expected_size=trajectory_size),
    as_finite_vector(md.velocity.z, expected_size=trajectory_size),
    as_finite_vector(md.position.t, expected_size=trajectory_size),
    as_finite_vector(md.orientation.z, expected_size=trajectory_size),
    as_finite_vector(md.orientationRate.z, expected_size=trajectory_size),
  )
  if any(field is None for field in fields):
    return None

  position_x, position_y, position_z, velocity_x, velocity_y, velocity_z, t_idxs, plan_yaw, plan_yaw_rate = fields
  if np.any(np.diff(t_idxs) <= 0.0):
    return None

  path_xyz = np.column_stack((position_x, position_y, position_z))
  speed_forward = np.linalg.norm(np.column_stack((velocity_x, velocity_y, velocity_z)), axis=1)
  return path_xyz, speed_forward, t_idxs, plan_yaw, plan_yaw_rate
