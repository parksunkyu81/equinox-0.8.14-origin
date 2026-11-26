import math

import numpy as np


def as_finite_vector(values, expected_size=None, minimum_size=None):
  """Return a finite 1-D float array, or None when a model field is incomplete."""
  # A capnp list goes through list() first: ~41 us against ~51 for asarray on
  # the EON, and this runs ~27 times a model frame in plannerd.
  items = None
  try:
    if isinstance(values, np.ndarray):
      vector = np.asarray(values, dtype=float)
    else:
      items = list(values)
      vector = np.array(items, dtype=float)
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
  # cannot overflow it. ~5 us against ~24 for isfinite().all(). For a capnp
  # list the Python sum of the floats already in hand is cheaper still: ~3 us
  # against ~20 for ndarray.sum() on the EON's numpy.
  if items is not None:
    if not math.isfinite(sum(vector.tolist())):
      return None
  elif not np.isfinite(vector.sum()):
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
  # Strictly increasing. For finite floats b - a <= 0 exactly when b <= a, so
  # this is np.diff's test without the diff (each numpy call is tens of us on
  # the EON).
  if (t_idxs[1:] <= t_idxs[:-1]).any():
    return None

  # Filled column by column rather than column_stack (~40 us a call); the
  # same values land in the same places.
  path_xyz = np.empty((position_x.size, 3))
  path_xyz[:, 0] = position_x
  path_xyz[:, 1] = position_y
  path_xyz[:, 2] = position_z
  # linalg.norm(axis=1) of three columns is sqrt((x*x + y*y) + z*z), in this
  # order; written out it skips two column_stacks and the norm dispatch.
  speed_forward = np.sqrt(velocity_x * velocity_x + velocity_y * velocity_y + velocity_z * velocity_z)
  return path_xyz, speed_forward, t_idxs, plan_yaw, plan_yaw_rate
