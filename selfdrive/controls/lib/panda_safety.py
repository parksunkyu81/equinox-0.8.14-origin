def panda_safety_config_matches(panda_states, safety_configs, alternative_experience,
                                ignored_safety_modes):
  """Return whether every connected Panda has the safety configuration expected by CarParams."""
  if len(panda_states) < len(safety_configs):
    return False

  for i, panda_state in enumerate(panda_states):
    if i < len(safety_configs):
      expected = safety_configs[i]
      if (panda_state.safetyModel != expected.safetyModel or
          panda_state.safetyParam != expected.safetyParam or
          panda_state.alternativeExperience != alternative_experience):
        return False
    elif panda_state.safetyModel not in ignored_safety_modes:
      return False

  return True


def update_panda_safety_readiness(ready, match_counter, matches, required_frames):
  """Latch readiness after a consecutive run of matching Panda safety states."""
  if ready:
    return True, match_counter

  match_counter = match_counter + 1 if matches else 0
  return match_counter >= required_frames, match_counter
