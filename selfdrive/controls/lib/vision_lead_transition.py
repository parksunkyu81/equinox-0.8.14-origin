def update_vision_lead_transition(previous_present, last_mono_time,
                                  current_present, current_mono_time):
  new_observation = current_mono_time != 0 and current_mono_time != last_mono_time
  lead_lost = bool(new_observation and previous_present and not current_present)
  if new_observation:
    return bool(current_present), current_mono_time, lead_lost
  return bool(previous_present), last_mono_time, False
