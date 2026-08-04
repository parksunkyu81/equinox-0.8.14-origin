LIVE_PARAMETERS_INPUT_GRACE_SECONDS = 0.3


class LiveParametersValidity:
  """Keep a sane estimate valid across a very short upstream input glitch."""

  def __init__(self, grace_seconds=LIVE_PARAMETERS_INPUT_GRACE_SECONDS):
    self.grace_seconds = grace_seconds
    self.has_sane_sample = False
    self.last_sane_input_time = 0.0

  def update(self, input_checks_ok, parameters_sane, now):
    if input_checks_ok:
      if parameters_sane:
        self.has_sane_sample = True
        self.last_sane_input_time = now
      # Payload sanity remains available separately as liveParameters.valid.
      return True

    return self.has_sane_sample and parameters_sane and \
           now - self.last_sane_input_time <= self.grace_seconds
