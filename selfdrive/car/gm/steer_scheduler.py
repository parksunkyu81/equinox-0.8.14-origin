GM_STEER_STEP = 2
GM_STEER_COMMAND_PERIOD = 0.020
GM_STEER_MIN_COMMAND_INTERVAL = 0.018
GM_STEER_DEADLINE_EARLY_TOLERANCE = 0.002
GM_STEER_GAP_FAULT_THRESHOLD = 0.035
GM_STEER_RATE_UP = 7
GM_STEER_RATE_DOWN = 17


class GMSteeringCommandScheduler:
  """50 Hz monotonic scheduler synchronized by Panda TX loopback.

  Loopback updates select the next rolling counter but never suppress a due
  command. When an acknowledgement is missing, the same counter is retried so
  the EPS never observes a counter gap. Scheduling from the actual send time
  prevents catch-up bursts after a delayed control loop.
  """
  def __init__(self, period=GM_STEER_COMMAND_PERIOD,
               min_interval=GM_STEER_MIN_COMMAND_INTERVAL,
               early_tolerance=GM_STEER_DEADLINE_EARLY_TOLERANCE):
    self.period = float(period)
    self.min_interval = float(min_interval)
    self.early_tolerance = float(early_tolerance)
    self.next_deadline = None
    self.last_send_time = None
    self.last_interval = 0.0
    self.deadline_lag = 0.0
    self.last_loopback_counter = None
    self.last_sent_counter = None
    self.loopback_changed = False
    self.loopback_acked = True
    self.gap_fault = False

  def update(self, now, loopback_counter):
    now = float(now)
    loopback_counter = int(loopback_counter) % 4
    self.loopback_changed = self.last_loopback_counter is not None and \
                            loopback_counter != self.last_loopback_counter
    self.last_loopback_counter = loopback_counter
    self.loopback_acked = self.last_sent_counter is None or \
                          loopback_counter == self.last_sent_counter

    if self.next_deadline is None:
      self.next_deadline = now

    due = now + self.early_tolerance >= self.next_deadline
    separated = self.last_send_time is None or \
                now - self.last_send_time + 1e-9 >= self.min_interval
    if not due or not separated:
      return False, self.last_sent_counter

    self.last_interval = 0.0 if self.last_send_time is None else now - self.last_send_time
    self.deadline_lag = max(now - self.next_deadline, 0.0)
    self.gap_fault = self.last_send_time is not None and \
                     self.last_interval > GM_STEER_GAP_FAULT_THRESHOLD

    # Only advance from a counter confirmed by Panda loopback. If the previous
    # command did not pass safety/TX, retry its counter instead of creating a
    # rolling-counter gap at the EPS.
    counter = (loopback_counter + 1) % 4
    self.last_sent_counter = counter
    self.last_send_time = now
    self.next_deadline = now + self.period
    return True, counter
