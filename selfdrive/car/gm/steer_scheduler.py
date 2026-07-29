GM_STEER_STEP = 2
GM_STEER_COMMAND_PERIOD = 0.020
GM_STEER_MIN_SAFE_INTERVAL = 0.018
GM_STEER_DEADLINE_EARLY_TOLERANCE = GM_STEER_COMMAND_PERIOD - GM_STEER_MIN_SAFE_INTERVAL
GM_STEER_GAP_FAULT_THRESHOLD = 0.035
GM_STEER_RATE_UP = 7
GM_STEER_RATE_DOWN = 17


class GMSteeringLimitTracker:
  """Track actuator limiting only when a GM steering command is transmitted.

  controls runs at 100 Hz while GM steering commands are sent at 50 Hz. Comparing
  the new 100 Hz request with the held actuator output on a non-send frame creates
  a false rate-limit indication. Hold the result from the most recent transmitted
  command instead, and clear it immediately when lateral control is inactive.
  """
  def __init__(self, torque_tolerance=1):
    self.torque_tolerance = int(torque_tolerance)
    self.requested_torque = 0
    self.applied_torque = 0
    self.limited = False

  def update(self, command_sent, active, requested_torque=None, applied_torque=None):
    if not active:
      self.requested_torque = 0
      self.applied_torque = 0
      self.limited = False
      return self.limited

    if not command_sent:
      return self.limited

    if requested_torque is None or applied_torque is None:
      raise ValueError("sent steering commands require requested and applied torque")

    self.requested_torque = int(requested_torque)
    self.applied_torque = int(applied_torque)
    self.limited = abs(self.applied_torque - self.requested_torque) > self.torque_tolerance
    return self.limited


class GMSteeringCommandScheduler:
  """50 Hz monotonic GM scheduler synchronized by Panda TX loopback.

  GM EPS can fault when steering commands arrive too close together or when the
  rolling counter is duplicated/skipped. Never send in the control cycle where
  a new Panda TX loopback is observed, and suppress a due command until the
  previous command is acknowledged.

  Scheduling is based on monotonic time rather than frame parity. On EON the
  controls loop can run slower than 100 Hz, and a loopback commonly arrives on
  the nominal even frame. A frame-parity gate then waits two more control cycles
  and turns a safe acknowledgement into a 40-60 ms command gap. An overdue,
  acknowledged command instead resumes on the first safe control cycle.
  """
  def __init__(self, step=GM_STEER_STEP, period=GM_STEER_COMMAND_PERIOD,
               min_safe_interval=GM_STEER_MIN_SAFE_INTERVAL):
    self.step = int(step)
    self.period = float(period)
    self.min_safe_interval = float(min_safe_interval)
    self.early_tolerance = max(self.period - self.min_safe_interval, 0.0)
    self.next_deadline = None
    self.last_send_time = None
    self.last_interval = 0.0
    self.deadline_lag = 0.0
    self.time_since_last_send = 0.0
    self.last_loopback_counter = None
    self.last_sent_counter = None
    self.loopback_changed = False
    self.loopback_acked = True
    self.gap_fault = False
    self.unacked_fault = False
    self.due = False
    self.block_reason = "initial_sync"

  def update(self, now, frame, loopback_counter):
    now = float(now)
    frame = int(frame)
    loopback_counter = int(loopback_counter) % 4

    first_loopback = self.last_loopback_counter is None
    self.loopback_changed = first_loopback or loopback_counter != self.last_loopback_counter
    self.last_loopback_counter = loopback_counter
    self.loopback_acked = self.last_sent_counter is None or \
                          loopback_counter == self.last_sent_counter

    if self.last_send_time is None:
      self.time_since_last_send = 0.0
      self.deadline_lag = 0.0
      self.gap_fault = False
    else:
      self.time_since_last_send = max(now - self.last_send_time, 0.0)
      self.deadline_lag = max(self.time_since_last_send - self.period, 0.0)
      self.gap_fault = self.time_since_last_send > GM_STEER_GAP_FAULT_THRESHOLD

    # The first loopback value only establishes the camera/Panda counter. This
    # is the same initial synchronization performed by the official controller.
    if first_loopback:
      self.next_deadline = now + self.period
      self.due = False
      self.unacked_fault = False
      self.block_reason = "initial_sync"
      return False, None

    separated = self.last_send_time is None or \
                self.time_since_last_send + 1e-9 >= self.min_safe_interval
    self.due = self.next_deadline is not None and \
               now + self.early_tolerance + 1e-9 >= self.next_deadline
    self.unacked_fault = self.due and not self.loopback_acked

    if not self.due:
      self.block_reason = "not_due"
      return False, None

    # A changed loopback means the previous command has just been observed from
    # Panda TX. Do not enqueue another command in the same control cycle.
    if self.loopback_changed:
      self.block_reason = "loopback_changed"
      return False, None

    # Never retry the same counter when TX acknowledgement is missing. A retry
    # is also invalid data to the EPS; wait for the real loopback instead.
    if not self.loopback_acked:
      self.block_reason = "unacked"
      return False, None

    if not separated:
      self.block_reason = "min_interval"
      return False, None

    self.last_interval = 0.0 if self.last_send_time is None else max(now - self.last_send_time, 0.0)
    self.deadline_lag = 0.0 if self.last_send_time is None else max(self.last_interval - self.period, 0.0)
    self.gap_fault = self.last_send_time is not None and \
                     self.last_interval > GM_STEER_GAP_FAULT_THRESHOLD

    # Advance only from a counter confirmed by Panda TX loopback.
    counter = (loopback_counter + 1) % 4
    self.last_sent_counter = counter
    self.last_send_time = now
    self.next_deadline = now + self.period
    self.time_since_last_send = 0.0
    self.loopback_acked = False
    self.block_reason = "sent"
    return True, counter
