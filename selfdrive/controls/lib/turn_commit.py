"""Turn-commit mode: two taps of the stalk mean "I am not taking this corner".

openpilot already plans intersection turns -- a drive-log scan of this car
found the lateral plan asking for 74 deg while the driver was holding 74, and
tracking the driver to within a couple of degrees anywhere below 90. What stops
it finishing one is not the plan but the driver: the steering command saturates
against the 300-unit ceiling for seconds, the "hold the wheel" prompt fires,
and the driver grabs the wheel, which cuts the torque back by design.

This mode is the driver saying otherwise for one corner. It changes nothing
about how the car steers -- no new target angle, no new limit, no touching the
torque ceiling, the 10 kph floor or the curvature clamp -- it only stops the
saturation prompt while the corner is being taken, and marks the episode so the
turn can be measured afterwards.

What the same logs say to expect, from the sustained-saturation windows where
openpilot was steering alone with the driver's hands off:

    14-16 kph   113 deg held (max 161)   ~23 m radius
    22-26 kph    93 deg held (max  97)   ~27 m radius

so a turn of roughly 22-29 m radius. Wide junctions are inside that; ordinary
ones need 15-20 m and are not. There is no data at all between 16 and 22 kph --
in 40 segments the driver always took over there -- which is the gap the
episode log exists to fill.

The blinker reads the stalk, not the lamp -- measured on-runs run from 0.20 s to
42 s with a cluster at 2.0-2.2 s where the stalk's own three blinks time out --
so what the trigger watches is the gap between one signal ending and the next
starting, not how long either lasted. That is the measurement that separates
intent: across 60 segments the shortest gap a driver produced by ordinary
signalling was 1.65 s, and the next shortest 11.57 s.

The gesture is therefore "signal, off, signal again straight away", and it works
whichever way the driver signals -- a light tap whose three blinks run out on
their own, or a latched signal switched off and back on. The second signal has
to stay on, because the blinker going out is what ends the mode.
"""

from common.realtime import DT_CTRL


# How quickly the stalk has to come back on after going off. Measured from the
# falling edge, deliberately, not between the two rising edges: a light tap of
# a GM stalk runs its own three blinks and holds the signal on for about 2.1 s,
# so an interval measured rising-to-rising cannot fit two light taps and the
# gesture would only work if the driver latched the stalk by hand.
#
# The gap is what separates intent cleanly. Across 60 segments every gap
# between one signal ending and the next starting was 1.65 s or longer, and the
# next one up was 11.57 s, while a deliberate re-tap lands inside half a second.
DOUBLE_TAP_GAP_S = 1.2

# Above this the mode neither arms nor stays alive. Not a tuning knob for how
# hard the car turns -- it is the band the measurements above cover, and the
# band where a junction turn happens at all.
MAX_SPEED_KPH = 20.0

# A junction turn of ~25 m radius covers about 31 m of arc, which is 5.6 s at
# 20 kph. Eight seconds finishes that with margin and still bounds the mode to
# one corner rather than leaving it latched down a curvy road.
TIMEOUT_S = 8.0

# The corner has actually begun once the wheel passes this, and is over once it
# comes back through it. Both directions use the same angle: the release check
# is armed only after the start is seen, so a corner cannot end before it
# begins and the mode cannot release on the first frame at near-zero angle.
TURN_STARTED_DEG = 30.0

KPH_PER_MS = 3.6


class TurnCommit:
  """State machine for one committed corner.

  update() is called once per control frame and returns whether the mode is
  live. It never returns a steering value: the caller keeps its own lateral
  output and uses this only to decide whether to prompt the driver.
  """

  def __init__(self, dt=DT_CTRL):
    self.dt = float(dt)
    self.active = False
    self.direction = ''          # 'left' or 'right' while active, else ''
    self.elapsed = 0.0
    self.turn_started = False
    # Why and which way the last corner ended. Both hold until the next corner
    # arms rather than lasting a single frame: just_released is the edge a
    # caller acts on, and making the description outlive it by a frame or two
    # means a caller that logs slightly later in the loop still sees it.
    self.release_reason = ''
    self.release_direction = ''
    self.just_released = False

    self._prev_left = False
    self._prev_right = False
    # When each stalk last went off, so the next rising edge can measure the gap.
    self._left_off_t = None
    self._right_off_t = None
    self._now = 0.0

  def reset(self):
    # Only mode state. The blinker edge state below it is the stalk's own
    # history and belongs to no particular corner -- wiping it here would stop
    # a driver re-arming immediately after a release, which is exactly what a
    # release on 'blinker off' invites them to do.
    self.active = False
    self.direction = ''
    self.elapsed = 0.0
    self.turn_started = False

  def _double_tap(self, left_blinker, right_blinker):
    """Return 'left'/'right' when a stalk comes back on right after going off.

    How long each signal stayed on does not matter, which is what lets the same
    gesture work whether the driver taps the stalk lightly and lets its own
    three blinks finish, or turns a latched signal off and straight back on.
    """
    armed = ''

    if left_blinker and not self._prev_left:
      if self._left_off_t is not None and \
         self._now - self._left_off_t <= DOUBLE_TAP_GAP_S:
        armed = 'left'
      # Either way this signal starts fresh: an unmatched gap must not stay
      # available for the signal after this one.
      self._left_off_t = None
      # Signalling the other way is a different intention, not the second half
      # of this one.
      self._right_off_t = None
    elif self._prev_left and not left_blinker:
      self._left_off_t = self._now

    if right_blinker and not self._prev_right:
      if self._right_off_t is not None and \
         self._now - self._right_off_t <= DOUBLE_TAP_GAP_S:
        armed = 'right'
      self._right_off_t = None
      self._left_off_t = None
    elif self._prev_right and not right_blinker:
      self._right_off_t = self._now

    self._prev_left = bool(left_blinker)
    self._prev_right = bool(right_blinker)
    return armed

  def _release_reason(self, engaged, kph, blinker_on, steering_pressed, angle_deg):
    if not engaged:
      return 'disengaged'
    if steering_pressed:
      return 'driver'
    if not blinker_on:
      return 'blinker off'
    if kph > MAX_SPEED_KPH:
      return 'over speed'
    if self.elapsed >= TIMEOUT_S:
      return 'timeout'
    if self.turn_started and abs(angle_deg) < TURN_STARTED_DEG:
      return 'turn complete'
    return ''

  def update(self, engaged, v_ego, left_blinker, right_blinker,
             steering_angle_deg, steering_pressed):
    self._now += self.dt
    self.just_released = False

    kph = float(v_ego) * KPH_PER_MS
    angle_deg = float(steering_angle_deg)
    armed = self._double_tap(left_blinker, right_blinker)

    if self.active:
      self.elapsed += self.dt
      if abs(angle_deg) >= TURN_STARTED_DEG:
        self.turn_started = True

      blinker_on = (left_blinker if self.direction == 'left' else right_blinker)
      reason = self._release_reason(engaged, kph, blinker_on,
                                    steering_pressed, angle_deg)
      if reason:
        self.release_reason = reason
        self.release_direction = self.direction
        self.just_released = True
        self.reset()
      return self.active

    # Arming is deliberately stricter than staying alive: the second tap has to
    # land while engaged and already inside the speed band, so the mode is
    # never entered on a guess about what the car is about to be doing.
    if armed and engaged and not steering_pressed and kph <= MAX_SPEED_KPH:
      self.active = True
      self.direction = armed
      self.elapsed = 0.0
      self.release_reason = ''
      self.release_direction = ''
      self.turn_started = abs(angle_deg) >= TURN_STARTED_DEG

    return self.active
