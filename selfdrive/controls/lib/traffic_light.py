"""Traffic light guess for the HUD, from the driving model's own plan.

Ported 2026-11-27 from OPTECH-KR/openpilot optech-spst-dev
(selfdrive/controls/lib/optech_signal.py), itself StarPilot's stop light
detector. The model has no traffic light output, so the light is read from
what the model plans to do:

- red: the model's 10 s path ends well short of where the current speed would
  take the car, and no lead car explains it (or the model stops clearly
  before the lead). On/off hysteresis, a speed-dependent filter and a short
  hold keep it from flickering. Nothing above MAX_SPEED, nothing while
  turning at low speed with the blinker on.
- green: after stopping for a red, the model plans to go again for
  GREEN_TIME. green_event is True for that one update (the alert).

Changes from OPTECH, from a replay of 6.6 h of our own rlogs (11-23..11-26,
50 stops at the front of the queue): no red while the model's path bends
more than TURN_PATH_Y sideways (it slows for the turn, not a light); a red has
to last RED_SHOW_DELAY while moving before it shows (most false reds were
sub-second flickers); and pulling away from a red stop turns the light green
at once instead of leaving the red on its hold. Red shown at 37 of 50 such
stops (OPTECH 39), false reds while moving 41 -> 21 per hour (205 -> 68 s/h),
red after pulling away 17 -> 5 s/h, green 18 -> 30 of 50.

Display and alert only: it never touches the car's controls.
"""
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.realtime import DT_MDL

THRESHOLD = 1. - 1. / 2.718281828459045  # a filter at this has seen 'true' for about its time constant
MODEL_TIME = 8.0           # s: the model's 10 s horizon minus 2
PLANNER_TIME = 10.0
CRUISING_SPEED = 5.0       # m/s; a path shorter than CRUISING_SPEED * PLANNER_TIME means 'stays stopped'
ON_MARGIN = 2.5            # m below the stop threshold -> stopping
OFF_MARGIN = 4.0           # m above the threshold -> not stopping
HOLD_STRONG_MARGIN = 10.0  # m: a clear stop keeps the red HOLD_TIME
HOLD_TIME = 4.0
LEAD_BLOCK_MARGIN = 15.0   # m: a lead within threshold + this may be why the model stops
MODEL_LEAD_MARGIN = 3.0    # m: the model stops at least this far before the lead -> a light before it
MODEL_BRAKE_MIN = 0.5      # m/s^2 the model must plan to brake for 'stops before the lead'
HANDOFF_MAX_LEAD_SPEED = 2.0  # m/s: a red stays red behind a lead that has (almost) stopped
MAX_SPEED = 100. / 3.6
LOW_SPEED_FILTER = 0.35    # s, below 56 km/h
HIGH_SPEED_FILTER = 0.8    # s, from 72 km/h
LIGHT_BOOST_HIGH = 1.2
LIGHT_MAX_TIME = 9.0
LEAD_CLEAR_LOW, LEAD_CLEAR_HIGH = 0.6, 0.35  # s
SPEED_BP = [0., 56. / 3.6, 72. / 3.6]
TURN_MAX_SPEED = 24. / 3.6
TURN_STEER_DEG = 45.
STOPPED_SPEED = 0.3        # m/s
GREEN_TIME = 0.3           # s the model has to plan to go before it is a green light
GREEN_SHOW_TIME = 3.0      # s the green stays on the HUD (or until the car moves off)
TURN_PATH_Y = 5.0          # m: the model path bends this far sideways -> a turn, not a light
RED_SHOW_DELAY = 1.0       # s a red has to last while moving before it shows

NONE, RED, GREEN = 0, 1, 2


class TrafficLight:
  def __init__(self):
    self.light_filter = FirstOrderFilter(0., HIGH_SPEED_FILTER, DT_MDL)
    self.lead_clear_filter = FirstOrderFilter(0., LEAD_CLEAR_LOW, DT_MDL)
    self.model_stopping = False
    self.red = False
    self.hold_until = 0.
    self.stopped_for_light = False
    self.go_time = 0.
    self.green_until = 0.
    self.green_event = False
    self.state = NONE
    self.t = 0.
    self.red_time = 0.

  def reset_red(self):
    self.light_filter.x = 0.
    self.lead_clear_filter.x = 0.
    self.model_stopping = False
    self.red = False
    self.red_time = 0.
    self.hold_until = 0.

  def _show(self):
    return GREEN if self.t < self.green_until else (RED if self.red else NONE)

  def update(self, v_ego, model_length, model_accel, standstill, blinker, steering_angle,
             lead_status, lead_dist, lead_speed, path_y=0.):
    """Once per modelV2. v_ego m/s; model_length: modelV2.position.x[-1] (m in 10 s);
    model_accel: planned acceleration now (m/s^2); lead_*: radarState.leadOne;
    path_y: max |modelV2.position.y| (m)."""
    self.t += DT_MDL
    now = self.t
    self.green_event = False
    model_stopped = model_length < CRUISING_SPEED * PLANNER_TIME

    if standstill or v_ego < STOPPED_SPEED:
      if self.red or (self.stopped_for_light and model_stopped):
        self.stopped_for_light = True
      if self.stopped_for_light:
        if model_stopped:
          self.go_time = 0.
          self.red = True
        else:
          self.go_time += DT_MDL
          if self.go_time >= GREEN_TIME:
            self.green_event = True
            self.green_until = now + GREEN_SHOW_TIME
            self.stopped_for_light = False
            self.reset_red()
      self.state = self._show()
      return self.state

    if self.stopped_for_light:
      # pulled away from a red stop (the driver went before the model did): it is green
      self.green_until = now + GREEN_SHOW_TIME
      self.reset_red()
    self.stopped_for_light = False
    self.go_time = 0.
    if v_ego > 3.:
      self.green_until = 0.

    if (blinker and v_ego <= TURN_MAX_SPEED and abs(steering_angle) >= TURN_STEER_DEG) or v_ego > MAX_SPEED or path_y > TURN_PATH_Y:
      self.reset_red()
      self.state = self._show()
      return self.state

    self.light_filter.update_alpha(interp(v_ego, SPEED_BP, [LOW_SPEED_FILTER, LOW_SPEED_FILTER, HIGH_SPEED_FILTER]))
    self.lead_clear_filter.update_alpha(interp(v_ego, SPEED_BP, [LEAD_CLEAR_LOW, LEAD_CLEAR_LOW, LEAD_CLEAR_HIGH]))
    boost = interp(v_ego, SPEED_BP, [1., 1., LIGHT_BOOST_HIGH])
    cap = interp(v_ego, SPEED_BP, [0., 0., 1.])
    model_time = MODEL_TIME * boost
    if cap > 0.:
      model_time = min(model_time, LIGHT_MAX_TIME * cap + MODEL_TIME * (1. - cap))
    stop_threshold = max(v_ego * model_time, 0.)
    if self.model_stopping:
      self.model_stopping = model_length < stop_threshold + OFF_MARGIN
    else:
      self.model_stopping = model_length < max(stop_threshold - ON_MARGIN, 0.)

    lead_relevant = lead_status and lead_dist < stop_threshold + LEAD_BLOCK_MARGIN
    target_before_lead = (self.model_stopping and lead_relevant and v_ego > CRUISING_SPEED and
                          model_accel <= -MODEL_BRAKE_MIN and model_length + MODEL_LEAD_MARGIN < lead_dist)
    handoff = lead_relevant and self.red and lead_speed < HANDOFF_MAX_LEAD_SPEED
    if target_before_lead or handoff:
      lead_cleared = True
    else:
      self.lead_clear_filter.update(0. if lead_relevant else 1.)
      lead_cleared = self.lead_clear_filter.x >= THRESHOLD

    self.light_filter.update(1. if (self.model_stopping and lead_cleared) else 0.)
    detector = self.light_filter.x >= THRESHOLD ** 2 and lead_cleared
    if detector and (model_stopped or model_length < max(stop_threshold - HOLD_STRONG_MARGIN, 0.)):
      self.hold_until = now + HOLD_TIME
    hold_ok = (not lead_relevant) or target_before_lead
    self.red = bool(detector or handoff or (hold_ok and now < self.hold_until))
    self.red_time = self.red_time + DT_MDL if self.red else 0.
    shown_red = self.red and self.red_time >= RED_SHOW_DELAY - 1e-6
    self.state = GREEN if now < self.green_until else (RED if shown_red else NONE)
    return self.state
