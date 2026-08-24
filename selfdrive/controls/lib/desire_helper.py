from cereal import log
from common.realtime import DT_MDL
from common.conversions import Conversions as CV
from common.params import Params

AUTO_LCA_START_TIME = 1.0

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 50 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

# --- Step 1 of the hands-free 90-degree-turn goal: OBSERVATION ONLY ---
# The driving model already accepts a turn desire (modeld reads
# lateralPlan.desire and one-hots it into the model), and Desire.turnLeft /
# turnRight are defined, but nothing in this repo ever sets them. Before
# building anything on top of that, we need to know one thing: does this model
# actually predict a 90-degree turn path when told a turn is coming?
#
# This wiring answers that from a drive log without touching control. It is
# gated to the DISENGAGED state on purpose -- feeding a desire changes what the
# model predicts, and that prediction feeds the MPC, so doing this while
# engaged would change steering. Disengaged, the prediction is logged and
# steers nothing.
#
# Set to False to disable the experiment entirely.
TURN_DESIRE_OBSERVE = True
# Only below lane-change speed, so this can never shadow a real lane change
# (which requires >= LANE_CHANGE_SPEED_MIN and owns the blinker up there).
TURN_DESIRE_MAX_SPEED = LANE_CHANGE_SPEED_MIN
TURN_DESIRE_MIN_SPEED = 1.0  # m/s, must actually be rolling
# modeld turns desire into a rising-edge pulse, so a held value reaches the
# model exactly once. Re-assert it periodically, the way the keepLeft/keepRight
# pulse already does, so a multi-second turn keeps signalling.
TURN_DESIRE_REPULSE_S = 1.0

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.LateralPlan.Desire.none

    self.lane_change_enabled = Params().get_bool('LaneChangeEnabled')
    self.auto_lane_change_enabled = Params().get_bool('AutoLaneChangeEnabled')
    self.auto_lane_change_timer = 0.0
    self.prev_torque_applied = False

    self.turn_desire_pulse_timer = 0.0
    self.turn_desire_active = False

  def update(self, carstate, active, lane_change_prob):
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if (not active) or (self.lane_change_timer > LANE_CHANGE_TIME_MAX) or (not one_blinker) or (not self.lane_change_enabled):
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      torque_applied = carstate.steeringPressed and \
                       ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                        (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right)) or \
                        self.auto_lane_change_enabled and \
                       (AUTO_LCA_START_TIME+0.25) > self.auto_lane_change_timer > AUTO_LCA_START_TIME

      blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                            (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

      # State transitions
      # off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        if carstate.leftBlinker:
          self.lane_change_direction = LaneChangeDirection.left
        elif carstate.rightBlinker:
          self.lane_change_direction = LaneChangeDirection.right

        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
        elif torque_applied and (not blindspot_detected or self.prev_torque_applied):
          self.lane_change_state = LaneChangeState.laneChangeStarting
        elif torque_applied and blindspot_detected and self.auto_lane_change_timer != 10.0:
          self.auto_lane_change_timer = 10.0
        elif not torque_applied and self.auto_lane_change_timer == 10.0 and not self.prev_torque_applied:
          self.prev_torque_applied = True

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    if self.lane_change_state == LaneChangeState.off:
      self.auto_lane_change_timer = 0.0
      self.prev_torque_applied = False
    elif self.auto_lane_change_timer < (AUTO_LCA_START_TIME+0.25): # stop afer 3 sec resume from 10 when torque applied
      self.auto_lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.LateralPlan.Desire.keepLeft, log.LateralPlan.Desire.keepRight):
        self.desire = log.LateralPlan.Desire.none

    self._update_turn_desire(carstate, active, v_ego, one_blinker)

  def _update_turn_desire(self, carstate, active, v_ego, one_blinker):
    """Signal turn intent to the model from the blinker, while disengaged only.

    Runs last so it can only fill a desire slot the lane-change state machine
    left empty -- it never overrides a real lane change.
    """
    eligible = bool(
      TURN_DESIRE_OBSERVE and
      not active and
      self.desire == log.LateralPlan.Desire.none and
      one_blinker and
      TURN_DESIRE_MIN_SPEED <= v_ego < TURN_DESIRE_MAX_SPEED
    )
    if not eligible:
      self.turn_desire_pulse_timer = 0.0
      self.turn_desire_active = False
      return

    self.turn_desire_active = True
    self.turn_desire_pulse_timer += DT_MDL
    if self.turn_desire_pulse_timer >= TURN_DESIRE_REPULSE_S:
      # Hold `none` for one frame so the next frame is a fresh rising edge and
      # modeld emits another pulse.
      self.turn_desire_pulse_timer = 0.0
      self.desire = log.LateralPlan.Desire.none
    else:
      self.desire = (log.LateralPlan.Desire.turnLeft if carstate.leftBlinker
                     else log.LateralPlan.Desire.turnRight)
