from cereal import log
from common.realtime import DT_MDL
from common.conversions import Conversions as CV
from common.params import Params

AUTO_LCA_START_TIME = 1.0

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 50 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

# A straight-road lane change and an intersection turn are indistinguishable at
# the blinker -- both are "blinker on, under LANE_CHANGE_SPEED_MIN". They part
# ways at the wheel. Through steerRatio 16.8 and a 2.72 m wheelbase, the
# 90-degree turn this fork targets (curvature 0.067-0.10) needs roughly 175-260
# deg of steering wheel, while a lane change peaks near 60. Only call it a turn
# once the wheel is past anything a lane change would ask for, so a lane change
# can never pull a turn desire. Hysteresis keeps the desire alive while the
# wheel unwinds through the corner exit.
TURN_DESIRE_ENTER_STEER_DEG = 120.0
TURN_DESIRE_EXIT_STEER_DEG = 60.0

# The model publishes its own read of which desire fits the scene in
# modelV2.meta.desireState, and it does so from vision alone -- in the
# 2026-08-26 city drive nothing ever fed it a turn desire, yet turnLeft/
# turnRight still peaked at 0.81/0.86. Using it as the primary trigger is what
# lets the desire land BEFORE the driver turns in, which the steering-angle
# gate above can never do. Sample size behind this threshold is 3 turn-ins
# (pre-turn peaks 0.569 / 0.188 / 0.033), so treat it as a starting point.
# Explicit driver declaration. The GM bus carries only BCMTurnSignals
# (off/left/right) -- stalk press duration is not on the bus at all -- so every
# gesture has to be read off the lamp's on/off pattern.
# Double tap: off then back on in the same direction inside this gap. A driver
# never does that by accident.
#   A plain "hold the blinker a long time" gesture was measured and dropped:
#   over 44 min (routes 2026-08-26--01-04-22 and --03-31-54) blinker episodes
#   that led to a >120 deg turn ran 6.7-56.6 s while ones that did not ran up
#   to 44.8 s, so declaring at 3 s mis-fired on 12 of 24 non-turn episodes and
#   even 7 s still mis-fired on 6 while losing a real turn. No usable split.
TURN_DESIRE_DOUBLE_TAP_GAP_S = 1.5
# A declaration outlives the blinker: a tapped stalk self-cancels a few seconds
# later, and the turn still has to be driven. Cleared early by the opposite
# blinker or by the corner finishing.
TURN_DECLARE_TIMEOUT_S = 20.0

TURN_DESIRE_MODEL_PROB = 0.35
# Index into the DESIRE_LEN = 8 one-hot, matching LateralPlan.Desire.
MODEL_DESIRE_TURN_LEFT = 1
MODEL_DESIRE_TURN_RIGHT = 2

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

    # Turn desire defaults on; write "0" to the param to disable without a rebuild.
    turn_desire_param = Params().get('TurnDesireEnabled')
    self.turn_desire_enabled = turn_desire_param not in (b'0', '0')
    self.turn_desire_direction = LaneChangeDirection.none
    self.turn_pulse_timer = 0.0
    self.blinker_phase_s = 0.0
    self.last_blinker_dir = LaneChangeDirection.none
    self.prev_blinker_dir = LaneChangeDirection.none
    self.turn_declared_dir = LaneChangeDirection.none
    self.turn_declare_s = 0.0

  def update(self, carstate, active, lane_change_prob, model_desire_state=None):
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

    # Turn desire. Below LANE_CHANGE_SPEED_MIN a blinker means an intersection
    # turn, not a lane change, and the state machine above deliberately parks
    # at LaneChangeState.off there -- so nothing was ever signalled to the
    # model. turnLeft/turnRight ride the same one-hot desire slot as the lane
    # change desires (modeld builds vec_desire[desire] = 1.0 with no special
    # casing), so populating self.desire here is the whole wiring.
    base_ok = (active and self.turn_desire_enabled and
               below_lane_change_speed and not carstate.standstill and
               self.lane_change_state == LaneChangeState.off)
    turn_ok = base_ok and one_blinker

    # --- driver declaration, tracked independently of the blinker so it can
    # outlive a stalk that self-cancels.
    blinker_dir = LaneChangeDirection.none
    if one_blinker:
      blinker_dir = (LaneChangeDirection.left if carstate.leftBlinker
                     else LaneChangeDirection.right)

    # blinker_phase_s is time spent in the current lamp state; on a rising edge
    # it is therefore the length of the gap that just ended.
    if blinker_dir != self.prev_blinker_dir:
      if (blinker_dir != LaneChangeDirection.none and
          self.last_blinker_dir == blinker_dir and
          self.blinker_phase_s <= TURN_DESIRE_DOUBLE_TAP_GAP_S):
        self.turn_declared_dir = blinker_dir
        self.turn_declare_s = TURN_DECLARE_TIMEOUT_S
      self.blinker_phase_s = 0.0
    self.blinker_phase_s += DT_MDL
    if blinker_dir != LaneChangeDirection.none:
      self.last_blinker_dir = blinker_dir
    self.prev_blinker_dir = blinker_dir

    # Signalling the other way is an explicit change of mind.
    if (blinker_dir != LaneChangeDirection.none and
        self.turn_declared_dir != LaneChangeDirection.none and
        blinker_dir != self.turn_declared_dir):
      self.turn_declared_dir = LaneChangeDirection.none
      self.turn_declare_s = 0.0

    if self.turn_declare_s > 0.0:
      self.turn_declare_s -= DT_MDL
    else:
      self.turn_declared_dir = LaneChangeDirection.none

    # Three ways in:
    #   1. driver declaration (double tap) -> the only trigger that lands
    #      before the wheel moves, and the only one that survives a turn taken
    #      without the blinker latched;
    #   2. the model reads the scene as a turn -> measured at 0/18 pre-turn
    #      detections on 2026-08-26--01-04-22, so it contributes nothing today;
    #      kept because it also produced 0 spurious fires and one route is thin
    #      evidence to delete on;
    #   3. the wheel is past anything a lane change asks for -> backstop for
    #      unsignalled turns, which were 13 of 18 on that same route.
    # ISO 8855: positive steeringAngleDeg is a left turn. Model and wheel both
    # additionally need the blinker to agree, so neither fires on its own; a
    # declaration stands alone by design.
    steer_deg = carstate.steeringAngleDeg
    already_turning = self.turn_desire_direction != LaneChangeDirection.none
    threshold = TURN_DESIRE_EXIT_STEER_DEG if already_turning else TURN_DESIRE_ENTER_STEER_DEG

    model_left = model_right = False
    if model_desire_state is not None and len(model_desire_state) > MODEL_DESIRE_TURN_RIGHT:
      model_left = model_desire_state[MODEL_DESIRE_TURN_LEFT] > TURN_DESIRE_MODEL_PROB
      model_right = model_desire_state[MODEL_DESIRE_TURN_RIGHT] > TURN_DESIRE_MODEL_PROB

    declared_left = base_ok and self.turn_declared_dir == LaneChangeDirection.left
    declared_right = base_ok and self.turn_declared_dir == LaneChangeDirection.right
    signalled_left = turn_ok and carstate.leftBlinker and (model_left or steer_deg > threshold)
    signalled_right = turn_ok and carstate.rightBlinker and (model_right or steer_deg < -threshold)

    if declared_left or signalled_left:
      self.turn_desire_direction = LaneChangeDirection.left
    elif declared_right or signalled_right:
      self.turn_desire_direction = LaneChangeDirection.right
    else:
      self.turn_desire_direction = LaneChangeDirection.none
      self.turn_pulse_timer = 0.0

    if self.turn_desire_direction != LaneChangeDirection.none:
      # modeld only forwards a desire to the model on a rising edge, so a held
      # value reaches it exactly once. Re-assert for a single frame once per
      # second, the same trick the keep pulse above uses.
      if self.turn_pulse_timer <= 0.0:
        self.turn_pulse_timer = 1.0
        self.desire = (log.LateralPlan.Desire.turnLeft
                       if self.turn_desire_direction == LaneChangeDirection.left
                       else log.LateralPlan.Desire.turnRight)
      else:
        self.turn_pulse_timer -= DT_MDL
        self.desire = log.LateralPlan.Desire.none
