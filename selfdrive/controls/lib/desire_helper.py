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
# ...and a floor under it, because the lamp state bounces. Across the two
# 2026-08-26 city routes the detector saw exactly two same-direction gaps and
# BOTH were artifacts, not gestures: 0.23 s at t=225 (declared left, the driver
# then turned right) and 0.55 s at t=931 (48 km/h, no turn at all). The driver
# has not once actually used this gesture in 108 min of logs, so treat the whole
# declaration path as unproven -- this floor only rules out what is too fast for
# a hand on a stalk.
TURN_DESIRE_DOUBLE_TAP_MIN_GAP_S = 0.3
# A declaration outlives the blinker: a tapped stalk self-cancels a few seconds
# later, and the turn still has to be driven. The timeout only runs while the
# car is moving, so declaring before a red light and turning after it still
# works. It is the backstop, not the normal exit -- the normal exit is the
# corner finishing (TURN_DECLARE_DONE_S below).
TURN_DECLARE_TIMEOUT_S = 20.0
# Once the wheel has actually gone round, the corner is over as soon as it
# unwinds and stays unwound this long. Without this the declaration ran its
# full timeout and kept pulsing turn desire at the model long after the turn,
# which stops the car settling back onto lane centre.
TURN_DECLARE_DONE_S = 0.5
# Turn desire only arms at intersection speed. LANE_CHANGE_SPEED_MIN (50 km/h)
# is far too loose a bar for "this must be an intersection": on
# 2026-08-26--09-33-54 a right blinker tapped twice 0.54 s apart at 48 km/h --
# a lane-change signal, not a turn -- declared a turn and pushed turnRight at
# the model for 7.4 s through a left-hand bend, pinning the model's own
# desireState[turnRight] at 0.97 the whole way. Turn-in speed across the two
# 2026-08-26 city routes never exceeded 23.6 km/h, so 30 leaves margin for the
# approach while shutting out the 31-48 km/h band where the false fire lived.
TURN_DESIRE_MAX_SPEED = 30 * CV.KPH_TO_MS

# GM's BCMTurnSignals state drops long before the wheel moves: on the same
# route 9 of 11 turns past 120 deg had the lamp already off at turn-in. The
# worst case signalled for 1.1 s, went dark 1.5 s BEFORE the wheel started
# moving, and then took 445 deg of steering with no desire, because both the
# model and wheel triggers below require a blinker to agree with. Latch the
# last direction for a few seconds so they still have something to agree with.
TURN_BLINKER_LATCH_S = 4.0

TURN_DESIRE_MODEL_PROB = 0.35
# The model's desireState is downstream of what we feed it: one pulse of
# turnRight pinned modelV2.meta.desireState[turnRight] at 0.97 for 9 s on
# 2026-08-26--09-33-54 t=931, so the model trigger can keep re-arming itself off
# its own echo -- a latch with no way out. Give a model-only desire a short
# leash: if the wheel has not gone the same way by then, drop it and lock the
# model trigger out until the driver signals again.
TURN_DESIRE_MODEL_MAX_S = 3.0
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
    self.turn_started = False
    self.turn_unwound_s = 0.0
    self.blinker_latch_dir = LaneChangeDirection.none
    self.blinker_latch_s = 0.0
    self.turn_model_only_s = 0.0
    self.model_trigger_blocked = False

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
               v_ego <= TURN_DESIRE_MAX_SPEED and not carstate.standstill and
               self.lane_change_state == LaneChangeState.off)

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
          TURN_DESIRE_DOUBLE_TAP_MIN_GAP_S <= self.blinker_phase_s <= TURN_DESIRE_DOUBLE_TAP_GAP_S and
          v_ego <= TURN_DESIRE_MAX_SPEED):
        self.turn_declared_dir = blinker_dir
        self.turn_declare_s = TURN_DECLARE_TIMEOUT_S
      if blinker_dir != LaneChangeDirection.none:
        # A fresh stalk gesture is the one thing that lifts the model lockout.
        self.model_trigger_blocked = False
      self.blinker_phase_s = 0.0
    self.blinker_phase_s += DT_MDL
    if blinker_dir != LaneChangeDirection.none:
      self.last_blinker_dir = blinker_dir
    self.prev_blinker_dir = blinker_dir

    # Blinker latch: hold the last signalled direction for a few seconds after
    # the lamp goes dark, so a stalk that self-cancels before turn-in still
    # counts. Frozen at standstill for the same reason the declaration timeout
    # is -- signalling, stopping at the light, then turning is the normal case.
    if blinker_dir != LaneChangeDirection.none:
      self.blinker_latch_dir = blinker_dir
      self.blinker_latch_s = TURN_BLINKER_LATCH_S
    elif self.blinker_latch_s > 0.0:
      if not carstate.standstill:
        self.blinker_latch_s -= DT_MDL
      if self.blinker_latch_s <= 0.0:
        self.blinker_latch_dir = LaneChangeDirection.none
    else:
      self.blinker_latch_dir = LaneChangeDirection.none

    turn_ok = base_ok and self.blinker_latch_dir != LaneChangeDirection.none

    # Signalling the other way is an explicit change of mind.
    if (blinker_dir != LaneChangeDirection.none and
        self.turn_declared_dir != LaneChangeDirection.none and
        blinker_dir != self.turn_declared_dir):
      self.turn_declared_dir = LaneChangeDirection.none
      self.turn_declare_s = 0.0

    # Corner finished: the wheel went round and has come back. This is the
    # normal way a declaration ends.
    if self.turn_declared_dir != LaneChangeDirection.none:
      if abs(carstate.steeringAngleDeg) > TURN_DESIRE_EXIT_STEER_DEG:
        self.turn_started = True
        self.turn_unwound_s = 0.0
      elif self.turn_started:
        self.turn_unwound_s += DT_MDL
        if self.turn_unwound_s >= TURN_DECLARE_DONE_S:
          self.turn_declared_dir = LaneChangeDirection.none
          self.turn_declare_s = 0.0
    else:
      self.turn_started = False
      self.turn_unwound_s = 0.0

    # Timeout backstop, held while stopped so a red light doesn't eat it.
    if self.turn_declare_s > 0.0:
      if not carstate.standstill:
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
    # Sign: positive steeringAngleDeg is a LEFT turn. Verified 2026-08-26 rather
    # than assumed, because the model frame is the trap here -- modelV2 y is
    # positive to the RIGHT (laneLines[1], the left line, sits at y = -1.38 and
    # laneLines[2], the right line, at +1.49), and liveLocationKalman's
    # calibrated frame is z-down, so both of those read backwards if taken for
    # the usual left-positive convention. Cross-checks that agree: at t=105 on
    # 2026-08-26--07-38-23 a left blinker accompanied +348 deg of wheel, and the
    # laneChangeRight episodes on --09-33-54 all ran -5 deg of mean wheel angle.
    # Model and wheel both additionally need a blinker (latched, above) to
    # agree, so neither fires on its own; a declaration stands alone by design.
    steer_deg = carstate.steeringAngleDeg
    already_turning = self.turn_desire_direction != LaneChangeDirection.none
    threshold = TURN_DESIRE_EXIT_STEER_DEG if already_turning else TURN_DESIRE_ENTER_STEER_DEG

    model_left = model_right = False
    if (model_desire_state is not None and not self.model_trigger_blocked and
        len(model_desire_state) > MODEL_DESIRE_TURN_RIGHT):
      model_left = model_desire_state[MODEL_DESIRE_TURN_LEFT] > TURN_DESIRE_MODEL_PROB
      model_right = model_desire_state[MODEL_DESIRE_TURN_RIGHT] > TURN_DESIRE_MODEL_PROB

    declared_left = base_ok and self.turn_declared_dir == LaneChangeDirection.left
    declared_right = base_ok and self.turn_declared_dir == LaneChangeDirection.right
    signalled_left = (turn_ok and self.blinker_latch_dir == LaneChangeDirection.left and
                      (model_left or steer_deg > threshold))
    signalled_right = (turn_ok and self.blinker_latch_dir == LaneChangeDirection.right and
                       (model_right or steer_deg < -threshold))

    if declared_left or signalled_left:
      self.turn_desire_direction = LaneChangeDirection.left
    elif declared_right or signalled_right:
      self.turn_desire_direction = LaneChangeDirection.right
    else:
      self.turn_desire_direction = LaneChangeDirection.none
      self.turn_pulse_timer = 0.0

    # A declaration is the driver speaking and keeps its own timeout above; only
    # a model-only desire is on the leash, and only the wheel turning the same
    # way counts as backing it up -- the 931 false fire had 106 deg of wheel,
    # all of it the other way.
    if self.turn_desire_direction == LaneChangeDirection.left:
      wheel_backs_desire = steer_deg > TURN_DESIRE_EXIT_STEER_DEG
    elif self.turn_desire_direction == LaneChangeDirection.right:
      wheel_backs_desire = steer_deg < -TURN_DESIRE_EXIT_STEER_DEG
    else:
      wheel_backs_desire = False

    if (self.turn_desire_direction != LaneChangeDirection.none and
        not (declared_left or declared_right) and not wheel_backs_desire):
      self.turn_model_only_s += DT_MDL
      if self.turn_model_only_s >= TURN_DESIRE_MODEL_MAX_S:
        self.model_trigger_blocked = True
        self.turn_desire_direction = LaneChangeDirection.none
        self.turn_pulse_timer = 0.0
    else:
      self.turn_model_only_s = 0.0

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
