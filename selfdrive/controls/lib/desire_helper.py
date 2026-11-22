import math

from cereal import log
from common.realtime import DT_MDL
from common.conversions import Conversions as CV
from common.params import Params
from selfdrive.controls.lib.atc_desire import (
  SideState, classify_maneuver_type, BLINKER_NONE, BLINKER_LEFT, BLINKER_RIGHT,
)

AUTO_LCA_START_TIME = 1.0

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
Desire = log.LateralPlan.Desire

# carrot's minimum (desire_lib/constants.py LANE_CHANGE_SPEED_MIN), for the
# driver blinker and AutoTurnControl alike: navigation lane changes happen in
# town, before turns and exits. Was 50 km/h (comma's value) until 2026-11-22.
# Turns have no minimum.
LANE_CHANGE_SPEED_MIN = 30 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.
ATC_LANE_CHANGE_SPEED_MIN = LANE_CHANGE_SPEED_MIN

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: Desire.none,
    LaneChangeState.preLaneChange: Desire.none,
    LaneChangeState.laneChangeStarting: Desire.none,
    LaneChangeState.laneChangeFinishing: Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: Desire.none,
    LaneChangeState.preLaneChange: Desire.none,
    LaneChangeState.laneChangeStarting: Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: Desire.none,
    LaneChangeState.preLaneChange: Desire.none,
    LaneChangeState.laneChangeStarting: Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: Desire.laneChangeRight,
  },
}

TURN_LEFT, TURN_RIGHT, TURN_NONE = Desire.turnLeft, Desire.turnRight, Desire.none

ATC_LANE_TYPES = ("fork left", "fork right", "atc left", "atc right")

# Turn latch. Once a turn desire starts it stays on until the car has turned.
# carrot dropped it when the wheel passed 80 deg and the model's yaw rate was
# falling, i.e. at the apex, and the navigation moves on to the next
# instruction a few metres past the turn point, which also ended it: replaying
# 2026-11-22--01-11-43 (12 intersection turns) the desire lasted ~2 s per turn.
# Heading is integrated from the wheel angle (Equinox geometry; carState.yawRate
# reads 0 for whole segments on this car).
TURN_LATCH_WHEELBASE = 2.72
TURN_LATCH_STEER_RATIO = 16.8
TURN_DONE_HEADING_DEG = 45.   # turned at least this much ...
TURN_DONE_WHEEL_DEG = 45.     # ... and the wheel is unwinding below this
TURN_DONE_MAX_HEADING_DEG = 100.  # or turned this much, whatever the wheel
TURN_ABORT_DIST_M = 120.      # still not turned this far after the start (approach <= 60 m + arc)
TURN_ABORT_WRONG_DEG = 30.    # turned this much the other way
TURN_ABORT_SPEED = 40 * CV.KPH_TO_MS
TURN_REARM_S = 10.            # after a finished turn, no new turn desire for this long


class DesireHelper:
  """Lane change / turn desire.

  Two paths share one lane-change state:
    - the driver's blinker alone: the original 0.8.14 logic (_update_legacy),
      unchanged;
    - an AutoTurnControl request (navigation turn / fork / pre-turn lane
      change), alone or with the driver's blinker on the same side: carrot's
      DesireHelper logic (_update_atc), ported from carrotpilot carrot-wip
      77dd33357a73 desire_helper.py with its default settings
      (LaneChangeNeedTorque 0, LaneChangeBsd 0, LaneLineCheck 0,
      LaneChangeDelay 0). Left out: the Tesla / HKG lamp control, voice and
      Bluetooth requests, the lever counters and the App LANECHANGE command
      -- none exist on this car.
  A driver blinker opposite to the ATC side cancels ATC until the navigation
  instruction changes (carrot's atc_active = 2) and hands back to the legacy path.

  Departure from carrot: an ATC lane change (fork / pre-turn lane) or
  intersection turn never starts on its own. carrot lights the lamp itself; this car cannot (gm/carcontroller.py
  sends no turn signal) and has no blind-spot signal, so a self-started change
  would cross lanes with no signal to the cars behind. It needs the driver's
  blinker on the same side or a nudge of the wheel toward that side.
  """

  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = Desire.none

    params = Params()
    self.lane_change_enabled = params.get_bool('LaneChangeEnabled')
    self.auto_lane_change_enabled = params.get_bool('AutoLaneChangeEnabled')
    self.auto_lane_change_timer = 0.0
    self.prev_torque_applied = False

    # ── AutoTurnControl (carrot) ──
    self.left = SideState("left")
    self.right = SideState("right")
    self.maneuver_type = "none"     # "none" / "turn" / "lane_change"
    self.turn_direction = TURN_NONE
    self.enable_turn_desires = True
    self.turn_desire_state = False
    self.turn_disable_count = 0
    self.blinker_ignore = False
    self.atc_type = ""
    self.atc_active = 0             # 0: none, 1: ATC acting, 2: cancelled by an opposite driver blinker
    self.auto_lane_change_enable = False
    self.next_lane_change = False
    self.prev_desire_enabled = False
    self.atc_owner = False          # the current maneuver belongs to the ATC path
    self.atc_turn_confirmed = False  # driver confirmed this ATC turn (blinker or nudge)
    self.atc_blinker = BLINKER_NONE  # side ATC asks for this frame (UI / log)
    # turn latch (see TURN_DONE_*)
    self.turn_latched = TURN_NONE
    self.turn_heading_deg = 0.      # turned toward the latched side so far
    self.turn_dist_m = 0.
    self.turn_end_reason = ""
    self.sides_valid = False

  # ─────────────────────────────────────────────
  # entry
  # ─────────────────────────────────────────────
  def update(self, carstate, active, lane_change_prob, modeldata=None, atc_type=""):
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    driver_st = (BLINKER_LEFT if carstate.leftBlinker else 0) | (BLINKER_RIGHT if carstate.rightBlinker else 0)

    if modeldata is not None:
      self._process_sides(carstate, modeldata)
      self._check_desire_state(modeldata, carstate)
    else:
      self.sides_valid = False

    if not self.lane_change_enabled or not self.sides_valid:
      atc_type = ""
    atc_st, atc_enabled = self._update_atc_blinker(atc_type, driver_st)
    self.atc_blinker = atc_st

    use_atc = atc_st != BLINKER_NONE or (self.atc_owner and driver_st == BLINKER_NONE)
    if use_atc:
      self._update_atc(carstate, active, lane_change_prob, driver_st, atc_st, atc_enabled)
      self.atc_owner = (self.lane_change_state != LaneChangeState.off or
                        self.turn_direction != TURN_NONE)
    else:
      if self.atc_owner:
        # Driver took over with the other blinker (or ATC ended mid-way under a
        # driver blinker): drop the ATC maneuver and let the legacy path start fresh.
        self._reset_maneuver()
        self.atc_owner = False
        self.prev_one_blinker = False
      self.prev_desire_enabled = False
      self._update_legacy(carstate, active, lane_change_prob, one_blinker)

    self._apply_turn_latch(carstate, active, driver_st)

    if self.sides_valid:
      # carrot commits every frame: lane_change_available_released is an edge.
      self.left.commit_last()
      self.right.commit_last()
    self.prev_one_blinker = one_blinker
    self._finish_desire()

  def _reset_maneuver(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.turn_direction = TURN_NONE
    self.maneuver_type = "none"
    self.lane_change_timer = 0.0

  def _finish_desire(self):
    if self.turn_direction != TURN_NONE:
      self.desire = self.turn_direction
      self.lane_change_direction = (LaneChangeDirection.left if self.turn_direction == TURN_LEFT
                                    else LaneChangeDirection.right)
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (Desire.keepLeft, Desire.keepRight):
        self.desire = Desire.none

  # ─────────────────────────────────────────────
  # legacy (0.8.14) driver-blinker path, unchanged
  # ─────────────────────────────────────────────
  def _update_legacy(self, carstate, active, lane_change_prob, one_blinker):
    v_ego = carstate.vEgo
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

  # ─────────────────────────────────────────────
  # carrot: model / sides
  # ─────────────────────────────────────────────
  def _process_sides(self, carstate, modeldata):
    try:
      lines, probs, edges = modeldata.laneLines, modeldata.laneLineProbs, modeldata.roadEdges
      if len(lines) < 4 or len(probs) < 4 or len(edges) < 2:
        self.sides_valid = False
        return
      # left: outer laneLines[0], current laneLines[1], edge[0], cur_prob laneLineProbs[1]
      self.left.update_lane_geometry(lines[0], probs[0], lines[1], edges[0], cur_prob=probs[1])
      # right: outer laneLines[3], current laneLines[2], edge[1], cur_prob laneLineProbs[2]
      self.right.update_lane_geometry(lines[3], probs[3], lines[2], edges[1], cur_prob=probs[2])
    except (ValueError, IndexError, TypeError):
      self.sides_valid = False
      return
    self.sides_valid = True

    # No camera lane-line type on this car: raw 0 (carrot reads HKG leftLaneLine).
    self.left.update_lane_line_info(0)
    self.right.update_lane_line_info(0)

    ignore_bsd = False   # LaneChangeBsd 0
    self.left.update_obstacles(carstate.leftBlindspot, ignore_bsd, bsd_hold_sec=2.0)
    self.right.update_obstacles(carstate.rightBlindspot, ignore_bsd, bsd_hold_sec=2.0)

    # LaneLineCheck 0: only a yellow line (HKG colour 2) blocks; raw 0 here.
    left_line_ok = self.left.lane_line_info_raw // 10 != 2
    right_line_ok = self.right.lane_line_info_raw // 10 != 2
    self.left.compute_lane_change_available(lane_line_info_lt_20=left_line_ok, ignore_bsd=ignore_bsd)
    self.right.compute_lane_change_available(lane_line_info_lt_20=right_line_ok, ignore_bsd=ignore_bsd)

    self.left.update_triggers()
    self.right.update_triggers()

  def _check_desire_state(self, modeldata, carstate):
    try:
      desire_state = modeldata.meta.desireState
      self.turn_desire_state = len(desire_state) > 2 and (desire_state[1] + desire_state[2]) > 0.1
    except (AttributeError, IndexError, TypeError):
      self.turn_desire_state = False

    # carrot set turn_disable_count here when the wheel passed 80 deg with the
    # model's yaw rate falling -- the apex, mid-turn. The turn latch now ends
    # the turn and sets it (_apply_turn_latch).
    self.turn_disable_count = max(0, self.turn_disable_count - 1)

  def _apply_turn_latch(self, carstate, active, driver_st):
    """Hold a started turn desire until the car has turned (TURN_DONE_*)."""
    if self.turn_latched == TURN_NONE:
      if self.turn_direction == TURN_NONE:
        return
      self.turn_latched = self.turn_direction
      self.turn_heading_deg = 0.
      self.turn_dist_m = 0.
      self.turn_end_reason = ""

    left = self.turn_latched == TURN_LEFT
    v = max(0., carstate.vEgo)
    k = math.tan(math.radians(carstate.steeringAngleDeg) / TURN_LATCH_STEER_RATIO) / TURN_LATCH_WHEELBASE
    self.turn_heading_deg += math.degrees(k * v * DT_MDL) * (1. if left else -1.)
    self.turn_dist_m += v * DT_MDL

    opposite_blinker = driver_st == (BLINKER_RIGHT if left else BLINKER_LEFT)
    opposite_press = carstate.steeringPressed and (
      carstate.steeringTorque < 0 if left else carstate.steeringTorque > 0)
    wheel_toward = carstate.steeringAngleDeg * (1. if left else -1.)

    reason = ""
    if self.turn_heading_deg >= TURN_DONE_MAX_HEADING_DEG or (
        self.turn_heading_deg >= TURN_DONE_HEADING_DEG and wheel_toward < TURN_DONE_WHEEL_DEG):
      reason = "done"
    elif not active:
      reason = "inactive"
    elif opposite_blinker or opposite_press:
      reason = "cancel"
    elif v > TURN_ABORT_SPEED:
      reason = "speed"
    elif self.turn_heading_deg <= -TURN_ABORT_WRONG_DEG:
      reason = "wrong way"
    elif self.turn_dist_m > TURN_ABORT_DIST_M and self.turn_heading_deg < TURN_DONE_HEADING_DEG:
      reason = "distance"

    if reason:
      self.turn_end_reason = reason
      self.turn_latched = TURN_NONE
      self.turn_direction = TURN_NONE
      self.lane_change_direction = LaneChangeDirection.none
      if reason == "done":
        self.turn_disable_count = int(TURN_REARM_S / DT_MDL)
      return

    # still turning: keep the desire whatever the navigation or the FSM say
    self.turn_direction = self.turn_latched
    self.maneuver_type = "turn"
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.left if left else LaneChangeDirection.right
    self.atc_owner = True

  def _get_selected_side(self, blinker_state):
    return self.left if blinker_state == BLINKER_LEFT else self.right

  @staticmethod
  def _is_last_lane(side):
    return side.lane_exist_count.counter <= 0 and not side.lane_change_available_geom

  def _update_atc_blinker(self, atc_type, driver_blinker_state):
    atc_blinker_state = BLINKER_NONE

    if atc_type in ("turn left", "turn right"):
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type == "turn left" else BLINKER_RIGHT
        self.atc_active = 1
        self.blinker_ignore = False
    elif atc_type in ATC_LANE_TYPES:
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type in ("fork left", "atc left") else BLINKER_RIGHT
        self.atc_active = 1
    else:
      self.atc_active = 0

    # Opposite driver blinker: ATC off
    if driver_blinker_state != BLINKER_NONE and atc_blinker_state != BLINKER_NONE and driver_blinker_state != atc_blinker_state:
      atc_blinker_state = BLINKER_NONE
      self.atc_active = 2

    atc_desire_enabled = atc_blinker_state in (BLINKER_LEFT, BLINKER_RIGHT)

    # blinker_ignore
    if driver_blinker_state == BLINKER_NONE:
      self.blinker_ignore = False
    if self.blinker_ignore:
      atc_blinker_state = BLINKER_NONE
      atc_desire_enabled = False

    # Ignore one frame when the type changes
    if self.atc_type != atc_type:
      atc_desire_enabled = False
      self.atc_turn_confirmed = False   # a new instruction needs a new confirmation
    self.atc_type = atc_type

    return atc_blinker_state, atc_desire_enabled

  # ─────────────────────────────────────────────
  # carrot: ATC maneuver FSM
  # ─────────────────────────────────────────────
  def _update_atc(self, carstate, lateral_active, lane_change_prob, driver_st, atc_st, atc_enabled):
    v_ego = carstate.vEgo
    below_lane_change_speed = v_ego < ATC_LANE_CHANGE_SPEED_MIN
    driver_enabled = driver_st in (BLINKER_LEFT, BLINKER_RIGHT)

    desire_enabled = driver_enabled or atc_enabled
    blinker_state = driver_st if driver_enabled else atc_st

    side = self._get_selected_side(blinker_state) if blinker_state in (BLINKER_LEFT, BLINKER_RIGHT) else None
    atc_lane_change_requested = atc_enabled and self.atc_type in ATC_LANE_TYPES
    # carrot lets a right fork start on its own; here any ATC request (fork,
    # pre-turn lane, or a turn read as a lane change) needs the driver's
    # blinker or nudge, since the car cannot light its own lamp.
    atc_needs_driver = atc_enabled and not driver_enabled
    atc_lane_change_manual_only = atc_lane_change_requested and not driver_enabled
    atc_lane_change_only = atc_lane_change_requested and not driver_enabled
    # Do not treat a blocked->available retry as permission to cross solid/unknown lines.
    atc_lane_change_retry_line_blocked = (
      atc_lane_change_only and
      side is not None and
      side.lane_line_info_mod not in (0, 5)
    )

    auto_lane_change_trigger = False
    if desire_enabled and side is not None:
      auto_lane_change_trigger = (
        self.auto_lane_change_enable and
        (not atc_needs_driver) and
        side.edge_available and
        (side.lane_available_trigger or side.lane_appeared) and
        (not side.side_object_detected) and
        (side.bsd_hold_counter == 0)
      )
    else:
      self.auto_lane_change_enable = False
      self.next_lane_change = False

    # ───────────────────────── FSM ─────────────────────────
    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self._reset_maneuver()

    else:
      if desire_enabled and side is not None:
        new_type = classify_maneuver_type(
          blinker_state=blinker_state,
          carstate=carstate,
          side=side,
          turn_desire_state=self.turn_desire_state,
          atc_type=self.atc_type,
          old_type=self.maneuver_type,
        )
      else:
        new_type = "none"

      # switching rules
      if self.maneuver_type == "lane_change" and new_type == "turn" and self.lane_change_state not in (
        LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting
      ):
        self.maneuver_type = "turn"
        self.lane_change_state = LaneChangeState.off
      elif self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
        self.maneuver_type = new_type

      # ─ TURN mode ─
      # A fork / lane request read as a turn at low speed must not steer across
      # without the driver either.
      if atc_lane_change_manual_only and self.maneuver_type == "turn":
        self.maneuver_type = "lane_change"

      # An ATC turn waits for the driver too: the blinker on that side, or a
      # nudge toward it, which then holds until the instruction changes.
      turn_nudge = carstate.steeringPressed and (
        (carstate.steeringTorque > 0) if blinker_state == BLINKER_LEFT else (carstate.steeringTorque < 0))
      if driver_enabled or turn_nudge:
        self.atc_turn_confirmed = True

      if desire_enabled and self.maneuver_type == "turn" and self.enable_turn_desires:
        self.lane_change_state = LaneChangeState.off
        if self.turn_disable_count > 0 or not (driver_enabled or self.atc_turn_confirmed):
          self.turn_direction = TURN_NONE
          self.lane_change_direction = LaneChangeDirection.none
        else:
          self.turn_direction = TURN_LEFT if blinker_state == BLINKER_LEFT else TURN_RIGHT
          self.lane_change_direction = (LaneChangeDirection.left if blinker_state == BLINKER_LEFT
                                        else LaneChangeDirection.right)

      # ─ Lane change FSM ─
      else:
        self.turn_direction = TURN_NONE

        if self.lane_change_state == LaneChangeState.off:
          if desire_enabled and not self.prev_desire_enabled and not below_lane_change_speed and side is not None:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_ll_prob = 1.0
            # Not the last lane on that side: no automatic ATC lane change
            self.auto_lane_change_enable = self._is_last_lane(side)
            self.next_lane_change = False

        elif self.lane_change_state == LaneChangeState.preLaneChange:
          if side is None:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
          else:
            self.lane_change_direction = LaneChangeDirection.left if blinker_state == BLINKER_LEFT else LaneChangeDirection.right

            torque_cond = (carstate.steeringTorque > 0) if blinker_state == BLINKER_LEFT else (carstate.steeringTorque < 0)
            torque_applied = carstate.steeringPressed and torque_cond

            # LaneChangeBsd 0: BSD holds, torque overrides it
            bsd_active = side.bsd_hold_counter > 0
            side_clear_without_line = (side.lane_available or side.edge_available) and \
                                      (not side.side_object_detected) and (not bsd_active)
            atc_driver_confirm = atc_lane_change_requested and driver_enabled
            atc_geometry_release = atc_lane_change_only and auto_lane_change_trigger
            atc_line_release = (atc_driver_confirm or atc_geometry_release) and side_clear_without_line

            # Arm automatic ATC only after this side has actually become the last lane.
            # Keep it latched so a newly appearing lane can start the maneuver later.
            if atc_lane_change_only and self._is_last_lane(side):
              self.auto_lane_change_enable = True

            if not desire_enabled or below_lane_change_speed:
              self.lane_change_state = LaneChangeState.off
              self.lane_change_direction = LaneChangeDirection.none
            else:
              # LaneLineCheck 0: no solid-line block; LaneChangeDelay 0
              block_released = side.lane_change_available_released
              block_released_auto = block_released and (driver_enabled or self.auto_lane_change_enable) and \
                                    not atc_lane_change_retry_line_blocked
              start_gate = side.lane_change_available_geom or \
                           side.lane_line_info_edge_detect or block_released_auto or atc_line_release
              can_start = False
              if start_gate:
                if bsd_active:
                  if torque_applied:
                    can_start = True
                elif self.next_lane_change:   # LaneChangeNeedTorque 0
                  if torque_applied:
                    can_start = True
                elif driver_enabled:
                  if side.lane_change_available or atc_line_release:
                    can_start = True
                else:
                  if torque_applied or ((not atc_needs_driver) and (
                    auto_lane_change_trigger or side.lane_line_info_edge_detect or block_released_auto
                  )):
                    if side.lane_change_available or atc_line_release:
                      can_start = True
              if can_start:
                self.lane_change_state = LaneChangeState.laneChangeStarting

        elif self.lane_change_state == LaneChangeState.laneChangeStarting:
          self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
          if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
            self.lane_change_state = LaneChangeState.laneChangeFinishing

        elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
          if self.lane_change_ll_prob > 0.99:
            self.lane_change_direction = LaneChangeDirection.none
            if desire_enabled:
              self.lane_change_state = LaneChangeState.preLaneChange
              # Another change in the same direction needs the driver's nudge.
              self.next_lane_change = True
            else:
              self.lane_change_state = LaneChangeState.off

    # timer
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_desire_enabled = desire_enabled

    # Opposite steering torque cancels
    steering_pressed_cancel = carstate.steeringPressed and (
      (carstate.steeringTorque < 0 and blinker_state == BLINKER_LEFT) or
      (carstate.steeringTorque > 0 and blinker_state == BLINKER_RIGHT)
    )
    if steering_pressed_cancel and self.lane_change_state != LaneChangeState.off:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state = LaneChangeState.off
      self.blinker_ignore = True
