"""Lane geometry and maneuver classification for AutoTurnControl lane changes / turns.

Ported from carrotpilot (ajouatom/openpilot, branch carrot-wip, commit
77dd33357a73, 2026-11-21), openpilot/selfdrive/controls/lib/desire_lib/:
  - ExistCounter            <- hysteresis.py, unchanged
  - calculate_lane_width    <- lane_math.py, unchanged
  - classify_maneuver_type  <- maneuver_classifier.py, unchanged
  - SideState               <- side_state.py, geometry / triggers / availability
    unchanged. Obstacles keep only the blind-spot hold: carrot's side-radar
    checks read radarState.leadLeft/leadRight and corner radar tracks, which
    0.8.14 (and this car) do not have, so side_object_detected stays False.
    The Equinox has no blind-spot signal either (gm/carstate.py never sets
    left/rightBlindspot), so the hold is wired but idle.
"""
from bisect import bisect_right
from dataclasses import dataclass, field
from collections import deque

from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS

BLINKER_NONE = 0
BLINKER_LEFT = 1
BLINKER_RIGHT = 2
BLINKER_BOTH = 3


class ExistCounter:
  def __init__(self, sustain_sec: float = 0.2):
    self.counter = 0
    self.true_count = 0
    self.false_count = 0
    self.threshold = int(sustain_sec / DT_MDL)

  def update(self, exist_flag: bool):
    if exist_flag:
      self.true_count += 1
      self.false_count = 0
      if self.true_count >= self.threshold:
        self.counter = max(self.counter + 1, 1)
    else:
      self.false_count += 1
      self.true_count = 0
      if self.false_count >= self.threshold:
        self.counter = min(self.counter - 1, -1)
    return self.counter


def _t_weights(t):
  i = min(max(bisect_right(T_IDXS, t) - 1, 0), len(T_IDXS) - 2)
  return i, (t - T_IDXS[i]) / (T_IDXS[i + 1] - T_IDXS[i])

# Lane width is read at 1 s and 2 s on the model's fixed time grid. np.interp
# turned the whole 33-point capnp list into an array on every call: ~100 us
# each on the EON, 8 per model frame (0.9 of the 1.0 ms _process_sides took,
# with or without ATC). The two neighbouring points give the same value.
_T1 = _t_weights(1.0)
# SideState.room: the road edge this far from our lane line (1 s and 2 s
# ahead) means a lane fits on that side. In the rightmost lane on the
# 2026-11-22--22-42-17 drive the edge read 1.3-3.0 m.
EDGE_ROOM_M = 3.5
_T2 = _t_weights(2.0)


def _y_at(y, iw):
  i, w = iw
  return y[i] * (1. - w) + y[i + 1] * w


def calculate_lane_width(lane, lane_prob, current_lane, road_edge):
  current_lane_y = _y_at(current_lane.y, _T1)
  lane_y = _y_at(lane.y, _T1)
  distance_to_lane = abs(current_lane_y - lane_y)

  road_edge_y = _y_at(road_edge.y, _T1)
  distance_to_road_edge = abs(current_lane_y - road_edge_y)
  distance_to_road_edge_far = abs(current_lane_y - _y_at(road_edge.y, _T2))

  lane_valid = lane_prob > 0.5
  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge, distance_to_road_edge_far, lane_valid


@dataclass
class SideState:
  name: str  # "left" / "right"

  # lane/edge distances
  lane_width: float = 0.0
  lane_width_diff: float = 0.0
  dist_to_edge: float = 0.0
  dist_to_edge_far: float = 0.0

  # current lane prob (ego lane line prob on that side)
  cur_prob: float = 1.0
  current_lane_missing: bool = False

  # counters
  lane_exist_count: ExistCounter = field(default_factory=lambda: ExistCounter(0.2))
  lane_width_count: ExistCounter = field(default_factory=lambda: ExistCounter(0.2))
  edge_count: ExistCounter = field(default_factory=lambda: ExistCounter(0.2))
  edge_room_count: ExistCounter = field(default_factory=lambda: ExistCounter(0.2))

  # availability
  lane_available: bool = False
  edge_available: bool = False
  # Room for the car on this side, for the ATC tap lane change: a confident
  # outer lane line (prob > 0.5) a lane width away, or the road edge at least
  # EDGE_ROOM_M from our lane line. lane_available alone also trusts the
  # outer line's position when the model gives it ~0 probability, and
  # edge_available counts a 2.5 m shoulder as room.
  room: bool = False

  # smoothing
  lane_width_queue: deque = field(default_factory=lambda: deque(maxlen=int(1.0 / DT_MDL)))

  # lane line info (HKG camera lane type; 0 on this car)
  lane_line_info_raw: int = 0
  lane_line_info_mod: int = 0
  last_lane_line_mod: int = 0
  lane_line_info_edge_detect: bool = False

  # transitions
  lane_available_last: bool = False
  edge_available_last: bool = False
  lane_available_trigger: bool = False
  lane_appeared: bool = False

  # obstacles
  side_object_detected: bool = False

  # BSD hold (after detection)
  bsd_hold_counter: int = 0
  bsd_detected_now: bool = False

  # computed lane change availability (includes BSD+object)
  lane_change_available_geom: bool = False
  lane_change_available: bool = False
  lane_change_available_last: bool = False
  lane_change_available_released: bool = False
  lane_width_sum: float = 0.0

  def update_lane_geometry(self, lane_outer, lane_outer_prob, lane_current, road_edge, cur_prob: float):
    lane_w, dist_edge, dist_edge_far, lane_valid = calculate_lane_width(
      lane_outer, lane_outer_prob, lane_current, road_edge
    )

    self.lane_exist_count.update(bool(lane_valid))

    # running mean (O(1))
    if len(self.lane_width_queue) == self.lane_width_queue.maxlen:
      self.lane_width_sum -= self.lane_width_queue.popleft()
    self.lane_width_queue.append(lane_w)
    self.lane_width_sum += lane_w
    self.lane_width = self.lane_width_sum / len(self.lane_width_queue)

    self.lane_width_diff = (self.lane_width_queue[-1] - self.lane_width_queue[0]) if len(self.lane_width_queue) >= 2 else 0.0

    self.dist_to_edge = float(dist_edge)
    self.dist_to_edge_far = float(dist_edge_far)

    min_lane_width = 2.5
    self.lane_width_count.update(self.lane_width > min_lane_width)
    self.edge_count.update(self.dist_to_edge > min_lane_width)

    available_count = int(0.2 / DT_MDL)
    self.lane_available = self.lane_width_count.counter > available_count
    self.edge_available = (self.edge_count.counter > available_count) and (self.dist_to_edge_far > min_lane_width)

    self.edge_room_count.update(self.dist_to_edge > EDGE_ROOM_M and self.dist_to_edge_far > EDGE_ROOM_M)
    self.room = ((self.lane_available and self.lane_exist_count.counter > available_count) or
                 self.edge_room_count.counter > available_count)

    self.cur_prob = float(cur_prob)
    self.current_lane_missing = self.cur_prob < 0.3

  def update_lane_line_info(self, lane_line_info_raw: int):
    self.lane_line_info_raw = int(lane_line_info_raw)
    mod = self.lane_line_info_raw % 10
    # edge_detect: true on the transition into an allowed 0/5 lane-line type.
    self.lane_line_info_edge_detect = (mod in (0, 5)) and (self.last_lane_line_mod not in (0, 5))
    self.last_lane_line_mod = mod
    self.lane_line_info_mod = mod

  def update_obstacles(self, blindspot: bool, ignore_bsd: bool, bsd_hold_sec: float = 2.0):
    # Hold BSD detection for a short period after it is observed.
    self.bsd_detected_now = bool(blindspot)
    if self.bsd_detected_now and not ignore_bsd:
      self.bsd_hold_counter = int(bsd_hold_sec / DT_MDL)
    elif not ignore_bsd:
      self.bsd_hold_counter = max(0, self.bsd_hold_counter - 1)
    else:
      self.bsd_hold_counter = 0

  def compute_lane_change_available(self, lane_line_info_lt_20: bool, ignore_bsd: bool):
    # geometric availability
    self.lane_change_available_geom = (self.lane_available or self.edge_available) and lane_line_info_lt_20

    # Include BSD/object checks in final lane-change availability.
    bsd_active = (self.bsd_hold_counter > 0) and (not ignore_bsd)
    self.lane_change_available = self.lane_change_available_geom and (not self.side_object_detected) and (not bsd_active)
    self.lane_change_available_released = self.lane_change_available and not self.lane_change_available_last

  def update_triggers(self):
    # Preserve the existing lane_available_trigger behavior.
    self.lane_available_trigger = False
    if self.lane_width_diff > 0.8 and (self.lane_width < self.dist_to_edge):
      self.lane_available_trigger = True

    # Use >= so the appeared state is not missed after the threshold.
    # Limit distant edges so intersections do not over-trigger lane_appeared.
    appeared_now = self.lane_exist_count.counter >= int(0.2 / DT_MDL)
    self.lane_appeared = (self.lane_appeared or appeared_now) and (self.dist_to_edge < 4.0)

  def commit_last(self):
    self.lane_available_last = self.lane_available
    self.edge_available_last = self.edge_available
    self.lane_change_available_last = self.lane_change_available


def classify_maneuver_type(blinker_state: int, carstate, side: SideState, turn_desire_state: bool,
                           atc_type: str, old_type: str):
  if blinker_state == 0:
    return "none"

  v_kph = carstate.vEgo * 3.6
  accel = carstate.aEgo

  score_turn = 0
  if v_kph < 30.0:
    score_turn += 1
  elif v_kph < 40.0 and accel < -1.0:
    score_turn += 1

  # no lane line or road edge on that side -> more likely a turn
  if v_kph < 40.0 and (not side.lane_available) and (not side.edge_available):
    score_turn += 1

  # the outer lane line only just appeared
  if v_kph < 40.0 and side.lane_exist_count.counter < int(0.5 / DT_MDL):
    score_turn += 1

  if turn_desire_state:
    score_turn += 1

  if atc_type in ("turn left", "turn right"):
    score_turn += 2
  elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
    score_turn -= 2

  edge_far = side.dist_to_edge_far > 4.0

  if score_turn >= 2:
    if edge_far:
      return "turn"
    return old_type
  return "lane_change"


ATC_LEFT_TYPES = ("turn left", "fork left", "atc left")
ATC_RIGHT_TYPES = ("turn right", "fork right", "atc right")
ATC_LANE_LEFT_TYPES = ("fork left", "atc left")
ATC_LANE_RIGHT_TYPES = ("fork right", "atc right")


# What the ATC prompt tells the driver, chosen when it comes up and held for
# it (events.py reads PROMPT_STATE through the alert callback; both run in
# controlsd):
#   PROMPT_AUTO   "○측 차선으로 이동하세요 / 깜빡이를 톡 치면 차가 옮깁니다"
#                 -- lane stage, from ATC_TAP speed (15 km/h) with room there,
#                 so the tap moves the car (desire_helper.py)
#   PROMPT_TURN   "○회전 준비 / 깜빡이를 켜 주세요" -- turn stage: the blinker
#                 sends the turn desire, room does not matter
#   PROMPT_MANUAL "○측 차선으로 이동하세요 / 직접 옮겨 주세요" -- otherwise
PROMPT_AUTO, PROMPT_TURN, PROMPT_MANUAL = "auto", "turn", "manual"
# voice: the turn prompt speaks "좌회전 / 우회전 깜빡이를 켜 주세요" instead of the
# chime (False when the driver's blinker on that side is already on).
PROMPT_STATE = {"kind": PROMPT_MANUAL, "voice": False}
# "좌/우회전 중" while the turn desire is out (controlsd fills it each cycle,
# events.py turn_in_progress_alert reads it). sub: "" / "sat" (the steering is
# at its 300 limit) / "slow" (under 10 km/h, where LKAS is off).
TURN_STATE = {"left": False, "sub": ""}
PROMPT_AUTO_SPEED_MIN = 15. / 3.6   # = desire_helper.ATC_TAP_SPEED_MIN
# A turn instruction ("atc left/right" is its lane stage) this close to the
# point is the turn: the prompt comes at ~45 m, and the navigation's turn
# stage only starts at 43-60 m, so on the 2026-11-23 drives most turn prompts
# came up as "차선으로 이동하세요" instead of "회전 준비".
PROMPT_TURN_M = 60.
# Turn instructions (xTurnInfo 1 / 2) get their own spoken prompt once the point
# is this close (2026-11-24, on request). A blinker put on 60-40 m out is what
# starts the turn: inside desire_helper.ATC_TURN_NO_LANE_CHANGE_M (60) it cannot
# start a lane change, and it is still on at ATC_TURN_FORCE_M (40), where the
# turn stage reads it. 55 leaves the navigation distance (updated about once a
# second, 14 m at 50 km/h) room to land inside 60.
TURN_PROMPT_M = 55.
# For a turn instruction the lane prompt ("○측 차선으로 이동하세요") comes only
# this far out; closer than this it would land seconds before the turn prompt
# and be cut off by it, so the turn prompt is the only one.
TURN_LANE_PROMPT_MIN_M = 80.


class AtcLaneGuide:
  """Tells the driver once, just before the point, which way the navigation goes.

  The car never makes an ATC lane change (desire_helper.py): the driver steers.
  controlsd runs this on every modelV2 frame and raises the prompt alert
  ("좌측/우측 차선으로 이동하세요") while `prompt` is set.

  One prompt per instruction (turn, pre-turn lane or fork), PROMPT_S long with
  one chime, when the point is PROMPT_AHEAD_S away at the current speed, but
  never further than PROMPT_MAX_M nor closer than PROMPT_MIN_M: about 45-50 m
  before an intersection in town (the turn stage), 100-130 m before a fork at
  60-80 km/h. Not on the straight road far before the point (2026-11-23, on
  request: the prompt at the start of the 160-350 m range came too early).
  It does not look at the lanes: the camera cannot tell a yellow centre line
  from a white one. Shown even when the driver's blinker on the instruction's
  side is already on (2026-11-23, on request: 9 of 24 instructions had no
  prompt for that reason), and always for the full PROMPT_S (on request: a
  blinker put on right after it came up hid it after 1.2-1.8 s). Re-armed after the instruction has been gone
  REARM_S, and at once when a new one starts: the side changes, or the
  distance jumps back up by NEW_INSTR_JUMP_M (2026-11-23--01-36-34: a left
  turn right after a right turn, and a right turn right after a left, got no
  prompt).
  """
  PROMPT_S = 4.
  REARM_S = 5.
  PROMPT_AHEAD_S = 6.
  PROMPT_MIN_M = 45.
  PROMPT_MAX_M = 150.
  NEW_INSTR_JUMP_M = 30.

  def __init__(self):
    # Its own copy of DesireHelper's side geometry (same SideState, same model
    # frames) for SideState.room, which lives in plannerd.
    self.left = SideState("left")
    self.right = SideState("right")
    self.kind = PROMPT_MANUAL
    self.prompted = False
    self.turn_prompted = False   # the spoken turn prompt of this instruction is done
    self.t = 0.
    self.away_t = 0.
    self.prompt = BLINKER_NONE
    self.last_want = BLINKER_NONE
    self.last_dist = 0.

  def _geometry(self, md):
    try:
      lines, probs, edges = md.laneLines, md.laneLineProbs, md.roadEdges
      if len(lines) < 4 or len(probs) < 4 or len(edges) < 2:
        return False
      self.left.update_lane_geometry(lines[0], probs[0], lines[1], edges[0], cur_prob=probs[1])
      self.right.update_lane_geometry(lines[3], probs[3], lines[2], edges[1], cur_prob=probs[2])
    except (ValueError, IndexError, TypeError, AttributeError):
      return False
    return True

  def _pick_kind(self, atc_type, want, v_ego, room_ok, dist_m):
    if atc_type.startswith(("turn left", "turn right")):
      return PROMPT_TURN
    if atc_type.startswith(("atc left", "atc right")) and 0. < dist_m <= PROMPT_TURN_M:
      return PROMPT_TURN
    side = self.left if want == BLINKER_LEFT else self.right
    if room_ok and side.room and v_ego >= PROMPT_AUTO_SPEED_MIN:
      return PROMPT_AUTO
    return PROMPT_MANUAL

  def trigger_m(self, v_ego):
    return min(self.PROMPT_MAX_M, max(self.PROMPT_MIN_M, v_ego * self.PROMPT_AHEAD_S))

  def update(self, atc_type, dist_m, v_ego, left_blinker, right_blinker, dt=DT_MDL, md=None, turn_info=0):
    room_ok = self._geometry(md) if md is not None else False
    want = BLINKER_NONE
    if not atc_type.endswith("prepare"):
      if atc_type.startswith(ATC_LEFT_TYPES):
        want = BLINKER_LEFT
      elif atc_type.startswith(ATC_RIGHT_TYPES):
        want = BLINKER_RIGHT

    if want == BLINKER_NONE:
      self.away_t += dt
      if self.away_t >= self.REARM_S:
        self.prompted = False
        self.turn_prompted = False
      self.prompt = BLINKER_NONE
      return self.prompt
    self.away_t = 0.

    # a new instruction re-arms at once
    if want != self.last_want or dist_m > max(self.last_dist, 0.) + self.NEW_INSTR_JUMP_M:
      self.prompted = False
      self.turn_prompted = False
      self.prompt = BLINKER_NONE
    self.last_want = want
    self.last_dist = float(dist_m)

    is_turn = turn_info in (1, 2) or atc_type.startswith(("turn left", "turn right"))
    if is_turn and not self.turn_prompted and 0. < dist_m <= TURN_PROMPT_M:
      # The turn prompt, spoken; it replaces a lane prompt still on screen.
      self.prompted = self.turn_prompted = True
      self.t = 0.
      self.prompt = want
      self.kind = PROMPT_TURN
      blinker_on = left_blinker if want == BLINKER_LEFT else right_blinker
      PROMPT_STATE.update(kind=self.kind, voice=not blinker_on)
    elif not self.prompted:
      trigger = self.trigger_m(v_ego)
      if is_turn:
        trigger = trigger if trigger >= TURN_LANE_PROMPT_MIN_M else 0.
      if 0. < dist_m <= trigger:
        self.prompted = True
        self.t = 0.
        self.prompt = want
        self.kind = self._pick_kind(atc_type, want, v_ego, room_ok, dist_m)
        PROMPT_STATE.update(kind=self.kind, voice=False)
    elif self.prompt != BLINKER_NONE:
      self.t += dt
      if self.t >= self.PROMPT_S:   # always the full PROMPT_S, blinker or not
        self.prompt = BLINKER_NONE
    return self.prompt
