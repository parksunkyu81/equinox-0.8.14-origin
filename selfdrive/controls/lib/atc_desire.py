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

  # availability
  lane_available: bool = False
  edge_available: bool = False

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


class AtcLaneGuide:
  """Tells the driver when to change lanes for a navigation instruction.

  The car never makes an ATC lane change (desire_helper.py): the driver steers
  across. controlsd runs this on every modelV2 frame and raises the prompt
  alert while `prompt` is set:
    - when a fork / pre-turn lane instruction comes into range and the model
      sees a lane on that side, the prompt shows for PROMPT_S with one chime;
    - the driver's blinker on that side means the driver is on it: silent;
      once it goes off and the lane lines have settled (SETTLE_S), a lane still
      there means another change is needed, and the prompt shows again;
    - still not moved REMIND_S after a prompt: one reminder;
    - no lane on that side for SETTLE_S: in the right lane, silent;
    - an opposite blinker cancels it until the instruction changes.
  At most MAX_PROMPTS per instruction, so a flickering lane line cannot nag.
  Intersection turns ("turn left/right") get no lane prompt.
  """
  PROMPT_S = 4.
  REMIND_S = 10.
  SETTLE_S = 2.
  MAX_PROMPTS = 4

  def __init__(self):
    self.left = SideState("left")
    self.right = SideState("right")
    self._reset("")

  def _reset(self, atc_type):
    self.atc_type = atc_type
    self.state = "idle"     # idle / prompt / wait / driver / settle / cancel
    self.t = 0.
    self.gone_t = 0.
    self.reminded = False
    self.prompts = 0
    self.prompt = BLINKER_NONE

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

  def _start_prompt(self, want):
    self.state = "prompt"
    self.t = 0.
    self.prompts += 1
    self.prompt = want

  def update(self, atc_type, md, left_blinker, right_blinker, dt=DT_MDL):
    valid = self._geometry(md)
    if atc_type != self.atc_type:
      self._reset(atc_type)
    self.prompt = BLINKER_NONE

    if atc_type in ATC_LANE_LEFT_TYPES:
      want = BLINKER_LEFT
    elif atc_type in ATC_LANE_RIGHT_TYPES:
      want = BLINKER_RIGHT
    else:
      self.state = "idle"
      return self.prompt
    if not valid:
      return self.prompt

    own = left_blinker if want == BLINKER_LEFT else right_blinker
    other = right_blinker if want == BLINKER_LEFT else left_blinker
    lane_there = (self.left if want == BLINKER_LEFT else self.right).lane_available
    self.t += dt

    if other or self.state == "cancel":
      self.state = "cancel"
      return self.prompt
    if own:
      self.state = "driver"
      self.t = 0.
      return self.prompt
    if self.state == "driver":       # blinker just went off
      self.state = "settle"
      self.t = 0.
      return self.prompt
    if self.state == "settle":
      if self.t < self.SETTLE_S:
        return self.prompt
      self.state = "idle"
      self.reminded = False

    self.gone_t = 0. if lane_there else self.gone_t + dt
    if self.state in ("prompt", "wait") and self.gone_t > self.SETTLE_S:
      self.state = "idle"            # in the right lane now
      self.reminded = False
    if self.prompts >= self.MAX_PROMPTS and self.state != "prompt":
      return self.prompt

    if self.state == "idle":
      if lane_there:
        self._start_prompt(want)
      return self.prompt
    if self.state == "prompt":
      if self.t < self.PROMPT_S:
        self.prompt = want
        return self.prompt
      self.state = "wait"
      self.t = 0.
    if self.state == "wait" and lane_there and not self.reminded and self.t >= self.REMIND_S:
      self.reminded = True
      self._start_prompt(want)
    return self.prompt
