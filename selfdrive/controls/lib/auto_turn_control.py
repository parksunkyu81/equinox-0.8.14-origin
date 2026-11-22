"""AutoTurnControl (ATC): navigation-driven lane changes, turns and turn slowdown.

Ported from carrotpilot (ajouatom/openpilot, branch carrot-wip, commit
77dd33357a73, 2026-11-21), openpilot/selfdrive/carrot/carrot_serv.py:
  - TURN_TYPE_MAPPING    <- CarrotServ._update_tbt() turn_type_mapping
  - update_tbt()         <- the GuidePoint block of CarrotServ.update() + _update_tbt()
  - update()             <- the TBT part of CarrotServ.update_navi()
  - _update_auto_turn()  <- CarrotServ.update_auto_turn()
  - calculate_current_speed() <- CarrotServ.calculate_current_speed()
The decisions (thresholds, distances, mapping, pause rule, mode switch) are
carrot's, unchanged. What differs is only the plumbing:
  - Input is TmapNda's 'apilot' object, which arrives on UDP 2843 inside the
    road_limit packet road_speed_limiter.py already receives. It is the same
    JSON TmapNda sends carrotpilot on UDP 7706 (nTBTDist, nTBTTurnType, ...).
  - Distance between packets is counted down from vEgo * dt (carrot uses
    selfdriveState.distanceTraveled, which 0.8.14 does not have).
  - A stale link (no packet for STALE_S) clears the turn, like carrot's
    active_carrot <= 1 reset.
  - The map/road screen switch (AutoTurnMapChange) is not ported: EON has no
    map screen.
"""
import math

import numpy as np

# carrot_serv.py _update_tbt(): nTBTTurnType -> (navType, navModifier, xTurnInfo)
# xTurnInfo: 1 left turn, 2 right turn, 3 left fork/ramp, 4 right fork/ramp,
#            5 rotary, 6 tollgate, 7 u-turn, 8 arrive, 0 notification
TURN_TYPE_MAPPING = {
  12: ("turn", "left", 1),
  16: ("turn", "sharp left", 1),
  13: ("turn", "right", 2),
  19: ("turn", "sharp right", 2),
  102: ("off ramp", "slight left", 3),
  105: ("off ramp", "slight left", 3),
  112: ("off ramp", "slight left", 3),
  115: ("off ramp", "slight left", 3),
  101: ("off ramp", "slight right", 4),
  104: ("off ramp", "slight right", 4),
  111: ("off ramp", "slight right", 4),
  114: ("off ramp", "slight right", 4),
  7: ("fork", "left", 3),
  44: ("fork", "left", 3),
  17: ("fork", "left", 3),
  75: ("fork", "left", 3),
  76: ("fork", "left", 3),
  118: ("fork", "left", 3),
  6: ("fork", "right", 4),
  43: ("fork", "right", 4),
  73: ("fork", "right", 4),
  74: ("fork", "right", 4),
  123: ("fork", "right", 4),
  124: ("fork", "right", 4),
  117: ("fork", "right", 4),
  131: ("rotary", "slight right", 5),
  132: ("rotary", "slight right", 5),
  140: ("rotary", "slight left", 5),
  141: ("rotary", "slight left", 5),
  133: ("rotary", "right", 5),
  134: ("rotary", "sharp right", 5),
  135: ("rotary", "sharp right", 5),
  136: ("rotary", "sharp left", 5),
  137: ("rotary", "sharp left", 5),
  138: ("rotary", "sharp left", 5),
  139: ("rotary", "left", 5),
  142: ("rotary", "straight", 5),
  14: ("turn", "uturn", 7),
  201: ("arrive", "straight", 8),
  51: ("notification", "straight", 0),
  52: ("notification", "straight", 0),
  53: ("notification", "straight", 0),
  54: ("notification", "straight", 0),
  55: ("notification", "straight", 0),
  153: ("", "", 6),  # TG
  154: ("", "", 6),  # TG
  249: ("", "", 6),  # TG
}

# carrot_settings.json / params_keys.h defaults
DEFAULT_MODE = 0            # AutoTurnControl: 0 off, 1 lane/turn, 2 lane/turn + speed, 3 speed
DEFAULT_SPEED_TURN = 20     # AutoTurnControlSpeedTurn [km/h]
DEFAULT_TURN_END = 6        # AutoTurnControlTurnEnd [s]
# AutoNaviSpeedDecelRate [0.01 m/s^2]. carrot's default is 120 (1.2 m/s^2), set
# for cars that brake. This car slows by coasting only: the learned coast decel
# is 0.17-0.20 m/s^2 at 50-60 km/h (road_speed_limiter.py CAM_DECEL_MS2), so a
# 1.2 curve starts far too late and the car reaches the turn fast. 20 = 0.20.
DEFAULT_DECEL_RATE = 20
# Intersection right turns are held to this, lower than AutoTurnControlSpeedTurn.
# A 90-degree corner of ~10 m radius needs ~250-300 deg of wheel, and at the
# 300 torque cap openpilot alone reaches ~260 deg only at 11-15 km/h (~170 deg
# at 20-25). Left turns keep the param: they are wider. Never above the param.
RIGHT_TURN_SPEED_KPH = 15

NO_LIMIT_KPH = 250.
ATC_SAFE_SEC = 2.0          # update_auto_turn(): safe_sec = 2.0
STALE_S = 6.                # road_speed_limiter.py drops road_limit after 6 s


def calculate_current_speed(left_dist, safe_speed_kph, safe_time, safe_decel_rate):
  """carrot_serv.py CarrotServ.calculate_current_speed(), unchanged."""
  safe_speed = safe_speed_kph / 3.6
  safe_dist = safe_speed * safe_time
  decel_dist = left_dist - safe_dist

  if decel_dist <= 0:
    return safe_speed_kph

  # v_i^2 = v_f^2 + 2ad
  temp = safe_speed**2 + 2 * safe_decel_rate * decel_dist

  if temp < 0:
    speed_mps = safe_speed
  else:
    speed_mps = math.sqrt(temp)
  return max(safe_speed_kph, min(250, speed_mps * 3.6))


def _int(v, default):
  try:
    return int(v)
  except (TypeError, ValueError):
    return default


class AutoTurnControl:
  def __init__(self):
    # GuidePoint (carrot_serv.py __init__ defaults)
    self.nRoadLimitSpeed = 30
    self.nRoadLimitSpeed_counter = 0
    self.nTBTDist = 0
    self.nTBTTurnType = -1
    self.nTBTNextRoadWidth = 0
    self.nTBTDistNext = 0
    self.nTBTTurnTypeNext = -1

    self.navType, self.navModifier, self.xTurnInfo = "invalid", "", -1
    self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = "invalid", "", -1
    self.xDistToTurn = 0.
    self.xDistToTurnNext = 0.

    self.atc_paused = False
    self.atc_activate_count = 0

    # params
    self.autoTurnControl = DEFAULT_MODE
    self.autoTurnControlSpeedTurn = DEFAULT_SPEED_TURN
    self.autoTurnControlTurnEnd = DEFAULT_TURN_END
    self.autoNaviSpeedDecelRate = DEFAULT_DECEL_RATE * 0.01

    self.last_packet_t = -1e9

    # outputs
    self.atcType = "none"
    self.atcSpeed = 0.
    self.atcDist = 0.
    self.atcSpeedNext = 0.
    self.atcDistNext = 0.
    self.atc_desired = NO_LIMIT_KPH       # after the mode switch
    self.atc_desired_next = NO_LIMIT_KPH

  def set_params(self, mode, speed_turn, turn_end, decel_rate):
    self.autoTurnControl = int(mode)
    self.autoTurnControlSpeedTurn = int(speed_turn)
    self.autoTurnControlTurnEnd = int(turn_end)
    self.autoNaviSpeedDecelRate = float(decel_rate) * 0.01

  # ── packet ────────────────────────────────────────────────────────────────
  def update_tbt(self, apilot, now):
    """GuidePoint block of CarrotServ.update(): runs when a packet carries nRoadLimitSpeed."""
    if not isinstance(apilot, dict) or "nRoadLimitSpeed" not in apilot:
      return
    self.last_packet_t = now

    nRoadLimitSpeed = _int(apilot.get("nRoadLimitSpeed"), 20)
    if nRoadLimitSpeed > 0:
      if nRoadLimitSpeed > 200:
        nRoadLimitSpeed = (nRoadLimitSpeed - 20) / 10
      elif nRoadLimitSpeed == 120:
        nRoadLimitSpeed = 115  # 120 -> 115 fix bug
    else:
      nRoadLimitSpeed = 30
    if self.nRoadLimitSpeed != nRoadLimitSpeed:
      self.nRoadLimitSpeed_counter += 1
      if self.nRoadLimitSpeed_counter > 5:
        self.nRoadLimitSpeed = nRoadLimitSpeed
    else:
      self.nRoadLimitSpeed_counter = 0

    self.nTBTDist = _int(apilot.get("nTBTDist"), 0)
    self.nTBTTurnType = _int(apilot.get("nTBTTurnType"), -1)
    self.nTBTNextRoadWidth = _int(apilot.get("nTBTNextRoadWidth"), 0)
    self.nTBTDistNext = _int(apilot.get("nTBTDistNext"), 0)
    self.nTBTTurnTypeNext = _int(apilot.get("nTBTTurnTypeNext"), -1)
    self._update_tbt()

  def _update_tbt(self):
    if self.nTBTTurnType in TURN_TYPE_MAPPING:
      self.navType, self.navModifier, self.xTurnInfo = TURN_TYPE_MAPPING[self.nTBTTurnType]
    else:
      self.navType, self.navModifier, self.xTurnInfo = "invalid", "", -1

    if self.nTBTTurnTypeNext in TURN_TYPE_MAPPING:
      self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = TURN_TYPE_MAPPING[self.nTBTTurnTypeNext]
    else:
      self.navTypeNext, self.navModifierNext, self.xTurnInfoNext = "invalid", "", -1

    if self.nTBTDist > 0 and self.xTurnInfo > 0:
      self.xDistToTurn = self.nTBTDist
    if self.nTBTDistNext > 0 and self.xTurnInfoNext > 0:
      self.xDistToTurnNext = self.nTBTDistNext + self.nTBTDist

  # ── per cycle ─────────────────────────────────────────────────────────────
  def update(self, now, v_ego, delta_dist, steering_pressed, steering_torque):
    """TBT part of CarrotServ.update_navi(). Returns the ATC speed cap [km/h] (NO_LIMIT_KPH = none)."""
    self.xDistToTurn = self.xDistToTurn - delta_dist
    self.xDistToTurnNext = self.xDistToTurnNext - delta_dist

    if now - self.last_packet_t > STALE_S:
      # carrot: active_carrot <= 1 -> xTurnInfo = xTurnInfoNext = -1, nTBTTurnType* = -1
      self.navType = "invalid"
      self.xTurnInfo = self.xTurnInfoNext = -1
      self.nTBTTurnType = self.nTBTTurnTypeNext = -1

    if self.xTurnInfo < 0 or self.xDistToTurn < -50:
      if self.xDistToTurn > 0:
        self.xDistToTurn = 0
      self.xTurnInfo = -1
      self.xDistToTurnNext = 0
      self.xTurnInfoNext = -1

    v_ego_kph = v_ego * 3.6
    atc_desired, self.atcType, self.atcSpeed, self.atcDist = self._update_auto_turn(
      v_ego_kph, self.xTurnInfo, self.xDistToTurn, steering_pressed, steering_torque, True)
    atc_desired_next, _, self.atcSpeedNext, self.atcDistNext = self._update_auto_turn(
      v_ego_kph, self.xTurnInfoNext, self.xDistToTurnNext, steering_pressed, steering_torque, False)

    if self.autoTurnControl not in [2, 3]:    # auto turn speed control
      atc_desired = atc_desired_next = NO_LIMIT_KPH

    if self.autoTurnControl not in [1, 2]:    # auto turn control
      self.atcType = "none"

    self.atc_desired = atc_desired
    self.atc_desired_next = atc_desired_next
    return min(atc_desired, atc_desired_next)

  def _update_auto_turn(self, v_ego_kph, x_turn_info, x_dist_to_turn, steering_pressed, steering_torque,
                        check_steer=False):
    """CarrotServ.update_auto_turn(), unchanged apart from the map switch (not ported)."""
    turn_speed = self.autoTurnControlSpeedTurn
    right_turn_speed = min(turn_speed, RIGHT_TURN_SPEED_KPH)
    fork_speed = self.nRoadLimitSpeed
    stop_speed = 1
    turn_dist_for_speed = self.autoTurnControlTurnEnd * turn_speed / 3.6  # 5
    right_turn_dist_for_speed = self.autoTurnControlTurnEnd * right_turn_speed / 3.6
    fork_dist_for_speed = self.autoTurnControlTurnEnd * fork_speed / 3.6  # 5
    stop_dist_for_speed = 5
    start_fork_dist = np.interp(self.nRoadLimitSpeed, [30, 50, 100], [160, 200, 350])
    start_turn_dist = np.interp(self.nTBTNextRoadWidth, [5, 10], [43, 60])
    turn_info_mapping = {
      1: {"type": "turn left", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_fork_dist},
      2: {"type": "turn right", "speed": right_turn_speed, "dist": right_turn_dist_for_speed, "start": start_fork_dist},
      5: {"type": "straight", "speed": turn_speed, "dist": turn_dist_for_speed, "start": start_turn_dist},
      3: {"type": "fork left", "speed": fork_speed, "dist": fork_dist_for_speed, "start": start_fork_dist},
      4: {"type": "fork right", "speed": fork_speed, "dist": fork_dist_for_speed, "start": start_fork_dist},
      6: {"type": "straight", "speed": fork_speed, "dist": fork_dist_for_speed, "start": start_fork_dist},
      7: {"type": "straight", "speed": stop_speed, "dist": stop_dist_for_speed, "start": 1000},
      8: {"type": "straight", "speed": stop_speed, "dist": stop_dist_for_speed, "start": 1000},
    }

    default_mapping = {"type": "none", "speed": 0, "dist": 0, "start": 1000}

    mapping = turn_info_mapping.get(x_turn_info, default_mapping)

    atc_type = mapping["type"]
    atc_speed = mapping["speed"]
    atc_dist = mapping["dist"]
    atc_start_dist = mapping["start"]

    if x_dist_to_turn > atc_start_dist:
      atc_type += " prepare"
      if check_steer:
        self.atc_activate_count = min(0, self.atc_activate_count - 1)
    else:
      if check_steer:
        self.atc_activate_count = max(0, self.atc_activate_count + 1)
      if atc_type in ["turn left", "turn right"] and x_dist_to_turn > start_turn_dist:
        atc_type = "atc left" if atc_type == "turn left" else "atc right"

    # carrot: AutoTurnMapChange switches the HUD between map and road here. Not ported.

    if check_steer:
      if 0 <= x_dist_to_turn < atc_start_dist and atc_type in ["fork left", "fork right"]:
        if not self.atc_paused:
          if steering_pressed and steering_torque < 0 and atc_type in ["fork left", "atc left"]:
            self.atc_paused = True
          elif steering_pressed and steering_torque > 0 and atc_type in ["fork right", "atc right"]:
            self.atc_paused = True
      else:
        self.atc_paused = False

      if self.atc_paused:
        atc_type += " canceled"

    atc_desired = NO_LIMIT_KPH
    if atc_speed > 0 and x_dist_to_turn > 0:
      decel = self.autoNaviSpeedDecelRate
      safe_sec = ATC_SAFE_SEC
      atc_desired = min(atc_desired, calculate_current_speed(x_dist_to_turn - atc_dist, atc_speed, safe_sec, decel))

    return atc_desired, atc_type, atc_speed, atc_dist

  def speed_target(self):
    """(target speed at the point [km/h], metres to that point) of whichever turn sets the cap."""
    if self.atc_desired_next < self.atc_desired:
      return float(self.atcSpeedNext), max(0., float(self.xDistToTurnNext) - float(self.atcDistNext))
    return float(self.atcSpeed), max(0., float(self.xDistToTurn) - float(self.atcDist))
