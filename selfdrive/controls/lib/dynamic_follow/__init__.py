import math
from collections import deque
import numpy as np
import cereal.messaging as messaging
from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import interp, clip, mean
from common.conversions import Conversions as CV
from cereal.messaging import SubMaster
from common.params import Params

from selfdrive.controls.lib.dynamic_follow.support import LeadData, CarData, dfData
from selfdrive.controls.lib.dynamic_follow.follow_gap import (
  FOLLOW_LEARN_PARAM, FollowGap, MIN_FOLLOW_TR, floor_tr, read_gap_scale)
from selfdrive.controls.lib.follow_accel_cap import default_coast_curve, read_follow_learn
from common.data_collector import DataCollector

travis = False
DEFAULT_TR = 1.3   #1.45

# The traffic / stock / roadtrip / auto profiles, the auto-df network, Min TR
# and the multiplier are gone. The gap is follow_gap.FollowGap: one elastic
# band per speed with the brake-free floor, scaled by what FollowLearner
# (controlsd) learns from the driver's gas and brake corrections.
GAP_SCALE_REFRESH_S = 5.0
TOGGLE_REFRESH_S = 0.5    # ActiveStopAccelBoost / StandstillAutoLaunch


def speed_based_min_tr(v_ego):
  """Kept for callers of the old name: the floor is follow_gap.floor_tr."""
  return floor_tr(v_ego)

# Stop-and-go lead launch assist. This only lowers the MPC time gap; it does not
# bypass the longitudinal acceleration, jerk, or safety limits.
LEAD_LAUNCH_IDLE = 0
LEAD_LAUNCH_ARMED = 1
LEAD_LAUNCH_ACTIVE = 2
LEAD_LAUNCH_RELEASING = 3

LEAD_LAUNCH_TR = 1.0
LEAD_LAUNCH_MIN_EGO_KPH = 1.0
LEAD_LAUNCH_MAX_EGO_KPH = 25.0
LEAD_LAUNCH_ARM_MAX_EGO_KPH = 5.0
LEAD_STOP_SPEED_MS = 0.50
LEAD_RESTOP_SPEED_MS = 0.20
# A stopped GM radar lead can report up to roughly 0.5 m/s. Confirm that its
# distance is stable before arming, then require either a measurable distance
# increase or a stronger speed signal before declaring a launch.
LEAD_START_SPEED_MS = 0.30
LEAD_START_STRONG_SPEED_MS = 0.65
LEAD_START_REL_SPEED_MS = 0.05
LEAD_START_DISTANCE_DELTA_M = 0.20
LEAD_LAUNCH_CLOSING_REL_SPEED_MS = -0.3
LEAD_LAUNCH_MIN_DISTANCE_M = 1.5
LEAD_LAUNCH_MAX_ARM_DISTANCE_M = 30.0
LEAD_LAUNCH_DISTANCE_JUMP_M = 8.0
LEAD_LAUNCH_CONTINUOUS_NEW_LEAD_M = 2.0
LEAD_STOP_DISTANCE_STABILITY_M = 0.60
LEAD_STOP_CONFIRM_S = 0.5
LEAD_START_CONFIRM_S = 0.05
LEAD_LAUNCH_VISION_DROPOUT_FRAMES = 6
LEAD_LAUNCH_ARM_HOLD_S = 2.0
LEAD_LAUNCH_BRAKE_ARM_HOLD_S = 3.0
LEAD_LAUNCH_SPEED_FILTER_SAMPLES = 3
LEAD_LAUNCH_DISTANCE_FILTER_SAMPLES = 5
LEAD_LAUNCH_MAX_ACTIVE_S = 6.0
LEAD_LAUNCH_BLEND_IN_S = 0.15
LEAD_LAUNCH_BLEND_OUT_S = 1.5

# Rolling pull-away in a queue. On the 11-18 drive (109 of 150 engaged
# minutes in jams) the driver pressed the gas 76 times as the lead pulled away
# while the car was still rolling, because nothing above 5 km/h counted as a
# launch: the plan waited for the gap to open, and cut-ins came at a ~25 m gap
# against a usual ~6 m. With ActiveStopAccelBoost on, a lead clearly pulling
# away at 3-25 km/h now starts the same launch (boost and floor in
# stop_accel_boost.py, with all of its cancels) as a lead leaving a stop.
LEAD_ROLL_MIN_EGO_KPH = 3.0
LEAD_ROLL_REL_SPEED_MS = 0.5
LEAD_ROLL_REARM_VREL_MS = 0.2
LEAD_ROLL_MIN_DISTANCE_M = 3.0
LEAD_ROLL_MAX_DISTANCE_M = 25.0
LEAD_ROLL_CONFIRM_S = 0.3


class DistanceModController:
  def __init__(self, k_i, k_d, x_clip, mods):
    self._k_i = k_i
    self._k_d = k_d
    self._to_clip = x_clip  # reaches this with v_rel=3.5 mph for 4 seconds
    self._mods = mods

    self.i = 0  # never resets, even when new lead
    self.last_error = 0

  def update(self, error):
    """
    Relative velocity is a good starting point
    Returns: Multiplier for final y_dist output
    """

    if (d := self._k_d * (error - self.last_error)) < 0:  # only add if it will add distance
      self.i += d

    self.i += error * DT_MDL * self._k_i
    self.i = clip(self.i, self._to_clip[0], self._to_clip[-1])  # clip to reasonable range
    self._slow_reset()  # slowly reset from max to 0

    fact = interp(self.i, self._to_clip, self._mods)
    self.last_error = float(error)

    # print("I: {}, FACT: {}".format(round(self.i, 4), round(fact, 3)))
    return fact

  def _slow_reset(self):
    if abs(self.i) > 0.01:  # oscillation starts around 0.006
      reset_time = 15  # in x seconds i goes from max to 0
      sign = 1 if self.i > 0 else -1
      self.i -= sign * max(self._to_clip) / (reset_time / DT_MDL)


class DynamicFollow:
  def __init__(self):
    if not travis:
      self.pm = messaging.PubMaster(['dynamicFollowData'])
    else:
      self.pm = None

    # Dynamic follow variables
    self.TR = DEFAULT_TR
    self.v_ego_retention = 2.5
    self.v_rel_retention = 1.75

    self.sng_TR = DEFAULT_TR  # 재가속 정지 및 이동 TR
    self.sng_speed = 20 / CV.MS_TO_KPH   # 28.8 kph  (DEF:18.0)

    self._setup_collector()
    self._setup_changing_variables()

  def _setup_collector(self):
    #self.sm_collector = SubMaster(['liveTracks', 'laneSpeed'])
    self.sm_collector = SubMaster(['liveTracks'])
    self.log_auto_df = False  # self.op_params.get('log_auto_df')
    if not isinstance(self.log_auto_df, bool):
      self.log_auto_df = False
    self.data_collector = DataCollector(file_path='/data/df_data', keys=['v_ego', 'a_ego', 'a_lead', 'v_lead', 'x_lead', 'left_lane_speeds', 'middle_lane_speeds', 'right_lane_speeds', 'left_lane_distances', 'middle_lane_distances', 'right_lane_distances', 'profile', 'time'], log_data=self.log_auto_df)

  def _setup_changing_variables(self):
    self.TR = DEFAULT_TR
    # There is one profile now; 0 keeps dynamicFollowData.profilePred and the
    # data collector's 'profile' column meaningful.
    self.user_profile = 0
    self.model_profile = 0
    self.follow_gap = FollowGap()
    self.gap_scale = 1.0
    self.coast_curve = default_coast_curve()
    self.lift_margin = 1.0
    self.last_gap_scale_read = -GAP_SCALE_REFRESH_S
    self.last_toggle_read = -TOGGLE_REFRESH_S

    self.sng = False
    self.car_data = CarData()
    self.lead_data = LeadData()
    self.df_data = dfData()  # dynamic follow data
    self.base_TR = DEFAULT_TR
    self.base_TR_unadjusted = DEFAULT_TR

    self.lead_launch_state = LEAD_LAUNCH_IDLE
    self.lead_launch_blend = 0.0
    self.lead_launch_stopped_since = None
    self.lead_launch_start_since = None
    self.lead_launch_started_at = None
    self.lead_launch_armed_until = None
    self.lead_launch_armed_distance = None
    self.lead_launch_dropout_frames = 0
    self.lead_launch_speed_samples = deque(maxlen=LEAD_LAUNCH_SPEED_FILTER_SAMPLES)
    self.lead_launch_distance_samples = deque(maxlen=LEAD_LAUNCH_DISTANCE_FILTER_SAMPLES)
    self.lead_launch_filtered_speed = None
    self.last_launch_lead_status = False
    self.last_launch_lead_distance = None
    self.lead_roll_rearmed = True
    self.lead_roll_since = None
    self.lead_roll_count = 0

    self.last_cost = 0.0
    self._get_live_params()  # so they're defined just in case

  def update(self, CS):
    self._get_live_params()
    self._update_car(CS)

    if self.log_auto_df:
      self._gather_data()

    if self.lead_data.status:
      self._store_df_data()
    self.base_TR_unadjusted = self.follow_gap.update(
      self.car_data.v_ego, self.lead_data.status, self.lead_data.x_lead,
      self.lead_data.v_lead, self.lead_data.a_lead, self.gap_scale, DT_MDL)

    self.base_TR = self._apply_following_floor(self.base_TR_unadjusted)
    self.TR = self._update_lead_launch_TR(self.base_TR)

    if not travis:
      self._send_cur_state()

    return self.TR

  def _apply_following_floor(self, base_tr):
    # FollowGap already stays inside its band; this is the hard floor again so
    # nothing upstream of the MPC can ever go below it.
    return max(floor_tr(self.car_data.v_ego), min(2.7, float(base_tr)))

  def _gather_data(self):
    self.sm_collector.update(0)
    # live_tracks = [[i.dRel, i.vRel, i.aRel, i.yRel] for i in self.sm_collector['liveTracks']]
    if self.car_data.cruise_enabled:
      self.data_collector.update([self.car_data.v_ego,
                                  self.car_data.a_ego,
                                  self.lead_data.a_lead,
                                  self.lead_data.v_lead,
                                  self.lead_data.x_lead,
                                  #list(self.sm_collector['laneSpeed'].leftLaneSpeeds),
                                  #list(self.sm_collector['laneSpeed'].middleLaneSpeeds),
                                  #list(self.sm_collector['laneSpeed'].rightLaneSpeeds),

                                  #list(self.sm_collector['laneSpeed'].leftLaneDistances),
                                  #list(self.sm_collector['laneSpeed'].middleLaneDistances),
                                  #list(self.sm_collector['laneSpeed'].rightLaneDistances),
                                  self.user_profile,
                                  sec_since_boot()])

  def _send_cur_state(self):
    if self.pm is not None:
      dat = messaging.new_message('dynamicFollowData')
      dat.dynamicFollowData.mpcTR = self.TR
      dat.dynamicFollowData.profilePred = self.model_profile
      # Publish the confirmed launch immediately, including below 1 km/h. The
      # controller needs this phase to release its stopped state; catchupFactor
      # remains zero until the speed-dependent TR blend actually starts.
      dat.dynamicFollowData.leadCatchupActive = self.stop_accel_boost_enabled and self.lead_launch_state == LEAD_LAUNCH_ACTIVE
      dat.dynamicFollowData.catchupFactor = self.lead_launch_blend
      dat.dynamicFollowData.baseTR = self.base_TR
      dat.dynamicFollowData.leadLaunchState = self.lead_launch_state
      dat.dynamicFollowData.stopAccelBoostEnabled = self.stop_accel_boost_enabled
      filtered_lead_speed = (self.lead_launch_filtered_speed if self.lead_launch_filtered_speed is not None
                             else float(self.lead_data.v_lead or 0.0))
      dat.dynamicFollowData.leadSpeed = float(filtered_lead_speed)
      dat.dynamicFollowData.leadRelativeSpeed = float(filtered_lead_speed - self.car_data.v_ego)
      dat.dynamicFollowData.leadDistance = float(self.lead_data.x_lead or 0.0)
      dat.dynamicFollowData.egoSpeed = float(self.car_data.v_ego)
      dat.dynamicFollowData.rawTR = float(self.base_TR_unadjusted)
      dat.dynamicFollowData.learnedTROffset = 0.0
      #print("dat.dynamicFollowData.mpcTR ======================================== : ", dat.dynamicFollowData.mpcTR)
      #print("dat.dynamicFollowData.profilePred ======================================== : ", dat.dynamicFollowData.profilePred)
      self.pm.send('dynamicFollowData', dat)

  def _store_df_data(self):
    cur_time = sec_since_boot()
    # 시간 경과에 따른 사용자 지정 상대 가속도 저장
    if self.lead_data.status:
      if self.lead_data.new_lead:
        self.df_data.v_rels = []  # reset when new lead
      else:
        self.df_data.v_rels = self._remove_old_entries(self.df_data.v_rels, cur_time, self.v_rel_retention)
      self.df_data.v_rels.append({'v_ego': self.car_data.v_ego, 'v_lead': self.lead_data.v_lead, 'time': cur_time})

    # 더 나은 SNG를 위해 속도를 저장하십시오.
    self.df_data.v_egos = self._remove_old_entries(self.df_data.v_egos, cur_time, self.v_ego_retention)
    self.df_data.v_egos.append({'v_ego': self.car_data.v_ego, 'time': cur_time})

  @staticmethod
  def _remove_old_entries(lst, cur_time, retention):
    return [sample for sample in lst if cur_time - sample['time'] <= retention]

  def _reset_lead_launch(self, immediate=True):
    self.lead_launch_state = LEAD_LAUNCH_IDLE if immediate else LEAD_LAUNCH_RELEASING
    self.lead_launch_stopped_since = None
    self.lead_launch_start_since = None
    self.lead_launch_started_at = None
    self.lead_launch_armed_until = None
    self.lead_launch_armed_distance = None
    self.lead_launch_dropout_frames = 0
    self.lead_launch_speed_samples.clear()
    self.lead_launch_distance_samples.clear()
    self.lead_launch_filtered_speed = None
    self.lead_roll_since = None
    if immediate:
      self.lead_launch_blend = 0.0

  def _update_lead_launch_TR(self, base_TR):
    """Temporarily blend TR toward 1.0 when a confirmed stopped lead starts moving."""
    now = sec_since_boot()
    lead_status = bool(self.lead_data.status)
    v_ego = float(self.car_data.v_ego)
    v_ego_kph = v_ego * CV.MS_TO_KPH
    v_lead = float(self.lead_data.v_lead) if self.lead_data.v_lead is not None else 0.0
    x_lead = float(self.lead_data.x_lead) if self.lead_data.x_lead is not None else 0.0

    if not self.stop_accel_boost_enabled:
      self._reset_lead_launch(immediate=True)
      self.last_launch_lead_status = lead_status
      self.last_launch_lead_distance = x_lead if lead_status and math.isfinite(x_lead) else None
      return float(base_TR)

    lead_values_valid = bool(lead_status and math.isfinite(v_lead) and math.isfinite(x_lead))
    # Preserve a confirmed stationary lead while the driver holds the brake.
    # Brake still blocks the controller latch and all acceleration output. If
    # the brake is pressed after boost starts, cancel immediately as before.
    control_cancel = not self.car_data.cruise_enabled

    if control_cancel or (self.car_data.brake_pressed and
                          self.lead_launch_state == LEAD_LAUNCH_ACTIVE):
      self._reset_lead_launch(immediate=True)
    elif not lead_values_valid:
      # Vision-only leads can disappear for one or two model frames at a stop.
      # Preserve the confirmed stop/launch state for a short dropout, but
      # never extend the ARMED deadline because of missing observations.
      self.lead_launch_dropout_frames += 1
      if self.lead_launch_dropout_frames > LEAD_LAUNCH_VISION_DROPOUT_FRAMES:
        self._reset_lead_launch(immediate=True)
        self.last_launch_lead_status = False
        self.last_launch_lead_distance = None
    else:
      self.lead_launch_dropout_frames = 0
      self.lead_launch_speed_samples.append(v_lead)
      self.lead_launch_distance_samples.append(x_lead)
      self.lead_launch_filtered_speed = float(np.median(self.lead_launch_speed_samples))

      distance_jump = False
      if self.last_launch_lead_status and self.last_launch_lead_distance is not None:
        distance_jump = abs(x_lead - self.last_launch_lead_distance) > LEAD_LAUNCH_DISTANCE_JUMP_M

      filtered_v_lead = self.lead_launch_filtered_speed
      filtered_v_rel = filtered_v_lead - v_ego
      continuous_new_lead = bool(self.lead_data.new_lead and
                                 self.last_launch_lead_status and
                                 self.last_launch_lead_distance is not None and
                                 v_ego_kph <= LEAD_LAUNCH_ARM_MAX_EGO_KPH and
                                 abs(x_lead - self.last_launch_lead_distance) <= LEAD_LAUNCH_CONTINUOUS_NEW_LEAD_M)
      lead_changed = bool(distance_jump or (self.lead_data.new_lead and not continuous_new_lead))
      hard_cancel = ((x_lead <= LEAD_LAUNCH_MIN_DISTANCE_M and filtered_v_rel <= 0.0) or
                     filtered_v_rel <= LEAD_LAUNCH_CLOSING_REL_SPEED_MS)

      if hard_cancel or lead_changed:
        self._reset_lead_launch(immediate=True)
      else:
        distance_stable = bool(len(self.lead_launch_distance_samples) == LEAD_LAUNCH_DISTANCE_FILTER_SAMPLES and
                               max(self.lead_launch_distance_samples) - min(self.lead_launch_distance_samples) <=
                               LEAD_STOP_DISTANCE_STABILITY_M)
        lead_stopped = filtered_v_lead <= LEAD_STOP_SPEED_MS and distance_stable
        arm_conditions = (lead_stopped and
                          v_ego_kph <= LEAD_LAUNCH_ARM_MAX_EGO_KPH and
                          LEAD_LAUNCH_MIN_DISTANCE_M < x_lead <= LEAD_LAUNCH_MAX_ARM_DISTANCE_M)

        # A lead stopping again is a safety-critical cancellation. It may arm a
        # fresh launch only after another complete stop confirmation.
        lead_stopped_again = (filtered_v_lead <= LEAD_RESTOP_SPEED_MS and
                              filtered_v_rel <= 0.0)
        if self.lead_launch_state == LEAD_LAUNCH_ACTIVE and lead_stopped_again:
          self._reset_lead_launch(immediate=True)

        if arm_conditions:
          if self.lead_launch_stopped_since is None:
            self.lead_launch_stopped_since = now
          elif (now - self.lead_launch_stopped_since >= LEAD_STOP_CONFIRM_S and
                self.lead_launch_state in (LEAD_LAUNCH_IDLE, LEAD_LAUNCH_RELEASING)):
            self.lead_launch_state = LEAD_LAUNCH_ARMED
            self.lead_launch_start_since = None
            self.lead_launch_armed_until = now + LEAD_LAUNCH_ARM_HOLD_S
            self.lead_launch_armed_distance = x_lead
            self.lead_launch_blend = 0.0
          elif self.lead_launch_state == LEAD_LAUNCH_ARMED:
            self.lead_launch_armed_until = now + LEAD_LAUNCH_ARM_HOLD_S
        else:
          self.lead_launch_stopped_since = None

        if self.lead_launch_state == LEAD_LAUNCH_ARMED:
          if self.car_data.brake_pressed:
            self.lead_launch_armed_until = now + LEAD_LAUNCH_BRAKE_ARM_HOLD_S
          distance_opening = bool(self.lead_launch_armed_distance is not None and
                                  x_lead - self.lead_launch_armed_distance >= LEAD_START_DISTANCE_DELTA_M)
          launch_detected = (filtered_v_rel >= LEAD_START_REL_SPEED_MS and
                             ((filtered_v_lead >= LEAD_START_SPEED_MS and distance_opening) or
                              filtered_v_lead >= LEAD_START_STRONG_SPEED_MS))
          if launch_detected:
            if self.lead_launch_start_since is None:
              self.lead_launch_start_since = now
            elif (now - self.lead_launch_start_since >= LEAD_START_CONFIRM_S and
                  not self.car_data.brake_pressed):
              self.lead_launch_state = LEAD_LAUNCH_ACTIVE
              self.lead_launch_started_at = now
              self.lead_launch_start_since = None
              self.lead_launch_armed_until = None
          else:
            self.lead_launch_start_since = None

        elif self.lead_launch_state == LEAD_LAUNCH_ACTIVE:
          active_timed_out = (self.lead_launch_started_at is not None and
                              now - self.lead_launch_started_at >= LEAD_LAUNCH_MAX_ACTIVE_S)
          if v_ego_kph > LEAD_LAUNCH_MAX_EGO_KPH or active_timed_out:
            self._reset_lead_launch(immediate=False)

        # Rolling pull-away: the same launch, but while creeping in a queue
        # rather than from a stop. A new pull-away needs the lead to have
        # stopped pulling away since the last one (re-arm), so one long
        # opening does not chain launches back to back.
        if filtered_v_rel < LEAD_ROLL_REARM_VREL_MS:
          self.lead_roll_rearmed = True
        rolling = bool(self.lead_roll_rearmed and
                       self.lead_launch_state in (LEAD_LAUNCH_IDLE, LEAD_LAUNCH_RELEASING) and
                       LEAD_ROLL_MIN_EGO_KPH <= v_ego_kph <= LEAD_LAUNCH_MAX_EGO_KPH and
                       filtered_v_lead >= LEAD_START_STRONG_SPEED_MS and
                       filtered_v_rel >= LEAD_ROLL_REL_SPEED_MS and
                       LEAD_ROLL_MIN_DISTANCE_M <= x_lead <= LEAD_ROLL_MAX_DISTANCE_M)
        if rolling:
          if self.lead_roll_since is None:
            self.lead_roll_since = now
          elif now - self.lead_roll_since >= LEAD_ROLL_CONFIRM_S and not self.car_data.brake_pressed:
            self.lead_launch_state = LEAD_LAUNCH_ACTIVE
            self.lead_launch_started_at = now
            self.lead_launch_armed_distance = None   # no standstill gap to launch off
            self.lead_launch_armed_until = None
            self.lead_roll_rearmed = False
            self.lead_roll_since = None
            self.lead_roll_count += 1
        else:
          self.lead_roll_since = None

      self.last_launch_lead_status = True
      self.last_launch_lead_distance = x_lead

    if (self.lead_launch_state == LEAD_LAUNCH_ARMED and
        self.lead_launch_armed_until is not None and now > self.lead_launch_armed_until):
      self._reset_lead_launch(immediate=True)

    blend_target = float(self.lead_launch_state == LEAD_LAUNCH_ACTIVE and
                         LEAD_LAUNCH_MIN_EGO_KPH <= v_ego_kph <= LEAD_LAUNCH_MAX_EGO_KPH)
    if blend_target > self.lead_launch_blend:
      self.lead_launch_blend = min(blend_target,
                                   self.lead_launch_blend + DT_MDL / LEAD_LAUNCH_BLEND_IN_S)
    else:
      self.lead_launch_blend = max(blend_target,
                                   self.lead_launch_blend - DT_MDL / LEAD_LAUNCH_BLEND_OUT_S)

    if self.lead_launch_state == LEAD_LAUNCH_RELEASING and self.lead_launch_blend <= 0.0:
      self.lead_launch_state = LEAD_LAUNCH_IDLE

    base_TR = float(base_TR)
    launch_TR = min(base_TR, max(LEAD_LAUNCH_TR, MIN_FOLLOW_TR))
    return base_TR + self.lead_launch_blend * (launch_TR - base_TR)

  def standstill_launch_shift(self, v_ego, stop_distance):
    """Metres to add to the lead obstacle so the MPC launches off the stopped gap.

    The car stops wherever the driver brakes it, typically ~3.6 m behind the
    lead, while the MPC wants stop_distance (5.5 m) at standstill. Its plan for
    a launching lead therefore stays at zero until the lead has opened that
    difference too. With StandstillAutoLaunch on and a confirmed lead launch,
    the gap the car stopped at counts as the standstill gap instead, so any
    opening is a reason to go. Faded out between 5 and 15 km/h, by which time
    the normal following gap has taken over.
    """
    if not (getattr(self, 'standstill_auto_launch', False) and self.stop_accel_boost_enabled and
            self.lead_launch_state == LEAD_LAUNCH_ACTIVE and
            self.lead_launch_armed_distance is not None):
      return 0.0
    base = max(0.0, float(stop_distance) - float(self.lead_launch_armed_distance))
    return base * float(interp(float(v_ego) * CV.MS_TO_KPH, [5.0, 15.0], [1.0, 0.0]))

  def update_lead(self, v_lead=None, a_lead=None, x_lead=None, status=False, new_lead=False):
    self.lead_data.v_lead = v_lead
    self.lead_data.a_lead = a_lead
    self.lead_data.x_lead = x_lead

    self.lead_data.status = status
    self.lead_data.new_lead = new_lead

  def _update_car(self, CS):
    self.car_data.v_ego = CS.vEgo
    self.car_data.a_ego = CS.aEgo

    self.car_data.left_blinker = CS.leftBlinker
    self.car_data.right_blinker = CS.rightBlinker
    self.car_data.cruise_enabled = CS.cruiseState.enabled
    self.car_data.brake_pressed = CS.brakePressed
    #self.car_data.cruise_enabled = CS.adaptive_Cruise

  def _get_live_params(self):
    # ParamControl writes ASCII "0"/"1" values. Read it live so the menu
    # switch takes effect without restarting controls -- but not every model
    # frame: two file reads at 20 Hz were ~0.25 ms a frame of plannerd, and a
    # tap on the 자동 출발 tile showing up 0.5 s later changes nothing.
    now = sec_since_boot()
    params = None
    if now - self.last_toggle_read >= TOGGLE_REFRESH_S:
      self.last_toggle_read = now
      params = Params()
      self.stop_accel_boost_enabled = params.get_bool("ActiveStopAccelBoost")
      try:
        self.standstill_auto_launch = params.get_bool("StandstillAutoLaunch")
      except Exception:   # key not built into params yet: stay off
        self.standstill_auto_launch = False

    # gap_scale is learned in controlsd (follow_learner.py) and handed over
    # through its saved state; a few seconds of lag is irrelevant to it.
    if now - self.last_gap_scale_read >= GAP_SCALE_REFRESH_S:
      if params is None:
        params = Params()
      self.last_gap_scale_read = now
      try:
        raw = params.get(FOLLOW_LEARN_PARAM)
        self.gap_scale = read_gap_scale(raw)
        # Coast decel and lift margin for the planner's follow accel cap.
        self.coast_curve, self.lift_margin = read_follow_learn(raw)
      except Exception:
        self.gap_scale = 1.0
