"""방지턱 감속 계획 -- 사람이 방지턱을 넘는 방식.

  1. 다가갈 때: TmapNda가 방지턱을 알려주면 방지턱 HOLD_DIST_M(15m) 전까지
     BUMP_SPEED_KPH(10km/h)로 줄인다. 그 전에는 거리 기반 프로파일로 부드럽게 내려간다.
  2. 넘은 직후: 바로 원래 속도로 튀어 나가지 않고 10km/h에서 천천히(RECOVER_ACCEL_KPH_S)
     올리면서 NEXT_WAIT_M 동안 다음 방지턱 보고를 기다린다. TmapNda는 다음 방지턱으로
     넘어가는 데 0.5~4초 걸린다(2026-11-17 주행).
       * 그 사이 가까운(NEAR_NEXT_M 이내) 다음 방지턱이 보이면, 천천히 올리던 속도를 상한으로
         유지한 채 그 방지턱의 감속 프로파일로 이어 간다 (올렸다 다시 줄이는 반복이 작다).
       * 다음 방지턱이 없거나 멀면 제한을 바로 푼다.

내비 거리만 믿지 않고 자체 주행거리(v_ego 적분)로 남은 거리를 추정한다.
  - TmapNda는 방지턱 거리가 0 이하가 되면 150m 고정값을 넣어 보낸다(새 방지턱으로 받지 않음).
  - TMAP은 방지턱을 지나기 전에 SDI를 지우거나, 지난 뒤에도 잠깐 옛 거리를 보내기도 한다.
그래서
  - 추적 중에는 '더 짧게 보고된 거리'만 받아들이고, 보고가 끊겨도 적분으로 계속 간다.
  - 통과 후 새 방지턱은 '위치'로 구분한다: 위치 = 누적주행거리 + 보고거리.
      * 지난 방지턱 위치 + MIN_GAP_M 안쪽이면 지난 방지턱의 늦은 보고로 보고 무시
      * 보고값이 NEW_JUMP_M 이상 뛰어오르면(1m -> 46m처럼 다음 항목으로 넘어감) 바로 채택.
        예전엔 여기서도 두 번째 보고를 기다려 1~4초 늦게 잡았다(134.0초 38m -> 138.2초 14m).
      * 뛰어오르지 않고 서서히 나타난 위치는 예전처럼 CONFIRM_TRAVEL_M 동안 유지돼야 채택

외부 의존성 없음(단위 테스트 가능).
"""
import math

KPH_TO_MS = 1. / 3.6
SPEED_BUMP_CAM_TYPE = 22


class SpeedBumpPlanner:
  BUMP_SPEED_KPH = 10.
  HOLD_DIST_M = 15.        # 이 거리 전에 BUMP_SPEED_KPH 도달
  # 감속 프로파일 기울기. 이 차는 브레이크 없이 페달을 떼서만 줄이는데, 2026-11-17 주행에서
  # 10~18km/h 타력 감속은 0.15~0.2 m/s²였다. 1.0이면 목표가 차보다 먼저 떨어져, 방지턱
  # 직전에야 페달을 떼고 14~19km/h로 넘었다. 타력에 맞춰 일찍 떼게 한다
  # (20km/h면 방지턱 약 70m 전부터, 사람이 미리 발을 떼는 것과 같다).
  DECEL_MS2 = 0.2
  LOOKAHEAD_S = 1.5        # 플래너/차량 응답 지연 보상: 이만큼 앞 위치 기준으로 목표 계산
  # 보고된 거리는 얼마간 전 위치 기준이다 (GPS 지연 + TMAP 갱신 1Hz + TmapNda 전송 2Hz).
  # 통과 판단에만 이만큼의 주행거리를 빼 준다 (2026-11-16 분석).
  REPORT_LATENCY_S = 0.5
  # 보고는 1~2Hz로만 바뀐다. 위치 확인은 값이 바뀐 순간에만 하고 허용 오차도 한 주기
  # 동안 달리는 거리만큼 넓힌다.
  REPORT_PERIOD_S = 1.0
  PASS_MARGIN_M = 3.       # 방지턱 지점을 이만큼 지나면 통과로 판단 (지연 보정 후라 짧게)
  MIN_GAP_M = 15.          # 지난 방지턱에서 이만큼 이상 앞이어야 새 방지턱 후보 (GPS 지연 여유 포함)
  NEW_JUMP_M = 5.          # 보고값이 이만큼 뛰어오르면 다음 방지턱 항목으로 넘어간 것
  PLACEHOLDER_DIST = 150   # TmapNda가 거리 0 이하에서 넣는 고정값: 새 방지턱으로 받지 않는다
  CONFIRM_TRAVEL_M = 3.    # 서서히 나타난 후보의 위치 확인에 필요한 주행 거리
  CONFIRM_TOL_M = 3.       # 후보 위치 허용 오차의 기본값 (+ 속도 x REPORT_PERIOD_S)
  COAST_WITHOUT_REPORT_M = 150.  # 보고가 끊겨도 이 거리 안이면 적분으로 계속 추적
  # 통과 후: 10km/h에서 이 기울기로 천천히 올린다 (0.55 m/s² 정도, 사람이 살살 밟는 정도).
  RECOVER_ACCEL_KPH_S = 2.0
  NEXT_WAIT_M = 15.        # 통과 후 다음 방지턱 보고를 기다리는 거리. 없으면 제한을 푼다.
  NEAR_NEXT_M = 80.        # 다음 방지턱이 이 안이면 천천히 올리던 속도를 상한으로 이어 간다
  MAX_DT = 0.2

  def __init__(self):
    self.reset()

  def reset(self):
    self.tracking = False
    self.est_dist = 0.
    self.last_t = None
    self.odo = 0.               # 누적 주행거리(m)
    self.passed_pos = None      # 지난 방지턱 위치(odo 기준). None = 통과 이력 없음
    self.candidate = None       # (후보 위치, 처음 본 시점 odo)
    self.last_reported = None   # 직전 보고값 (값이 바뀌었는지 = 새 보고인지 판단)
    self.recover_kph = None     # 통과 후 천천히 올리는 속도 상한. None = 회복 구간 아님
    self.recover_start_odo = 0.

  @property
  def active(self):
    return self.tracking or self.recover_kph is not None

  @property
  def recovering(self):
    return self.recover_kph is not None

  @property
  def left_dist(self):
    """다가가는 방지턱까지 남은 거리(m). 회복 구간만이면 0."""
    return max(0., self.est_dist) if self.tracking else 0.

  def _clear_history(self):
    self.passed_pos = None
    self.candidate = None

  def _accept_new(self, reported, fresh, jumped, v_ego_ms):
    if reported == self.PLACEHOLDER_DIST:
      return False               # TmapNda 고정값
    if self.passed_pos is None:
      return True
    pos = self.odo + reported
    if pos < self.passed_pos + self.MIN_GAP_M:
      self.candidate = None      # 지난 방지턱의 늦은 보고
      return False
    if jumped:
      self._clear_history()      # 다음 항목으로 넘어감: 바로 채택
      return True
    if not fresh:
      return False               # 반복된 옛 값으로는 위치를 판단하지 않는다
    tol = self.CONFIRM_TOL_M + v_ego_ms * self.REPORT_PERIOD_S
    if self.candidate is None or abs(pos - self.candidate[0]) > tol:
      self.candidate = (pos, self.odo)   # 새 후보(또는 위치가 흔들림: 다시 확인)
      return False
    if self.odo - self.candidate[1] < self.CONFIRM_TRAVEL_M:
      return False
    self._clear_history()
    return True

  def update(self, cam_type, reported_dist, v_ego_ms, now):
    """매 프레임 호출. 방지턱 감속/회복 중이면 목표 속도(km/h), 아니면 None."""
    dt = 0. if self.last_t is None else min(max(now - self.last_t, 0.), self.MAX_DT)
    self.last_t = now
    v_ego_ms = max(0., v_ego_ms)
    self.odo += v_ego_ms * dt

    bump_report = cam_type == SPEED_BUMP_CAM_TYPE and reported_dist is not None and reported_dist > 0
    fresh = bump_report and reported_dist != self.last_reported
    jumped = bump_report and (self.last_reported is None or
                              reported_dist >= self.last_reported + self.NEW_JUMP_M)
    self.last_reported = reported_dist if bump_report else None
    if not self.tracking and cam_type != SPEED_BUMP_CAM_TYPE:
      # 방지턱 정보가 사라졌다 = 지난 방지턱과의 연결 끊김. 다음 방지턱은 바로 받는다.
      self._clear_history()

    if self.recover_kph is not None:
      self.recover_kph += self.RECOVER_ACCEL_KPH_S * dt

    if not self.tracking:
      if bump_report and self._accept_new(float(reported_dist), fresh, jumped, v_ego_ms):
        self.tracking = True
        self.est_dist = float(reported_dist)
        if self.est_dist > self.NEAR_NEXT_M:
          self.recover_kph = None    # 다음 방지턱이 멀다: 회복 상한 없이 프로파일만
      elif self.recover_kph is not None:
        if self.odo - self.recover_start_odo >= self.NEXT_WAIT_M:
          self.recover_kph = None    # 다음 방지턱 없음: 제한 해제
          return None
        return self.recover_kph
      else:
        return None
    else:
      self.est_dist -= v_ego_ms * dt
      if bump_report:
        self.est_dist = min(self.est_dist, float(reported_dist))
      elif self.est_dist > self.COAST_WITHOUT_REPORT_M:
        self.tracking = False   # 멀리서 정보가 끊김: 추적 포기
        self.recover_kph = None
        return None

      # 지연 보정은 통과 판단에만, 그 순간의 속도로 한다.
      if self.est_dist <= -self.PASS_MARGIN_M + v_ego_ms * self.REPORT_LATENCY_S:
        self.tracking = False
        self.passed_pos = self.odo + self.est_dist
        self.candidate = None
        # 넘은 직후: 10km/h에서 천천히 올리며 다음 방지턱을 기다린다
        self.recover_kph = self.BUMP_SPEED_KPH
        self.recover_start_odo = self.odo
        return self.recover_kph

    target = self.target_kph(self.est_dist - v_ego_ms * self.LOOKAHEAD_S)
    if self.recover_kph is not None:
      target = min(target, self.recover_kph)
    return target

  @classmethod
  def target_kph(cls, dist_m):
    """방지턱까지 dist_m 남았을 때 목표 속도. HOLD_DIST_M 안쪽은 BUMP_SPEED_KPH 고정."""
    v_bump = cls.BUMP_SPEED_KPH * KPH_TO_MS
    extra = max(0., dist_m - cls.HOLD_DIST_M)
    return math.sqrt(v_bump ** 2 + 2. * cls.DECEL_MS2 * extra) / KPH_TO_MS
