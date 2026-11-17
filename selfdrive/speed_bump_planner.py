"""방지턱 감속 계획 -- 방지턱마다 '인식 -> 감속 -> 통과(해제)' 이벤트로 움직인다.

방지턱 하나의 흐름 (phase):
  idle      방지턱 없음. 제한 없음.
  approach  TmapNda가 방지턱을 알려줌(이벤트 detect). 아직 SLOW_DIST_M(20m)보다 멀면 제한 없음.
  slow      방지턱 SLOW_DIST_M 안(이벤트 slow): 바로 BUMP_SPEED_KPH(10km/h).
  between   방지턱을 넘음(이벤트 pass). 10km/h에서 RAMP_KPH_S로 천천히 올리되
            BETWEEN_SPEED_KPH(20km/h)를 넘지 않는다.
              * 다음 방지턱이 NEAR_NEXT_M 안에서 보이면(detect) 20km/h 상한을 유지한 채 다가가고,
                그 방지턱 20m 안에서 다시 slow.
              * NEXT_WAIT_M 동안 다음 방지턱이 안 보이거나 멀면 제한을 푼다(이벤트 release).
            TmapNda는 넘은 뒤 다음 항목으로 바뀌는 데 0.5~4초 걸린다(2026-11-17 주행).

이번 프레임에 일어난 이벤트는 self.events에 담긴다 (speed_bump.jsonl에 그대로 기록).

내비 거리만 믿지 않고 자체 주행거리(v_ego 적분)로 남은 거리를 추정한다.
  - TmapNda는 방지턱 거리가 0 이하가 되면 150m 고정값을 넣어 보낸다(새 방지턱으로 받지 않음).
  - TMAP은 방지턱을 지나기 전에 SDI를 지우거나, 지난 뒤에도 잠깐 옛 거리를 보내기도 한다.
그래서
  - 추적 중에는 '더 짧게 보고된 거리'만 받아들이고, 보고가 끊겨도 적분으로 계속 간다.
  - 통과 후 새 방지턱은 '위치'로 구분한다: 위치 = 누적주행거리 + 보고거리.
      * 지난 방지턱 위치 + MIN_GAP_M 안쪽이면 지난 방지턱의 늦은 보고로 보고 무시
      * 보고값이 NEW_JUMP_M 이상 뛰어오르면(1m -> 46m처럼 다음 항목으로 넘어감) 바로 채택
        (두 번째 보고를 기다리던 때는 1~4초 늦게 잡았다)
      * 서서히 나타난 위치는 CONFIRM_TRAVEL_M 동안 유지돼야 채택

외부 의존성 없음(단위 테스트 가능).
"""

KPH_TO_MS = 1. / 3.6
SPEED_BUMP_CAM_TYPE = 22


class SpeedBumpPlanner:
  BUMP_SPEED_KPH = 10.
  SLOW_DIST_M = 20.        # 방지턱 이 거리 안에서 바로 BUMP_SPEED_KPH
  BETWEEN_SPEED_KPH = 20.  # 방지턱 사이 상한
  RAMP_KPH_S = 2.0         # 넘은 뒤 10 -> 20km/h를 5초에 걸쳐 천천히
  NEXT_WAIT_M = 15.        # 넘은 뒤 다음 방지턱 보고를 기다리는 거리. 없으면 release.
  NEAR_NEXT_M = 80.        # 다음 방지턱이 이 안이면 방지턱 사이(between)로 이어 간다
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
  MAX_DT = 0.2

  def __init__(self):
    self.reset()

  def reset(self):
    self.tracking = False       # 다가가는 방지턱이 있음 (approach 또는 slow)
    self.est_dist = 0.
    self.last_t = None
    self.odo = 0.               # 누적 주행거리(m)
    self.passed_pos = None      # 지난 방지턱 위치(odo 기준). None = 통과 이력 없음
    self.candidate = None       # (후보 위치, 처음 본 시점 odo)
    self.last_reported = None   # 직전 보고값 (값이 바뀌었는지 = 새 보고인지 판단)
    self.between = False        # 방지턱 사이 구간 (넘은 뒤 20km/h 상한)
    self.ramp_kph = self.BUMP_SPEED_KPH
    self.between_start_odo = 0.
    self.bump_no = 0            # 인식한 방지턱 번호 (로그용)
    self.events = []            # 이번 프레임의 이벤트: detect / slow / pass / release / lost
    self._slow = False

  @property
  def phase(self):
    if self.tracking and self._slow:
      return 'slow'
    if self.between:
      return 'between'
    if self.tracking:
      return 'approach'
    return 'idle'

  @property
  def active(self):
    """제한을 걸고 있는가 (controlsd가 10km/h 아래 목표를 허용하고 필터 없이 따른다)."""
    return self.phase in ('slow', 'between')

  @property
  def recovering(self):
    return self.between

  @property
  def recover_kph(self):
    return self.ramp_kph if self.between else None

  @property
  def left_dist(self):
    """다가가는 방지턱까지 남은 거리(m). 없으면 0."""
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

  def _target(self):
    if self.tracking and self._slow:
      return self.BUMP_SPEED_KPH
    if self.between:
      return self.ramp_kph
    return None

  def update(self, cam_type, reported_dist, v_ego_ms, now):
    """매 프레임 호출. 제한 중이면 목표 속도(km/h), 아니면 None."""
    self.events = []
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

    if self.between:
      self.ramp_kph = min(self.BETWEEN_SPEED_KPH, self.ramp_kph + self.RAMP_KPH_S * dt)

    # 1) 인식: 새 방지턱
    if not self.tracking and bump_report and \
       self._accept_new(float(reported_dist), fresh, jumped, v_ego_ms):
      self.tracking = True
      self._slow = False
      self.est_dist = float(reported_dist)
      self.bump_no += 1
      self.events.append('detect')
      if self.between and self.est_dist > self.NEAR_NEXT_M:
        self.between = False       # 다음 방지턱이 멀다: 방지턱 사이 상한 해제
        self.events.append('release')

    # 2) 추적: 거리 갱신, 20m 진입, 통과
    if self.tracking:
      if 'detect' not in self.events:
        self.est_dist -= v_ego_ms * dt
        if bump_report:
          self.est_dist = min(self.est_dist, float(reported_dist))
        elif self.est_dist > self.COAST_WITHOUT_REPORT_M:
          self.tracking = False      # 멀리서 정보가 끊김: 추적 포기
          self.events.append('lost')
      if self.tracking and not self._slow and self.est_dist <= self.SLOW_DIST_M:
        self._slow = True
        self.events.append('slow')
      # 지연 보정은 통과 판단에만, 그 순간의 속도로 한다.
      if self.tracking and self.est_dist <= -self.PASS_MARGIN_M + v_ego_ms * self.REPORT_LATENCY_S:
        self.tracking = False
        self._slow = False
        self.passed_pos = self.odo + self.est_dist
        self.candidate = None
        self.between = True
        self.ramp_kph = self.BUMP_SPEED_KPH
        self.between_start_odo = self.odo
        self.events.append('pass')

    # 3) 방지턱 사이: 다음 방지턱이 안 보이면 해제
    if self.between and not self.tracking and \
       self.odo - self.between_start_odo >= self.NEXT_WAIT_M:
      self.between = False
      self.events.append('release')

    return self._target()
