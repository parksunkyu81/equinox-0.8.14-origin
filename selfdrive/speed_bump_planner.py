"""방지턱 감속 계획.

목표: 방지턱 HOLD_DIST_M(50m) 전까지 BUMP_SPEED_KPH(10km/h)로 줄이고, 방지턱을 지나면 제한을 푼다.

내비 거리만 믿지 않고 자체 주행거리(v_ego 적분)로 남은 거리를 추정한다.
  - TmapNda는 방지턱 거리가 0 이하가 되면 150m 고정값을 넣어 보낸다.
  - TMAP은 방지턱을 지나기 전에 SDI를 지우거나, 지난 뒤에도 잠깐 옛 거리를 보내기도 한다.
그래서
  - 추적 중에는 '더 짧게 보고된 거리'만 받아들이고, 보고가 끊겨도 적분으로 계속 간다.
  - 통과 후 새 방지턱은 '위치'로 확인한다: 위치 = 누적주행거리 + 보고거리.
      * 지난 방지턱 위치 + MIN_GAP_M 안쪽이면 지난 방지턱의 늦은 보고로 보고 무시
      * CONFIRM_TRAVEL_M 이상 달리는 동안 위치가 CONFIRM_TOL_M 안에서 유지돼야 채택
        (150m 고정값은 차와 함께 앞으로 밀려가므로 채택되지 않음)

외부 의존성 없음(단위 테스트 가능).
"""
import math

KPH_TO_MS = 1. / 3.6
SPEED_BUMP_CAM_TYPE = 22


class SpeedBumpPlanner:
  BUMP_SPEED_KPH = 10.
  HOLD_DIST_M = 50.        # 이 거리 전에 BUMP_SPEED_KPH 도달
  DECEL_MS2 = 1.0          # 감속 프로파일 기울기 (편안한 감속)
  LOOKAHEAD_S = 1.5        # 플래너/차량 응답 지연 보상: 이만큼 앞 위치 기준으로 목표 계산
  # 보고된 거리는 얼마간 전 위치 기준이다 (GPS 지연 + TMAP 갱신 1Hz + TmapNda 전송 2Hz).
  # 그대로 쓰면 길게 잡혀, 방지턱을 지난 뒤에도 몇 초 더 10km/h에 묶였다
  # (2026-11-16 1132~1140초). 실제 지연은 아직 재지 못해(→ /data/log/speed_bump.jsonl)
  # 절반인 0.5초만 뺀다: 지연이 0이어도 방지턱 지점 전에는 풀리지 않고(20km/h에서 -0.2m),
  # 1초라도 지난 뒤 약 1초 안에 풀린다. 예전(보정 없음 + 여유 8m)은 3.7초였다.
  REPORT_LATENCY_S = 0.5
  # 보고는 1~2Hz로만 바뀐다. 그 사이엔 같은 값이 반복되므로, 위치 확인은 값이 바뀐
  # 순간에만 하고 허용 오차도 한 주기 동안 달리는 거리만큼 넓힌다. 고정 3m 오차로는
  # 20km/h만 넘어도 매번 후보가 리셋돼 연속 방지턱을 놓쳤다(같은 날 1147초).
  REPORT_PERIOD_S = 1.0
  PASS_MARGIN_M = 3.       # 방지턱 지점을 이만큼 지나면 통과로 판단 (지연 보정 후라 짧게)
  MIN_GAP_M = 15.          # 지난 방지턱에서 이만큼 이상 앞이어야 새 방지턱 후보 (GPS 지연 여유 포함)
  CONFIRM_TRAVEL_M = 3.    # 새 후보 위치 확인에 필요한 주행 거리
  CONFIRM_TOL_M = 3.       # 후보 위치 허용 오차의 기본값 (+ 속도 x REPORT_PERIOD_S)
  COAST_WITHOUT_REPORT_M = 150.  # 보고가 끊겨도 이 거리 안이면 적분으로 계속 추적
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

  @property
  def active(self):
    return self.tracking

  def _clear_history(self):
    self.passed_pos = None
    self.candidate = None

  def _accept_new(self, reported, fresh, v_ego_ms):
    if self.passed_pos is None:
      return True
    pos = self.odo + reported
    if pos < self.passed_pos + self.MIN_GAP_M:
      self.candidate = None      # 지난 방지턱의 늦은 보고
      return False
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
    """매 프레임 호출. 방지턱 감속 중이면 목표 속도(km/h), 아니면 None."""
    dt = 0. if self.last_t is None else min(max(now - self.last_t, 0.), self.MAX_DT)
    self.last_t = now
    v_ego_ms = max(0., v_ego_ms)
    self.odo += v_ego_ms * dt

    bump_report = cam_type == SPEED_BUMP_CAM_TYPE and reported_dist is not None and reported_dist > 0
    fresh = bump_report and reported_dist != self.last_reported
    self.last_reported = reported_dist if bump_report else None
    if not self.tracking and cam_type != SPEED_BUMP_CAM_TYPE:
      # 방지턱 정보가 사라졌다 = 지난 방지턱과의 연결 끊김. 다음 방지턱은 바로 받는다.
      self._clear_history()

    if not self.tracking:
      if not bump_report or not self._accept_new(float(reported_dist), fresh, v_ego_ms):
        return None
      self.tracking = True
      self.est_dist = float(reported_dist)
    else:
      self.est_dist -= v_ego_ms * dt
      if bump_report:
        self.est_dist = min(self.est_dist, float(reported_dist))
      elif self.est_dist > self.COAST_WITHOUT_REPORT_M:
        self.tracking = False   # 멀리서 정보가 끊김: 추적 포기
        return None

      # 지연 보정은 통과 판단에만, 그 순간의 속도로 한다. 추정 거리 자체에 넣으면
      # 처음 받은 순간(빠를 때)의 보정량이 방지턱 앞(느릴 때)까지 남아, 지연이 작으면
      # 방지턱에 닿기 전에 풀렸다.
      if self.est_dist <= -self.PASS_MARGIN_M + v_ego_ms * self.REPORT_LATENCY_S:
        self.tracking = False
        self.passed_pos = self.odo + self.est_dist
        self.candidate = None
        return None

    return self.target_kph(self.est_dist - v_ego_ms * self.LOOKAHEAD_S)

  @classmethod
  def target_kph(cls, dist_m):
    """방지턱까지 dist_m 남았을 때 목표 속도. HOLD_DIST_M 안쪽은 BUMP_SPEED_KPH 고정."""
    v_bump = cls.BUMP_SPEED_KPH * KPH_TO_MS
    extra = max(0., dist_m - cls.HOLD_DIST_M)
    return math.sqrt(v_bump ** 2 + 2. * cls.DECEL_MS2 * extra) / KPH_TO_MS
