#!/usr/bin/env python3

# --- LTP log debug helpers (schema-independent) ---
# These are ONLY for ltp_log(JSON lines) diagnostics.
LTP_EPS_DAMP_HOLD_S = 0.35     # should match latcontrol_torque EPS_DAMP_HOLD_S
LTP_EPS_CLIP_TH = 0.45         # clip spike threshold (qual_clip_ratio)
LTP_EPS_SAT_RATIO = 0.95       # steer_out_can close to steer_max => saturation proxy
# -*- coding: utf-8 -*-
"""
더 안전한 실시간 토크 추정기(Anchor Defaults 적용)
- MIN_VEL 중복 문제 해결 + 단위 명확화
- 더 안전한 PointBuckets.is_valid 로직
- latAccelOffset에 대한 온전성 경계 추가
- 직선 폴백 샘플링 완화(편향 감소)
- SVD를 중심으로 한 광범위한 예외 처리
- 더 강력한 페일세이프 + 더 명확한 로그
- (업데이트) 직선 샘플링이 더 엄격해짐(abs(latAcc)<0.2 & 2% -> 5%)
- (업데이트) 초기 적응 속도 향상, 안정화 속도 저하(감쇠 스케줄)
- (업데이트) 충분한 포인트가 수집되면 온전성 범위 좁아짐
- (수정) 'points'에 대해 filtered_params가 더 이상 생성되지 않음
- (수정) 데시메이션된 min_bucket_points가 int(min 1)임
- (수정) enough_points가 self.min_points_total을 일관되게 사용함
- (안전) 초기 타임라인 오류를 방지하기 위한 interp 가드
- (Anchor) 기본 latAccelFactor=2.4, friction=0.175  ← 변경
- (Weighted) 코너 데이터 가중치 상향, 직선 데이터 가중치 하향

[통합 개선]
- 직선 전용 버킷 추가(느슨한 yaw_rate<0.05, |latAcc|<0.3)
- 속도 기반 가변 반영(저속10%/중속20%/고속30%), warm-up 30초 동안 2배
- EMA(α=0.1) 기반 세션 간 누적 저장/복원

[버그픽스 추가]
- 정지/파킹 시 파라미터 **동결**(vEgo<0.3m/s + 2초 홀드)
- 추정값 NaN/비정상 시 **필터 업데이트 금지 및 안전값으로 대체**
- Raw/Filtered 필드는 항상 **숫자**로 채움(null 방지)
- 포인트 **디시메이션**으로 버킷 급증 억제
- 수동조향/비활성/정지 시 포인트 수집 차단 강화

[응답성 패치]
- 코너 전용 속도 임계 하향(20→약 10.8km/h, MIN_STEER_SPEED=3.0m/s 기준)
- 곡선에서 latAccelFactor **감소를 빠르게**, 증가는 느리게 (언더스티어(못 도는 느낌) 완화)
- 코너 포인트 채택 확률 소폭 상향(12%→18%)
- 워밍업 중 직선 블렌딩 상한 완화(0.6→0.45)

[요청 통합]
(1) 캐시 복원 points(코너+직선 혼합) → steer 기준으로 코너/직선 버킷에 분리 적재
(2) STEER_DELTA_DOWN_DIAG는 실제 CarControllerParams 기준 17 유지
(3) FORCE_TARGET_TUNING 밴드(±5%)가 너무 타이트 → ±8%로 완화
    + 추가: FORCE 밴드를 비대칭으로 하향을 더 넓힘(언더스티어 탈출 여지 확대)
(4) limited 반영 조건부 강화(clip/max/rate_limited 상황에서 limited_corner 반영 가중치 상향)
(5) 10km/h 이상 저속 코너(curve_active)에서는 업데이트 동결을 해제(정지/파킹 동결은 유지)

"""

import os
import sys
import signal
import atexit
import time
import json
import pickle
import tempfile
import numpy as np
import math
from collections import deque, defaultdict
from datetime import datetime, timezone, timedelta
from decimal import Decimal, InvalidOperation

import cereal.messaging as messaging
from cereal import car, log
from common.params import Params
from common.realtime import Priority, config_realtime_process, DT_MDL
from common.filter_simple import FirstOrderFilter
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from common.numpy_fast import interp, clip

# -----------------------------
# 로그 설정 (일자별 파일)
# -----------------------------
LTP_LOG_DIR = "/data/openpilot/ltp_logs"
KST = timezone(timedelta(hours=9))

# EPS proxy thresholds used by ltp_log snapshot/burst diagnostics
LTP_EPS_ERR_NEAR_MAX = LTP_EPS_SAT_RATIO
LTP_EPS_CTRL_NEAR_MAX = LTP_EPS_SAT_RATIO
LTP_EPS_CLIP_DEMAND = 0.70

# Burst / event trace (snapshot log is kept; burst logs are only emitted around key events)
LTP_BURST_LOG_DIR = os.path.join(LTP_LOG_DIR, "bursts")
ENABLE_BURST_TRACE = True
BURST_PRE_S = 2.0
BURST_POST_S = 2.5
BURST_COOLDOWN_S = 4.0
BURST_RING_MAX = 900
BURST_MAX_EVENTS_PER_FILE = 24
BURST_SAMPLE_LIMIT = 2400

# -----------------------------
# 이쿼녹스 토크 디폴트값 (요청값)
# -----------------------------
LIVE_TORQUE_TUNING_ENABLED = True  # False면 라이브 튜닝(학습/적응) OFF, 고정값만 publish
LAT_ACCEL_FACTOR_ANCHOR = 1.90
FRICTION_ANCHOR = 0.255

# -----------------------------
# Force targets (disable CP/cache overrides)
# -----------------------------
FORCE_TARGET_TUNING = True  # latAccelFactor/friction anchors are enforced

# ✅ FORCE 밴드: 비대칭(하향 넓게, 상향 보수적으로)
TARGET_FACTOR_BAND_UP = 0.06  # +6% (이쿼녹스 디젤: 고속 과민 방지)
TARGET_FACTOR_BAND_DOWN = 0.18  # -18% (저속/중속 코너 보정 여지)

TARGET_FRICTION_BAND = 0.18

# -----------------------------
# Adaptive FORCE band relaxation
#  - 초기에는 앵커 근처(안전)로 강하게 묶고,
#  - 코너 포인트가 충분히 쌓이면(force band를 점진적으로 확장) raw 추정을 반영
# -----------------------------
FORCE_BAND_RELAX_ENABLED = True

# relax 시작/완전 해제 구간(코너+limited 코너 포인트 기준)
FORCE_RELAX_START_MULT = 0.8  # 10~55km/h에서 필요한 보정이 더 빨리 살아나기 시작
FORCE_RELAX_FULL_MULT = 3.0  # 3.0 * min_points_total에서 완화 완료

# 완화 완료 시 밴드(앵커 대비)
FORCE_FACTOR_BAND_UP_MAX = 0.14  # +14%
FORCE_FACTOR_BAND_DOWN_MAX = 0.30  # -30%
FORCE_FRICTION_BAND_MAX = 0.28  # ±28%

# 절대 안전 클램프(혹시 모를 발산/오입력 방지)
LAT_ACCEL_FACTOR_ABS_MIN = 1.20
LAT_ACCEL_FACTOR_ABS_MAX = 4.50
FRICTION_ABS_MIN = 0.05
FRICTION_ABS_MAX = 0.60
# ✅ 직선 쏠림 보완: offset 학습 허용(단, 직선 샘플이 실제로 들어온 프레임에서만 업데이트 게이트)
DISABLE_LATACCEL_OFFSET_LEARNING = True

# Published latAccelFactor assist.
# Lower latAccelFactor commands more steering torque. Keep the learner stable, but
# make the published value slightly more assertive when clip-quality persists and
# there is no real EPS/max-limit evidence.
LAT_ACCEL_FACTOR_AGGRESSIVE_ASSIST = True
LAT_ACCEL_FACTOR_ASSIST_MIN_SCALE = 0.955  # assist는 최대 -4.5%까지만 개입
LAT_ACCEL_FACTOR_ASSIST_MAX_DELTA = 0.10
LAT_ACCEL_FACTOR_ASSIST_CLIP_RATIO_START = 0.16
LAT_ACCEL_FACTOR_ASSIST_CLIP_RATIO_FULL = 0.44
LAT_ACCEL_FACTOR_ASSIST_RATE_STRONG_LIMIT = 0.13

# ===== Live Torque Tuning "B-plan" + Warm Start =====
# B안: 직선 오프셋 업데이트는 '최근 구간(윈도우) 품질'로만 결정
# 제한(limited) 코너 샘플은 저장하되, 업데이트 반영 비중을 낮춤(기본 30%)
# ===== B-plan / Safety knobs =====
LIMITED_CORNER_WEIGHT = 0.30

# --- B안 Offset Update Presets ---
# env: LTP_OFFSET_PRESET = conservative | balanced | aggressive
#  - conservative: 안전 최우선(느리게)
#  - balanced:    실사용 밸런스(권장)
#  - aggressive:  빠르게 따라가되 step/clamp로 안전 확보
LTP_OFFSET_PRESET = os.environ.get("LTP_OFFSET_PRESET", "balanced").lower().strip()
_LTP_OFFSET_PRESETS = {
    "conservative": {"min_ok_abs": 45, "min_ok_frac": 0.70, "min_ratio": 0.90, "step_max": 0.002, "cooldown_s": 8.0},
    "balanced": {"min_ok_abs": 35, "min_ok_frac": 0.65, "min_ratio": 0.85, "step_max": 0.003, "cooldown_s": 5.0},
    "aggressive": {"min_ok_abs": 30, "min_ok_frac": 0.60, "min_ratio": 0.80, "step_max": 0.005, "cooldown_s": 3.0},
}
_LTP_OFFSET_CFG = _LTP_OFFSET_PRESETS.get(LTP_OFFSET_PRESET, _LTP_OFFSET_PRESETS["balanced"])

STRAIGHT_OK_MIN_SAMPLES = int(_LTP_OFFSET_CFG["min_ok_abs"])
STRAIGHT_OK_MIN_FRAC = float(_LTP_OFFSET_CFG["min_ok_frac"])
STRAIGHT_OK_MIN_RATIO = float(_LTP_OFFSET_CFG["min_ratio"])
OFFSET_UPDATE_MAX_STEP = float(_LTP_OFFSET_CFG["step_max"])
OFFSET_UPDATE_MIN_INTERVAL_S = float(_LTP_OFFSET_CFG["cooldown_s"])

# 직선 오프셋(윈도우) 아웃라이어 제거(MAD 기반)
STRAIGHT_MAD_K = 3.5
STRAIGHT_MAD_EPS = 1e-3

# Straight offset / soft liveValid helpers
STRAIGHT_OK_RECENT_S = 2.5  # straight_ok가 최근에 있었는지(오프셋 업데이트 트리거)
SOFT_LIVEVALID_MIN_S = 6.0  # 재시작/저속에서 빠르게 liveValid 진입(안전한 클램프 값 사용)
SOFT_LIVEVALID_MIN_POINTS_LOWSPD = 250
SOFT_LIVEVALID_MIN_POINTS_MIDSPD = 450
SOFT_LIVEVALID_MIN_POINTS_HISPD = 700

# Warm start(재시작 복원) 상태 파일
LTP_STATE_VERSION = 2
LTP_STATE_SAVE_INTERVAL_S = 20.0  # faster warm-state persistence
LTP_STATE_PATH = os.path.join(LTP_LOG_DIR, "ltp_state.json")
LTP_STATE_PATH_PKL = os.path.join(LTP_LOG_DIR, "ltp_state.pkl")

# Warm restore compatibility
LTP_WARM_COMPAT_MAX_VERSION_GAP = 0  # major tuning change: do not restore old bucket/filter state across VERSION changes
LTP_STATE_SAVE_MIN_DELTA_PTS = 300  # save sooner when many new points are added

# VERSION 변경 시 이전 warm-state/LiveTorqueParameters/사용자 anchor Param을 확실히 버린다.
# 기존에는 VERSION mismatch로 warm-state만 스킵하고, 종료 시 EMA merge나 Runtime Param이
# 예전 값을 다시 주입할 수 있어서 source anchor(1.90/0.255)로 초기화되지 않는 문제가 있었다.
LTP_VERSION_PARAM_KEY = "LiveTorqueLastAppliedVersion"
LTP_CLEAR_RUNTIME_ANCHOR_PARAMS_ON_VERSION_CHANGE = True
LTP_RUNTIME_ANCHOR_PARAM_KEYS = ("TorqueMaxLatAccel", "TorqueFriction")

# Tunables
# -----------------------------
HISTORY = 5  # secs
POINTS_PER_BUCKET = 1500
MIN_POINTS_TOTAL = 1500
MIN_POINTS_TOTAL_QLOG = 500
FIT_POINTS_TOTAL = 2000
FIT_POINTS_TOTAL_QLOG = 600

# Velocity thresholds (m/s). 10 km/h ≈ 2.78 m/s
MIN_VEL_MS = 11.11  # 40 km/h (직선/오프셋 임계 상향: 직선 학습 비중↓) (직선/기본 임계 유지)
MIN_VEL_MS_BIAS = 5.56  # 20 km/h (직선 bias/오프셋 업데이트 최소 속도 - 저속 학습 조기 진입)
MIN_VEL_MS_STRAIGHT = 5.56  # 20 km/h (직선 포인트 수집 최소 속도 - 저속에서도 샘플 수집)

# 직선 샘플링에서 rate-limit 차단을 완화(오검출/경미한 제한은 허용)
STRAIGHT_RATE_LIM_ALLOW_DELTA = 0.02  # |desired-applied|가 이 값 이하이면 rate-limit이어도 직선 OK
RATE_LIMITED_STRONG_ERR = 0.45  # 이 이상이면 '강한' rate-limit로 간주

# 30~60km/h 구간에서만 델타 제한 완화(고속은 그대로).
# NOTE: 실제 조향 델타 제한(컨트롤러)도 동일하게 적용해야 체감이 같이 좋아집니다.
MIDSPD_KPH_IN_LO = 25.0
MIDSPD_KPH_IN_HI = 30.0
MIDSPD_KPH_OUT_LO = 60.0
MIDSPD_KPH_OUT_HI = 70.0
MIDSPD_DELTA_UP_GAIN = 0.24  # 이쿼녹스 디젤: 중속 반응 보강, 과한 튐 방지
MIDSPD_DELTA_DOWN_GAIN = 0.14

MIN_VEL_CURVE_MS = 3.00  # ✅ 10.8 km/h (CarControllerParams.MIN_STEER_SPEED=3.0m/s 기준)
FREEZE_UPDATE_MS = 3.00  # ✅ 10.8 km/h 이상 저속 코너에서는 업데이트 동결 해제
PARK_VEL_MS = 0.30  # 정지/파킹 판정
FREEZE_AFTER_STOP_S = 2.0  # 정지 후 추가 홀드

# Sample decimation (버킷 폭증 억제)
CORNER_KEEP_PROB = 0.18  # ✅ 코너 포인트 채택 확률(12% → 18%)
STRAIGHT_KEEP_PROB = 0.07  # ✅ 직선 포인트 채택 확률(2% → 5%)

# 이쿼녹스 디젤 저속 코너 학습 보정
# - 10.8~20km/h 코너 데이터는 수집하되 영향력을 낮춰 저속 와리가리/유턴 노이즈 과반영을 방지
# - 20~30km/h는 거의 정상 반영, 30km/h 이상은 기존 반영
LOWSPD_CORNER_KPH = 20.0
MIDLOW_CORNER_KPH = 30.0
LOWSPD_CORNER_KEEP_SCALE = 0.50
LOWSPD_CORNER_TIMES_SCALE = 0.55
MIDLOW_CORNER_KEEP_SCALE = 0.75
MIDLOW_CORNER_TIMES_SCALE = 0.80

# Point replication cap (times)
MAX_POINT_TIMES = 8

# --- 고조향/리밋 샘플 수집 옵션 ---
RATE_LIMITED_MIX_ENABLED = False
RATE_LIMITED_MIX_KEEP_PROB_MULT = 0.20
RATE_LIMITED_MIX_TIMES_MULT = 0.50
RATE_LIMITED_MIX_ABS_STEER_MIN = 0.70
RATE_LIMITED_MIX_ABS_STEER_MAX = 0.98

FRICTION_FACTOR = 1.5  # ~85% of data coverage
FACTOR_SANITY = 0.3  # latAccelFactor sanity band ±30%
FRICTION_SANITY = 0.5  # friction sanity band ±50%
OFFSET_SANITY_ABS = 0.25  # ✅ |latAccelOffset| ≤ 0.25 (offset 폭주로 한쪽 박힘 방지)

STEER_MIN_THRESHOLD = 0.02
MIN_FILTER_DECAY = 40
MAX_FILTER_DECAY = 200
LAT_ACC_THRESHOLD = 3.0  # absolute lateral acceleration limit (m/s^2)

# steer bucket ranges (코너용) — 1.0 범위까지 확장
STEER_BUCKET_BOUNDS = [
    (-1.0, -0.9), (-0.9, -0.7), (-0.7, -0.5),
    (-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1), (-0.1, 0.0),
    (0.0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5),
    (0.5, 0.7), (0.7, 0.9), (0.9, 1.0001)
]
MIN_BUCKET_POINTS = np.array([30, 60, 80, 100, 300, 500, 500, 500, 500, 300, 100, 80, 60, 30])

# 직선 전용 버킷 (steer≈0 영역만 사용)
STRAIGHT_STEER_MAX = 0.15
STRAIGHT_BUCKET_BOUNDS = [(-STRAIGHT_STEER_MAX, STRAIGHT_STEER_MAX)]
MIN_BUCKET_POINTS_STRAIGHT = np.array([200])

# 직선 판별 파라미터
STRAIGHT_YAW_RATE_MAX = 0.04  # rad/s (≈2.9°/s)
STRAIGHT_LATACC_MAX = 0.15  # ✅ m/s^2

# 직선 오프셋(Offset) 학습용: '거의 0 조향'만 허용
STRAIGHT_STEER_MAX_FOR_OFFSET = 0.09  # normalized steer
STRAIGHT_SAMPLE_RECENT_S = 1.0

# warm-up
WARMUP_SECS = 30.0
EMA_ALPHA = 0.1

# ✅ 비대칭 적응(곡선 즉응성 향상) - (이번 적용에서 "감소 빠르게 / 증가 느리게"로 변경)
CURVE_YAWRATE_MIN = 0.05  # 곡률 감지 최소 요레이트(rad/s)
CURVE_UP_FAST_MULT = 0.40  # (기존 변수는 유지하되, update 로직을 바꿔 실사용 의미가 달라짐)
CURVE_DOWN_SLOW_MULT = 1.5
STRAIGHT_WARMUP_MAX_W = 0.25

# latActive gate / 직선 오프셋 업데이트 게이트
LAT_ACTIVE_STALE_S = 0.5
STRAIGHT_POINTS_SOFT_CAP = int(POINTS_PER_BUCKET * 0.6)

MAX_RESETS = 5.0
MAX_INVALID_THRESHOLD = 10
MIN_ENGAGE_BUFFER = 2  # secs

# -----------------------------
# Controller diagnostics (for steer limit flags)
# -----------------------------
STEER_MAX_DIAG = 300
STEER_DELTA_UP_DIAG = 10
STEER_DELTA_DOWN_DIAG = 17
STEER_SAT_THRESHOLD = 0.98
STEER_CLIP_EPS = 0.05  # ignore small desired/applied gaps that are normal actuator lag
STEER_CLIP_MIN_DES = 0.18  # ignore 'clip' inference when desired is small
STEER_CLIP_REL = 0.18  # relative gap threshold (fraction of desired)
STEER_DELTA_UP_NORM = STEER_DELTA_UP_DIAG / float(STEER_MAX_DIAG)
STEER_DELTA_DOWN_NORM = STEER_DELTA_DOWN_DIAG / float(STEER_MAX_DIAG)

# -----------------------------
# Live tuning update gating (clip/rate/quality window)
# -----------------------------
# 최근 N초 품질 윈도우에서 제한/개입 비율이 높으면 업데이트(학습) 동결
QUALITY_WIN_S = 10.0
QUALITY_MIN_SAMPLES = 10

QUALITY_CLIP_FREEZE_RATIO = 0.35  # legacy fallback; hysteresis thresholds below are used
QUALITY_RATE_FREEZE_RATIO = 0.30  # rate_limited(약+강) 비율
QUALITY_RATE_STRONG_FREEZE_RATIO = 0.22  # strong rate_limited 비율
QUALITY_STEER_PRESSED_FREEZE_RATIO = 0.30  # steeringPressed 비율
# Quality freeze should react to real sustained under-actuation, not every mild steer lag.
QUALITY_CLIP_MIN_DES = 0.25
QUALITY_CLIP_MIN_DELTA_ERR = 0.18
# Hysteresis + low-speed branch (reduce sawtooth near thresholds)
QUALITY_LOW_SPEED_KPH = 45.0
# Clip enter/exit (high speed vs low speed)
QUALITY_CLIP_FREEZE_ENTER_HIGH = 0.60
QUALITY_CLIP_FREEZE_EXIT_HIGH  = 0.38
QUALITY_CLIP_FREEZE_ENTER_LOW  = 0.82    # 저속 코너 학습은 살리되, 나쁜 데이터 과반영 방지
QUALITY_CLIP_FREEZE_EXIT_LOW   = 0.58
# Both(clip+pressed) hysteresis (low-speed only)
QUALITY_BOTH_FREEZE_ENTER_LOW  = 0.28
QUALITY_BOTH_FREEZE_EXIT_LOW   = 0.20
# Conditional shorter hold when just over threshold (clip/both only)
QUALITY_NEAR_THRESH_MARGIN = 0.03
QUALITY_FREEZE_HOLD_S_SHORT = 0.45


# steeringPressed 디바운스 (스파이크 민감도 완화)
STEER_PRESSED_ON_FRAMES = 2  # ≈20ms @100Hz
STEER_PRESSED_OFF_FRAMES = 7  # ≈70ms @100Hz
STEER_PRESSED_DRIVER_TORQUE_MIN = 0.80

# steeringPressed + steer_clip 동시 발생 비율 (체감 악화 1순위 케이스)
QUALITY_BOTH_FREEZE_RATIO = 0.18

QUALITY_FREEZE_HOLD_S = 0.70  # 트리거 시 최소 홀드

# rate_limited 구간: 과도/준정상 분기 + latAccelFactor 하향 금지
RATE_LIM_TRANSIENT_DES_DELTA = 0.012  # desired 변화량이 이 이상이면 과도로 간주(정규화 steer)
RATE_LIM_TRANSIENT_HOLD_S = 0.6  # 과도 판단을 잠깐 유지(초)

RATE_LIM_STEADY_BLEND_W = 0.20  # 준정상 rate-limit 구간에서 업데이트 반영 비중(0~1)
RATE_LIM_STEADY_FRICTION_BLEND_W = 0.15  # 준정상 rate-limit 구간에서 friction 반영 비중(0~1)


# Applied profile: Equinox 2020 Diesel
# - CarControllerParams matched: STEER_MAX=300, STEER_DELTA_UP=10, STEER_DELTA_DOWN=17, MIN_STEER_SPEED=3.0m/s
# - Corner learning starts at 3.00m/s (~10.8km/h); straight/offset learning remains >=20km/h
VERSION = 25  # reset-fix: clear stale state/cache/runtime anchors on VERSION change


def slope2rot(slope):
    sin = np.sqrt(slope ** 2 / (slope ** 2 + 1))
    cos = np.sqrt(1 / (slope ** 2 + 1))
    return np.array([[cos, -sin], [sin, cos]])


def merge_with_cache(old_val, new_val, alpha=EMA_ALPHA):
    if old_val is None or np.isnan(old_val):
        return new_val
    if new_val is None or np.isnan(new_val):
        return old_val
    return (1.0 - alpha) * float(old_val) + alpha * float(new_val)


def _finite(val):
    return val is not None and np.isfinite(val)


def _sanitize_num(val, fallback):
    return float(val) if _finite(val) else float(fallback)


def _median_safe(vals):
    v = [x for x in vals if x is not None and np.isfinite(x)]
    if len(v) == 0:
        return None
    v.sort()
    n = len(v)
    return float(v[n // 2]) if (n % 2 == 1) else float(0.5 * (v[n // 2 - 1] + v[n // 2]))


def _mean_safe(vals):
    v = [x for x in vals if x is not None and np.isfinite(x)]
    if len(v) == 0:
        return None
    return float(sum(v) / len(v))


def _slugify_label(val, max_len: int = 48):
    s = str(val or "event").strip().lower()
    out = []
    for ch in s:
        if ch.isalnum():
            out.append(ch)
        elif ch in (" ", "-", "_", ".", "/", "|", ":"):
            out.append("_")
    slug = "".join(out)
    while "__" in slug:
        slug = slug.replace("__", "_")
    slug = slug.strip("._")
    if not slug:
        slug = "event"
    return slug[:max_len]


class NPQueue:
    """Fixed-size ring buffer for points (fast, no np.append reallocations)."""

    def __init__(self, maxlen, rowsize):
        self.maxlen = int(maxlen)
        self.rowsize = int(rowsize)
        self._buf = np.empty((self.maxlen, self.rowsize), dtype=np.float64)
        self._size = 0
        self._head = 0  # next write index

    def __len__(self):
        return int(self._size)

    def append(self, pt):
        self._buf[self._head] = pt
        self._head = (self._head + 1) % self.maxlen
        self._size = min(self._size + 1, self.maxlen)

    def to_numpy(self):
        """Return points in chronological order as (N, rowsize)."""
        if self._size == 0:
            return np.empty((0, self.rowsize), dtype=np.float64)
        if self._size < self.maxlen:
            return self._buf[:self._size].copy()
        return np.concatenate((self._buf[self._head:], self._buf[:self._head]), axis=0)


class BucketMoments:
    __slots__ = ("cap", "n", "sx", "sy", "sxx", "syy", "sxy")

    def __init__(self, cap: int):
        self.cap = float(max(1, int(cap)))
        self.n = 0.0
        self.sx = 0.0
        self.sy = 0.0
        self.sxx = 0.0
        self.syy = 0.0
        self.sxy = 0.0

    def _scale(self, s: float):
        s = float(max(0.0, s))
        self.n *= s
        self.sx *= s
        self.sy *= s
        self.sxx *= s
        self.syy *= s
        self.sxy *= s

    def add(self, x: float, y: float, w: int = 1):
        w = int(max(1, w))
        x = float(x);
        y = float(y)

        if self.n > self.cap:
            self._scale(self.cap / max(self.n, 1e-9))

        ww = float(w)
        self.n += ww
        self.sx += ww * x
        self.sy += ww * y
        self.sxx += ww * x * x
        self.syy += ww * y * y
        self.sxy += ww * x * y

        if self.n > self.cap:
            self._scale(self.cap / max(self.n, 1e-9))

    def to_dict(self):
        return {
            "n": float(self.n),
            "sx": float(self.sx), "sy": float(self.sy),
            "sxx": float(self.sxx), "syy": float(self.syy),
            "sxy": float(self.sxy),
        }

    def from_dict(self, d):
        try:
            self.n = float(d.get("n", 0.0))
            self.sx = float(d.get("sx", 0.0));
            self.sy = float(d.get("sy", 0.0))
            self.sxx = float(d.get("sxx", 0.0));
            self.syy = float(d.get("syy", 0.0))
            self.sxy = float(d.get("sxy", 0.0))
        except Exception:
            self.n = 0.0
            self.sx = self.sy = self.sxx = self.syy = self.sxy = 0.0


class PointBuckets:
    def __init__(self, x_bounds, min_points, min_points_total,
                 require_coverage: bool = False,
                 coverage_ratio: float = 0.25,
                 require_symmetry: bool = False,
                 cap_per_bucket: int = POINTS_PER_BUCKET):
        self.x_bounds = x_bounds
        self.buckets = {bounds: BucketMoments(cap=int(cap_per_bucket)) for bounds in x_bounds}
        self.buckets_min_points = {bounds: int(min_point) for bounds, min_point in zip(x_bounds, min_points)}
        self.min_points_total = int(min_points_total)

        self.require_coverage = bool(require_coverage)
        self.coverage_ratio = float(coverage_ratio)
        self.require_symmetry = bool(require_symmetry)

    def bucket_lengths(self):
        return [int(round(v.n)) for v in self.buckets.values()]

    def __len__(self):
        return int(round(sum(v.n for v in self.buckets.values())))

    def is_valid(self):
        bucket_check = all(
            (v.n >= float(min_pts)) for v, min_pts in zip(self.buckets.values(), self.buckets_min_points.values()))
        total_check = (float(sum(v.n for v in self.buckets.values())) >= float(self.min_points_total))

        if bucket_check:
            return True
        if not total_check:
            return False
        if not self.require_coverage:
            return True

        min_hits = max(2, int(math.ceil(len(self.x_bounds) * 0.50)))
        hits = 0
        neg_hits = 0
        pos_hits = 0

        for (low, high), bm in self.buckets.items():
            tgt = self.buckets_min_points[(low, high)]
            need = max(30, int(tgt * self.coverage_ratio))
            if bm.n >= float(need):
                hits += 1
                if high <= 0.0:
                    neg_hits += 1
                if low >= 0.0:
                    pos_hits += 1

        if hits < min_hits:
            return False
        if self.require_symmetry and (neg_hits == 0 or pos_hits == 0):
            return False
        return True

    def show_bucket_status(self):
        total = float(sum(v.n for v in self.buckets.values()))
        total_ok = total >= float(self.min_points_total)

        hits = 0
        for (low, high), bm in self.buckets.items():
            tgt = self.buckets_min_points[(low, high)]
            need = max(30, int(tgt * self.coverage_ratio)) if self.require_coverage else tgt
            if bm.n >= float(need):
                hits += 1

        lines = []
        lines.append(f"[TOTAL] {int(round(total))} / {int(self.min_points_total)} => {'OK' if total_ok else 'FAIL'}\n")
        if self.require_coverage:
            lines.append(
                f"[COVERAGE] hits={hits}/{len(self.x_bounds)} ratio={self.coverage_ratio:.2f} "
                f"symmetry={'ON' if self.require_symmetry else 'OFF'}\n"
            )

        cross_bounds = [(low, high) for (low, high) in self.x_bounds if (low < 0.0 < high)]
        non_cross_bounds = [(low, high) for (low, high) in self.x_bounds if not (low < 0.0 < high)]

        def _fmt_bucket(bounds):
            if bounds is None:
                return "[N/A] 0/0 (X)"
            bm = self.buckets.get(bounds, None)
            tgt = self.buckets_min_points.get(bounds, None)
            low, high = bounds
            if bm is None or tgt is None:
                return f"[{low:.2f}~{high:.2f}] 0/0 (X)"

            cur = int(round(bm.n))
            tgt_i = int(tgt)
            ok = cur >= tgt_i
            return f"[{low:.2f}~{high:.2f}]{cur}/{tgt_i}({'O' if ok else 'X'})"

        for b in cross_bounds:
            lines.append(f"{_fmt_bucket(b)}\n")

        pair_map = {}
        ordered_keys = []

        def _pair_key(low, high):
            a = round(min(abs(low), abs(high)), 2)
            b = round(max(abs(low), abs(high)), 2)
            return (a, b)

        for (low, high) in non_cross_bounds:
            k = _pair_key(low, high)
            if k not in pair_map:
                pair_map[k] = {'-': None, '+': None}
                ordered_keys.append(k)

            if high <= 0.0:
                pair_map[k]['-'] = (low, high)
            elif low >= 0.0:
                pair_map[k]['+'] = (low, high)

        for k in ordered_keys:
            left = _fmt_bucket(pair_map[k]['-'])
            right = _fmt_bucket(pair_map[k]['+'])
            lines.append(f"{left} {right}\n")

        return "".join(lines)

    def add_point(self, x, y, times: int = 1):
        times = max(1, min(MAX_POINT_TIMES, int(times)))
        for bound_min, bound_max in self.x_bounds:
            if (x >= bound_min) and (x < bound_max):
                self.buckets[(bound_min, bound_max)].add(float(x), float(y), w=times)
                break

    def get_points(self, num_points=None):
        return np.empty((0, 3), dtype=np.float64)

    def load_points(self, points):
        if points is None:
            return
        for x, y in points:
            try:
                self.add_point(float(x), float(y), times=1)
            except Exception:
                continue

    def aggregate_moments(self):
        n = sx = sy = sxx = syy = sxy = 0.0
        for bm in self.buckets.values():
            n += bm.n
            sx += bm.sx;
            sy += bm.sy
            sxx += bm.sxx;
            syy += bm.syy
            sxy += bm.sxy
        return n, sx, sy, sxx, syy, sxy

    def to_state_dict(self):
        out = {}
        for (low, high), bm in self.buckets.items():
            key = f"{low:.3f},{high:.3f}"
            out[key] = bm.to_dict()
        return {
            "min_points_total": int(self.min_points_total),
            "buckets": out,
        }

    def from_state_dict(self, st):
        if not st or "buckets" not in st:
            return
        b = st.get("buckets", {})
        for (low, high), bm in self.buckets.items():
            key = f"{low:.3f},{high:.3f}"
            if key in b:
                bm.from_dict(b[key])


class TorqueEstimator:
    def __init__(self, CP, decimated=False):
        self.hist_len = int(HISTORY / DT_MDL)
        self.lag = CP.steerActuatorDelay + 0.2

        self.offline_latAccelFactor = LAT_ACCEL_FACTOR_ANCHOR
        self.offline_friction = FRICTION_ANCHOR
        self.resets = 0

        if decimated:
            self.min_bucket_points = np.maximum((MIN_BUCKET_POINTS // 10), 1).astype(int)
            self.min_points_total = MIN_POINTS_TOTAL_QLOG
            self.fit_points = FIT_POINTS_TOTAL_QLOG
            self.min_bucket_points_straight = np.maximum((MIN_BUCKET_POINTS_STRAIGHT // 10), 1).astype(int)
            self.min_points_total_straight = max(int(MIN_POINTS_TOTAL_QLOG * 0.3), 100)
        else:
            self.min_bucket_points = MIN_BUCKET_POINTS
            self.min_points_total = MIN_POINTS_TOTAL
            self.fit_points = FIT_POINTS_TOTAL
            self.min_bucket_points_straight = MIN_BUCKET_POINTS_STRAIGHT
            self.min_points_total_straight = max(int(MIN_POINTS_TOTAL * 0.3), 200)

        self.points = []
        self.is_valid = False

        self._last_lat_active_cc = None
        self._last_lat_active_cc_t = None
        self._lat_active_src = "unknown"
        self._last_enabled_cc = None
        self._last_enabled_cc_t = None

        self._last_lkas_enable = None
        self._last_lkas_enable_t = None

        self._straight_win_ok = 0
        self._straight_win_total = 0
        self._straight_win_ok_ratio = 0.0
        self._last_offset_update_t = -1e9

        if (not FORCE_TARGET_TUNING) and CP.lateralTuning.which() == 'torque':
            try:
                fric_cp = float(CP.lateralTuning.torque.friction)
                fact_cp = float(CP.lateralTuning.torque.latAccelFactor)
                if np.isfinite(fric_cp) and np.isfinite(fact_cp) and (fact_cp > 0.5) and (fric_cp > 0.08):
                    self.offline_friction = fric_cp
                    self.offline_latAccelFactor = fact_cp
                else:
                    cloudlog.warning(
                        f"torque estimator: CP torque values out of sanity (latAccelFactor={fact_cp}, friction={fric_cp}); using anchors")
            except Exception:
                cloudlog.warning("torque estimator: failed to read CP torque tuning; using anchors")

        self.version = VERSION
        self.base_params = {
            'latAccelFactor': float(self.offline_latAccelFactor),
            'latAccelOffset': 0.0,
            'frictionCoefficient': float(self.offline_friction),
        }

        self.start_time = None
        self.last_vego = None
        self.last_yaw_rate = None
        self.last_time = None
        self._ltp_prev_t = None
        self._ltp_eps_damp_until = 0.0
        # controlsState (latcontrol) debug mirror
        self._lc_prev_t = None
        self._lc_dt = None
        self._lc_eps_evt = False
        self._lc_eps_damp = False
        self._lc_eps_damp_until = 0.0
        self.last_is_frozen = False
        self.stop_freeze_until = 0.0

        self._straight_bias_win_s = 10.0
        self._straight_bias = deque()
        self.last_straight_sampled = False
        self.last_straight_w = 0.0
        self._last_straight_sampled_t = -1e9

        self.last_lat_active = False
        self.last_steer_desired = None
        self.last_steer_applied = None
        self.last_rate_limited = False
        self.last_rate_limited_strong = False
        self.last_delta_err = 0.0
        self.last_rate_lim_w = 0.0
        self.last_delta_lim_up = float(STEER_DELTA_UP_NORM)
        self.last_delta_lim_dn = float(STEER_DELTA_DOWN_NORM)
        self.last_max_limited = False
        self.last_steer_clip = False
        self._last_clip_quality = False
        self._last_clip_raw = False
        self._prev_steer_applied = None
        self._prev_steer_applied_t = None

        self._prev_steer_desired = None
        self._prev_steer_desired_t = None

        self._last_desired_delta = 0.0
        self._last_applied_delta = 0.0
        self._rate_transient_until = 0.0

        self._qual_win = deque()
        self._qual_freeze_until = 0.0
        # qual_freeze extend debug
        self._qual_freeze_extend_cnt = 0
        self._qual_freeze_ext_evt = None
        # steeringPressed debounce state
        self._steer_pressed_cnt = 0
        self._last_steer_override_db = False

        self._last_valid_applied = None

        self._log_dir = LTP_LOG_DIR
        self._log_date = None
        self._log_path = None

        # snapshot + burst/event trace state
        self._burst_dir = LTP_BURST_LOG_DIR
        self._burst_ring = deque(maxlen=BURST_RING_MAX)
        self._burst_active = None
        self._burst_seq = 0
        self._burst_prev_flags = {}
        self._burst_prev_ext_cnt = 0
        self._burst_last_start_t = -1e9
        self._burst_last_close_t = -1e9
        self._last_burst_path = None
        self._last_burst_trigger = None
        self._last_burst_meta = None
        self._ltp_dt = None
        self._ltp_eps_evt_proxy = False
        self._ltp_eps_damp_proxy = False
        self._last_v_kph = 0.0

        self.reset()

        # warm/soft state flags + debug cache
        self._warm_restored = False
        self._warm_restore_source = None
        self._last_state_save_pts = 0
        self._last_straight_ok_t = -1e9
        self._last_driver_torque = 0.0
        self._last_eps_torque = 0.0
        self._last_allowed_torque = 0.0
        self._last_steer_out_can = 0.0
        self._last_steer_max = 0.0
        self._latAF_assist_active = False
        self._latAF_assist_base = 0.0
        self._latAF_assist_scale = 1.0
        self._latAF_assist_delta = 0.0
        self._latAF_assist_clip_ratio = 0.0

        initial_params = {
            'latAccelFactor': self.offline_latAccelFactor,
            'latAccelOffset': 0.0,
            'frictionCoefficient': self.offline_friction,
        }
        initial_points = []
        self.decay = MIN_FILTER_DECAY

        self.min_lataccel_factor = (1.0 - FACTOR_SANITY) * self.offline_latAccelFactor
        self.max_lataccel_factor = (1.0 + FACTOR_SANITY) * self.offline_latAccelFactor
        self.min_friction = (1.0 - FRICTION_SANITY) * self.offline_friction
        self.max_friction = (1.0 + FRICTION_SANITY) * self.offline_friction
        self.max_offset_abs = OFFSET_SANITY_ABS
        self._car_fingerprint = getattr(CP, 'carFingerprint', None)
        self._car_tune_type = CP.lateralTuning.which() if hasattr(CP, 'lateralTuning') else None

        self._pending_straight_restore = None
        warm_loaded, warm_params, warm_decay = self._try_restore_warm_state(CP)
        if warm_loaded:
            initial_params = warm_params
            self.decay = warm_decay

        if not warm_loaded:
            params = Params()
            params_cache = params.get("LiveTorqueCarParams")
            torque_cache = params.get("LiveTorqueParameters")
            if params_cache is not None and torque_cache is not None:
                try:
                    cache_ltp = log.Event.from_bytes(torque_cache).liveTorqueParameters
                    cache_CP = car.CarParams.from_bytes(params_cache)
                    if self.get_restore_key(cache_CP, cache_ltp.version) == self.get_restore_key(CP, VERSION):
                        if cache_ltp.liveValid:
                            if FORCE_TARGET_TUNING:
                                initial_params = {
                                    'latAccelFactor': self.offline_latAccelFactor,
                                    'latAccelOffset': merge_with_cache(0.0, cache_ltp.latAccelOffsetFiltered),
                                    'frictionCoefficient': self.offline_friction
                                }
                            else:
                                initial_params = {
                                    'latAccelFactor': merge_with_cache(self.offline_latAccelFactor,
                                                                       cache_ltp.latAccelFactorFiltered),
                                    'latAccelOffset': merge_with_cache(0.0, cache_ltp.latAccelOffsetFiltered),
                                    'frictionCoefficient': merge_with_cache(self.offline_friction,
                                                                            cache_ltp.frictionCoefficientFiltered)
                                }

                        initial_points = cache_ltp.points
                        self.decay = cache_ltp.decay

                        if initial_points is not None:
                            for x, y in initial_points:
                                try:
                                    x = float(x);
                                    y = float(y)
                                except Exception:
                                    continue
                                if abs(x) <= STRAIGHT_STEER_MAX:
                                    self.straight_points.add_point(x, y, times=1)
                                else:
                                    self.corner_points.add_point(x, y, times=1)

                        cloudlog.info("restored torque params from EMA cache (split corner/straight)")
                except Exception:
                    cloudlog.exception("failed to restore cached torque params")
                    try:
                        params.remove("LiveTorqueCarParams")
                        params.remove("LiveTorqueParameters")
                    except Exception as e:
                        cloudlog.warning(f"torque cache remove error: {e}")

        self.filtered_params = {
            'latAccelFactor': FirstOrderFilter(initial_params['latAccelFactor'], self.decay, DT_MDL),
            'latAccelOffset': FirstOrderFilter(initial_params['latAccelOffset'], self.decay, DT_MDL),
            'frictionCoefficient': FirstOrderFilter(initial_params['frictionCoefficient'], self.decay, DT_MDL),
        }

    def _midspd_weight(self, v_kph: float) -> float:
        """
      중속(대략 25~90kph) 구간을 더 신뢰하도록 샘플 가중치 부여.
      - 저속(정지/근거리)은 글리치/노이즈가 많아 가중치 낮춤
      - 중속은 가중치 높임
      - 고속은 필요시 약간 낮춤(과도한 고속 편향 방지)
      """
        try:
            vk = float(v_kph)
        except Exception:
            return 0.0

        # 0~8: 0
        # 8~25: 0 -> 1 램프
        # 25~90: 1 유지
        # 90~130: 1 -> 0.4
        # 130~180: 0.4 -> 0.2
        w = float(interp(vk,
                         [0.0, 8.0, 25.0, 90.0, 130.0, 180.0],
                         [0.0, 0.0, 1.0, 1.0, 0.4, 0.2]))
        return float(clip(w, 0.0, 1.0))

    def straight_weight(self, vego: float, yaw_rate: float, t: float) -> float:
        if yaw_rate is None or abs(yaw_rate) > STRAIGHT_YAW_RATE_MAX:
            return 0.0
        if vego is None:
            return 0.0

        if vego < 20.0:
            w = 0.05
        elif vego < 33.0:
            w = 0.10
        else:
            w = 0.15

        if (self.start_time is not None) and (t - self.start_time < WARMUP_SECS):
            w *= 1.5

        return float(max(0.0, min(STRAIGHT_WARMUP_MAX_W, w)))

    def _get_lat_active(self, t: float) -> bool:
        try:
            if self._last_lkas_enable_t is not None and (t - float(self._last_lkas_enable_t)) <= LAT_ACTIVE_STALE_S:
                self._lat_active_src = "carState.lkasEnable"
                return bool(self._last_lkas_enable)
        except Exception:
            pass

        try:
            if self._last_lat_active_cc_t is not None and (t - float(self._last_lat_active_cc_t)) <= LAT_ACTIVE_STALE_S:
                self._lat_active_src = "carControl.latActive"
                return bool(self._last_lat_active_cc)
        except Exception:
            pass

        try:
            if self._last_enabled_cc_t is not None and (t - float(self._last_enabled_cc_t)) <= LAT_ACTIVE_STALE_S:
                if bool(getattr(self, "_last_enabled_cc", False)):
                    if bool(getattr(self, "_last_steer_override", False)):
                        return False

                    des = getattr(self, "last_steer_desired", None)
                    app = getattr(self, "last_steer_applied", None)
                    if des is None or app is None:
                        return False
                    if (not np.isfinite(float(des))) or (not np.isfinite(float(app))):
                        return False

                    if abs(float(des)) < 0.03 and abs(float(app)) < 0.03:
                        return False

                    self._lat_active_src = "inferred(enabled)"
                    return True
        except Exception:
            pass

        return bool(getattr(self, "last_lat_active", False))

    def _update_straight_window_stats(self):
        try:
            win = list(self._straight_bias) if hasattr(self, "_straight_bias") else []
            ok = [s for s in win if len(s) >= 6 and bool(s[5])]
            # NOTE: straight window의 total을 "ok 샘플"로 두어야 오프셋 업데이트 최소 샘플 조건이 현실적으로 만족됩니다.
            total = ok
            self._straight_win_ok = int(len(ok))
            self._straight_win_total = int(len(total))
            self._straight_win_ok_ratio = float(
                self._straight_win_ok / self._straight_win_total) if self._straight_win_total > 0 else 0.0
        except Exception:
            self._straight_win_ok = 0
            self._straight_win_total = 0
            self._straight_win_ok_ratio = 0.0

    def _straight_min_ok_required(self, total: int) -> int:
        try:
            tot = int(max(int(total), 0))
            frac = float(STRAIGHT_OK_MIN_FRAC) if 'STRAIGHT_OK_MIN_FRAC' in globals() else 0.65
            return int(max(int(STRAIGHT_OK_MIN_SAMPLES), int(math.ceil(float(tot) * frac))))
        except Exception:
            return int(STRAIGHT_OK_MIN_SAMPLES)

    def _offset_update_allowed(self) -> bool:
        if DISABLE_LATACCEL_OFFSET_LEARNING:
            return False
        t = float(getattr(self, "last_time", 0.0) or 0.0)
        vego = float(getattr(self, "last_vego", 0.0) or 0.0)
        # 저속에서도 straight_ok가 충분하면 오프셋 업데이트 허용
        if vego < float(MIN_VEL_MS_BIAS):
            return False
        if not self._get_lat_active(t):
            return False
        try:
            if self._freeze_check(t, vego):
                return False
        except Exception:
            pass
        try:
            if (t - float(getattr(self, '_last_straight_ok_t', -1e9))) > float(STRAIGHT_OK_RECENT_S):
                return False
        except Exception:
            return False

        self._update_straight_window_stats()
        min_ok = self._straight_min_ok_required(self._straight_win_total)
        if self._straight_win_ok < int(min_ok):
            return False
        if self._straight_win_ok_ratio < STRAIGHT_OK_MIN_RATIO:
            return False
        if (t - float(self._last_offset_update_t)) < OFFSET_UPDATE_MIN_INTERVAL_S:
            return False
        return True

    def _soft_livevalid_min_points(self, v_kph: float) -> int:
        try:
            vk = float(v_kph) if v_kph is not None and np.isfinite(v_kph) else 0.0
            if vk < 30.0:
                return int(SOFT_LIVEVALID_MIN_POINTS_LOWSPD)
            elif vk < 60.0:
                return int(SOFT_LIVEVALID_MIN_POINTS_MIDSPD)
            else:
                return int(SOFT_LIVEVALID_MIN_POINTS_HISPD)
        except Exception:
            return int(SOFT_LIVEVALID_MIN_POINTS_MIDSPD)

    def get_restore_key(self, CP, version):
        which = CP.lateralTuning.which()
        return (CP.carFingerprint, which, version)

    def _try_restore_warm_state(self, CP):
        try:
            path_json = str(LTP_STATE_PATH)
            path_pkl = str(LTP_STATE_PATH_PKL)
            st = None
            src = None
            if os.path.exists(path_json):
                src = path_json
                with open(path_json, "r", encoding="utf-8") as f:
                    st = json.load(f)
            elif os.path.exists(path_pkl):
                src = path_pkl
                with open(path_pkl, "rb") as f:
                    st = pickle.load(f)
            else:
                return False, {}, self.decay

            st_ver = int(st.get("version", -1))
            # allow legacy warm-state schema versions (backward compatible)
            if st_ver <= 0 or st_ver > int(LTP_STATE_VERSION):
                return False, {}, self.decay

            fp = st.get("carFingerprint", None)
            if fp is not None and str(fp) != str(getattr(CP, "carFingerprint", None)):
                return False, {}, self.decay

            tune = st.get("tuneType", None)
            cur_tune = CP.lateralTuning.which() if hasattr(CP, "lateralTuning") else None
            if tune is not None and str(tune) != str(cur_tune):
                return False, {}, self.decay

            ltp_ver = st.get("ltpVersion", None)
            restore_buckets = True
            if ltp_ver is not None:
                try:
                    gap = abs(int(ltp_ver) - int(self.version))
                    if gap > int(LTP_WARM_COMPAT_MAX_VERSION_GAP):
                        # VERSION이 바뀐 경우에는 이전 anchor/filter/bucket을 모두 버린다.
                        # 기존 filtered 값만 복원해도 새 anchor(예: 1.90/0.255)를 오래 덮어써서
                        # 첫 주행 튜닝이 꼬일 수 있다.
                        cloudlog.warning(
                            f"LiveTorque: warm state skipped due to VERSION mismatch (saved={ltp_ver}, current={self.version})"
                        )
                        return False, {}, self.decay
                except Exception:
                    return False, {}, self.decay

            filt = st.get("filtered", {}) or {}
            decay = float(filt.get("decay", self.decay))
            decay = float(np.clip(decay, MIN_FILTER_DECAY, MAX_FILTER_DECAY))

            init = {
                'latAccelFactor': float(filt.get('latAccelFactor', self.base_params['latAccelFactor'])),
                'latAccelOffset': float(filt.get('latAccelOffset', self.base_params['latAccelOffset'])),
                'frictionCoefficient': float(filt.get('frictionCoefficient', self.base_params['frictionCoefficient'])),
            }

            b = st.get("buckets", {}) or {}

            # Legacy compat: older warm-state may store mixed 'points' instead of bucket moments.
            # If buckets are missing, split points by steer magnitude into corner vs straight buckets.
            if restore_buckets:
                try:
                    has_bucket_keys = isinstance(b, dict) and any(
                        k in b for k in ("corner", "straight", "limited_corner"))
                except Exception:
                    has_bucket_keys = False

                if not has_bucket_keys:
                    try:
                        legacy_points = st.get("points", None) or st.get("pointsMixed", None) or st.get("allPoints",
                                                                                                        None)
                        legacy_corner = st.get("cornerPoints", None) or st.get("pointsCorner", None)
                        legacy_straight = st.get("straightPoints", None) or st.get("pointsStraight", None)

                        def _route_points(pts):
                            c_list, s_list = [], []
                            if not isinstance(pts, list):
                                return c_list, s_list
                            for pt in pts:
                                if not isinstance(pt, (list, tuple)) or len(pt) < 2:
                                    continue
                                try:
                                    x = float(pt[0]);
                                    y = float(pt[1])
                                except Exception:
                                    continue
                                if not (np.isfinite(x) and np.isfinite(y)):
                                    continue
                                if abs(x) <= float(STRAIGHT_STEER_MAX):
                                    s_list.append((x, y))
                                else:
                                    c_list.append((x, y))
                            return c_list, s_list

                        if isinstance(legacy_points, list) and len(legacy_points) > 0:
                            c_pts, s_pts = _route_points(legacy_points)
                            if len(c_pts):
                                self.corner_points.load_points(c_pts)
                            if len(s_pts):
                                self.straight_points.load_points(s_pts)

                        if isinstance(legacy_corner, list) and len(legacy_corner) > 0:
                            self.corner_points.load_points(
                                [(p[0], p[1]) for p in legacy_corner if isinstance(p, (list, tuple)) and len(p) >= 2])
                        if isinstance(legacy_straight, list) and len(legacy_straight) > 0:
                            self.straight_points.load_points(
                                [(p[0], p[1]) for p in legacy_straight if isinstance(p, (list, tuple)) and len(p) >= 2])
                    except Exception:
                        pass
            if restore_buckets:
                try:
                    self.corner_points.from_state_dict(b.get("corner", None))
                except Exception:
                    pass
                try:
                    self.limited_corner_points.from_state_dict(b.get("limited_corner", None))
                except Exception:
                    pass
                try:
                    self.straight_points.from_state_dict(b.get("straight", None))
                except Exception:
                    pass
            else:
                # 버전 차이로 버킷 복원을 스킵했을 때도, 필터 값만으로 빠른 liveValid를 지원
                pass

            win = st.get("straightWindow", {}) or {}
            samples = win.get("samples", None)
            if isinstance(samples, list) and len(samples) > 0:
                self._pending_straight_restore = samples

            cloudlog.warning(
                f"LiveTorque: warm state restored ({src})" + (" [filtered-only]" if not restore_buckets else ""))
            self._warm_restored = True
            self._warm_restore_source = src
            return True, init, decay
        except Exception as e:
            cloudlog.exception(f"LiveTorque: warm restore failed: {e}")
            return False, {}, self.decay

    def _apply_pending_straight_restore(self, t_now: float):
        if not self._pending_straight_restore:
            return
        try:
            for it in self._pending_straight_restore:
                if not (isinstance(it, (list, tuple)) and len(it) >= 6):
                    continue
                age_s = float(it[0])
                if age_s < 0.0 or age_s > float(self._straight_bias_win_s) * 1.5:
                    continue
                tt = float(t_now) - float(age_s)
                self._straight_bias.append((tt, float(it[1]), float(it[2]), float(it[3]), float(it[4]), bool(it[5])))
            self._pending_straight_restore = None
            self._update_straight_window_stats()
        except Exception as e:
            cloudlog.exception(f"LiveTorque: apply pending straight window failed: {e}")
            self._pending_straight_restore = None

    def save_warm_state(self, reason: str = "periodic"):
        try:
            os.makedirs(LTP_LOG_DIR, exist_ok=True)

            t_ref = float(self.last_time) if (self.last_time is not None and np.isfinite(self.last_time)) else float(
                time.monotonic())

            samples = []
            for (tt, steer_app, yaw_rate, latacc, vego, ok) in list(self._straight_bias):
                try:
                    age = float(max(0.0, min(float(self._straight_bias_win_s), t_ref - float(tt))))
                    samples.append([age, float(steer_app), float(yaw_rate), float(latacc), float(vego), bool(ok)])
                except Exception:
                    continue

            try:
                latF = float(self.filtered_params['latAccelFactor'].x)
                latO = float(self.filtered_params['latAccelOffset'].x)
                fric = float(self.filtered_params['frictionCoefficient'].x)
            except Exception:
                latF = float(self.base_params['latAccelFactor'])
                latO = float(self.base_params['latAccelOffset'])
                fric = float(self.base_params['frictionCoefficient'])

            st = {
                "version": int(LTP_STATE_VERSION),
                "restoreKey": [str(getattr(self, "_car_fingerprint", None)),
                               str(getattr(self, "_car_tune_type", None))],
                "ltpVersion": int(self.version),
                "carFingerprint": str(self._car_fingerprint),
                "tuneType": str(self._car_tune_type),
                "savedAtMono": float(t_ref),
                "reason": str(reason),

                "filtered": {
                    "latAccelFactor": float(latF),
                    "latAccelOffset": float(latO),
                    "frictionCoefficient": float(fric),
                    "decay": float(self.decay),
                },

                "buckets": {
                    "corner": self.corner_points.to_state_dict(),
                    "limited_corner": self.limited_corner_points.to_state_dict(),
                    "straight": self.straight_points.to_state_dict(),
                },

                "straightWindow": {
                    "win_s": float(self._straight_bias_win_s),
                    "samples": samples[-800:],
                },
            }

            d = os.path.dirname(LTP_STATE_PATH)
            os.makedirs(d, exist_ok=True)
            with tempfile.NamedTemporaryFile("w", delete=False, dir=d, prefix="ltp_state_", suffix=".tmp",
                                             encoding="utf-8") as tf:
                json.dump(st, tf, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
                tf.flush()
                os.fsync(tf.fileno())
                tmp_name = tf.name
            os.replace(tmp_name, LTP_STATE_PATH)

            d2 = os.path.dirname(LTP_STATE_PATH_PKL)
            os.makedirs(d2, exist_ok=True)
            with tempfile.NamedTemporaryFile("wb", delete=False, dir=d2, prefix="ltp_state_", suffix=".tmp") as tf2:
                pickle.dump(st, tf2, protocol=pickle.HIGHEST_PROTOCOL)
                tf2.flush()
                os.fsync(tf2.fileno())
                tmp2 = tf2.name
            os.replace(tmp2, LTP_STATE_PATH_PKL)

            try:
                self._last_state_save_pts = int(
                    len(self.corner_points) + len(self.straight_points) + len(self.limited_corner_points))
            except Exception:
                pass
            cloudlog.info(f"LiveTorque: warm state saved ({reason})")
        except Exception as e:
            cloudlog.exception(f"LiveTorque: warm state save failed: {e}")

    def reset(self):
        self.resets += 1
        self.invalid_values_tracker = 0.0
        self.decay = MIN_FILTER_DECAY
        self.raw_points = defaultdict(lambda: deque(maxlen=self.hist_len))
        try:
            if hasattr(self, '_qual_win') and self._qual_win is not None:
                self._qual_win.clear()
            self._qual_freeze_until = 0.0
            self._qual_freeze_extend_cnt = 0
            self._qual_freeze_ext_evt = None
            self._last_clip_quality = False
            self._last_clip_raw = False
            self._steer_pressed_cnt = 0
            self._last_steer_override_db = False

            self._rate_transient_until = 0.0
            self._last_desired_delta = 0.0
            self._last_applied_delta = 0.0
        except Exception:
            pass
        self.corner_points = PointBuckets(
            x_bounds=STEER_BUCKET_BOUNDS,
            min_points=self.min_bucket_points,
            min_points_total=self.min_points_total,
        )
        self.limited_corner_points = PointBuckets(
            x_bounds=STEER_BUCKET_BOUNDS,
            min_points=self.min_bucket_points,
            min_points_total=self.min_points_total,
        )

        self.high_steer_kept_pos = 0
        self.high_steer_kept_neg = 0
        self.straight_points = PointBuckets(
            x_bounds=STRAIGHT_BUCKET_BOUNDS,
            min_points=self.min_bucket_points_straight,
            min_points_total=self.min_points_total_straight,
        )

    def _estimate_params_from_moments(self, n, sx, sy, sxx, syy, sxy):
        if n is None or float(n) < 8.0:
            return np.nan, np.nan, np.nan

        try:
            n = float(n);
            sx = float(sx);
            sy = float(sy);
            sxx = float(sxx);
            syy = float(syy);
            sxy = float(sxy)

            S = np.array([
                [sxx, sx, sxy],
                [sx, n, sy],
                [sxy, sy, syy],
            ], dtype=np.float64)

            if not np.all(np.isfinite(S)):
                return np.nan, np.nan, np.nan

            w, V = np.linalg.eigh(S)
            v = V[:, 0]
            if v.shape[0] != 3 or (not np.isfinite(v).all()) or abs(float(v[2])) < 1e-9:
                return np.nan, np.nan, np.nan

            slope = float(-v[0] / v[2])
            offset = float(-v[1] / v[2])

            mean_r = (sy - slope * sx - offset * n) / max(n, 1e-9)

            Er2 = (
                          syy +
                          (slope * slope) * sxx +
                          (offset * offset) * n +
                          2.0 * slope * offset * sx -
                          2.0 * slope * sxy -
                          2.0 * offset * sy
                  ) / max(n, 1e-9)

            var_r = float(max(0.0, Er2 - (mean_r * mean_r)))
            std_perp = float(math.sqrt(var_r) / math.sqrt(1.0 + slope * slope))
            friction_coeff = float(std_perp * FRICTION_FACTOR)

            if not np.isfinite(slope) or not np.isfinite(offset) or not np.isfinite(
                    friction_coeff) or friction_coeff < 0.0:
                return np.nan, np.nan, np.nan

            return slope, offset, friction_coeff
        except Exception as e:
            cloudlog.exception(f"Error computing live torque params from moments: {e}")
            return np.nan, np.nan, np.nan

    def estimate_params_corner(self):
        # normal corner + limited corner(가중치 적용)
        n1, sx1, sy1, sxx1, syy1, sxy1 = self.corner_points.aggregate_moments()
        n2, sx2, sy2, sxx2, syy2, sxy2 = self.limited_corner_points.aggregate_moments()

        # ✅ (4) limited 반영 조건부 강화:
        # clip/max/rate_limited 상황이면 limited_corner 반영 비중을 올려 "한계 근처" 기울기 추정이 너무 약해지지 않게 함
        w = float(LIMITED_CORNER_WEIGHT)
        try:
            limited_now = (bool(self.last_steer_clip) or bool(self.last_max_limited) or
                           bool(self.last_rate_limited_strong) or bool(self.last_rate_limited))
            abs_s = abs(float(self.last_steer_desired)) if (
                        self.last_steer_desired is not None and np.isfinite(self.last_steer_desired)) else 0.0

            if limited_now:
                # 한계 근처 데이터는 기울기 추정에 더 큰 비중 부여(특히 고조향)
                w = max(w, 0.70)
                if abs_s >= 0.70:
                    w = max(w, 0.80)
                if bool(self.last_steer_clip) or bool(self.last_max_limited):
                    w = max(w, 0.85)
        except Exception:
            pass

        n = n1 + (w * n2)
        sx = sx1 + (w * sx2)
        sy = sy1 + (w * sy2)
        sxx = sxx1 + (w * sxx2)
        syy = syy1 + (w * syy2)
        sxy = sxy1 + (w * sxy2)

        return self._estimate_params_from_moments(n, sx, sy, sxx, syy, sxy)

    def _estimate_straight_offset_from_window(self):
        win = list(self._straight_bias) if hasattr(self, "_straight_bias") else []
        if len(win) == 0:
            return float('nan'), 0, 0, 0.0

        vals = [float(latacc) for (_, _, _, latacc, _, ok) in win if ok and np.isfinite(latacc)]
        # NOTE: total/min_ok/ratio는 "ok 샘플(엄격 필터 통과)" 기준으로 계산해야 실제 업데이트가 가능합니다.
        total = len(vals)  # ok 샘플 수
        ok_n = total
        ratio = 1.0 if total > 0 else 0.0
        min_ok = self._straight_min_ok_required(total)
        if ok_n < int(min_ok):
            return float('nan'), ok_n, total, ratio

        med = float(np.median(vals))
        abs_dev = [abs(v - med) for v in vals]
        mad = float(np.median(abs_dev)) if len(abs_dev) else 0.0

        if mad > STRAIGHT_MAD_EPS:
            thr = float(STRAIGHT_MAD_K * mad)
            vals_f = [v for v in vals if abs(v - med) <= thr]
            if len(vals_f) >= int(max(1, int(min_ok) * 0.7)):
                vals = vals_f

        try:
            off = float(np.median(vals))
        except Exception:
            off = float('nan')
        return off, ok_n, total, ratio

    def _update_decay(self):
        if self.decay < 100:
            self.decay = min(self.decay + 3 * DT_MDL, 100)
        else:
            self.decay = min(self.decay + DT_MDL, MAX_FILTER_DECAY)

    def update_params(self, params):
        self._update_decay()
        base_decay = self.decay

        yaw = abs(self.last_yaw_rate) if self.last_yaw_rate is not None else 0.0
        v = self.last_vego if self.last_vego is not None else 0.0
        curve_active = (v > MIN_VEL_CURVE_MS) and (yaw > CURVE_YAWRATE_MIN)

        for param, value in params.items():
            if not _finite(value):
                continue

            cur = self.filtered_params[param].x
            decay_use = base_decay

            if param == 'latAccelOffset':
                decay_use = min(MAX_FILTER_DECAY, max(base_decay, base_decay * 1.5))
                try:
                    value = float(np.clip(float(value), float(cur) - float(OFFSET_UPDATE_MAX_STEP),
                                          float(cur) + float(OFFSET_UPDATE_MAX_STEP)))
                except Exception:
                    pass

            # ✅ (2) latAccelFactor 감소를 더 빠르게 / 증가는 느리게 (언더스티어 완화)
            if param == 'latAccelFactor' and curve_active:
                if value < cur:
                    # 감소는 빠르게(더 잘 도는 쪽)
                    decay_use = max(MIN_FILTER_DECAY * 0.40, base_decay * 0.40)
                elif value > cur:
                    # 증가는 느리게(과대추정 방지)
                    decay_use = min(MAX_FILTER_DECAY, base_decay * 1.5)

            self.filtered_params[param].update_alpha(decay_use)
            self.filtered_params[param].update(value)

    def _dynamic_bands(self):
        if FORCE_TARGET_TUNING:
            # ✅ (1) FORCE 밴드 비대칭 + (개선) 포인트가 쌓일수록 점진 완화(=raw 반영)
            anchorF = float(self.offline_latAccelFactor)
            anchorR = float(self.offline_friction)

            down = float(TARGET_FACTOR_BAND_DOWN)
            up = float(TARGET_FACTOR_BAND_UP)
            fr_b = float(TARGET_FRICTION_BAND)

            if bool(FORCE_BAND_RELAX_ENABLED):
                try:
                    pts = int(len(self.corner_points) + len(self.limited_corner_points))
                    start_pts = int(max(1, round(float(self.min_points_total) * float(FORCE_RELAX_START_MULT))))
                    full_pts = int(
                        max(start_pts + 1, round(float(self.min_points_total) * float(FORCE_RELAX_FULL_MULT))))
                    r = float(np.clip((pts - start_pts) / float(full_pts - start_pts), 0.0, 1.0))
                    down = float(down + r * (float(FORCE_FACTOR_BAND_DOWN_MAX) - down))
                    up = float(up + r * (float(FORCE_FACTOR_BAND_UP_MAX) - up))
                    fr_b = float(fr_b + r * (float(FORCE_FRICTION_BAND_MAX) - fr_b))
                except Exception:
                    pass

            min_factor = (1.0 - down) * anchorF
            max_factor = (1.0 + up) * anchorF
            min_fric = (1.0 - fr_b) * anchorR
            max_fric = (1.0 + fr_b) * anchorR

            # 절대 안전 클램프(상식 범위 밖으로 튀는 것 방지)
            min_factor = float(max(min_factor, float(LAT_ACCEL_FACTOR_ABS_MIN)))
            max_factor = float(min(max_factor, float(LAT_ACCEL_FACTOR_ABS_MAX)))
            min_fric = float(max(min_fric, float(FRICTION_ABS_MIN)))
            max_fric = float(min(max_fric, float(FRICTION_ABS_MAX)))
            return min_factor, max_factor, min_fric, max_fric

        if len(self.corner_points) > 1.5 * self.min_points_total:
            factor_band = FACTOR_SANITY * 0.5
            friction_band = FRICTION_SANITY * 0.5
        else:
            factor_band = FACTOR_SANITY
            friction_band = FRICTION_SANITY

        min_factor = (1.0 - factor_band) * self.offline_latAccelFactor
        max_factor = (1.0 + factor_band) * self.offline_latAccelFactor
        min_fric = (1.0 - friction_band) * self.offline_friction
        max_fric = (1.0 + friction_band) * self.offline_friction
        return min_factor, max_factor, min_fric, max_fric

    def _coerce_params(self, latFactor, latOffset, friction):
        curF = _sanitize_num(self.filtered_params['latAccelFactor'].x, self.offline_latAccelFactor)
        curO = _sanitize_num(self.filtered_params['latAccelOffset'].x, 0.0)
        curR = _sanitize_num(self.filtered_params['frictionCoefficient'].x, self.offline_friction)

        min_factor, max_factor, min_fric, max_fric = self._dynamic_bands()

        latF = np.clip(_sanitize_num(latFactor, curF), min_factor, max_factor)
        latO = np.clip(_sanitize_num(latOffset, curO), -self.max_offset_abs, self.max_offset_abs)
        fric = np.clip(_sanitize_num(friction, curR), min_fric, max_fric)
        return float(latF), float(latO), float(fric)

    def _apply_lat_factor_assist(self, latF, minF=None, maxF=None):
        base = float(_sanitize_num(latF, self.offline_latAccelFactor))
        self._latAF_assist_active = False
        self._latAF_assist_base = base
        self._latAF_assist_scale = 1.0
        self._latAF_assist_delta = 0.0
        self._latAF_assist_clip_ratio = 0.0

        try:
            if not bool(LAT_ACCEL_FACTOR_AGGRESSIVE_ASSIST):
                return base

            if minF is None or maxF is None:
                minF, maxF, _, _ = self._dynamic_bands()

            clip_ratio = float(getattr(
                self,
                "_qual_clip_quality_ratio",
                getattr(self, "_qual_clip_ratio", 0.0)
            ) or 0.0)
            clip_ratio = float(np.clip(clip_ratio, 0.0, 1.0))
            self._latAF_assist_clip_ratio = clip_ratio

            v_ego = float(self.last_vego) if (self.last_vego is not None and np.isfinite(self.last_vego)) else 0.0
            if (not bool(getattr(self, "last_lat_active", False))) or (v_ego < float(MIN_VEL_CURVE_MS)):
                return base

            eps_or_max_guard = bool(
                getattr(self, "last_max_limited", False) or
                getattr(self, "_lc_eps_evt", False) or
                getattr(self, "_lc_eps_damp", False)
            )
            rate_strong_ratio = float(getattr(self, "_qual_rate_strong_ratio", 0.0) or 0.0)
            rate_strong_guard = bool(
                getattr(self, "last_rate_limited_strong", False) or
                (rate_strong_ratio >= float(LAT_ACCEL_FACTOR_ASSIST_RATE_STRONG_LIMIT))
            )
            if eps_or_max_guard or rate_strong_guard:
                return base

            start = float(LAT_ACCEL_FACTOR_ASSIST_CLIP_RATIO_START)
            full = float(LAT_ACCEL_FACTOR_ASSIST_CLIP_RATIO_FULL)
            if full <= start or clip_ratio <= start:
                return base

            weight = float(np.clip((clip_ratio - start) / (full - start), 0.0, 1.0))
            scale = 1.0 - ((1.0 - float(LAT_ACCEL_FACTOR_ASSIST_MIN_SCALE)) * weight)
            assisted = float(base * scale)

            max_delta = max(0.0, float(LAT_ACCEL_FACTOR_ASSIST_MAX_DELTA))
            if max_delta > 0.0:
                assisted = max(base - max_delta, assisted)

            assisted = float(np.clip(assisted, minF, maxF))
            if assisted < (base - 1e-6):
                self._latAF_assist_active = True
                self._latAF_assist_scale = float(assisted / base) if abs(base) > 1e-9 else 1.0
                self._latAF_assist_delta = float(base - assisted)
                return assisted
        except Exception:
            pass

        return base

    def _freeze_check(self, t, vego):
        if vego is not None and vego < PARK_VEL_MS:
            self.stop_freeze_until = max(self.stop_freeze_until, t + FREEZE_AFTER_STOP_S)
        self.last_is_frozen = (t < self.stop_freeze_until)
        return self.last_is_frozen

    def is_sane(self, latF, latO, fric):
        minF, maxF, minR, maxR = self._dynamic_bands()
        return (
                _finite(latF) and _finite(latO) and _finite(fric) and
                (minF <= latF <= maxF) and (-self.max_offset_abs <= latO <= self.max_offset_abs) and
                (minR <= fric <= maxR)
        )

    def handle_log(self, t, which, msg):
        # Runtime toggle: when live tuning disabled, skip learning/update work
        if not LIVE_TORQUE_TUNING_ENABLED:
            return

        if self.start_time is None:
            self.start_time = t

        if which == "carControl":
            self.raw_points["carControl_t"].append(t + self.lag)

            applied = None
            try:
                applied = float(msg.actuatorsOutput.steer)
            except Exception:
                applied = None

            desired = None
            try:
                desired = float(msg.actuators.steer)
            except Exception:
                desired = None

            self._last_lat_active_cc = bool(msg.latActive)
            self._last_lat_active_cc_t = float(t)

            try:
                en = getattr(msg, 'enabled', None)
                if en is not None:
                    self._last_enabled_cc = bool(en)
                    self._last_enabled_cc_t = float(t)
            except Exception:
                pass

            self.last_lat_active = bool(msg.latActive)

            applied_used = None
            if applied is not None and np.isfinite(applied):
                applied_used = float(applied)
                self._last_valid_applied = applied_used
            else:
                applied_used = self._last_valid_applied

            if applied_used is not None and np.isfinite(applied_used):
                self.raw_points["steer_torque"].append(-float(applied_used))
            else:
                prev = self.raw_points["steer_torque"][-1] if len(self.raw_points["steer_torque"]) > 0 else 0.0
                self.raw_points["steer_torque"].append(float(prev))

            self.raw_points["active"].append(msg.latActive)

            self.last_steer_desired = desired
            self.last_steer_applied = applied

            # (optional) steer output CAN / steerMax / torque-limit debug fields (fork별 필드명이 다를 수 있음)
            try:
                ao = getattr(msg, 'actuatorsOutput', None)
                if ao is not None:
                    for nm in ['steerOutputCan', 'steerCan', 'steerCanOutput', 'steerOutput']:
                        v_can = getattr(ao, nm, None)
                        if v_can is not None:
                            self._last_steer_out_can = float(v_can)
                            break
                    for nm in ['steerMax', 'steerMaxOutput', 'steerMaxCan']:
                        v_m = getattr(ao, nm, None)
                        if v_m is not None:
                            self._last_steer_max = float(v_m)
                            break
            except Exception:
                pass

            try:
                tl = getattr(msg, 'torqueLimits', None)
                if tl is not None:
                    for nm in ['steerMax', 'allowedSteer', 'allowedTorque', 'maxSteer']:
                        v_a = getattr(tl, nm, None)
                        if v_a is not None:
                            self._last_allowed_torque = float(v_a)
                            break
            except Exception:
                pass

            # Fallback(derive): if fork doesn't publish torqueLimits/steerCan fields
            try:
                self._last_steer_max = float(STEER_MAX_DIAG)
                if applied_used is not None and np.isfinite(applied_used):
                    torq = float(applied_used) * float(STEER_MAX_DIAG)
                    self._last_steer_out_can = float(torq)
                    self._last_allowed_torque = float(torq)
            except Exception:
                pass

            # desired/applied delta (rate-limit 과도/준정상 분기용)
            try:
                if (desired is not None) and np.isfinite(desired) and (
                        self._prev_steer_desired is not None) and np.isfinite(self._prev_steer_desired):
                    self._last_desired_delta = float(desired) - float(self._prev_steer_desired)
                else:
                    self._last_desired_delta = 0.0
            except Exception:
                self._last_desired_delta = 0.0

            try:
                if (applied is not None) and np.isfinite(applied) and (
                        self._prev_steer_applied is not None) and np.isfinite(self._prev_steer_applied):
                    self._last_applied_delta = float(applied) - float(self._prev_steer_applied)
                else:
                    self._last_applied_delta = 0.0
            except Exception:
                self._last_applied_delta = 0.0

            steer_clip = False
            max_limited = False
            rate_limited = False

            if (desired is not None) and (applied is not None) and np.isfinite(desired) and np.isfinite(applied):
                des_abs = abs(desired)
                app_abs = abs(applied)
                clip_gap = des_abs - app_abs
                steer_clip = (des_abs >= STEER_CLIP_MIN_DES) and (
                            clip_gap > max(STEER_CLIP_EPS, STEER_CLIP_REL * des_abs))
                max_limited = steer_clip and (abs(applied) >= STEER_SAT_THRESHOLD)

                if (self._prev_steer_applied is not None) and (self._prev_steer_applied_t is not None):
                    d = float(applied) - float(self._prev_steer_applied)
                    delta_err = abs(float(desired) - float(applied))
                    if abs(d) > 1e-6 and (delta_err > 0.005):
                        v_kph = float(self.last_vego) * 3.6 if (
                                    self.last_vego is not None and np.isfinite(self.last_vego)) else 0.0
                        w_mid = self._midspd_weight(v_kph)
                        lim_up = STEER_DELTA_UP_NORM * (1.0 + MIDSPD_DELTA_UP_GAIN * w_mid)
                        lim_dn = STEER_DELTA_DOWN_NORM * (1.0 + MIDSPD_DELTA_DOWN_GAIN * w_mid)
                        lim = lim_up if abs(desired) > abs(applied) else lim_dn
                        self.last_rate_lim_w = float(w_mid)
                        self.last_delta_lim_up = float(lim_up)
                        self.last_delta_lim_dn = float(lim_dn)
                        if (abs(d) >= 0.85 * lim) and (abs(d) <= 1.25 * lim):
                            rate_limited = True
                            # strong rate-limit: 체감 '툭툭'에 더 가까운 구간만 별도 플래그
                            self.last_delta_err = float(delta_err)
                            strong = (float(delta_err) >= RATE_LIMITED_STRONG_ERR) and (max(des_abs, app_abs) >= 0.18)
                            self.last_rate_limited_strong = bool(strong)

            self.last_steer_clip = bool(steer_clip)
            self.last_max_limited = bool(max_limited)
            if (desired is not None) and (applied is not None) and np.isfinite(desired) and np.isfinite(applied):
                try:
                    self.last_delta_err = float(abs(float(desired) - float(applied)))
                except Exception:
                    self.last_delta_err = 0.0
            else:
                self.last_delta_err = 0.0
            # rate_limited 과도 구간 홀드(원하는 토크 변화 직후는 지연/과도 영향이 커서 학습 제외)
            try:
                if bool(rate_limited) or bool(self.last_rate_limited_strong):
                    if (abs(float(getattr(self, '_last_desired_delta', 0.0))) >= float(
                            RATE_LIM_TRANSIENT_DES_DELTA)) or bool(self.last_rate_limited_strong) or (
                            float(getattr(self, 'last_delta_err', 0.0)) >= float(RATE_LIMITED_STRONG_ERR)):
                        self._rate_transient_until = max(float(getattr(self, '_rate_transient_until', 0.0) or 0.0),
                                                         float(t) + float(RATE_LIM_TRANSIENT_HOLD_S))
            except Exception:
                pass

            self.last_rate_limited = bool(rate_limited)
            if not bool(rate_limited):
                self.last_rate_limited_strong = False

            if applied is not None and np.isfinite(applied):
                self._prev_steer_applied = float(applied)
                self._prev_steer_applied_t = float(t)

            if desired is not None and np.isfinite(desired):
                self._prev_steer_desired = float(desired)
                self._prev_steer_desired_t = float(t)

        elif which == "carState":
            self.raw_points["carState_t"].append(t + self.lag)
            self.raw_points["vego"].append(msg.vEgo)
            self.raw_points["steer_override"].append(msg.steeringPressed)
            # driver / EPS torque (가능하면)
            try:
                td = getattr(msg, 'steeringTorqueDriver', None)
                if td is None:
                    td = getattr(msg, 'steeringTorque', None)
                if td is not None and np.isfinite(td):
                    self._last_driver_torque = float(td)
                self.raw_points.setdefault('driver_torque', deque(maxlen=self.hist_len)).append(
                    float(self._last_driver_torque) if hasattr(self, '_last_driver_torque') else 0.0)
            except Exception:
                self.raw_points.setdefault('driver_torque', deque(maxlen=self.hist_len)).append(
                    float(getattr(self, '_last_driver_torque', 0.0) or 0.0))

            try:
                te = getattr(msg, 'steeringTorqueEps', None)
                if te is None:
                    te = getattr(msg, 'steeringTorqueEPS', None)
                if te is not None and np.isfinite(te):
                    self._last_eps_torque = float(te)
                self.raw_points.setdefault('eps_torque', deque(maxlen=self.hist_len)).append(
                    float(getattr(self, '_last_eps_torque', 0.0) or 0.0))
            except Exception:
                self.raw_points.setdefault('eps_torque', deque(maxlen=self.hist_len)).append(
                    float(getattr(self, '_last_eps_torque', 0.0) or 0.0))

            # raw steeringPressed + debounce(스파이크 완화)

            sp_raw = bool(getattr(msg, "steeringPressed", False))

            self._last_steer_override = bool(sp_raw)

            # update debounce counter

            if sp_raw:

                self._steer_pressed_cnt = min(int(getattr(self, "_steer_pressed_cnt", 0) or 0) + 1, 1000)

            else:

                self._steer_pressed_cnt = max(int(getattr(self, "_steer_pressed_cnt", 0) or 0) - 1, -1000)

            sp_db = bool(getattr(self, "_last_steer_override_db", False))

            if (not sp_db) and (self._steer_pressed_cnt >= int(STEER_PRESSED_ON_FRAMES)):

                sp_db = True

            elif sp_db and (self._steer_pressed_cnt <= -int(STEER_PRESSED_OFF_FRAMES)):

                sp_db = False

            self._last_steer_override_db = bool(sp_db)

            try:
                lkas_en = getattr(msg, 'lkasEnable', None)
                if lkas_en is None:
                    lkas_en = getattr(msg, 'lkasEnabled', None)
                if lkas_en is not None:
                    self._last_lkas_enable = bool(lkas_en)
                    self._last_lkas_enable_t = float(t)
                    self.raw_points['lkas_enable'].append(bool(lkas_en))
                else:
                    self.raw_points['lkas_enable'].append(False)
            except Exception:
                self.raw_points['lkas_enable'].append(False)

        elif which == "controlsState":
            # Read latcontrol-decided epsEvt/epsDamp from ControlsState.lateralTorqueState (capnp fields @14/@15)
            try:
                lts = getattr(msg, "lateralTorqueState", None)
                if lts is None:
                    return
                # dt from logMonoTime spacing
                if self._lc_prev_t is not None and np.isfinite(self._lc_prev_t):
                    self._lc_dt = float(t - float(self._lc_prev_t))
                else:
                    self._lc_dt = None
                self._lc_prev_t = float(t)

                self._lc_eps_evt = bool(getattr(lts, "epsEvt", False))
                self._lc_eps_damp = bool(getattr(lts, "epsDamp", False))
            except Exception:
                return

        elif which == "liveLocationKalman":
            self._apply_pending_straight_restore(t)
            if len(self.raw_points['steer_torque']) == self.hist_len:
                yaw_rate = msg.angularVelocityCalibrated.value[2]
                roll = msg.orientationNED.value[0]
                self.last_yaw_rate = float(yaw_rate)
                self.last_time = float(t)

                lat_active = self._get_lat_active(float(t))
                lat_src = self._lat_active_src

                enabled_cs = None
                try:
                    if self._last_enabled_cc_t is not None and (
                            float(t) - float(self._last_enabled_cc_t)) <= LAT_ACTIVE_STALE_S:
                        enabled_cs = bool(self._last_enabled_cc)
                except Exception:
                    enabled_cs = None

                if (not lat_active) and (enabled_cs is True) and (self._last_lkas_enable_t is None or (
                        float(t) - float(self._last_lkas_enable_t)) > LAT_ACTIVE_STALE_S):
                    try:
                        des = self.last_steer_desired
                        app = self.last_steer_applied
                        if (des is not None) and (app is not None) and np.isfinite(des) and np.isfinite(app):
                            if abs(float(des)) > 0.01 and abs(float(app)) > 0.01:
                                lat_active = True
                                lat_src = "inferred(enabled)"
                    except Exception:
                        pass

                self.last_lat_active = bool(lat_active)
                self._lat_active_src = lat_src

                if len(self.raw_points['carState_t']) >= 2 and len(self.raw_points['steer_override']) >= 2:
                    steer_override = (np.interp(
                        np.arange(t - MIN_ENGAGE_BUFFER, t, DT_MDL),
                        self.raw_points['carState_t'],
                        self.raw_points['steer_override']
                    ) > 0.5)
                else:
                    steer_override = np.array([False], dtype=bool)

                if len(self.raw_points['carState_t']) >= 2 and len(self.raw_points['vego']) >= 2:
                    vego = float(np.interp(t, self.raw_points['carState_t'], self.raw_points['vego']))
                    self.last_vego = vego
                    self._last_v_kph = float(vego) * 3.6
                else:
                    return

                if len(self.raw_points['carControl_t']) >= 2 and len(self.raw_points['steer_torque']) >= 2:
                    steer = float(np.interp(t, self.raw_points['carControl_t'], self.raw_points['steer_torque']))
                else:
                    return

                is_frozen = self._freeze_check(t, vego)

                lateral_acc = (vego * yaw_rate) - (np.sin(roll) * ACCELERATION_DUE_TO_GRAVITY)

                steer_app = self.last_steer_applied
                straight_ok = (
                        (vego is not None and vego > MIN_VEL_MS_BIAS) and
                        (yaw_rate is not None and abs(yaw_rate) <= STRAIGHT_YAW_RATE_MAX) and
                        (abs(lateral_acc) <= STRAIGHT_LATACC_MAX) and
                        (steer is not None and np.isfinite(steer) and abs(steer) <= STRAIGHT_STEER_MAX_FOR_OFFSET) and
                        (steer_app is not None and np.isfinite(steer_app) and abs(
                            steer_app) <= STRAIGHT_STEER_MAX_FOR_OFFSET) and
                        (not any(steer_override)) and
                        (lat_active) and
                        (not is_frozen) and
                        (not self.last_steer_clip) and
                        (not self.last_max_limited) and
                        (
                                (not self.last_rate_limited_strong) or
                                (float(getattr(self, 'last_delta_err', 0.0)) <= STRAIGHT_RATE_LIM_ALLOW_DELTA)
                        )
                )
                # ✅ BUGFIX: straight_ok가 True인 순간을 기록(오프셋 업데이트 게이트)
                if straight_ok:
                    self._last_straight_ok_t = float(t)

                self._straight_bias.append((float(t), None if steer_app is None else float(steer_app),
                                            None if yaw_rate is None else float(yaw_rate),
                                            float(lateral_acc), float(vego), bool(straight_ok)))
                while len(self._straight_bias) > 0 and (
                        float(t) - self._straight_bias[0][0]) > self._straight_bias_win_s:
                    self._straight_bias.popleft()

                self._update_straight_window_stats()

                min_steer = 0.005
                max_latacc = LAT_ACC_THRESHOLD + 1.0

                # === 코너 데이터 수집 (약 10.8 km/h부터, MIN_STEER_SPEED=3.0m/s 기준) ===
                if (not any(steer_override)) and (lat_active) and (vego > MIN_VEL_CURVE_MS) and (
                        abs(steer) > min_steer) and (abs(lateral_acc) <= max_latacc) and (not is_frozen):
                    abs_s = abs(steer)

                    if abs_s >= 0.9:
                        keep_prob = 0.65
                    elif abs_s >= 0.7:
                        keep_prob = 0.45
                    elif abs_s >= 0.5:
                        keep_prob = 0.28
                    else:
                        keep_prob = CORNER_KEEP_PROB

                    if abs_s >= 0.7:
                        pos = float(self.high_steer_kept_pos)
                        neg = float(self.high_steer_kept_neg)
                        tot = pos + neg + 1e-6
                        imbalance = (pos - neg) / tot

                        if steer >= 0.0:
                            keep_prob *= (1.0 - 0.25 * max(imbalance, 0.0)) * (1.0 + 0.25 * max(-imbalance, 0.0))
                        else:
                            keep_prob *= (1.0 - 0.25 * max(-imbalance, 0.0)) * (1.0 + 0.25 * max(imbalance, 0.0))

                        keep_prob = float(np.clip(keep_prob, 0.05, 0.95))

                    is_clip = bool(self.last_steer_clip)
                    is_max_limited = bool(self.last_max_limited)
                    is_rate_limited_strong = bool(self.last_rate_limited_strong)
                    is_rate_limited = bool(self.last_rate_limited) or is_rate_limited_strong
                    is_limited = (is_clip or is_max_limited or is_rate_limited)

                    times_lat = 1
                    if abs(lateral_acc) > 1.2:
                        times_lat = 2
                    elif abs(lateral_acc) > 0.8:
                        times_lat = 2

                    times_steer = 1
                    if abs_s >= 0.9:
                        times_steer = 6
                    elif abs_s >= 0.7:
                        times_steer = 3
                    elif abs_s >= 0.5:
                        times_steer = 2

                    times = int(np.clip(times_lat * times_steer, 1, 8))

                    # ✅ 10.8~30km/h 저속 코너 샘플은 학습하되 영향력은 낮춤
                    # - 유턴/주차장/골목 저속 조향은 lateral accel이 작고 EPS 특성이 달라 전체 튜닝을 과하게 끌 수 있음
                    # - 그래서 포인트 채택 확률과 복제 횟수를 속도별로 줄여 안정성을 확보
                    v_kph_sample = float(vego) * 3.6
                    if v_kph_sample < float(LOWSPD_CORNER_KPH):
                        keep_prob *= float(LOWSPD_CORNER_KEEP_SCALE)
                        times = max(1, int(round(float(times) * float(LOWSPD_CORNER_TIMES_SCALE))))
                    elif v_kph_sample < float(MIDLOW_CORNER_KPH):
                        keep_prob *= float(MIDLOW_CORNER_KEEP_SCALE)
                        times = max(1, int(round(float(times) * float(MIDLOW_CORNER_TIMES_SCALE))))

                    keep_prob = float(np.clip(keep_prob, 0.02, 0.95))

                    if not is_limited:
                        if np.random.random() < keep_prob:
                            self.corner_points.add_point(float(steer), float(lateral_acc), times=times)
                            if abs_s >= 0.7:
                                if steer >= 0.0:
                                    self.high_steer_kept_pos += 1
                                else:
                                    self.high_steer_kept_neg += 1
                    else:
                        # ✅ limited corner: clip/max/rate-limit 구간을 더 적극적으로 수집
                        keep_prob_l = float(keep_prob) * 0.55
                        if is_clip or is_max_limited:
                            keep_prob_l = float(keep_prob) * 0.75
                        elif is_rate_limited_strong:
                            keep_prob_l = float(keep_prob) * 0.65
                        elif is_rate_limited:
                            keep_prob_l = float(keep_prob) * 0.60

                        keep_prob_l = float(np.clip(keep_prob_l, 0.03, 0.55))

                        times_l = max(1, int(round(times * 0.75)))
                        if is_clip or is_max_limited:
                            times_l = max(1, int(round(times * 0.90)))

                        if np.random.random() < keep_prob_l:
                            self.limited_corner_points.add_point(float(steer), float(lateral_acc), times=times_l)

                        if RATE_LIMITED_MIX_ENABLED and is_rate_limited and (not is_clip) and (not is_max_limited):
                            if (abs_s >= RATE_LIMITED_MIX_ABS_STEER_MIN) and (abs_s <= RATE_LIMITED_MIX_ABS_STEER_MAX):
                                keep_prob_m = float(np.clip(keep_prob * RATE_LIMITED_MIX_KEEP_PROB_MULT, 0.02, 0.25))
                                times_m = max(1, int(times * RATE_LIMITED_MIX_TIMES_MULT))
                                if np.random.random() < keep_prob_m:
                                    self.corner_points.add_point(float(steer), float(lateral_acc), times=times_m)
                                    if abs_s >= 0.7:
                                        if steer >= 0.0:
                                            self.high_steer_kept_pos += 1
                                        else:
                                            self.high_steer_kept_neg += 1

                # === 직선 전용 버킷 수집 (직선은 40km/h 유지) ===
                self.last_straight_sampled = False
                if (abs(steer) <= STRAIGHT_STEER_MAX) and (vego > MIN_VEL_MS_STRAIGHT) and (
                        abs(lateral_acc) < STRAIGHT_LATACC_MAX) and (not any(steer_override)) and (lat_active) and (
                not is_frozen) and (not self.last_steer_clip) and (not self.last_max_limited) and (
                not self.last_rate_limited_strong) and (len(self.straight_points) < STRAIGHT_POINTS_SOFT_CAP):
                    w = self.straight_weight(vego, yaw_rate, t)
                    self.last_straight_w = float(w)
                    if w > 0.0:
                        prob = STRAIGHT_KEEP_PROB * (w / max(STRAIGHT_WARMUP_MAX_W, 1e-3))
                        if np.random.random() < prob:
                            self.straight_points.add_point(float(steer), float(lateral_acc), times=1)
                            self.last_straight_sampled = True
                            self._last_straight_sampled_t = float(t)

                try:
                    burst_sample = self._make_burst_sample(float(t), lateral_acc=float(lateral_acc), straight_ok=bool(straight_ok))
                    self._append_burst_sample(burst_sample)
                    self._detect_burst_triggers(float(t), burst_sample)
                    self._flush_active_burst(reason="auto", t_now=float(t), force=False)
                except Exception:
                    pass


    def _eps_proxy_state(self, now_mono: float, update_state: bool = False):
        try:
            now_mono = float(now_mono)
        except Exception:
            now_mono = float(time.monotonic())

        try:
            _allowed = float(getattr(self, "_last_allowed_torque", 0.0) or 0.0)
        except Exception:
            _allowed = 0.0
        try:
            _steer_out = float(getattr(self, "_last_steer_out_can", 0.0) or 0.0)
        except Exception:
            _steer_out = 0.0
        try:
            _steer_max = float(getattr(self, "_last_steer_max", 0.0) or 0.0)
        except Exception:
            _steer_max = 0.0

        a_allowed = abs(_allowed)
        a_out = abs(_steer_out)
        a_max = abs(_steer_max)

        eps_evt = False
        if a_max > 1.0:
            if a_allowed >= float(LTP_EPS_ERR_NEAR_MAX) * a_max:
                eps_evt = True
            if a_out >= float(LTP_EPS_CTRL_NEAR_MAX) * a_max:
                eps_evt = True

        if bool(getattr(self, "last_steer_clip", False)) and a_max > 1.0:
            if (a_allowed >= float(LTP_EPS_CLIP_DEMAND) * a_max) or (a_out >= float(LTP_EPS_CLIP_DEMAND) * a_max):
                eps_evt = True

        ltp_dt = None
        if update_state:
            try:
                if self._ltp_prev_t is not None and np.isfinite(self._ltp_prev_t):
                    ltp_dt = float(now_mono - float(self._ltp_prev_t))
            except Exception:
                ltp_dt = None
            self._ltp_prev_t = float(now_mono)
            if eps_evt:
                self._ltp_eps_damp_until = max(float(self._ltp_eps_damp_until),
                                               float(now_mono) + float(LTP_EPS_DAMP_HOLD_S))

        eps_damp = bool(eps_evt or (float(now_mono) < float(getattr(self, "_ltp_eps_damp_until", 0.0) or 0.0)))

        if update_state:
            self._ltp_dt = None if ltp_dt is None else float(ltp_dt)
            self._ltp_eps_evt_proxy = bool(eps_evt)
            self._ltp_eps_damp_proxy = bool(eps_damp)

        return bool(eps_evt), bool(eps_damp), ltp_dt

    def _make_burst_sample(self, t_now: float, lateral_acc=None, straight_ok=None):
        try:
            mono_t = float(t_now)
        except Exception:
            mono_t = float(self.last_time) if (self.last_time is not None and np.isfinite(self.last_time)) else float(time.monotonic())

        try:
            v_kph = float(self.last_vego) * 3.6 if (self.last_vego is not None and np.isfinite(self.last_vego)) else 0.0
        except Exception:
            v_kph = 0.0
        self._last_v_kph = float(v_kph)

        try:
            yaw_rate = float(self.last_yaw_rate) if (self.last_yaw_rate is not None and np.isfinite(self.last_yaw_rate)) else None
        except Exception:
            yaw_rate = None

        try:
            lat_active = bool(self._get_lat_active(mono_t))
        except Exception:
            lat_active = bool(getattr(self, "last_lat_active", False))

        try:
            eps_evt_proxy, eps_damp_proxy, _ = self._eps_proxy_state(mono_t, update_state=False)
        except Exception:
            eps_evt_proxy, eps_damp_proxy = False, bool(mono_t < float(getattr(self, "_ltp_eps_damp_until", 0.0) or 0.0))

        sample_clip_raw = bool(getattr(self, "last_steer_clip", False) or getattr(self, "last_max_limited", False))
        try:
            sample_des_abs = abs(float(self.last_steer_desired)) if (
                self.last_steer_desired is not None and np.isfinite(self.last_steer_desired)) else 0.0
        except Exception:
            sample_des_abs = 0.0
        try:
            sample_delta_err = abs(float(getattr(self, "last_delta_err", 0.0) or 0.0))
        except Exception:
            sample_delta_err = 0.0
        sample_clip_quality = bool(
            sample_clip_raw and (
                bool(getattr(self, "last_max_limited", False)) or
                bool(getattr(self, "last_rate_limited_strong", False)) or
                bool(getattr(self, "_lc_eps_evt", False) or eps_evt_proxy) or
                (float(sample_des_abs) >= float(QUALITY_CLIP_MIN_DES)) or
                (float(sample_delta_err) >= float(QUALITY_CLIP_MIN_DELTA_ERR))
            )
        )

        sample = {
            "mono_t": round(float(mono_t), 4),
            "ts": datetime.now(KST).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            "v_kph": round(float(v_kph), 3),
            "yaw_rate": (None if yaw_rate is None else round(float(yaw_rate), 6)),
            "lateral_acc": (None if lateral_acc is None or not np.isfinite(lateral_acc) else round(float(lateral_acc), 6)),
            "straight_ok": (None if straight_ok is None else bool(straight_ok)),
            "latActive": bool(lat_active),
            "latActive_src": str(getattr(self, "_lat_active_src", "unknown")),
            "enabled_cs": (None if self._last_enabled_cc is None else bool(self._last_enabled_cc)),
            "steeringPressed": bool(getattr(self, "_last_steer_override", False)),
            "steeringPressed_db": bool(getattr(self, "_last_steer_override_db", False)),
            "steer_des": (None if self.last_steer_desired is None else round(float(self.last_steer_desired), 6)),
            "steer_app": (None if self.last_steer_applied is None else round(float(self.last_steer_applied), 6)),
            "steer_clip": bool(getattr(self, "last_steer_clip", False)),
            "clip_quality": bool(sample_clip_quality),
            "max_limited": bool(getattr(self, "last_max_limited", False)),
            "rate_limited": bool(getattr(self, "last_rate_limited", False)),
            "rate_limited_strong": bool(getattr(self, "last_rate_limited_strong", False)),
            "rate_transient": bool((getattr(self, "last_rate_limited", False) or getattr(self, "last_rate_limited_strong", False)) and (
                mono_t < float(getattr(self, "_rate_transient_until", 0.0) or 0.0) or bool(getattr(self, "last_rate_limited_strong", False))
            )),
            "delta_err": round(float(getattr(self, "last_delta_err", 0.0) or 0.0), 6),
            "qual_freeze": bool(mono_t < float(getattr(self, "_qual_freeze_until", 0.0) or 0.0)),
            "qual_n": int(getattr(self, "_qual_n", 0) or 0),
            "qual_clip_ratio": round(float(getattr(self, "_qual_clip_ratio", 0.0) or 0.0), 6),
            "qual_clip_raw_ratio": round(float(getattr(self, "_qual_clip_raw_ratio", getattr(self, "_qual_clip_ratio", 0.0)) or 0.0), 6),
            "qual_clip_quality_ratio": round(float(getattr(self, "_qual_clip_quality_ratio", getattr(self, "_qual_clip_ratio", 0.0)) or 0.0), 6),
            "qual_rate_ratio": round(float(getattr(self, "_qual_rate_ratio", 0.0) or 0.0), 6),
            "qual_rate_strong_ratio": round(float(getattr(self, "_qual_rate_strong_ratio", 0.0) or 0.0), 6),
            "qual_both_ratio": round(float(getattr(self, "_qual_both_ratio", 0.0) or 0.0), 6),
            "qual_steer_pressed_ratio": round(float(getattr(self, "_qual_steer_pressed_ratio", 0.0) or 0.0), 6),
            "qual_freeze_extend_cnt": int(getattr(self, "_qual_freeze_extend_cnt", 0) or 0),
            "qual_freeze_ext_primary": (None if not getattr(self, "_qual_freeze_ext_evt", None) else str(getattr(self, "_qual_freeze_ext_evt", {}).get("primary", None))),
            "eps_evt_proxy": bool(eps_evt_proxy),
            "eps_damp_proxy": bool(eps_damp_proxy),
            "lc_eps_evt": bool(getattr(self, "_lc_eps_evt", False)),
            "lc_eps_damp": bool(getattr(self, "_lc_eps_damp", False)),
            "driver_torque": round(float(getattr(self, "_last_driver_torque", 0.0) or 0.0), 6),
            "eps_torque": round(float(getattr(self, "_last_eps_torque", 0.0) or 0.0), 6),
            "allowed_torque": round(float(getattr(self, "_last_allowed_torque", 0.0) or 0.0), 6),
            "steer_out_can": round(float(getattr(self, "_last_steer_out_can", 0.0) or 0.0), 6),
            "steer_max": round(float(getattr(self, "_last_steer_max", 0.0) or 0.0), 6),
            "latAF_f": round(float(_sanitize_num(self.filtered_params['latAccelFactor'].x, self.offline_latAccelFactor)), 6),
            "latAO_f": round(float(_sanitize_num(self.filtered_params['latAccelOffset'].x, 0.0)), 6),
            "fric_f": round(float(_sanitize_num(self.filtered_params['frictionCoefficient'].x, self.offline_friction)), 6),
            "latAO_evt": getattr(self, "_latAO_evt", None),
            "latAO_blk": getattr(self, "_latAO_blk", None),
            "straight_sampled": bool(getattr(self, "last_straight_sampled", False)),
            "straight_w_last": round(float(getattr(self, "last_straight_w", 0.0) or 0.0), 6),
        }
        return sample

    def _append_burst_sample(self, sample):
        if not ENABLE_BURST_TRACE or sample is None:
            return
        try:
            self._burst_ring.append(sample)
            if self._burst_active is not None:
                samples = self._burst_active.get("samples", [])
                if len(samples) == 0 or float(sample.get("mono_t", -1e9)) > float(samples[-1].get("mono_t", -1e9)):
                    samples.append(sample)
                    if len(samples) > int(BURST_SAMPLE_LIMIT * 2):
                        del samples[:len(samples) - int(BURST_SAMPLE_LIMIT * 2)]
        except Exception:
            pass

    def _append_burst_event(self, active, t_now: float, reason: str, sample=None, extra=None):
        if active is None:
            return
        if len(active["events"]) >= int(BURST_MAX_EVENTS_PER_FILE):
            return
        event = {
            "mono_t": round(float(t_now), 4),
            "reason": str(reason),
            "v_kph": None if sample is None else sample.get("v_kph"),
            "steer_clip": None if sample is None else sample.get("steer_clip"),
            "clip_quality": None if sample is None else sample.get("clip_quality"),
            "rate_limited_strong": None if sample is None else sample.get("rate_limited_strong"),
            "qual_freeze": None if sample is None else sample.get("qual_freeze"),
            "steeringPressed_db": None if sample is None else sample.get("steeringPressed_db"),
            "extra": extra,
        }
        active["events"].append(event)
        active["trigger_reasons"].append(str(reason))
        active["until"] = max(float(active.get("until", t_now)), float(t_now) + float(BURST_POST_S))
        self._last_burst_trigger = str(reason)

    def _start_or_extend_burst(self, t_now: float, reason: str, sample=None, extra=None):
        if not ENABLE_BURST_TRACE:
            return
        try:
            t_now = float(t_now)
        except Exception:
            t_now = float(self.last_time) if (self.last_time is not None and np.isfinite(self.last_time)) else float(time.monotonic())

        if sample is None:
            sample = self._make_burst_sample(t_now)

        active = self._burst_active
        if active is None:
            if (t_now - float(getattr(self, "_burst_last_close_t", -1e9) or -1e9)) < float(BURST_COOLDOWN_S):
                return
            pre_samples = [s for s in list(self._burst_ring) if float(s.get("mono_t", -1e9)) >= (t_now - float(BURST_PRE_S))]
            self._burst_seq += 1
            active = {
                "seq": int(self._burst_seq),
                "trigger": str(reason),
                "start_t": float(t_now),
                "started_wall": datetime.now(KST).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                "until": float(t_now) + float(BURST_POST_S),
                "samples": list(pre_samples),
                "events": [],
                "trigger_reasons": [],
            }
            self._burst_active = active
            self._burst_last_start_t = float(t_now)
        self._append_burst_event(active, t_now, reason, sample=sample, extra=extra)
        # ensure current sample is present
        if sample is not None:
            samples = active.get("samples", [])
            if len(samples) == 0 or float(sample.get("mono_t", -1e9)) > float(samples[-1].get("mono_t", -1e9)):
                samples.append(sample)

    def _detect_burst_triggers(self, t_now: float, sample):
        if not ENABLE_BURST_TRACE or sample is None:
            return

        try:
            eps_evt_proxy, _, _ = self._eps_proxy_state(t_now, update_state=False)
        except Exception:
            eps_evt_proxy = False

        try:
            burst_des_abs = abs(float(self.last_steer_desired)) if (
                self.last_steer_desired is not None and np.isfinite(self.last_steer_desired)) else 0.0
        except Exception:
            burst_des_abs = 0.0
        try:
            burst_delta_err = abs(float(getattr(self, "last_delta_err", 0.0) or 0.0))
        except Exception:
            burst_delta_err = 0.0
        burst_clip_raw = bool(getattr(self, "last_steer_clip", False) or getattr(self, "last_max_limited", False))
        clip_for_burst = bool(
            burst_clip_raw and (
                bool(getattr(self, "last_max_limited", False)) or
                bool(getattr(self, "last_rate_limited_strong", False)) or
                bool(getattr(self, "_lc_eps_evt", False) or eps_evt_proxy) or
                (float(burst_des_abs) >= float(QUALITY_CLIP_MIN_DES)) or
                (float(burst_delta_err) >= float(QUALITY_CLIP_MIN_DELTA_ERR))
            )
        )
        current_flags = {
            "steer_clip": bool(clip_for_burst),
            "rate_limited_strong": bool(getattr(self, "last_rate_limited_strong", False)),
            "qual_freeze": bool(float(t_now) < float(getattr(self, "_qual_freeze_until", 0.0) or 0.0)),
            "eps_evt_any": bool(getattr(self, "_lc_eps_evt", False) or eps_evt_proxy),
            "clip_pressed": bool(clip_for_burst and getattr(self, "_last_steer_override_db", False)),
            "latAO_blk": bool(getattr(self, "_latAO_blk", None) is not None),
            "latAO_evt": bool(getattr(self, "_latAO_evt", None) is not None),
        }
        prev = getattr(self, "_burst_prev_flags", {}) or {}

        if current_flags["steer_clip"] and not prev.get("steer_clip", False):
            self._start_or_extend_burst(t_now, "steer_clip_rise", sample=sample)
        if current_flags["rate_limited_strong"] and not prev.get("rate_limited_strong", False):
            self._start_or_extend_burst(t_now, "rate_limited_strong_rise", sample=sample)
        if current_flags["qual_freeze"] and not prev.get("qual_freeze", False):
            self._start_or_extend_burst(t_now, "qual_freeze_enter", sample=sample)
        if current_flags["eps_evt_any"] and not prev.get("eps_evt_any", False):
            extra = {
                "lc_eps_evt": bool(getattr(self, "_lc_eps_evt", False)),
                "eps_evt_proxy": bool(eps_evt_proxy),
            }
            self._start_or_extend_burst(t_now, "eps_evt_rise", sample=sample, extra=extra)
        if current_flags["clip_pressed"] and not prev.get("clip_pressed", False):
            self._start_or_extend_burst(t_now, "clip_pressed_rise", sample=sample)
        if current_flags["latAO_blk"] and not prev.get("latAO_blk", False):
            self._start_or_extend_burst(t_now, "latAO_block", sample=sample, extra=getattr(self, "_latAO_blk", None))
        if current_flags["latAO_evt"] and not prev.get("latAO_evt", False):
            self._start_or_extend_burst(t_now, "latAO_update", sample=sample, extra=getattr(self, "_latAO_evt", None))

        ext_cnt = int(getattr(self, "_qual_freeze_extend_cnt", 0) or 0)
        prev_ext_cnt = int(getattr(self, "_burst_prev_ext_cnt", 0) or 0)
        if ext_cnt > prev_ext_cnt:
            self._start_or_extend_burst(
                t_now,
                "qual_freeze_extend",
                sample=sample,
                extra=getattr(self, "_qual_freeze_ext_evt", None),
            )
        self._burst_prev_ext_cnt = ext_cnt
        self._burst_prev_flags = current_flags

    def _flush_active_burst(self, reason: str = "auto", t_now=None, force: bool = False):
        if not ENABLE_BURST_TRACE or self._burst_active is None:
            return None

        active = self._burst_active
        if t_now is None:
            t_now = float(self.last_time) if (self.last_time is not None and np.isfinite(self.last_time)) else float(time.monotonic())
        try:
            t_now = float(t_now)
        except Exception:
            t_now = float(time.monotonic())

        if (not force) and t_now < float(active.get("until", t_now)) and len(active.get("samples", [])) < int(BURST_SAMPLE_LIMIT):
            return None

        os.makedirs(self._burst_dir, exist_ok=True)
        trigger_slug = _slugify_label(active.get("trigger", "burst"))
        stamp = datetime.now(KST).strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self._burst_dir, f"ltp_burst_{stamp}_{int(active.get('seq', 0)):04d}_{trigger_slug}.jsonl")

        samples = list(active.get("samples", []))
        if len(samples) > int(BURST_SAMPLE_LIMIT):
            idx = np.linspace(0, len(samples) - 1, int(BURST_SAMPLE_LIMIT), dtype=int)
            samples = [samples[int(i)] for i in idx]

        meta = {
            "kind": "meta",
            "seq": int(active.get("seq", 0)),
            "trigger": str(active.get("trigger", "burst")),
            "trigger_reasons": list(active.get("trigger_reasons", [])),
            "start_t": round(float(active.get("start_t", t_now)), 4),
            "end_t": round(float(t_now), 4),
            "started_wall": str(active.get("started_wall", "")),
            "closed_wall": datetime.now(KST).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            "close_reason": str(reason),
            "sample_count": int(len(samples)),
            "event_count": int(len(active.get("events", []))),
            "pre_s": float(BURST_PRE_S),
            "post_s": float(BURST_POST_S),
        }

        with open(path, "w", encoding="utf-8") as f:
            f.write(json.dumps(meta, ensure_ascii=False) + "\n")
            for ev in active.get("events", []):
                rec = {"kind": "event"}
                rec.update(ev)
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            for smp in samples:
                rec = {"kind": "sample"}
                rec.update(smp)
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")

        self._last_burst_path = path
        self._last_burst_meta = meta
        self._burst_last_close_t = float(t_now)
        self._burst_active = None
        return path

    def _log_to_file(self, msg):
        try:
            os.makedirs(self._log_dir, exist_ok=True)
            now = datetime.now(KST)
            date_str = now.strftime("%Y_%m_%d")
            if self._log_date != date_str or self._log_path is None:
                self._log_date = date_str
                self._log_path = os.path.join(self._log_dir, f"ltp_log_{date_str}.log")

            ltp = msg.liveTorqueParameters

            v_kph = None
            if self.last_vego is not None and np.isfinite(self.last_vego):
                v_kph = round(float(self.last_vego) * 3.6, 2)

            yaw = None
            if self.last_yaw_rate is not None and np.isfinite(self.last_yaw_rate):
                yaw = round(float(self.last_yaw_rate), 5)

            win = list(self._straight_bias) if hasattr(self, "_straight_bias") else []
            straight_ok_samples = [s for s in win if len(s) >= 6 and bool(s[5])]
            steer_ok = [s[1] for s in straight_ok_samples if s[1] is not None]
            latacc_ok = [s[3] for s in straight_ok_samples if s[3] is not None]
            eligible = [s for s in win if
                        len(s) >= 5 and s[4] is not None and np.isfinite(s[4]) and float(s[4]) >= MIN_VEL_MS_BIAS and s[
                            2] is not None]
            yaw_pass = [s for s in eligible if abs(float(s[2])) <= STRAIGHT_YAW_RATE_MAX]
            straight_bias_mean = _mean_safe(steer_ok)
            straight_bias_abs_med = _median_safe([abs(x) for x in steer_ok if x is not None])
            straight_latacc_med = _median_safe(latacc_ok)
            straight_yaw_pass_ratio = None
            if len(eligible) >= 10:
                straight_yaw_pass_ratio = float(len(yaw_pass) / len(eligible))


            # --- LTP debug: EPS damping frequency and dt (schema-independent) ---
            now_mono = float(self.last_time) if (self.last_time is not None and np.isfinite(self.last_time)) else float(time.monotonic())
            eps_evt, eps_damp, ltp_dt = self._eps_proxy_state(now_mono, update_state=True)

            rec = {
                "ts": now.strftime("%Y-%m-%d %H:%M:%S"),
                "ltp_dt": (None if ltp_dt is None else round(float(ltp_dt), 4)),
                "eps_evt": bool(getattr(self, "_lc_eps_evt", False)),
                "eps_evt_proxy": bool(eps_evt),
                "eps_damp": bool(getattr(self, "_lc_eps_damp", False)),
                "eps_damp_proxy": bool(eps_damp),
                "lc_dt": (None if getattr(self, "_lc_dt", None) is None else round(float(getattr(self, "_lc_dt", 0.0) or 0.0), 4)),
                "lc_eps_evt": bool(getattr(self, "_lc_eps_evt", False)),
                "lc_eps_damp": bool(getattr(self, "_lc_eps_damp", False)),
                "liveValid": bool(ltp.liveValid),
                "v_kph": v_kph,
                "yaw_rate": yaw,
                "latActive": bool(self._get_lat_active(float(self.last_time) if self.last_time is not None else 0.0)),
                "latActive_src": str(getattr(self, "_lat_active_src", "unknown")),
                "enabled_cs": (None if self._last_enabled_cc is None else bool(self._last_enabled_cc)),
                "steeringPressed": (
                    None if not hasattr(self, "_last_steer_override") else bool(self._last_steer_override)),
                "steeringPressed_db": bool(getattr(self, "_last_steer_override_db", False)),
                "latAO_evt": getattr(self, "_latAO_evt", None),
                "latAO_blk": getattr(self, "_latAO_blk", None),
                "driver_torque": (None if not hasattr(self, "_last_driver_torque") else round(
                    float(getattr(self, "_last_driver_torque", 0.0) or 0.0), 3)),
                "eps_torque": (None if not hasattr(self, "_last_eps_torque") else round(
                    float(getattr(self, "_last_eps_torque", 0.0) or 0.0), 3)),
                "allowed_torque": (None if not hasattr(self, "_last_allowed_torque") else round(
                    float(getattr(self, "_last_allowed_torque", 0.0) or 0.0), 3)),
                "steer_out_can": (None if not hasattr(self, "_last_steer_out_can") else round(
                    float(getattr(self, "_last_steer_out_can", 0.0) or 0.0), 5)),
                "steer_max": (None if not hasattr(self, "_last_steer_max") else round(
                    float(getattr(self, "_last_steer_max", 0.0) or 0.0), 5)),
                "steer_des": (None if self.last_steer_desired is None else round(float(self.last_steer_desired), 5)),
                "steer_app": (None if self.last_steer_applied is None else round(float(self.last_steer_applied), 5)),
                "steer_clip": bool(self.last_steer_clip),
                "clip_quality": bool(getattr(self, "_last_clip_quality", self.last_steer_clip)),
                "rate_limited": bool(self.last_rate_limited),
                "rate_limited_strong": bool(getattr(self, "last_rate_limited_strong", False)),
                "delta_err": round(float(getattr(self, "last_delta_err", 0.0)), 5),
                "rate_lim_w": round(float(getattr(self, "last_rate_lim_w", 0.0)), 4),
                "delta_lim_up_eff": round(float(getattr(self, "last_delta_lim_up", STEER_DELTA_UP_NORM)), 6),
                "delta_lim_dn_eff": round(float(getattr(self, "last_delta_lim_dn", STEER_DELTA_DOWN_NORM)), 6),
                "max_limited": bool(self.last_max_limited),
                "rate_transient": bool(
                    (self.last_rate_limited or getattr(self, "last_rate_limited_strong", False)) and (
                                (self.last_time or 0.0) < float(
                            getattr(self, "_rate_transient_until", 0.0) or 0.0) or bool(
                            getattr(self, "last_rate_limited_strong", False)))),
                "qual_freeze": bool((self.last_time or 0.0) < float(getattr(self, "_qual_freeze_until", 0.0) or 0.0)),

                "qual_freeze_until": (None if not hasattr(self, "_qual_freeze_until") else round(float(getattr(self, "_qual_freeze_until", 0.0) or 0.0), 3)),
                "qual_freeze_extend_cnt": int(getattr(self, "_qual_freeze_extend_cnt", 0) or 0),
                "qual_freeze_ext_primary": (None if not getattr(self, "_qual_freeze_ext_evt", None) else str(getattr(self, "_qual_freeze_ext_evt", {}).get("primary", None))),
                "qual_freeze_hold_s": (None if not getattr(self, "_qual_freeze_ext_evt", None) else float(getattr(self, "_qual_freeze_ext_evt", {}).get("hold_s", 0.0))),
                "qual_n": int(getattr(self, "_qual_n", 0) or 0),
                "qual_clip_ratio": (None if not hasattr(self, "_qual_clip_ratio") else round(
                    float(getattr(self, "_qual_clip_ratio", 0.0) or 0.0), 4)),
                "qual_clip_raw_ratio": (None if not hasattr(self, "_qual_clip_raw_ratio") else round(
                    float(getattr(self, "_qual_clip_raw_ratio", 0.0) or 0.0), 4)),
                "qual_clip_quality_ratio": (None if not hasattr(self, "_qual_clip_quality_ratio") else round(
                    float(getattr(self, "_qual_clip_quality_ratio", 0.0) or 0.0), 4)),
                "qual_rate_ratio": (None if not hasattr(self, "_qual_rate_ratio") else round(
                    float(getattr(self, "_qual_rate_ratio", 0.0) or 0.0), 4)),
                "qual_rate_strong_ratio": (None if not hasattr(self, "_qual_rate_strong_ratio") else round(
                    float(getattr(self, "_qual_rate_strong_ratio", 0.0) or 0.0), 4)),
                "qual_both_ratio": (None if not hasattr(self, "_qual_both_ratio") else round(
                    float(getattr(self, "_qual_both_ratio", 0.0) or 0.0), 4)),
                "qual_steer_pressed_ratio": (None if not hasattr(self, "_qual_steer_pressed_ratio") else round(
                    float(getattr(self, "_qual_steer_pressed_ratio", 0.0) or 0.0), 4)),
                "latAF_raw": round(float(ltp.latAccelFactorRaw), 5),
                "latAF_f": round(float(ltp.latAccelFactorFiltered), 5),
                "latAF_assist_active": bool(getattr(self, "_latAF_assist_active", False)),
                "latAF_assist_base": round(float(getattr(self, "_latAF_assist_base", 0.0) or 0.0), 5),
                "latAF_assist_scale": round(float(getattr(self, "_latAF_assist_scale", 1.0) or 1.0), 4),
                "latAF_assist_delta": round(float(getattr(self, "_latAF_assist_delta", 0.0) or 0.0), 5),
                "latAF_assist_clip_ratio": round(float(getattr(self, "_latAF_assist_clip_ratio", 0.0) or 0.0), 4),
                "latAO_raw": round(float(ltp.latAccelOffsetRaw), 5),
                "latAO_f": round(float(ltp.latAccelOffsetFiltered), 5),
                "fric_raw": round(float(ltp.frictionCoefficientRaw), 5),
                "fric_f": round(float(ltp.frictionCoefficientFiltered), 5),
                "total_pts": int(ltp.totalBucketPoints),
                "decay": round(float(ltp.decay), 3),
                "resets": int(self.resets),

                "straight_bias_mean": (None if straight_bias_mean is None else round(float(straight_bias_mean), 6)),
                "straight_bias_abs_med": (
                    None if straight_bias_abs_med is None else round(float(straight_bias_abs_med), 6)),
                "straight_yaw_pass_ratio": (
                    None if straight_yaw_pass_ratio is None else round(float(straight_yaw_pass_ratio), 4)),
                "straight_latacc_med": (None if straight_latacc_med is None else round(float(straight_latacc_med), 6)),
                "straight_samples": int(len(straight_ok_samples)),
                "straight_sampled": bool(getattr(self, "last_straight_sampled", False)),
                "straight_w_last": (
                    None if not hasattr(self, "last_straight_w") else round(float(self.last_straight_w), 4)),
                "burst_active": bool(self._burst_active is not None),
                "burst_last_trigger": (None if self._last_burst_trigger is None else str(self._last_burst_trigger)),
                "burst_last_path": (None if self._last_burst_path is None else str(self._last_burst_path)),
            }

            with open(self._log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")

        except Exception:
            cloudlog.exception("live torque: failed to write ltp log")

    def get_msg(self, valid=True, with_points=False):
        msg = messaging.new_message('liveTorqueParameters')
        msg.valid = valid
        liveTorqueParameters = msg.liveTorqueParameters
        liveTorqueParameters.version = VERSION

        if not LIVE_TORQUE_TUNING_ENABLED:
            fF = LAT_ACCEL_FACTOR_ANCHOR
            fO = 0.0
            fR = FRICTION_ANCHOR

            liveTorqueParameters.liveValid = True
            liveTorqueParameters.latAccelFactorRaw = fF
            liveTorqueParameters.latAccelOffsetRaw = fO
            liveTorqueParameters.frictionCoefficientRaw = fR
            liveTorqueParameters.latAccelFactorFiltered = fF
            liveTorqueParameters.latAccelOffsetFiltered = fO
            liveTorqueParameters.frictionCoefficientFiltered = fR

            liveTorqueParameters.totalBucketPoints = 0
            liveTorqueParameters.bucketPoints = "[LiveTorqueTuningDisabled]\n"
            liveTorqueParameters.decay = float(self.decay)
            liveTorqueParameters.maxResets = int(self.resets)

            if with_points:
                liveTorqueParameters.points = []
            return msg

        curF = _sanitize_num(self.filtered_params['latAccelFactor'].x, self.offline_latAccelFactor)
        curO = _sanitize_num(self.filtered_params['latAccelOffset'].x, 0.0)
        curR = _sanitize_num(self.filtered_params['frictionCoefficient'].x, self.offline_friction)

        liveTorqueParameters.latAccelFactorRaw = float(curF)
        liveTorqueParameters.latAccelOffsetRaw = float(curO)
        liveTorqueParameters.frictionCoefficientRaw = float(curR)

        # ✅ (3) 저속 코너에서 업데이트 동결 해제:
        # - 정지/파킹(last_is_frozen)은 그대로 동결
        # - 속도 < 약 10.8km/h 이더라도 curve_active가 아니면 동결
        v = float(self.last_vego) if (self.last_vego is not None and np.isfinite(self.last_vego)) else 0.0
        yaw_abs = abs(float(self.last_yaw_rate)) if (
                    self.last_yaw_rate is not None and np.isfinite(self.last_yaw_rate)) else 0.0
        curve_active_now = (v > MIN_VEL_CURVE_MS) and (yaw_abs > CURVE_YAWRATE_MIN)

        freeze_update = bool(self.last_is_frozen) or ((v < FREEZE_UPDATE_MS) and (not curve_active_now))

        # -----------------------------
        # Update gating: clip/rate_limited/quality window (학습 품질 보호)
        # -----------------------------
        t_now = float(self.last_time) if (self.last_time is not None and np.isfinite(self.last_time)) else 0.0

        # -----------------------------
        # Per-frame debug markers (latAO offset learning)
        #  - latAO_evt: offset update happened this frame
        #  - latAO_blk: candidate existed but was blocked (why)
        # -----------------------------
        self._latAO_evt = None
        self._latAO_blk = None

        clip_now = bool(self.last_steer_clip) or bool(self.last_max_limited)
        rate_strong_now = bool(getattr(self, "last_rate_limited_strong", False))
        rate_now = bool(self.last_rate_limited) or rate_strong_now
        steer_pressed_db_now = bool(getattr(self, "_last_steer_override_db", getattr(self, "_last_steer_override", False)))
        steer_pressed_raw_now = bool(getattr(self, "_last_steer_override", steer_pressed_db_now))
        try:
            driver_torque_now = abs(float(getattr(self, "_last_driver_torque", 0.0) or 0.0))
        except Exception:
            driver_torque_now = 0.0
        steer_pressed_now = bool(steer_pressed_db_now or (
            steer_pressed_raw_now and driver_torque_now >= float(STEER_PRESSED_DRIVER_TORQUE_MIN)))
        try:
            des_abs_quality = abs(float(self.last_steer_desired)) if (
                self.last_steer_desired is not None and np.isfinite(self.last_steer_desired)) else 0.0
        except Exception:
            des_abs_quality = 0.0
        try:
            delta_err_quality = abs(float(getattr(self, "last_delta_err", 0.0) or 0.0))
        except Exception:
            delta_err_quality = 0.0
        eps_evt_quality = bool(getattr(self, "_lc_eps_evt", False) or getattr(self, "_ltp_eps_evt_proxy", False))
        clip_quality_now = bool(
            clip_now and (
                bool(self.last_max_limited) or
                bool(rate_strong_now) or
                bool(eps_evt_quality) or
                (float(des_abs_quality) >= float(QUALITY_CLIP_MIN_DES)) or
                (float(delta_err_quality) >= float(QUALITY_CLIP_MIN_DELTA_ERR))
            )
        )
        if steer_pressed_now:
            clip_quality_now = False
        self._last_clip_quality = bool(clip_quality_now)
        self._last_clip_raw = bool(clip_now)

        # rate-limited 과도/준정상 분기(홀드 포함)
        rate_transient_now = bool(rate_now and (
                rate_strong_now or
                (t_now < float(getattr(self, "_rate_transient_until", 0.0) or 0.0)) or
                (abs(float(getattr(self, "_last_desired_delta", 0.0) or 0.0)) >= float(RATE_LIM_TRANSIENT_DES_DELTA))
        ))
        rate_quasi_steady_now = bool(rate_now and (not rate_transient_now))
        rate_quality_now = bool(rate_now and (not steer_pressed_now) and (not rate_transient_now))
        rate_strong_quality_now = bool(rate_strong_now and (not steer_pressed_now))

        # 최근 N초 품질 윈도우 통계
        try:
            if not hasattr(self, "_qual_win") or self._qual_win is None:
                self._qual_win = deque()
            self._qual_win.append((t_now, clip_quality_now, rate_quality_now, rate_strong_quality_now, steer_pressed_now,
                                   rate_transient_now, clip_now))
            while len(self._qual_win) > 0 and (t_now - float(self._qual_win[0][0])) > float(QUALITY_WIN_S):
                self._qual_win.popleft()

            n = len(self._qual_win)
            self._qual_n = int(n)
            if n >= int(QUALITY_MIN_SAMPLES):
                clip_ratio = float(sum(1 for s in self._qual_win if bool(s[1])) / n)
                clip_raw_ratio = float(sum(1 for s in self._qual_win if bool(s[6] if len(s) > 6 else s[1])) / n)
                rate_ratio = float(sum(1 for s in self._qual_win if bool(s[2])) / n)
                rate_strong_ratio = float(sum(1 for s in self._qual_win if bool(s[3])) / n)
                steer_pressed_ratio = float(sum(1 for s in self._qual_win if bool(s[4])) / n)

                both_ratio = float(sum(1 for s in self._qual_win if (bool(s[1]) and bool(s[4]))) / n)

                # export for logging
                self._qual_clip_ratio = clip_ratio
                self._qual_clip_quality_ratio = clip_ratio
                self._qual_clip_raw_ratio = clip_raw_ratio
                self._qual_rate_ratio = rate_ratio
                self._qual_rate_strong_ratio = rate_strong_ratio
                self._qual_steer_pressed_ratio = steer_pressed_ratio
                self._qual_both_ratio = both_ratio

                # --- quality freeze gate (enter/exit hysteresis + low-speed branch + conditional hold) ---
                v_kph_now = float(getattr(self, "_last_v_kph", 0.0) or 0.0)
                low_speed = (v_kph_now > 0.0) and (v_kph_now < float(QUALITY_LOW_SPEED_KPH))

                # Clip hysteresis latch (speed-dependent)
                clip_enter = float(QUALITY_CLIP_FREEZE_ENTER_LOW if low_speed else QUALITY_CLIP_FREEZE_ENTER_HIGH)
                clip_exit  = float(QUALITY_CLIP_FREEZE_EXIT_LOW  if low_speed else QUALITY_CLIP_FREEZE_EXIT_HIGH)
                if not hasattr(self, "_qual_clip_latched"):
                    self._qual_clip_latched = False
                if (not bool(self._qual_clip_latched)) and (clip_ratio >= clip_enter):
                    self._qual_clip_latched = True
                elif bool(self._qual_clip_latched) and (clip_ratio < clip_exit):
                    self._qual_clip_latched = False
                clip_trig = bool(self._qual_clip_latched)

                # Both(clip+pressed) hysteresis latch (low-speed only; else keep original threshold w/o latch)
                if not hasattr(self, "_qual_both_latched"):
                    self._qual_both_latched = False
                if low_speed:
                    both_enter = float(QUALITY_BOTH_FREEZE_ENTER_LOW)
                    both_exit  = float(QUALITY_BOTH_FREEZE_EXIT_LOW)
                    if (not bool(self._qual_both_latched)) and (both_ratio >= both_enter):
                        self._qual_both_latched = True
                    elif bool(self._qual_both_latched) and (both_ratio < both_exit):
                        self._qual_both_latched = False
                else:
                    self._qual_both_latched = False
                both_trig = (bool(self._qual_both_latched) if low_speed else (both_ratio >= float(QUALITY_BOTH_FREEZE_RATIO)))

                rate_trig = (rate_ratio >= float(QUALITY_RATE_FREEZE_RATIO))
                rate_strong_trig = (rate_strong_ratio >= float(QUALITY_RATE_STRONG_FREEZE_RATIO))
                pressed_trig = (steer_pressed_ratio >= float(QUALITY_STEER_PRESSED_FREEZE_RATIO))

                freeze_by_ratio = (clip_trig or both_trig or rate_trig or rate_strong_trig or pressed_trig)
                if freeze_by_ratio:
                    hold_s = float(QUALITY_FREEZE_HOLD_S)
                    near_margin = float(QUALITY_NEAR_THRESH_MARGIN)
                    clip_near = clip_trig and (clip_ratio < (clip_enter + near_margin)) and (not rate_strong_trig)
                    both_near = low_speed and bool(self._qual_both_latched) and (both_ratio < (float(QUALITY_BOTH_FREEZE_ENTER_LOW) + near_margin)) and (not rate_strong_trig)
                    if (clip_near or both_near) and (not rate_trig) and (not pressed_trig):
                        hold_s = float(QUALITY_FREEZE_HOLD_S_SHORT)

                    prev_until = float(getattr(self, "_qual_freeze_until", 0.0) or 0.0)
                    new_until = max(prev_until, t_now + hold_s)
                    if new_until > prev_until + 1e-6:
                        # Extend event (A안): record only when until is actually pushed out
                        try:
                            self._qual_freeze_extend_cnt = int(getattr(self, "_qual_freeze_extend_cnt", 0) or 0) + 1
                        except Exception:
                            self._qual_freeze_extend_cnt = 1

                        trig_list = []
                        try:
                            clip_thr = float(clip_enter)
                            both_thr = float(QUALITY_BOTH_FREEZE_ENTER_LOW if low_speed else QUALITY_BOTH_FREEZE_RATIO)
                            pressed_thr = float(QUALITY_STEER_PRESSED_FREEZE_RATIO)
                            rate_strong_thr = float(QUALITY_RATE_STRONG_FREEZE_RATIO)

                            if bool(clip_trig) or (float(clip_ratio) >= clip_thr):
                                trig_list.append("clip")
                            if bool(pressed_trig) or (float(steer_pressed_ratio) >= pressed_thr):
                                trig_list.append("pressed")
                            if bool(rate_strong_trig) or (float(rate_strong_ratio) >= rate_strong_thr):
                                trig_list.append("rate_strong")
                            if bool(both_trig) or (float(both_ratio) >= both_thr):
                                trig_list.append("both")

                            scores = {
                                "clip": (float(clip_ratio) / clip_thr) if clip_thr > 1e-9 else 0.0,
                                "pressed": (float(steer_pressed_ratio) / pressed_thr) if pressed_thr > 1e-9 else 0.0,
                                "rate_strong": (float(rate_strong_ratio) / rate_strong_thr) if rate_strong_thr > 1e-9 else 0.0,
                                "both": (float(both_ratio) / both_thr) if both_thr > 1e-9 else 0.0,
                            }
                            exceeded = {k: v for k, v in scores.items() if v >= 1.0}
                            primary = max(exceeded, key=exceeded.get) if exceeded else max(scores, key=scores.get)
                        except Exception:
                            primary = "unknown"

                        self._qual_freeze_ext_evt = {
                            "t_now": float(t_now),
                            "prev_until": float(prev_until),
                            "new_until": float(new_until),
                            "hold_s": float(hold_s),
                            "primary": str(primary),
                            "triggers": ",".join(trig_list),
                            "clip_ratio": float(clip_ratio),
                            "clip_quality_ratio": float(clip_ratio),
                            "clip_raw_ratio": float(clip_raw_ratio),
                            "pressed_ratio": float(steer_pressed_ratio),
                            "rate_strong_ratio": float(rate_strong_ratio),
                            "both_ratio": float(both_ratio),
                            "low_speed": bool(low_speed),
                        }
                    self._qual_freeze_until = float(new_until)
            else:
                # not enough samples yet
                self._qual_clip_ratio = float(getattr(self, "_qual_clip_ratio", 0.0) or 0.0)
                self._qual_clip_quality_ratio = float(getattr(self, "_qual_clip_quality_ratio", self._qual_clip_ratio) or 0.0)
                self._qual_clip_raw_ratio = float(getattr(self, "_qual_clip_raw_ratio", self._qual_clip_ratio) or 0.0)
                self._qual_rate_ratio = float(getattr(self, "_qual_rate_ratio", 0.0) or 0.0)
                self._qual_rate_strong_ratio = float(getattr(self, "_qual_rate_strong_ratio", 0.0) or 0.0)
                self._qual_steer_pressed_ratio = float(getattr(self, "_qual_steer_pressed_ratio", 0.0) or 0.0)
        except Exception:
            pass

        qual_freeze_now = bool(t_now < float(getattr(self, "_qual_freeze_until", 0.0) or 0.0))

        if self.corner_points.is_valid():
            latFactor_c, latOffset_c, friction_c = self.estimate_params_corner()

            latFactor_s = np.nan
            latOffset_s, win_ok_n, win_total_n, win_ratio = self._estimate_straight_offset_from_window()
            min_ok_win_n = self._straight_min_ok_required(win_total_n)
            use_straight = (
                    (win_ok_n >= int(min_ok_win_n)) and
                    (win_ratio >= float(STRAIGHT_OK_MIN_RATIO)) and
                    np.isfinite(latOffset_s)
            )

            w = self.straight_weight(self.last_vego, self.last_yaw_rate, self.last_time) if use_straight else 0.0

            self._update_straight_window_stats()
            min_ok_win = self._straight_min_ok_required(self._straight_win_total)
            straight_offset_ok = (
                    np.isfinite(latOffset_s) and
                    (self._straight_win_ok >= int(min_ok_win)) and
                    (self._straight_win_ok_ratio >= STRAIGHT_OK_MIN_RATIO) and
                    (abs(float(latOffset_s)) <= 0.15)
            )

            latFactor_blend = latFactor_c
            friction_use = friction_c

            latOffset_base = _sanitize_num(self.filtered_params['latAccelOffset'].x, 0.0)
            latOffset_blend = float(latOffset_base)

            win = list(self._straight_bias) if hasattr(self, "_straight_bias") else []
            steer_ok = [s[1] for s in win if isinstance(s, (list, tuple)) and len(s) >= 6 and bool(s[5]) and (
                        s[1] is not None) and np.isfinite(s[1])]
            bias_mean = _mean_safe(steer_ok)
            bias_ok = (bias_mean is None) or (abs(float(bias_mean)) <= float(STRAIGHT_STEER_MAX_FOR_OFFSET) * 0.50)

            if straight_offset_ok and bias_ok:
                w_off = min(float(w), 0.10)
                latOffset_blend = (1.0 - w_off) * float(latOffset_base) + w_off * float(latOffset_s)

            if _finite(latFactor_blend): liveTorqueParameters.latAccelFactorRaw = float(latFactor_blend)
            if _finite(latOffset_blend): liveTorqueParameters.latAccelOffsetRaw = float(latOffset_blend)
            if _finite(friction_use):    liveTorqueParameters.frictionCoefficientRaw = float(friction_use)

            sane = self.is_sane(latFactor_blend, latOffset_blend, friction_use)
            enough_points = len(self.corner_points) >= self.min_points_total

            latF_use, latO_use, fric_use = self._coerce_params(latFactor_blend, latOffset_blend, friction_use)

            if sane or enough_points:
                if not sane and enough_points:
                    cloudlog.warning("live torque: forcing liveValid=True (enough points) with clamped params")
                liveTorqueParameters.liveValid = True

                # 최종 학습 업데이트 동결 조건
                freeze_update_total = bool(freeze_update) or bool(qual_freeze_now) or bool(rate_transient_now)

                # -----------------------------
                # Straight offset(latAO) update debug (candidate / gating / block reasons)
                #  - offset_candidate: straight_offset_ok + bias_ok + finite(latOffset_s)
                #  - offset_gate_pass: candidate + cooldown + no clip/rate + not freeze_update_total
                # -----------------------------
                try:
                    offset_candidate = bool(straight_offset_ok and bias_ok and np.isfinite(latOffset_s))
                except Exception:
                    offset_candidate = False
                try:
                    cooldown_ok = bool(self._offset_update_allowed())
                except Exception:
                    cooldown_ok = False

                offset_gate_pass = bool(offset_candidate and cooldown_ok and (not clip_now) and (not rate_now) and (
                    not freeze_update_total))

                # Log "blocked" only at low rate to avoid spam
                if offset_candidate and (not offset_gate_pass):
                    try:
                        last_blk_t = float(getattr(self, "_latAO_blk_last_t", -1e9) or -1e9)
                        if (t_now - last_blk_t) >= 1.0:
                            self._latAO_blk_last_t = float(t_now)
                            reasons = []
                            if bool(freeze_update_total):
                                if bool(freeze_update): reasons.append("stop/low-speed")
                                if bool(qual_freeze_now): reasons.append("quality_window")
                                if bool(rate_transient_now): reasons.append("rate_transient")
                                if not (bool(freeze_update) or bool(qual_freeze_now) or bool(rate_transient_now)):
                                    reasons.append("freeze_total")
                            if bool(clip_now): reasons.append("clip/max_limited")
                            if bool(rate_now): reasons.append("rate_limited")
                            if not bool(cooldown_ok): reasons.append("cooldown")

                            self._latAO_blk = {
                                "t": float(t_now),
                                "clip_now": bool(clip_now),
                                "steeringPressed": bool(steer_pressed_now),
                                "rate_now": bool(rate_now),
                                "rate_transient": bool(rate_transient_now),
                                "qual_freeze": bool(qual_freeze_now),
                                "qual_n": int(getattr(self, "_qual_n", 0) or 0),
                                "qual_clip_ratio": float(getattr(self, "_qual_clip_ratio", 0.0) or 0.0),
                                "qual_clip_raw_ratio": float(getattr(self, "_qual_clip_raw_ratio", 0.0) or 0.0),
                                "qual_clip_quality_ratio": float(getattr(self, "_qual_clip_quality_ratio", 0.0) or 0.0),
                                "qual_rate_ratio": float(getattr(self, "_qual_rate_ratio", 0.0) or 0.0),
                                "qual_rate_strong_ratio": float(getattr(self, "_qual_rate_strong_ratio", 0.0) or 0.0),
                                "qual_steer_pressed_ratio": float(
                                    getattr(self, "_qual_steer_pressed_ratio", 0.0) or 0.0),
                                "win_ok": (None if win_ok_n is None else int(win_ok_n)),
                                "win_total": (None if win_total_n is None else int(win_total_n)),
                                "win_ratio": (None if win_ratio is None else float(win_ratio)),
                                "win_min_ok_req": (None if min_ok_win_n is None else int(min_ok_win_n)),
                                "straight_win_ok": int(getattr(self, "_straight_win_ok", 0) or 0),
                                "straight_win_total": int(getattr(self, "_straight_win_total", 0) or 0),
                                "straight_win_ok_ratio": float(getattr(self, "_straight_win_ok_ratio", 0.0) or 0.0),
                                "latO_s": (None if not np.isfinite(latOffset_s) else float(latOffset_s)),
                                "bias_mean": (None if bias_mean is None else float(bias_mean)),
                                "bias_ok": bool(bias_ok),
                                "cooldown_ok": bool(cooldown_ok),
                                "reasons": "|".join(reasons) if len(reasons) else "blocked",
                            }
                    except Exception:
                        pass

                if not freeze_update_total:
                    offset_updated = False
                    latO_target = None
                    # 기본 업데이트 값
                    upd = {
                        'latAccelFactor': latF_use,
                        'frictionCoefficient': fric_use,
                    }

                    # clip/max_limited 구간: friction 업데이트는 동결(가짜 마찰 추정 방지)
                    if bool(clip_now):
                        try:
                            upd.pop('frictionCoefficient', None)
                        except Exception:
                            pass

                    # rate_limited(준정상) 구간: 업데이트는 약하게만 반영 + latAccelFactor 하향 금지
                    if bool(rate_quasi_steady_now):
                        try:
                            cur_latF = float(
                                _sanitize_num(self.filtered_params['latAccelFactor'].x, self.offline_latAccelFactor))
                            cur_fric = float(
                                _sanitize_num(self.filtered_params['frictionCoefficient'].x, self.offline_friction))

                            # latAF 하향 금지: 목표값이 cur보다 작으면 cur로 클램프
                            latF_tgt = float(max(float(latF_use), float(cur_latF)))

                            # 준정상 반영 비중(블렌드)
                            wF = float(np.clip(float(RATE_LIM_STEADY_BLEND_W), 0.0, 1.0))
                            wR = float(np.clip(float(RATE_LIM_STEADY_FRICTION_BLEND_W), 0.0, 1.0))

                            upd['latAccelFactor'] = float(cur_latF + wF * (latF_tgt - cur_latF))
                            upd['frictionCoefficient'] = float(cur_fric + wR * (float(fric_use) - cur_fric))
                        except Exception:
                            # 최소 안전: 하향 금지라도 보장
                            try:
                                cur_latF = float(_sanitize_num(self.filtered_params['latAccelFactor'].x,
                                                               self.offline_latAccelFactor))
                                upd['latAccelFactor'] = float(max(float(latF_use), float(cur_latF)))
                            except Exception:
                                pass
                    if offset_gate_pass:
                        try:
                            latO_prev = float(_sanitize_num(self.filtered_params['latAccelOffset'].x, 0.0))
                            _, latO_s_use, _ = self._coerce_params(latF_use, float(latOffset_s), fric_use)
                            if _finite(latO_s_use):
                                upd['latAccelOffset'] = float(latO_s_use)
                                offset_updated = True
                                latO_target = float(latO_s_use)
                        except Exception:
                            pass
                    self.update_params(upd)
                    if offset_updated:
                        try:
                            self._last_offset_update_t = float(self.last_time or 0.0)
                        except Exception:
                            pass
                        try:
                            latO_new = float(_sanitize_num(self.filtered_params['latAccelOffset'].x, 0.0))
                            self._latAO_evt = {
                                "t": float(t_now),
                                "prev": float(latOffset_base),
                                "target": (None if latO_target is None else float(latO_target)),
                                "new": float(latO_new),
                                "clip_now": bool(clip_now),
                                "steeringPressed": bool(steer_pressed_now),
                                "rate_now": bool(rate_now),
                                "rate_transient": bool(rate_transient_now),
                                "qual_freeze": bool(qual_freeze_now),
                                "qual_n": int(getattr(self, "_qual_n", 0) or 0),
                                "qual_clip_ratio": float(getattr(self, "_qual_clip_ratio", 0.0) or 0.0),
                                "qual_clip_raw_ratio": float(getattr(self, "_qual_clip_raw_ratio", 0.0) or 0.0),
                                "qual_clip_quality_ratio": float(getattr(self, "_qual_clip_quality_ratio", 0.0) or 0.0),
                                "qual_rate_ratio": float(getattr(self, "_qual_rate_ratio", 0.0) or 0.0),
                                "qual_rate_strong_ratio": float(getattr(self, "_qual_rate_strong_ratio", 0.0) or 0.0),
                                "qual_steer_pressed_ratio": float(
                                    getattr(self, "_qual_steer_pressed_ratio", 0.0) or 0.0),
                                "win_ok": (None if win_ok_n is None else int(win_ok_n)),
                                "win_total": (None if win_total_n is None else int(win_total_n)),
                                "win_ratio": (None if win_ratio is None else float(win_ratio)),
                                "win_min_ok_req": (None if min_ok_win_n is None else int(min_ok_win_n)),
                                "straight_win_ok": int(getattr(self, "_straight_win_ok", 0) or 0),
                                "straight_win_total": int(getattr(self, "_straight_win_total", 0) or 0),
                                "straight_win_ok_ratio": float(getattr(self, "_straight_win_ok_ratio", 0.0) or 0.0),
                                "bias_mean": (None if bias_mean is None else float(bias_mean)),
                                "bias_ok": bool(bias_ok),
                            }
                        except Exception:
                            pass
                else:
                    try:
                        reasons = []
                        if bool(freeze_update): reasons.append("stop/low-speed")
                        if bool(qual_freeze_now): reasons.append("quality_window")
                        if bool(clip_now): reasons.append("clip/max_limited")
                        if bool(rate_transient_now): reasons.append("rate_transient")
                        cloudlog.info("live torque: params frozen: " + ",".join(reasons) if len(
                            reasons) else "live torque: params frozen")
                    except Exception:
                        cloudlog.info("live torque: params frozen")
                self.invalid_values_tracker = max(0.0, self.invalid_values_tracker - 0.5)
            else:
                cloudlog.warning(
                    f"live torque: params outside acceptable bounds; marking invalid "
                    f"(latAccelFactor={latFactor_blend}, offset={latOffset_blend}, friction={fric_use})"
                )
                liveTorqueParameters.liveValid = False
                self.invalid_values_tracker += 1.0
                if self.invalid_values_tracker > MAX_INVALID_THRESHOLD and len(
                        self.corner_points) < self.min_points_total:
                    self.reset()
                if self.invalid_values_tracker > (3 * MAX_INVALID_THRESHOLD):
                    cloudlog.warning("live torque: sustained invalid values; hard reset")
                    self.reset()
        else:
            # Corner 포인트가 아직 부족해도(저속/재시작 직후) straight 오프셋은 먼저 학습하도록
            try:
                latOffset_s, win_ok_n, win_total_n, win_ratio = self._estimate_straight_offset_from_window()
                self._update_straight_window_stats()
                min_ok_win = self._straight_min_ok_required(self._straight_win_total)
                straight_offset_ok = (
                        np.isfinite(latOffset_s) and
                        (self._straight_win_ok >= int(min_ok_win)) and
                        (self._straight_win_ok_ratio >= STRAIGHT_OK_MIN_RATIO) and
                        (abs(float(latOffset_s)) <= 0.15)
                )

                if (not freeze_update) and straight_offset_ok and self._offset_update_allowed():
                    try:
                        # step-limit + clamp는 _coerce_params에서 처리
                        latF_use, latO_use, fric_use = self._coerce_params(curF, float(latOffset_s), curR)
                        if _finite(latO_use):
                            self.update_params({'latAccelOffset': float(latO_use)})
                            self._last_offset_update_t = float(self.last_time or 0.0)
                    except Exception:
                        pass
            except Exception:
                pass

            # Soft liveValid: warm restore가 있거나, 일정 시간/포인트가 쌓이면 안전한 클램프 값으로 liveValid=True
            v_kph = float(self.last_vego) * 3.6 if (self.last_vego is not None and np.isfinite(self.last_vego)) else 0.0
            soft_min_pts = self._soft_livevalid_min_points(v_kph)
            pts_corner = int(len(self.corner_points) + len(self.limited_corner_points))
            t_since_start = 0.0
            try:
                if self.start_time is not None and self.last_time is not None:
                    t_since_start = float(self.last_time) - float(self.start_time)
            except Exception:
                t_since_start = 0.0

            soft_ready = bool(getattr(self, '_warm_restored', False)) or (
                        (t_since_start >= float(SOFT_LIVEVALID_MIN_S)) and (pts_corner >= int(soft_min_pts)))
            liveTorqueParameters.liveValid = True if soft_ready else False

        if with_points:
            pts_corner = self.corner_points.get_points()
            pts_straight = self.straight_points.get_points()
            pts = np.vstack([pts_corner, pts_straight]) if len(pts_corner) or len(pts_straight) else np.empty((0, 3))
            liveTorqueParameters.points = (pts[:, [0, 2]].tolist() if len(pts) else [])

        minF, maxF, minR, maxR = self._dynamic_bands()
        fF = float(
            np.clip(_sanitize_num(self.filtered_params['latAccelFactor'].x, self.offline_latAccelFactor), minF, maxF))
        fO = float(np.clip(_sanitize_num(self.filtered_params['latAccelOffset'].x, 0.0), -self.max_offset_abs,
                           self.max_offset_abs))
        fR = float(
            np.clip(_sanitize_num(self.filtered_params['frictionCoefficient'].x, self.offline_friction), minR, maxR))

        fF = self._apply_lat_factor_assist(fF, minF, maxF)
        liveTorqueParameters.latAccelFactorFiltered = fF
        liveTorqueParameters.latAccelOffsetFiltered = fO
        liveTorqueParameters.frictionCoefficientFiltered = fR
        liveTorqueParameters.totalBucketPoints = len(self.corner_points) + len(self.straight_points) + len(
            self.limited_corner_points)
        liveTorqueParameters.bucketPoints = (
                "[Corner]\n" + self.corner_points.show_bucket_status() +
                "[Corner-Limited]\n" + self.limited_corner_points.show_bucket_status() +
                "[Straight]\n" + self.straight_points.show_bucket_status()
        )
        liveTorqueParameters.decay = self.decay
        liveTorqueParameters.maxResets = self.resets

        self._log_to_file(msg)
        return msg


def main(sm=None, pm=None):
    config_realtime_process(2, Priority.CTRL_LOW)

    if sm is None:
        sm = messaging.SubMaster(['carControl', 'carState', 'controlsState', 'liveLocationKalman'], poll=['liveLocationKalman'])

    if pm is None:
        pm = messaging.PubMaster(['liveTorqueParameters'])

    params = Params()

    # -----------------------------
    # Load live torque tuning/anchors from Params (user settings)
    #  - IsLiveTorque (bool): enable/disable live tuning
    #  - TorqueMaxLatAccel (int-like, x0.1): latAccelFactor anchor
    #  - TorqueFriction (int-like, x0.001): friction anchor
    #
    # ✅ Runtime reload support:
    #   When the Param values change while running, they will be reloaded periodically
    #   and applied immediately (toggle + new anchors).
    # -----------------------------
    global LIVE_TORQUE_TUNING_ENABLED, LAT_ACCEL_FACTOR_ANCHOR, FRICTION_ANCHOR

    USER_PARAM_RELOAD_INTERVAL_S = float(os.environ.get("LTP_PARAMS_RELOAD_INTERVAL_S", "0.5"))
    _last_user_param_check_wall = time.monotonic()

    _last_is_live = None
    _last_lat_raw = None
    _last_fric_raw = None

    def _reload_user_torque_params(estimator_ref=None, force_log: bool = False):
        global LIVE_TORQUE_TUNING_ENABLED, LAT_ACCEL_FACTOR_ANCHOR, FRICTION_ANCHOR
        nonlocal _last_is_live, _last_lat_raw, _last_fric_raw
        changed = False

        # IsLiveTorque (bool)
        # Params.get_bool()은 Param이 없을 때 False를 반환할 수 있으므로,
        # 키가 실제로 존재할 때만 override한다. 없으면 코드 기본값(True)을 유지한다.
        try:
            raw_live = params.get('IsLiveTorque')
            if raw_live is not None:
                is_live = bool(params.get_bool('IsLiveTorque'))
                if _last_is_live is None or is_live != _last_is_live:
                    LIVE_TORQUE_TUNING_ENABLED = bool(is_live)
                    _last_is_live = bool(is_live)
                    changed = True
        except Exception:
            pass

        # TorqueMaxLatAccel (x0.1)
        try:
            v = params.get("TorqueMaxLatAccel", encoding="utf8")
            if v is not None:
                v = str(v).strip()
            if v:
                if _last_lat_raw is None or v != _last_lat_raw:
                    torque_lat_accel_factor = float(Decimal(v) * Decimal('0.1'))  # LAT_ACCEL_FACTOR
                    if np.isfinite(torque_lat_accel_factor) and torque_lat_accel_factor > 0.5:
                        LAT_ACCEL_FACTOR_ANCHOR = float(torque_lat_accel_factor)
                        _last_lat_raw = v
                        changed = True
        except (InvalidOperation, Exception):
            pass

        # TorqueFriction (x0.001)
        try:
            v = params.get("TorqueFriction", encoding="utf8")
            if v is not None:
                v = str(v).strip()
            if v:
                if _last_fric_raw is None or v != _last_fric_raw:
                    torque_friction = float(Decimal(v) * Decimal('0.001'))  # FRICTION
                    if np.isfinite(torque_friction) and torque_friction > 0.01:
                        FRICTION_ANCHOR = float(torque_friction)
                        _last_fric_raw = v
                        changed = True
        except (InvalidOperation, Exception):
            pass

        # Keep estimator's offline anchors and sanity bounds in sync (used for warm-start and fallbacks)
        try:
            if estimator_ref is not None:
                estimator_ref.offline_latAccelFactor = float(LAT_ACCEL_FACTOR_ANCHOR)
                estimator_ref.offline_friction = float(FRICTION_ANCHOR)
                estimator_ref.base_params['latAccelFactor'] = float(LAT_ACCEL_FACTOR_ANCHOR)
                estimator_ref.base_params['frictionCoefficient'] = float(FRICTION_ANCHOR)
                estimator_ref.min_lataccel_factor = (1.0 - FACTOR_SANITY) * float(LAT_ACCEL_FACTOR_ANCHOR)
                estimator_ref.max_lataccel_factor = (1.0 + FACTOR_SANITY) * float(LAT_ACCEL_FACTOR_ANCHOR)
                estimator_ref.min_friction = (1.0 - FRICTION_SANITY) * float(FRICTION_ANCHOR)
                estimator_ref.max_friction = (1.0 + FRICTION_SANITY) * float(FRICTION_ANCHOR)
        except Exception:
            pass

        if changed or force_log:
            try:
                cloudlog.info(
                    "LiveTorque: runtime params applied IsLiveTorque=%s LAT_ACCEL_FACTOR_ANCHOR=%.5f FRICTION_ANCHOR=%.5f"
                    % (LIVE_TORQUE_TUNING_ENABLED, LAT_ACCEL_FACTOR_ANCHOR, FRICTION_ANCHOR)
                )
            except Exception:
                pass

    def _clear_ltp_persistent_state(reason: str, clear_runtime_anchor_params: bool = False):
        """Clear every persistent source that can override source anchors on startup."""
        try:
            cloudlog.warning("LiveTorque: clearing persistent state (%s)" % str(reason))
        except Exception:
            pass

        try:
            for pth in (LTP_STATE_PATH, LTP_STATE_PATH_PKL):
                if os.path.exists(pth):
                    os.remove(pth)
        except Exception:
            pass

        for key in ("LiveTorqueCarParams", "LiveTorqueParameters"):
            try:
                params.remove(key)
            except Exception:
                pass

        if clear_runtime_anchor_params:
            for key in LTP_RUNTIME_ANCHOR_PARAM_KEYS:
                try:
                    params.remove(key)
                except Exception:
                    pass

    startup_version_reset = False

    # VERSION이 바뀌면 source anchor 기준으로 완전 초기화한다.
    # 단순히 VERSION만 올리면 warm-state는 스킵되더라도 LiveTorqueParameters/Runtime Param이
    # 예전 filtered 값을 다시 섞을 수 있으므로, 부팅 초기에 먼저 제거한다.
    try:
        saved_version = params.get(LTP_VERSION_PARAM_KEY, encoding="utf8")
        saved_version = str(saved_version).strip() if saved_version is not None else ""
        if saved_version != str(VERSION):
            startup_version_reset = True
            _clear_ltp_persistent_state(
                "VERSION changed %s -> %s" % (saved_version or "none", VERSION),
                clear_runtime_anchor_params=bool(LTP_CLEAR_RUNTIME_ANCHOR_PARAMS_ON_VERSION_CHANGE),
            )
            try:
                params.put(LTP_VERSION_PARAM_KEY, str(VERSION).encode("utf8"))
            except Exception:
                pass
    except Exception:
        pass

    # Initial load (once at startup). VERSION reset 이후에 읽어야 예전 Runtime Param이 anchor를 덮지 않는다.
    _reload_user_torque_params(estimator_ref=None, force_log=True)

    try:
        do_reset = (os.environ.get("LTP_RESET", "0") == "1")
        v = params.get("LiveTorqueReset")
        if v is not None and v.strip() in [b"1", b"true", b"True", b"YES", b"yes"]:
            do_reset = True
        if do_reset:
            _clear_ltp_persistent_state(
                "manual reset",
                clear_runtime_anchor_params=bool(LTP_CLEAR_RUNTIME_ANCHOR_PARAMS_ON_VERSION_CHANGE),
            )
            try:
                params.put("LiveTorqueReset", b"0")
            except Exception:
                pass
            try:
                params.put(LTP_VERSION_PARAM_KEY, str(VERSION).encode("utf8"))
            except Exception:
                pass
    except Exception:
        pass

    CP = car.CarParams.from_bytes(params.get("CarParams", block=True))
    estimator = TorqueEstimator(CP)
    # Sync offline anchors to the latest runtime Params values
    try:
        _reload_user_torque_params(estimator_ref=estimator, force_log=True)
    except Exception:
        pass

    # VERSION/manual reset 직후에는 filtered 값/버킷/decay도 source anchor로 강제 초기화한다.
    try:
        if startup_version_reset:
            estimator.reset()
            estimator._straight_bias = deque()
            estimator._pending_straight_restore = None
            estimator._straight_win_ok = 0
            estimator._straight_win_total = 0
            estimator._straight_win_ok_ratio = 0.0
            estimator.filtered_params['latAccelFactor'].x = float(estimator.offline_latAccelFactor)
            estimator.filtered_params['latAccelOffset'].x = 0.0
            estimator.filtered_params['frictionCoefficient'].x = float(estimator.offline_friction)
            estimator.decay = float(MIN_FILTER_DECAY)
            cloudlog.warning(
                "LiveTorque: VERSION reset applied filtered anchors latAF=%.5f friction=%.5f"
                % (float(estimator.offline_latAccelFactor), float(estimator.offline_friction))
            )
    except Exception:
        pass

    def cache_params(sig, frame):
        signal.signal(sig, signal.SIG_DFL)
        cloudlog.warning("caching torque params with EMA merge")

        params = Params()
        params.put("LiveTorqueCarParams", CP.as_builder().to_bytes())

        try:
            estimator.save_warm_state(reason=f"signal {sig}")
        except Exception:
            pass
        try:
            estimator._flush_active_burst(reason=f"signal_{sig}", force=True)
        except Exception:
            pass

        msg = estimator.get_msg(with_points=False)

        try:
            prev_bytes = params.get("LiveTorqueParameters")
            if prev_bytes is not None:
                prev = log.Event.from_bytes(prev_bytes).liveTorqueParameters
                cur = msg.liveTorqueParameters

                # VERSION이 다른 이전 캐시와 EMA merge하면 새 anchor 초기화가 다시 오염된다.
                # 같은 VERSION일 때만 EMA merge를 허용한다.
                if int(getattr(prev, 'version', -1)) == int(getattr(cur, 'version', VERSION)):
                    cur.latAccelFactorFiltered = merge_with_cache(prev.latAccelFactorFiltered, cur.latAccelFactorFiltered,
                                                                  EMA_ALPHA)
                    cur.latAccelOffsetFiltered = merge_with_cache(prev.latAccelOffsetFiltered, cur.latAccelOffsetFiltered,
                                                                  EMA_ALPHA)
                    cur.frictionCoefficientFiltered = merge_with_cache(prev.frictionCoefficientFiltered,
                                                                       cur.frictionCoefficientFiltered, EMA_ALPHA)
                    cur.decay = max(prev.decay, cur.decay)
                else:
                    try:
                        cloudlog.warning(
                            "LiveTorque: skip EMA cache merge due to VERSION mismatch prev=%s cur=%s"
                            % (getattr(prev, 'version', None), getattr(cur, 'version', None))
                        )
                    except Exception:
                        pass
            params.put("LiveTorqueParameters", msg.to_bytes())
        except Exception:
            cloudlog.exception("EMA cache merge failed; saving current snapshot")
            params.put("LiveTorqueParameters", msg.to_bytes())

        sys.exit(0)

    if "REPLAY" not in os.environ:
        signal.signal(signal.SIGINT, cache_params)
        signal.signal(signal.SIGTERM, cache_params)
        try:
            signal.signal(signal.SIGHUP, cache_params)
        except Exception:
            pass
        try:
            signal.signal(signal.SIGQUIT, cache_params)
        except Exception:
            pass

    def _atexit_save():
        try:
            estimator.save_warm_state(reason="atexit")
        except Exception:
            pass
        try:
            estimator._flush_active_burst(reason="atexit", force=True)
        except Exception:
            pass

    atexit.register(_atexit_save)

    last_state_save_wall = time.monotonic()
    last_reset_check_wall = last_state_save_wall

    while True:
        sm.update()
        # ✅ runtime reset: LiveTorqueReset=1이면 즉시 warm-state 삭제 + 버킷/필터 리셋
        now_wall = time.monotonic()
        # ✅ runtime params reload: apply IsLiveTorque/TorqueMaxLatAccel/TorqueFriction changes while running
        if (now_wall - _last_user_param_check_wall) >= float(USER_PARAM_RELOAD_INTERVAL_S):
            _last_user_param_check_wall = now_wall
            try:
                _reload_user_torque_params(estimator_ref=estimator, force_log=False)
            except Exception:
                pass

        if (now_wall - last_reset_check_wall) >= 0.5:
            last_reset_check_wall = now_wall
            try:
                v = params.get("LiveTorqueReset")
                if v is not None and v.strip() in [b"1", b"true", b"True", b"YES", b"yes"]:
                    cloudlog.warning("LiveTorque: runtime reset requested")
                    try:
                        _clear_ltp_persistent_state(
                            "runtime reset",
                            clear_runtime_anchor_params=bool(LTP_CLEAR_RUNTIME_ANCHOR_PARAMS_ON_VERSION_CHANGE),
                        )
                    except Exception:
                        pass
                    try:
                        params.put(LTP_VERSION_PARAM_KEY, str(VERSION).encode("utf8"))
                    except Exception:
                        pass
                    try:
                        _reload_user_torque_params(estimator_ref=estimator, force_log=True)
                    except Exception:
                        pass
                    try:
                        estimator._flush_active_burst(reason="runtime_reset", force=True)
                    except Exception:
                        pass
                    try:
                        estimator.reset()
                        estimator._straight_bias = deque()
                        estimator._pending_straight_restore = None
                        estimator._straight_win_ok = 0;
                        estimator._straight_win_total = 0;
                        estimator._straight_win_ok_ratio = 0.0
                        estimator.filtered_params['latAccelFactor'].x = float(estimator.offline_latAccelFactor)
                        estimator.filtered_params['latAccelOffset'].x = 0.0
                        estimator.filtered_params['frictionCoefficient'].x = float(estimator.offline_friction)
                        estimator.decay = float(MIN_FILTER_DECAY)
                    except Exception:
                        pass
                    try:
                        params.put("LiveTorqueReset", b"0")
                    except Exception:
                        pass
            except Exception:
                pass
        for which in sm.updated.keys():
            if sm.updated[which]:
                t = sm.logMonoTime[which] * 1e-9
                estimator.handle_log(t, which, sm[which])

        now_wall = time.monotonic()
        # Periodic warm-state save + point-delta trigger (crash/restart 대비)
        pts_now = 0
        try:
            pts_now = int(
                len(estimator.corner_points) + len(estimator.straight_points) + len(estimator.limited_corner_points))
        except Exception:
            pts_now = 0

        if ((now_wall - last_state_save_wall) >= float(LTP_STATE_SAVE_INTERVAL_S)) or (
                (pts_now - int(getattr(estimator, '_last_state_save_pts', 0) or 0)) >= int(
                LTP_STATE_SAVE_MIN_DELTA_PTS) and (now_wall - last_state_save_wall) >= 5.0):
            try:
                estimator.save_warm_state(reason="periodic")
            except Exception:
                pass
            last_state_save_wall = now_wall

        if sm.frame % 5 == 0:
            pm.send('liveTorqueParameters', estimator.get_msg(valid=sm.all_checks()))


if __name__ == "__main__":
    main()
