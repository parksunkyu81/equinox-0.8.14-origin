> **중요:** 이 문서는 이전 동작 설명입니다. 현재 소스는 `PEDAL_FORCE_RECOVERY_STRICT_ZERO_KO.md`의 기본 OFF·엄격 0 고착 전용 방식으로 변경되었습니다.

> **최신 안내:** 이 문서는 ACCEL=0 전용 1차 구현 설명입니다. 현재 배포본에는 ACCEL>0인데 실제 차량이 감속하는 경우까지 처리하는 양수 페달 무효 watchdog이 추가되었습니다. 최신 적용 기준은 `PEDAL_FORCE_RECOVERY_COMPLETE_WATCHDOG_KO.md`를 확인하십시오.

# 콤마 페달 ACCEL=0 멍때림 복구 Watchdog

## 적용 목적

고속 주행 중 ACC가 활성화되어 있고 목표 속도를 회복해야 하는데 `actuators.accel == 0`으로 떨어지는 경우를 두 단계로 복구합니다.

1. **0~120ms:** 직전 정상 양수 가속 명령을 최대 `0.36m/s²`까지만 유지
2. **120ms 초과:** `0.36m/s²` 강제 복구 및 GM 콤마 페달 최소 `6%` 적용

정상적인 감속·운전자 개입·선행차 접근을 덮어쓰지 않도록 안전 차단 조건은 매 100Hz 프레임에서 우선 적용됩니다.

## 변경 파일

- `selfdrive/controls/lib/pedal_force_recovery.py`
- `selfdrive/controls/controlsd.py`
- `selfdrive/car/gm/carcontroller.py`
- `selfdrive/controls/tests/test_pedal_force_recovery.py`

## 복구 허용 조건

다음 조건을 모두 만족할 때만 Watchdog이 동작합니다.

- 콤마 페달 장착 차량
- openpilot 및 ACC 활성
- 종방향 제어 상태가 `pid`
- 브레이크·운전자 가속페달 입력 없음
- 정차 상태 아님
- 1km/h 초과
- 운전자 모니터링 강제 감속 상태 아님
- FCW 없음
- 현재 PID 목표 또는 미래 계획 속도가 실제 속도보다 `0.30m/s` 이상 높음
- 근거리 계획에서 실제 감속을 요구하지 않음
- 실제 코너 감속 목표가 적용 중이지 않음
- 접근 중인 선행차 위험 없음
- 수동 선행차 따라잡기 기능과 충돌하지 않음

## 즉시 복구 해제 조건

복구 중에도 다음 조건 중 하나가 발생하면 같은 100Hz 프레임에서 즉시 해제합니다.

- 브레이크 입력
- 운전자 가속페달 입력
- ACC 또는 openpilot 비활성
- FCW
- 플랜 오래됨 또는 무효
- 플랜이 감속 요구
- 코너 목표 속도가 실제 속도보다 낮음
- 선행차와 위험하게 접근 중
- 정차 또는 저속

## 기록되는 진단값

기존 cereal 스키마를 변경하지 않기 위해 기존 페달 진단 필드를 재사용합니다.

| controlsState 필드 | 기록값 |
|---|---|
| `pedalDeadzoneBoostCandidate` | ACC 주행 중 `accel=0` 사건 감지 |
| `pedalDeadzoneBoostActive` | 120ms 직전 가속 유지 단계 |
| `pedalDeadzoneRawCommand` | 복구 전 원본 accel |
| `pedalDeadzoneAppliedCommand` | 복구 후 최종 accel |
| `pedalDeadzoneFloor` | 페달 하한 `0.060` |
| `pedalDeadzoneAccelRequest` | 마지막 정상 양수 accel |
| `pedalDeadzoneVehicleAccel` | 차량 실측 `aEgo` |
| `pedalForceRecoveryActive` | 0.36 강제 복구 단계 |
| `pedalForceRecoveryDuration` | Watchdog 사건 지속 시간 |
| `pedalForceRecoveryCount` | Watchdog 진입 횟수 |

`swaglog`에는 다음 문자열로 차단 원인과 상태 전환이 남습니다.

```text
PedalForceRecovery blocked
PedalForceRecovery watchdog start
PedalForceRecovery force start
PedalForceRecovery end
```

대표 차단 원인:

```text
PLAN_INVALID
NO_SPEED_DEMAND
PLAN_DECEL
CURVE_DECEL
LEAD_RISK
BRAKE
DRIVER_GAS
FCW
MANUAL_CATCHUP
```

## 소스 적용

압축 파일의 `selfdrive` 폴더를 openpilot 저장소의 기존 `selfdrive`에 병합합니다.

```bash
cd /data/openpilot

# 수정 파일 확인
git status --short

# 문법 검사
python3 -m py_compile \
  selfdrive/controls/lib/pedal_force_recovery.py \
  selfdrive/controls/controlsd.py \
  selfdrive/car/gm/carcontroller.py

# 단위 테스트
python3 -m unittest \
  selfdrive.controls.tests.test_pedal_force_recovery -v
```

## 정차 차량 시험 순서

실도로보다 먼저 안전한 정차·벤치 환경에서 시험합니다.

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params
p = Params()
p.put("EquinoxSimAccelZero", "2")
print("1회 accel=0 복구 시험 설정 완료")
PY
```

정상 결과:

1. `accel=0` 주입
2. 최대 약 120ms 직전 양수 accel 유지
3. 계속 0이면 `pedalForceRecoveryActive=true`
4. 최종 accel 약 `0.36`
5. 콤마 페달 출력 최소 약 `6%`
6. 정상 PID 출력이 `0.36` 이상으로 100ms 유지되거나 안전 차단 조건 발생 시 해제

시험 후 파라미터를 해제합니다.

```bash
python3 - <<'PY'
from common.params import Params
Params().put("EquinoxSimAccelZero", "0")
print("복구 시험 해제")
PY
```

## 실차 시험 주의

- 처음에는 폐쇄된 평지 또는 저속 안전 구간에서 확인합니다.
- 선행차가 없는 상태에서 원본 accel, 적용 accel, 실제 페달, `aEgo`를 함께 기록합니다.
- 고속도로 시험 전 브레이크 입력 시 복구가 즉시 0으로 해제되는지 확인합니다.
- `LEAD_RISK`, `PLAN_DECEL`, `CURVE_DECEL` 차단은 안전 목적이므로 임계값을 임의로 완화하지 않습니다.
