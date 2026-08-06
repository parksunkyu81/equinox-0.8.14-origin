> **중요:** 이 문서는 이전 동작 설명입니다. 현재 소스는 `PEDAL_FORCE_RECOVERY_STRICT_ZERO_KO.md`의 기본 OFF·엄격 0 고착 전용 방식으로 변경되었습니다.

# 콤마 페달 완전 복구 Watchdog

## 1. 목적

이 수정은 콤마 페달 기반 GM 종방향 제어에서 발생할 수 있는 두 가지 멍때림을 함께 처리합니다.

1. `ACCEL=0`으로 가속 명령이 갑자기 사라지는 경우
2. `ACCEL>0`이고 콤마 페달 명령도 양수지만 실제 차량은 계속 감속하는 경우

두 번째 경우는 단순히 페달값이 양수인지 확인하는 것으로는 잡을 수 없습니다. 목표 속도 부족, 실제 차량 가속도 `aEgo`, 플랜 감속 여부와 선행차 위험을 함께 검사합니다.

> 이 기능은 소프트웨어에서 감지 가능한 무효 명령을 보정합니다. 6% 명령에도 차량 반응이 없다면 CAN 전달, Panda 또는 콤마 페달 하드웨어 이상일 수 있으므로 강제 출력을 무한 유지하지 않습니다.

## 2. 적용 파일

```text
controls/lib/pedal_force_recovery.py
controls/controlsd.py
car/gm/carcontroller.py
controls/tests/test_pedal_force_recovery.py
```

## 3. ACCEL=0 복구

```text
정상 양수 ACCEL
  ↓
ACCEL=0 발생
  ↓
최대 120ms 동안 직전 정상 ACCEL 유지
  ↓
계속 0이면 ACCEL 최소 0.36m/s²
  ↓
콤마 페달 최소 6%
```

직전 가속값은 최대 `0.36m/s²`로 제한하므로 이전의 강한 가속 명령을 그대로 유지하지 않습니다.

## 4. 양수 페달 무효 Watchdog

### 감지 조건

다음 조건이 동시에 `0.50초` 지속되어야 1단계가 시작됩니다.

```text
원본 ACCEL >= 0.01m/s²
현재 목표 속도 오차 >= 0.30m/s
실제 차량 가속도 aEgo <= -0.05m/s²
복구 공통 안전 조건 모두 통과
```

즉, 가속을 요청하고 목표 속도보다 느린데도 실제 차량이 감속하는 상태만 잡습니다.

### 1단계

```text
최종 ACCEL 최소 0.22m/s²
콤마 페달 최소 3.5%
최대 0.60초 확인
```

작은 양수 페달이 주행 저항을 이기지 못하는 경우를 먼저 부드럽게 보정합니다.

### 2단계

1단계 후에도 차량 반응이 확인되지 않으면 다음으로 상승합니다.

```text
최종 ACCEL 최소 0.36m/s²
콤마 페달 최소 6%
```

### 정상 복귀

실측 `aEgo >= 0.03m/s²`가 `0.25초` 연속 확인되면 강제 하한을 해제하고 원래 PID 출력으로 복귀합니다.

### 전달 이상 판정

6% 단계에서도 `2.0초` 동안 차량 반응이 없으면 다음 가능성이 큽니다.

- CAN 명령 누락
- Panda 전송 문제
- 콤마 페달 enable/counter/checksum 문제
- 콤마 페달 하드웨어가 명령을 적용하지 못함

이 경우 무한 강제 가속을 막기 위해 하한을 해제하고 다음 로그를 남깁니다.

```text
PedalPositiveWatchdog delivery fault suspected
```

그 후 1초 동안 재진입을 막습니다.

## 5. 즉시 취소 조건

다음 조건은 1단계와 2단계를 같은 100Hz 제어 프레임에서 즉시 해제합니다.

- 운전자 브레이크 입력
- 운전자 엑셀 입력
- ACC 해제
- openpilot 비활성
- 종방향 상태가 PID가 아님
- 정차 또는 1km/h 이하
- FCW
- soft disabling 또는 운전자 모니터링 감속
- 플랜 데이터 무효 또는 250ms 초과
- 플랜이 실제 감속을 요구함
- 코너 목표 속도가 현재 속도보다 낮아 실제 코너 감속이 필요함
- 접근 중인 선행차의 거리 또는 TTC가 위험함
- 수동 선행차 catch-up 기능이 복구를 차단함

## 6. 동작 값

| 구분 | 감지 시간 | ACCEL 하한 | 페달 하한 |
|---|---:|---:|---:|
| ACCEL=0 브리지 | 즉시~0.12초 | 직전 양수값, 최대 0.36 | 없음 |
| ACCEL=0 강제 복구 | 0.12초 이후 | 0.36 | 6% |
| 양수 무효 감지 | 0.50초 | 원본 유지 | 원본 유지 |
| 양수 무효 1단계 | 다음 0.60초 | 0.22 | 3.5% |
| 양수 무효 2단계 | 최대 2.0초 | 0.36 | 6% |
| 정상 반응 확인 | 0.25초 | PID로 복귀 | PID로 복귀 |

## 7. 진단 필드

기존 cereal 스키마를 변경하지 않기 위해 기존 진단 채널을 재사용합니다.

| 필드 | 의미 |
|---|---|
| `pedalDeadzoneBoostCandidate` | ACCEL=0 또는 양수 무효 후보 감지 |
| `pedalDeadzoneBoostActive` | 브리지·1단계·2단계 중 하나가 활성 |
| `pedalDeadzoneRawCommand` | 복구 전 원본 ACCEL |
| `pedalDeadzoneAppliedCommand` | 복구 후 최종 ACCEL |
| `pedalDeadzoneFloor` | 현재 적용되는 페달 하한, 0/0.035/0.060 |
| `pedalDeadzoneAccelRequest` | 현재 단계의 ACCEL 하한 |
| `pedalDeadzoneVehicleAccel` | 실측 `aEgo` |
| `pedalForceRecoveryActive` | 강한 6% 단계 활성 여부 |
| `pedalForceRecoveryCount` | 0 복구와 양수 무효 복구 진입 횟수 합계 |

단계 전환 로그:

```text
PedalPositiveWatchdog stage: 0->1
PedalPositiveWatchdog stage: 1->2
PedalPositiveWatchdog stage: 1->0
PedalPositiveWatchdog stage: 2->0
```

## 8. 적용 방법

### 전체 압축 사용

기존 `selfdrive`를 백업한 후 압축 내용을 교체합니다.

### PATCH 사용

`selfdrive` 루트에서 실행합니다.

```bash
cd /data/openpilot/selfdrive
patch -p1 < /data/PEDAL_FORCE_RECOVERY_COMPLETE_WATCHDOG.patch
```

## 9. 문법 및 단위 테스트

openpilot 루트가 `/data/openpilot`이면:

```bash
cd /data/openpilot

python3 -m py_compile \
  selfdrive/controls/lib/pedal_force_recovery.py \
  selfdrive/controls/controlsd.py \
  selfdrive/car/gm/carcontroller.py

python3 -m unittest \
  selfdrive.controls.tests.test_pedal_force_recovery -v
```

총 14개 테스트가 통과해야 합니다.

## 10. 실차 적용 전 시험 순서

1. 정차 상태에서 문법·단위 테스트 실행
2. 페달 CAN 송신값이 0%, 3.5%, 6%로 전환되는지 확인
3. 브레이크 입력 즉시 하한이 0으로 해제되는지 확인
4. 운전자 엑셀 입력 시 즉시 해제되는지 확인
5. 저속의 빈 폐쇄 구간에서 목표 속도를 낮게 설정하여 시험
6. 선행차 접근 상황에서는 복구가 차단되는지 로그 확인
7. 고속도로 시험은 마지막에 실시

고속도로에서 처음 시험할 때는 운전자가 즉시 수동으로 가속과 제동을 인계할 수 있는 상태를 유지해야 합니다.
