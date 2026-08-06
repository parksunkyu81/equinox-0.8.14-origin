# 콤마 페달 ACCEL=0 엄격 복구 모드

## 변경 이유

기존 복구는 정상적인 타력 주행의 `ACCEL=0`과 실제 제어 고착을 충분히 구분하지 못해 불필요한 페달 보정이 발생할 수 있었습니다.

최종 버전은 다음 원칙으로 변경했습니다.

- 기본값은 `OFF`
- 양수 페달 무효 자동 부스트는 완전히 제거
- `ACCEL=0`이어도 확인 단계에서는 페달을 전혀 올리지 않음
- 여러 증거가 동시에 확인된 실제 고착 후보에만 1회 제한 복구
- 브레이크, 운전자 엑셀, 코너, 선행차, 감속 플랜은 즉시 차단

## 엄격 복구 조건

다음 조건이 모두 만족돼야 합니다.

1. 직전에 `ACCEL >= 0.08m/s²`가 0.3초 이상 지속
2. 이후 `ACCEL=0`이 0.8초 이상 지속
3. 이 0.8초 동안 측정 속도가 최소 `0.20m/s` 감소
4. 목표 속도 오차가 최소 `0.12m/s` 더 커짐
5. 측정 가속도 `aEgo <= -0.06m/s²`인 프레임이 75% 이상
6. 현재 속도 오차가 `0.55m/s` 이상
7. 미래 플랜 속도 오차도 `0.35m/s` 이상
8. 직선 주행
9. 가까운 선행차 또는 접근 중인 선행차 없음
10. 브레이크·운전자 엑셀·FCW·감속 플랜 없음

0.8~1.2초 안에 모든 조건을 만족하지 못하면 후보를 폐기합니다. 일반적인 긴 타력 주행이 나중에 복구로 바뀌지 않습니다.

## 복구 출력

고착이 확정된 경우에만:

- `ACCEL=0.36m/s²`
- 콤마 페달 최소 6%
- 최대 0.6초
- 이후 5초 동안 재동작 금지

차량이 반응하거나 정상 PID 가속 명령이 돌아오면 즉시 종료합니다.

## 기본 상태

```text
PedalForceRecoveryEnabled=0
```

즉, 소스를 설치한 직후에는 복구 기능이 완전히 비활성화됩니다.

## 기능 켜기

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params
Params().put("PedalForceRecoveryEnabled", b"1")
print("strict zero recovery enabled")
PY
```

약 1초 안에 실행 중인 `controlsd`에 반영됩니다.

## 기능 완전히 끄기

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params
Params().put("PedalForceRecoveryEnabled", b"0")
print("pedal recovery disabled")
PY
```

이 값이 `0`이면 원래 PID 가속 명령을 그대로 통과시키며 강제 페달 출력이 발생하지 않습니다.

## 상태 확인

```bash
cd /data/openpilot
python3 selfdrive/debug/pedal_recovery_monitor.py
```

또는:

```bash
python3 - <<'PY'
from common.params import Params
p = Params()
print("PedalForceRecoveryEnabled", (p.get("PedalForceRecoveryEnabled") or b"0").decode())
PY
```

## 권장 사용

과잉 개입을 이미 경험했다면 우선 `0`으로 끄고 운행하십시오. 로그에서 동일한 `ACCEL=0` 고착이 반복적으로 확인된 뒤에만 안전한 시험 구간에서 `1`로 켜는 것을 권장합니다.
