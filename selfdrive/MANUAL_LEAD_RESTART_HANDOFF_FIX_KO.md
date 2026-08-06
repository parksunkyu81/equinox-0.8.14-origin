# 앞차 재출발 후 수동 가속 인계 지연 수정

## 원인

기존 `ManualLeadCatchup`은 운전자가 가속페달을 놓는 단 한 프레임에서 아래 조건을 모두 만족해야 했습니다.

- 앞차 속도 `0.8m/s` 이상
- 상대속도 `0.3m/s` 이상
- 차량속도 `1km/h` 초과
- 안전거리 조건 만족

GM 레이더는 약 15Hz로 갱신되므로, 운전자가 페달을 놓는 순간 레이더가 아직 이전 정지 속도를 보내면 `handoff_gate`로 즉시 취소됐습니다. 이후 앞차 속도가 정상으로 갱신돼도 가속 인계는 다시 시도되지 않아 일반 MPC 시작 조건을 기다리면서 지연이 발생했습니다.

또한 인계가 성공해도 가속 하한이 `0`부터 `0.8m/s³`로 올라가서 `0.36m/s²`에 도달하는 데 약 0.45초가 필요했습니다.

## 수정 동작

```text
앞차 정지
→ 운전자가 직접 가속페달 입력
→ 내 차가 1km/h 초과
→ 운전자가 가속페달 해제
→ 앞차 움직임이 명확하면 같은 제어 주기에 인계
→ 레이더가 늦으면 최대 0.8초 대기
→ 약한 움직임은 0.1초 확인 후 인계
→ 거리 기반 가속 하한 적용
```

### 즉시 인계

페달 해제 프레임에 다음 값이 이미 확인되면 같은 주기에 인계합니다.

- 앞차 속도 `0.8m/s` 이상
- 상대속도 `0.3m/s` 이상
- 안전거리 조건 만족

### 레이더 지연 보정

해제 프레임에서 앞차 속도가 아직 낮아도 즉시 취소하지 않고 최대 `0.8초` 동안 기다립니다.

- 앞차 속도 `0.45m/s` 이상
- 상대속도 `0.15m/s` 이상
- 위 상태 `0.10초` 확인

조건이 확인되면 가속 인계가 시작됩니다.

## 자동 출발 방지 유지

이번 수정은 자동 출발 기능이 아닙니다.

- 운전자의 실제 가속페달 입력이 반드시 필요
- 운전자가 차량을 `1km/h` 이상 움직여야 함
- 콤마 페달의 기존 `1km/h` 출력 제한 유지
- 운전자가 페달을 놓은 뒤에만 인계 가능
- 아무런 운전자 입력 없이 앞차만 출발하면 절대 활성화되지 않음

## 초기 응답 개선

인계 직후 가속 하한을 최대 `0.12m/s²`로 제한해 시작하고, 상승 jerk를 `1.6m/s³`로 조정했습니다.

이는 큰 급가속을 넣는 것이 아니라, 기존처럼 약 0.4~0.5초 동안 거의 0에 가까운 명령이 유지되는 지연만 줄이는 변경입니다. 최종 가속값은 계속 다음 제한을 받습니다.

- `ManualLeadCatchupMaxAccel`
- 속도별 가속 상한
- 앞차 거리 및 상대속도
- MPC 감속 요청
- TTC 및 안전거리

## 즉시 취소 조건

다음 조건은 기존과 동일하게 같은 제어 주기에서 취소합니다.

- 브레이크 입력
- 운전자 가속페달 재입력
- 차량속도 `1km/h` 이하
- ACC 또는 openpilot 해제
- 앞차 소실 또는 재정지
- FCW
- 감속 MPC 요청
- 안전거리 부족
- TTC 4초 미만
- 앞차에 접근 중
- 급커브 또는 큰 조향각

## 기능 활성화

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params

p = Params()
p.put("ManualLeadCatchupEnabled", b"1")
p.put("ManualLeadCatchupMaxAccel", b"0.50")

for key in ("ManualLeadCatchupEnabled", "ManualLeadCatchupMaxAccel"):
  value = p.get(key)
  print(key, value.decode() if value else "없음")
PY
```

첫 시험은 `0.50m/s²`로 시작하고 정상 동작을 확인한 뒤 필요할 때만 올리는 것을 권장합니다.

## 검증

```bash
cd /data/openpilot

python3 -m py_compile \
  selfdrive/controls/lib/manual_lead_catchup.py \
  selfdrive/controls/lib/longcontrol.py \
  selfdrive/controls/controlsd.py \
  selfdrive/car/gm/carcontroller.py

python3 -m unittest \
  selfdrive.controls.tests.test_manual_lead_catchup_restart \
  selfdrive.controls.tests.test_pedal_force_recovery_strict -v
```

정상 결과는 총 16개 테스트 `OK`입니다.
