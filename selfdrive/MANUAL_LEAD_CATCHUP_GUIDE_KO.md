# 수동 출발 후 앞차 거리 기반 가속 부스트 안내

## 1. 기능 목적

이 기능은 정차 중인 차량을 자동으로 출발시키지 않습니다.

반드시 운전자가 먼저 가속페달을 밟아 차량을 1km/h 이상 움직여야 합니다. 운전자가 가속페달에서 발을 뗀 뒤에만, 앞차와의 거리 및 상대속도를 계산하여 콤마 페달 제어로 빠르게 인계합니다. 페달 해제 순간 레이더 값이 아직 갱신되지 않은 경우 최대 0.8초 동안 기다린 뒤 앞차 출발을 확인합니다.

```text
앞차 출발
→ 운전자가 직접 가속페달 입력
→ 차량속도 1km/h 초과
→ 운전자가 가속페달 해제
→ 안전 조건 확인
→ 거리 기반 부스트 적용
→ 적정 추종거리 도달
→ 일반 MPC/PID 제어로 자동 인계
```

`car/gm/carcontroller.py`의 기존 제한은 변경하지 않았습니다.

```python
pedal_speed_allowed = CS.out.vEgo > V_CRUISE_ENABLE_MIN / CV.MS_TO_KPH
```

따라서 차량이 1km/h 이하이거나 운전자가 직접 출발하지 않은 상태에서는 콤마 페달 명령이 나오지 않습니다.

---

## 2. 부스트 자동 계산

부스트 지속시간은 고정된 1초가 아닙니다. 현재 앞차 거리에서 MPC가 사용하는 안전거리를 뺀 값을 매 제어 주기마다 계산합니다.

```text
안전거리 = 제동거리 + 현재 TR × 내 차 속도 + 정지거리
거리 여유 = 현재 앞차 거리 - 안전거리
```

현재 소스 기준값:

```text
COMFORT_BRAKE = 2.5m/s²
STOP_DISTANCE = 5.5m
```

거리 여유가 크고 앞차가 빠르게 멀어지면 부스트가 커집니다. 적정 추종거리에 가까워지면 부스트가 자동으로 줄어듭니다.

| 안전거리보다 남는 거리 | 기본 가속 하한 |
|---:|---:|
| 0.5m 이하 | 0.00m/s² |
| 1.0m | 0.15m/s² |
| 2.0m | 0.25m/s² |
| 3.5m | 0.36m/s² |
| 6.0m | 0.50m/s² |
| 10m 이상 | 0.65m/s² |

앞차가 내 차보다 빠르면 최대 0.15m/s²가 추가될 수 있습니다. 최종 가속 상한은 `ManualLeadCatchupMaxAccel`로 제한됩니다.

저속 급출발을 막기 위해 속도별 상한도 적용됩니다.

| 내 차 속도 | 부스트 가속 상한 |
|---:|---:|
| 1km/h | 0.40m/s² |
| 3km/h | 0.50m/s² |
| 5km/h | 0.58m/s² |
| 10km/h | 0.68m/s² |
| 20km/h 이상 | 0.70m/s² |

---

## 3. 활성화 조건

다음 조건이 모두 맞아야 기능이 시작됩니다.

- 오픈파일럿 및 ACC 활성
- 유효하고 최신인 종방향 계획
- 유효한 앞차
- 운전자가 저속에서 직접 가속페달 입력
- 운전자가 가속페달에서 발을 뗌
- 차량속도 1km/h 초과
- 앞차 움직임이 명확하면 해제 프레임에 즉시 인계
- 레이더 갱신이 늦으면 최대 0.8초 동안 대기
- 약한 출발은 앞차 속도 0.45m/s 이상, 상대속도 0.15m/s 이상을 0.10초 확인
- 현재 앞차 거리가 안전거리의 80% 이상
- 브레이크 미입력
- FCW 없음
- 급커브가 아님
- 조향각 절댓값 15도 이하

운전자의 가속페달 입력이 없으면 앞차가 출발하더라도 기능은 절대 활성화되지 않습니다.

---

## 4. 자동 종료와 즉시 취소

### 정상 종료

다음 조건이 0.3초 이상 유지되면 부스트를 부드럽게 0으로 낮추고 일반 MPC/PID로 넘깁니다.

```text
안전거리보다 남는 거리 ≤ 0.8m
앞차와 속도 차이 ≤ 0.3m/s
```

실제 부스트 지속시간은 상황에 따라 약 0.5~6초 사이에서 자동 결정됩니다. 6초는 오작동 방지용 최종 타임아웃입니다.

### 즉시 취소

다음 조건에서는 부스트를 즉시 취소합니다.

- 운전자 브레이크 입력
- 운전자 가속페달 재입력
- 차량속도 1km/h 이하
- 차량속도 30km/h 초과
- ACC 또는 오픈파일럿 해제
- 앞차 소실 또는 정지
- 유효하지 않거나 오래된 종방향 계획
- FCW 발생
- MPC 가속 요청이 -0.05m/s² 미만
- 현재 거리가 안전거리의 75% 미만
- 절대 앞차 거리 3.5m 미만
- 앞차에 접근 중이며 상대속도가 -0.2m/s 미만
- TTC 4초 미만
- 급커브 또는 조향각 15도 초과

안전 취소 직후에는 기존 `PedalForceRecovery`가 0.5초 동안 부스트를 다시 덮어쓰지 못하도록 차단합니다.

---

## 5. 기능 ON

`common/params.cc`가 변경됐으므로 전체 빌드와 재부팅을 먼저 완료해야 합니다.

차량을 정차하고 오픈파일럿이 해제된 상태에서 SSH로 실행합니다.

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params

p = Params()
p.put("ManualLeadCatchupEnabled", b"1")
p.put("ManualLeadCatchupMaxAccel", b"0.70")

print("Manual lead catch-up enabled")
PY
```

처음 실차 검증은 더 보수적인 `0.50m/s²`로 시작하는 것을 권장합니다.

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params

p = Params()
p.put("ManualLeadCatchupEnabled", b"1")
p.put("ManualLeadCatchupMaxAccel", b"0.50")
PY
```

검증 순서:

```text
0.50 → 0.60 → 최대 0.70m/s²
```

허용 범위는 0.40~0.70m/s²이며, 범위를 벗어난 값은 자동으로 제한됩니다.

---

## 6. 설정 확인

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params

p = Params()
for key in ("ManualLeadCatchupEnabled", "ManualLeadCatchupMaxAccel"):
  value = p.get(key)
  print(key, value.decode() if value else "없음")
PY
```

예상 출력:

```text
ManualLeadCatchupEnabled 1
ManualLeadCatchupMaxAccel 0.50
```

---

## 7. 기능 OFF

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params

p = Params()
p.put("ManualLeadCatchupEnabled", b"0")

print("Manual lead catch-up disabled")
PY
```

기능은 학습값을 저장하지 않으므로 별도의 초기화는 필요하지 않습니다.

---

## 8. 변경 파일

```text
controls/lib/manual_lead_catchup.py   신규
controls/lib/longcontrol.py
controls/controlsd.py
common/params.cc
manager/manager.py
MANUAL_LEAD_CATCHUP_GUIDE_KO.md       신규
```

`car/gm/carcontroller.py`의 정차 자동출발 방지와 1km/h 제한은 변경하지 않았습니다.

---

## 9. 실차 검증 권장 순서

1. `ManualLeadCatchupMaxAccel=0.50`으로 시작
2. 평탄하고 차량 통행이 없는 안전한 구간에서 시험
3. 운전자가 직접 1~3km/h까지 출발시킨 뒤 가속페달 해제
4. 앞차 거리가 클 때 부스트가 증가하는지 확인
5. 적정 추종거리에 가까워지면 부스트가 줄어드는지 확인
6. 브레이크 입력 시 즉시 해제되는지 확인
7. 앞차가 재정지하거나 가까워질 때 즉시 해제되는지 확인
8. 안정적일 때만 0.60, 이후 최대 0.70으로 단계 조정

실제 차량, 콤마 페달, Panda safety 및 레이더 노이즈는 이 정적 분석 환경에서 검증할 수 없습니다. 첫 시험은 반드시 즉시 브레이크를 밟을 수 있는 조건에서 수행해야 합니다.
