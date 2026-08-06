# TorqueCenterOffset 단계형 학습 안내

## 목적

직선 중앙 정렬 오프셋을 처음부터 40~60초 동안 기다리지 않고 빠르게 생성하되,
이후에는 더 긴 새 데이터로 정밀 보정합니다.

`Count`는 원시 샘플 수가 아니라 해당 속도 구간의 오프셋이 승인되어 Params에 저장된 횟수입니다.

## 단계별 기준

### 최초 학습 (`Count == 0`)

| 속도 구간 | 최소 관찰 시간 | 최소 승인 샘플 | 최소 승인 비율 | 1회 최대 변경량 |
|---|---:|---:|---:|---:|
| 20~40km/h | 20초 | 350 | 82% | 0.00010 |
| 40~60km/h | 18초 | 320 | 82% | 0.00015 |
| 60~100km/h | 20초 | 350 | 82% | 0.00020 |

최초 학습은 빠르지만 변경량을 작게 제한합니다.

### 정밀 학습 (`Count >= 1`)

| 속도 구간 | 최소 관찰 시간 | 최소 승인 샘플 | 최소 승인 비율 | 1회 최대 변경량 |
|---|---:|---:|---:|---:|
| 20~40km/h | 40초 | 700 | 90% | 0.00015 |
| 40~60km/h | 45초 | 800 | 90% | 0.00025 |
| 60~100km/h | 60초 | 1000 | 90% | 0.00050 |

첫 저장에 사용한 샘플은 즉시 삭제합니다. 따라서 두 번째 이후 `Count` 증가는
첫 학습 데이터를 재사용하지 않고 새로 수집한 40~60초 데이터로만 승인됩니다.

## 유지되는 안전 조건

다음 조건은 시간 단축과 관계없이 완화하지 않았습니다.

- 이쿼녹스 전용 fingerprint 일치
- 횡제어 활성
- 운전자 조향 개입 없음
- 롤 1도 이내
- 속도 구간별 yaw rate, 횡가속, 조향 출력 제한
- steer clip 및 max-limit 차단
- 강한 rate-limit 차단

GM 직선 미세 보정에서 발생하는 약한 rate-limit만 다음 조건으로 허용합니다.

- strong rate-limit이 아님
- `abs(desired - applied) <= 0.02`

## 실시간 확인

```bash
cd /data/openpilot
python3 selfdrive/debug/center_offset_monitor.py
```

예시:

```text
82.3 km/h bin=60_100 phase=bootstrap ok=287/332 ratio=0.86/0.82 obs=17.1/20s need_ok=350 savedCount=0 block=PASS
```

첫 저장 후에는 자동으로 다음처럼 바뀝니다.

```text
82.1 km/h bin=60_100 phase=refine ok=125/138 ratio=0.91/0.90 obs=6.9/60s need_ok=1000 savedCount=1 block=PASS
```

## 설정 확인

```bash
cd /data/openpilot
python3 - <<'PY'
from common.params import Params

p = Params()
for key in (
  "IsLiveTorque",
  "TorqueCenterOffsetEnabled",
  "TorqueCenterOffset20_40",
  "TorqueCenterOffset20_40Count",
  "TorqueCenterOffset40_60",
  "TorqueCenterOffset40_60Count",
  "TorqueCenterOffset60_100",
  "TorqueCenterOffset60_100Count",
):
  raw = p.get(key)
  print(key, raw.decode() if raw else "없음")
PY
```

## 학습값 초기화

처음부터 다시 학습할 때만 실행합니다.

```bash
cd /data/openpilot
python3 - <<'PY'
from common.params import Params

p = Params()
for suffix in ("20_40", "40_60", "60_100"):
  p.put(f"TorqueCenterOffset{suffix}", b"0.0")
  p.put(f"TorqueCenterOffset{suffix}Count", b"0")
print("TorqueCenterOffset staged learning reset")
PY
```

초기화 후 재부팅하면 각 구간은 다시 `bootstrap` 단계부터 시작합니다.
