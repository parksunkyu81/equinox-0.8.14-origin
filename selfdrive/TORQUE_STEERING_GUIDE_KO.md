# 이쿼녹스 토크 조향 및 속도별 중앙 정렬 사용 안내

## 1. 문서 목적

이 문서는 `CHEVROLET EQUINOX NO RADAR`로 인식되는 2020년형 이쿼녹스의 토크 조향 최적화 기능을 설명합니다.

포함된 기능은 다음과 같습니다.

- 라이브 토크 학습
- 10~40km/h 저속 토크 gain 학습
- 속도별 중앙 정렬 오프셋 학습
- 선택적 좌우 코너 토크 보정
- 선택적 GM 동적 `STEER_DELTA_UP/DOWN`

중앙 정렬 기능은 휠 얼라인먼트나 타이어 문제를 대신하지 않습니다. 오픈파일럿 횡제어가 활성화됐을 때 발생하는 작은 지속 편향을 보정하기 위한 기능입니다.

## 2. 속도별 중앙 정렬 구조

기존의 단일 `latAccelOffset` 대신 다음 세 구간을 독립적으로 학습합니다.

| 속도 구간 | Params | 최대 보정 | 특징 |
|---:|---|---:|---|
| 20~40km/h | `TorqueCenterOffset20_40` | ±0.006 | 가장 엄격하게 학습 |
| 40~60km/h | `TorqueCenterOffset40_60` | ±0.008 | 도심·간선도로용 |
| 60~100km/h | `TorqueCenterOffset60_100` | ±0.010 | 고속 직선용 |

속도별 실제 적용 방식은 다음과 같습니다.

- 10km/h 미만: 중앙 오프셋 미적용
- 10~20km/h: 저속 학습값의 일부만 적용
- 20~40km/h: 저속 오프셋 적용
- 40~60km/h: 중속 오프셋 적용
- 60km/h 이상: 고속 오프셋 100% 적용

속도 경계에서는 선형 보간하므로 40km/h 또는 60km/h에서 값이 갑자기 바뀌지 않습니다. 아직 학습되지 않은 구간은 가장 가까운 유효 구간 값을 임시로 이어받습니다.

## 3. 학습 안전 조건

중앙 오프셋은 다음 조건을 만족하는 직선 데이터만 학습합니다.

- 차량 fingerprint가 `CHEVROLET EQUINOX NO RADAR`
- `IsLiveTorque=1`
- `TorqueCenterOffsetEnabled=1`
- 오픈파일럿 횡제어 활성화
- 운전자가 핸들에 힘을 주지 않음
- 조향 rate limit 또는 `STEER_MAX` 포화가 아님
- 도로 roll 절댓값이 약 1도 이하
- 요레이트, 횡가속, 적용 조향값이 해당 속도 구간의 직선 기준 이하
- 최소 관찰 시간과 승인 샘플 수 충족

20~40km/h 구간은 교차로, 주차 차량 회피, 타이어 스크럽을 잘못 학습하지 않도록 가장 엄격한 기준을 사용합니다.

## 4. 기능 ON 방법

패치를 적용한 뒤 전체 빌드와 재부팅을 먼저 완료합니다. `common/params.cc`가 변경되었기 때문에 Python 파일만 덮어쓰는 방식은 사용할 수 없습니다.

차량을 정차하고 오픈파일럿 횡제어를 해제한 상태에서 SSH로 실행합니다.

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params

p = Params()
p.put("IsLiveTorque", b"1")
p.put("TorqueCenterOffsetEnabled", b"1")

# 중앙 정렬 검증 중에는 다른 보정은 우선 끕니다.
p.put("TorqueDirectionalCompEnabled", b"0")
p.put("DynamicSteerDeltaEnabled", b"0")

print("Live torque and speed-binned center alignment enabled")
PY
```

### 설정 확인

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params

p = Params()
keys = (
  "IsLiveTorque",
  "TorqueCenterOffsetEnabled",
  "TorqueDirectionalCompEnabled",
  "DynamicSteerDeltaEnabled",
  "TorqueCenterOffset20_40",
  "TorqueCenterOffset20_40Count",
  "TorqueCenterOffset40_60",
  "TorqueCenterOffset40_60Count",
  "TorqueCenterOffset60_100",
  "TorqueCenterOffset60_100Count",
)

for key in keys:
  value = p.get(key)
  print(key, value.decode() if value else "없음")
PY
```

처음에는 세 오프셋 값과 Count가 모두 0인 것이 정상입니다. 조건에 맞는 직선 데이터가 충분히 쌓인 뒤 천천히 변경됩니다.

## 5. 권장 학습 주행

- 타이어 공기압과 휠 얼라인먼트를 먼저 확인
- 평탄하고 차선이 명확한 도로 선택
- 같은 도로를 양방향으로 주행
- 강한 측풍이나 큰 도로 경사가 있는 구간 피하기
- 20~40, 40~60, 60~100km/h 구간을 각각 충분히 주행
- 핸들에는 손을 두되 불필요한 조향 토크를 주지 않기

한 방향으로만 주행하면 도로의 우측 경사를 차량 고유 편향으로 잘못 학습할 수 있으므로 양방향 데이터가 중요합니다.

## 6. 학습값 초기화

중앙 정렬을 포함한 전체 라이브 토크 학습을 초기화하려면 다음을 실행합니다.

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params
Params().put("LiveTorqueReset", b"1")
print("Live torque reset requested")
PY
```

이 명령은 기본 `latAccelFactor`, `friction`, 저속 gain, 속도별 중앙 오프셋 학습 상태에 영향을 줄 수 있습니다.

중앙 오프셋 값만 수동으로 초기화하려면 다음을 실행합니다.

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params

p = Params()
for key in (
  "TorqueCenterOffset20_40",
  "TorqueCenterOffset20_40Count",
  "TorqueCenterOffset40_60",
  "TorqueCenterOffset40_60Count",
  "TorqueCenterOffset60_100",
  "TorqueCenterOffset60_100Count",
):
  p.put(key, b"0")

print("Center-offset bins cleared")
PY
```

초기화 후 장치를 재부팅하거나 `torqued` 프로세스를 다시 시작하는 편이 안전합니다.

## 7. 기능 OFF 방법

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params
Params().put("TorqueCenterOffsetEnabled", b"0")
print("Center alignment disabled")
PY
```

OFF 상태에서는 학습된 Params 값이 남아 있어도 실제 토크 조향에는 중앙 오프셋이 적용되지 않습니다.

## 8. 좌우 코너 보정과의 차이

`TorqueCenterOffsetEnabled`는 직선에서 한쪽으로 밀리는 작은 편향을 보정합니다.

`TorqueDirectionalCompEnabled`는 왼쪽 코너와 오른쪽 코너의 추종력 차이를 최대 약 ±3% 범위에서 보정합니다. 중앙 정렬을 검증하는 동안에는 `0`으로 두는 것이 권장됩니다.

## 9. 동적 delta 사용

중앙 정렬 학습과 `DynamicSteerDeltaEnabled`는 서로 다른 기능입니다.

- 중앙 정렬: 직선 편향 보정
- 동적 delta: 토크가 목표값까지 올라가는 속도 조절

중앙 정렬이 안정된 뒤에만 동적 delta를 별도로 검증합니다. 첫 단계 권장값은 다음과 같습니다.

```bash
cd /data/openpilot

python3 - <<'PY'
from common.params import Params

p = Params()
p.put("DynamicSteerDeltaEnabled", b"1")
p.put("DynamicSteerDeltaMaxUp", b"9")
p.put("DynamicSteerDeltaMaxDown", b"17")
print("Dynamic steer delta enabled with MaxUp=9")
PY
```

Panda safety와 GM EPS 동작을 확인하지 않은 상태에서 바로 11 또는 12로 올리지 마십시오.

## 10. 문제 발생 시

다음 증상이 발생하면 중앙 정렬 기능을 즉시 OFF하고 학습값을 초기화합니다.

- 직선에서 한쪽으로 더 강하게 끌림
- 중앙 주변에서 좌우로 반복 흔들림
- 반대 방향 도로에서 편향이 심해짐
- 운전자 개입이 자주 필요함
- EPS temporary/permanent fault 발생

오픈파일럿을 끈 상태에서도 차량이 쏠린다면 먼저 타이어, 얼라인먼트, 핸들 물리 센터, 카메라 장착 상태를 점검해야 합니다.
