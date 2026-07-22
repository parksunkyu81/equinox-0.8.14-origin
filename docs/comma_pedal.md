# 콤마 페달 종방향 플래너 지침서

이 문서는 첨부된 `longitudinal_planner.py`를 기준으로 콤마 페달 차량의 종방향 가속도 제한과 MPC 전달 과정을 설명한다. 첨부 소스에 없는 최종 CAN 페달 변환, PID 하한, 제동 제어 동작은 이 문서의 범위에 포함하지 않는다.

## 핵심 동작

플래너는 현재 차량 속도로 최소·최대 가속도 제한을 계산하고, 코너·운전자 주의 저하·직전 가속도 조건을 반영한 뒤 MPC에 전달한다.

```text
현재 속도
  → 기본 가속도 범위 계산
  → 코너에서 최대 가속도 제한
  → 주의 저하 감속 적용
  → 직전 가속도와의 급격한 변화 제한
  → 저속 감속 상한 보정
  → MPC에 최소·최대 가속도 전달
```

여기서 최소·최대 가속도는 강제로 실행하는 명령이 아니라 MPC가 계획할 수 있는 범위다.

## 기본 가속도 배열

첨부 소스에는 다음 네 배열이 정의되어 있다.

```python
_A_CRUISE_MIN_V_FOLLOWING = [-1.5, -1.5, -1.2, -1.0, -0.8]
_A_CRUISE_MIN_V = [-0.8, -1.0, -0.8, -0.5, -0.3]

_A_CRUISE_MAX_V_FOLLOWING = [1.0, 0.9, 0.7, 0.5, 0.4]
_A_CRUISE_MAX_V = [0.8, 0.7, 0.6, 0.5, 0.4]

_A_CRUISE_MIN_BP = [0.0, 15.0, 30.0, 55.0, 85.0]
_A_CRUISE_MAX_BP = _A_CRUISE_MIN_BP
```

하지만 실제 `calc_cruise_accel_limits()`는 항상 `FOLLOWING` 배열만 사용한다.

```python
def calc_cruise_accel_limits(v_ego):
  a_cruise_min = interp(v_ego, _A_CRUISE_MIN_BP,
                        _A_CRUISE_MIN_V_FOLLOWING)
  a_cruise_max = interp(v_ego, _A_CRUISE_MAX_BP,
                        _A_CRUISE_MAX_V_FOLLOWING)
  return [a_cruise_min, a_cruise_max]
```

따라서 첨부 소스 기준으로는 다음과 같다.

- 앞차 유무에 따른 프로파일 선택이 없다.
- 앞차가 없어도 `FOLLOWING` 최소·최대 배열을 사용한다.
- `_A_CRUISE_MIN_V`와 `_A_CRUISE_MAX_V`는 정의만 되어 있고 실제 계산에는 사용되지 않는다.
- 플래너 내부에서 `leadOne.status`는 가속도 배열 선택에 사용되지 않고 `longitudinalPlan.hasLead` 발행에만 직접 사용된다. 다만 전체 `radarState`는 MPC에 전달되므로 MPC의 앞차 계획에는 계속 반영된다.
- 앞차 감지 확인 시간이나 프로파일 전환 램프도 없다.

## 속도 단위 주의

`Planner.update()`의 `v_ego`는 `carState.vEgo`이며 단위는 `m/s`다. 첨부 소스는 `calc_cruise_accel_limits(v_ego)`를 호출할 때 `km/h`로 변환하지 않는다.

따라서 기준점은 다음과 같이 해석된다.

| 기준점 | 실제 단위 | 환산 속도 |
|---:|---:|---:|
| 0 | 0 m/s | 0 km/h |
| 15 | 15 m/s | 54 km/h |
| 30 | 30 m/s | 108 km/h |
| 55 | 55 m/s | 198 km/h |
| 85 | 85 m/s | 306 km/h |

기준점을 `0, 15, 30, 55, 85 km/h`로 의도했다면 현재 구현과 단위가 일치하지 않는다. km/h 기준으로 사용하려면 호출 전에 `v_ego * CV.MS_TO_KPH`로 변환하거나 기준점 배열을 m/s 값으로 바꿔야 한다.

## 실제 속도별 기본 가속도 범위

현재 구현처럼 기준점을 m/s로 적용했을 때의 선형 보간 결과다.

| 차량 속도 | 최소 가속도 | 최대 가속도 |
|---:|---:|---:|
| 0 km/h | -1.500 m/s² | 1.000 m/s² |
| 10 km/h | -1.500 m/s² | 0.981 m/s² |
| 20 km/h | -1.500 m/s² | 0.963 m/s² |
| 30 km/h | -1.500 m/s² | 0.944 m/s² |
| 40 km/h | -1.500 m/s² | 0.926 m/s² |
| 50 km/h | -1.500 m/s² | 0.907 m/s² |
| 60 km/h | -1.467 m/s² | 0.878 m/s² |
| 70 km/h | -1.411 m/s² | 0.841 m/s² |
| 80 km/h | -1.356 m/s² | 0.804 m/s² |
| 90 km/h | -1.300 m/s² | 0.767 m/s² |
| 100 km/h | -1.244 m/s² | 0.730 m/s² |

일반적인 주행 속도에서는 최소 가속도가 오랫동안 `-1.5 m/s²` 부근에 머문다. 최소값이 음수라는 것은 MPC가 해당 수준까지 감속 계획을 만들 수 있다는 뜻이며, 실제 차량이 반드시 그 감속도를 실행한다는 뜻은 아니다.

## 코너 가속도 제한

코너에서는 종가속도와 횡가속도의 합이 속도별 총 가속도 한도를 넘지 않도록 최대 종가속도를 제한한다.

```python
_A_TOTAL_MAX_BP = [0.0, 25.0, 55.0]
_A_TOTAL_MAX_V = [2.5, 3.0, 4.0]

a_y = v_ego ** 2 * angle_steers * DEG_TO_RAD / (steerRatio * wheelbase)
a_x_allowed = sqrt(max(a_total_max ** 2 - a_y ** 2, 0.0))
```

최종 코너 제한은 다음과 같다.

```python
[기본 최소 가속도, min(기본 최대 가속도, 허용 종가속도)]
```

따라서 코너 제한은 최대 가속도만 낮추고 최소 감속 한도는 바꾸지 않는다. 조향각과 속도가 커질수록 횡가속도가 증가하므로 사용할 수 있는 종방향 가속도 여유가 줄어든다.

이 기준점에도 `v_ego`가 변환 없이 들어가므로 `_A_TOTAL_MAX_BP`의 단위 역시 m/s다.

## 운전자 주의 저하 감속

`controlsState.forceDecel`이 참이면 최대 가속도를 `AWARENESS_DECEL` 이하로 제한한다.

```python
AWARENESS_DECEL = -0.2

accel_max = min(accel_max, -0.2)
accel_min = min(accel_min, accel_max)
```

이 단계에서는 최대값도 음수가 되어 MPC가 부드러운 감속 계획을 만들도록 유도한다. 다만 바로 다음의 직전 가속도 연속성 보정이 우선되면 최종 최대값이 즉시 `-0.2 m/s²`까지 내려가지 않고 프레임마다 점진적으로 감소할 수 있다. 또한 `0.5 m/s` 미만에서는 마지막 저속 보정으로 최대값이 `-0.1 m/s²`까지 완화될 수 있다.

## 직전 가속도와의 연속성

플래너는 가속도 범위가 한 프레임에 급격히 바뀌지 않도록 직전 목표 가속도 `a_desired`를 기준으로 범위를 보정한다.

```python
accel_min = min(accel_min, a_desired + 0.05)
accel_max = max(accel_max, a_desired - 0.05)
```

이 처리는 새 제한이 현재 목표 가속도에서 너무 멀리 떨어져 MPC가 불연속적으로 움직이는 것을 줄인다. 결과적으로 기본 프로파일보다 직전 가속도 연속성이 우선될 수 있다.

## 극저속 감속 보정

차량 속도가 `0.5 m/s` 미만이면 최대 가속도를 최소 `-0.1 m/s²`까지 올린다.

```python
if v_ego < 0.5:
  accel_max = max(accel_max, AWARENESS_DECEL / 2)
```

일반 주행에서는 최대 가속도가 이미 양수이므로 변화가 없다. 주로 `forceDecel`로 최대값이 `-0.2 m/s²`가 된 상태에서 극저속 감속 요구를 `-0.1 m/s²`로 완화하는 역할을 한다.

## MPC 전달과 궤적 생성

보정된 범위는 다음 호출로 MPC에 전달된다.

```python
self.mpc.set_accel_limits(accel_min, accel_max)
self.mpc.set_cur_state(filtered_speed, a_desired)
```

MPC는 설정 속도, 레이더 앞차, 모델 위치·속도·가속도와 이전 가속도 연속성 조건을 함께 사용해 속도·가속도·저크 궤적을 계산한다.

모델 궤적 배열이 각각 33개이면 MPC 시간축에 맞게 보간하고, 그렇지 않으면 0으로 채운 배열을 전달한다.

계산 결과는 제어 시간축으로 다시 보간된다.

```python
v_desired_trajectory
a_desired_trajectory
j_desired_trajectory
```

다음 프레임의 `a_desired`는 `DT_MDL` 시점의 가속도 궤적에서 가져오며, 필터 속도는 이전·현재 가속도의 사다리꼴 적분으로 갱신한다.

## 초기화 조건

리셋 조건은 차량의 종방향 제어 방식에 따라 다르다.

- `openpilotLongitudinalControl` 차량: `longControlState == off`
- 그 외 차량: `controlsState.enabled == false`

리셋되면 필터 속도를 실제 차량 속도로 맞추고 목표 가속도를 `0`으로 초기화한다. 정차 상태에서는 이전 가속도 연속성 조건을 사용하지 않는다.

## 발행 데이터

첨부 소스는 `longitudinalPlan`에 다음 주요 데이터를 발행한다.

- `speeds`: 목표 속도 궤적
- `accels`: 목표 가속도 궤적
- `jerks`: 목표 저크 궤적
- `hasLead`: `radarState.leadOne.status`
- `longitudinalPlanSource`: MPC가 선택한 계획 소스
- `fcw`: MPC 충돌 카운터가 5를 초과했는지 여부

첨부 소스에는 `following`, `accelProfileFactor`, `accelLimitMax`를 발행하는 코드가 없다. 해당 필드를 이용한 진단 절차는 이 버전에 적용할 수 없다.

## 운전 중 예상되는 체감

- 앞차 유무와 관계없이 같은 `FOLLOWING` 가속도 범위를 사용한다.
- 일반적인 도심 속도에서는 최소 가속도가 약 `-1.5 m/s²`로 유지되어 MPC의 감속 계획 여유가 크다.
- 속도가 올라갈수록 최대 가속도가 점차 낮아져 가속이 완만해진다.
- 코너에서는 횡가속도가 커질수록 추가 가속이 제한된다.
- 운전자 주의 저하 감속이 활성화되면 최대 가속도까지 음수가 되어 감속 계획이 만들어진다.
- 첨부 소스만으로는 음수 가속도 계획이 실제 브레이크, 회생제동 또는 단순 가스 컷 중 무엇으로 실행되는지 판단할 수 없다.

## 진단 체크리스트

- `calc_cruise_accel_limits()` 입력 `v_ego`가 m/s인지 확인한다.
- 기준점이 의도한 단위와 실제 입력 단위가 일치하는지 확인한다.
- 앞차가 없어도 `FOLLOWING` 배열이 적용되는 현재 동작이 의도된 것인지 확인한다.
- `_A_CRUISE_MIN_V`와 `_A_CRUISE_MAX_V`가 사용되지 않는 것이 의도된 것인지 확인한다.
- 직선과 코너에서 최대 가속도 제한이 예상대로 달라지는지 기록한다.
- `forceDecel` 활성화 시 기본 최대 가속도가 `-0.2 m/s²`로 제한되고, 직전 가속도 연속성 보정에 따라 점진적으로 내려가는지 확인한다.
- `0.5 m/s` 미만에서 감속 상한이 `-0.1 m/s²`로 완화되는지 확인한다.
- MPC 목표 가속도와 최종 액추에이터 명령을 함께 기록해 계획과 실제 차량 동작을 구분한다.

## 안전 주의

가속도 제한값은 MPC 계획 범위이며 실제 제동 성능을 보장하지 않는다. 콤마 페달 장치가 가속만 제어하는 차량에서는 음수 계획이 실제 브레이크로 이어지지 않을 수 있다. 실차 적용 전 폐쇄된 환경에서 단계적으로 검증하고 운전자는 항상 직접 제동할 준비를 해야 한다.
