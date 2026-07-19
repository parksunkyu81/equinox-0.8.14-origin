# Torque Steer 지침서

이 문서는 현재 Equinox 소스의 **토크 기반 횡조향 제어**를 간단히 정리한 지침서입니다. 실제 차량 적용 전에는 반드시 로그와 폐쇄된 시험 환경에서 확인해야 합니다.

## 1. 제어 흐름

현재 조향 명령은 다음 순서로 만들어집니다.

1. 주행 경로에서 목표 곡률을 계산합니다.
2. 목표 곡률을 목표 횡가속도로 변환합니다.
3. 실제 횡가속도와의 오차를 PID, 피드포워드, 마찰 보상으로 토크 명령에 반영합니다.
4. 저속·고속 곡률 제한과 토크 변화율 제한을 적용합니다.
5. 운전자 조향 토크를 포함한 GM 표준 토크 제한을 적용합니다.
6. 최종 조향 명령을 50 Hz로 전송합니다.

## 2. 현재 주요 설정

| 항목 | 현재 값 또는 동작 |
|---|---|
| 최대 조향 토크 | `300` |
| 조향 명령 전송 | `50 Hz` (`20 ms` 주기) |
| 토크 증가 제한 | 전송 1회당 `7` |
| 토크 감소 제한 | 전송 1회당 `17` |
| 최소 조향 속도 | `10 km/h` |
| 고속 곡률 보호 시작 | `40 km/h` |
| 강한 운전자 개입 기준 | 운전자 토크 절댓값 `30` 이상 |
| 방향별 토크 보정 | 현재 비활성화 |

최종 CAN 전송 단계의 토크 증가·감소 제한은 속도별 동적 값이 아니라 **고정 7/17**입니다. 속도에 따른 조향감 변화는 주로 목표 곡률 제한, 토크 슬루 제한, 유효 `latAccelFactor`와 `friction` 보정에서 만들어집니다.

## 3. 속도별 조향 특성

- **10~35 km/h:** 저속 코너 진입과 회전이 둔하지 않도록 조향 응답을 보강합니다.
- **35~45 km/h:** 저속 보강에서 일반 조향으로 부드럽게 연결합니다.
- **45~60 km/h:** 중속 조향 응답을 안정적으로 이어갑니다.
- **60 km/h 이상:** 급격한 조향보다 안정성과 복원감을 우선합니다.
- **40 km/h 이상:** 목표 곡률의 크기와 변화율을 추가로 제한해 급격한 조향을 억제합니다.

## 4. 운전자 개입과 안전 동작

- 운전자가 핸들을 잡고 조향하면 적분기를 멈춰 토크가 계속 누적되지 않게 합니다.
- 강한 운전자 토크가 감지되면 Equinox 동적 조향 보강을 해제합니다.
- 운전자 토크는 최종 GM 토크 제한 계산에 항상 반영됩니다.
- 조향 비활성, 조향 오류 또는 최소 조향 속도 미만이면 조향 명령은 `0`이 됩니다.
- 토크 제한 여부는 계산값이 아니라 실제 전송된 요청 토크와 적용 토크를 기준으로 판단합니다.
- 명령 전송 간격이 비정상적으로 길어지면 gap fault 로그를 남깁니다.

이 기능은 운전자를 대신하는 자동운전 기능이 아닙니다. 운전자는 항상 조향을 즉시 인계할 수 있어야 합니다.

## 5. Live Torque 튜닝

### 역할

Live Torque는 실제 주행에서 조향 토크와 횡가속도의 관계를 학습해 다음 세 값을 보정합니다.

| 파라미터 | 역할 | 값이 변할 때의 영향 |
|---|---|---|
| `latAccelFactor` | 횡가속도를 조향 토크로 변환하는 비율 | 작아지면 같은 목표 횡가속도에서 토크가 강해지고, 커지면 약해짐 |
| `friction` | 핸들이 움직이기 시작할 때 필요한 마찰 보상 | 커지면 코너 진입과 방향 전환 초기 토크가 강해짐 |
| `latAccelOffset` | 직진 시 좌우 쏠림을 보정하는 중심 오프셋 | 양·음 방향에 따라 한쪽으로 치우친 토크를 보정 |

### 현재 기준값과 제한

- 기본 앵커값은 `latAccelFactor=2.05`, `friction=0.230`입니다.
- `TorqueMaxLatAccel`과 `TorqueFriction` 설정값이 정상 범위이면 앵커값으로 사용합니다.
- 학습 초기 안전 범위는 `latAccelFactor=1.95~2.12`, `friction=0.220~0.245`입니다.
- Equinox 프로파일은 좌우 샘플 분포, 포인트 수, 운전자 개입과 토크 제한 비율로 학습 신뢰도를 계산합니다.
- 신뢰도가 충분히 높아지면 허용 범위가 최대 `latAccelFactor=1.75~2.42`, `friction=0.165~0.305`까지 점진적으로 확장될 수 있습니다.
- `latAccelOffset` 학습은 현재 `CHEVROLET EQUINOX NO RADAR`에서만 활성화되며, `30~120 km/h`와 `-0.10~0.10` 범위로 제한됩니다.

### 적용 순서

1. `torqued.py`가 코너·직선 샘플을 수집하고 필터링된 학습값을 `liveTorqueParameters`로 발행합니다.
2. `controlsd.py`는 `IsLiveTorque`가 활성화되어 있고 메시지가 정상이며 `latAccelFactorFiltered > 0`이면 필터링된 세 값을 토크 제어기에 전달합니다.
3. Live Torque가 비활성화되어 있으면 ntune의 `latAccelFactor`, `friction`을 사용하고 `latAccelOffset`은 `0`으로 적용합니다.
4. `latcontrol_torque.py`는 학습값을 기준값으로 보존하고, 매 프레임 속도와 코너 강도에 맞춘 유효 `latAccelFactor`와 `friction`을 별도로 계산합니다.
5. 코너 보정이 끝나면 유효값은 원래 학습 기준값으로 복귀합니다. 따라서 동적 보정이 학습값에 누적되지는 않습니다.

### 학습과 저장

- 샘플이 부족하거나 품질 조건을 충족하지 못하면 학습 업데이트를 동결하고 안전 범위의 값을 유지합니다.
- 학습 상태는 `/data/openpilot/ltp_logs/ltp_state.json`과 `ltp_state.pkl`에 저장되어 재시작 후 복원됩니다.
- 종료 시 `LiveTorqueParameters` 캐시에 이전 값과 새 값을 EMA `0.1`로 합쳐 저장합니다.
- `LiveTorqueReset=1`은 저장 상태와 학습 버킷을 초기화하므로 재학습이 필요할 때만 사용합니다.

### 현재 소스의 주의사항

- `controlsd.py`의 `IsLiveTorque` 상태는 시작할 때 읽습니다. 설정을 변경한 뒤 실제 제어 적용 상태를 확실하게 맞추려면 프로세스 또는 장치를 재시작해야 합니다.
- `torqued.py`의 런타임 설정 갱신 코드에는 현재 `is_live = True`가 고정되어 있어, `IsLiveTorque`를 꺼도 학습 프로세스 내부는 활성 상태를 유지합니다. 실제 조향 제어에서 학습값을 사용할지는 시작 시 `controlsd.py`가 읽은 설정에 따라 결정됩니다.
- `controlsd.py`는 현재 `liveValid` 필드가 아니라 메시지 유효성과 `latAccelFactorFiltered > 0` 조건으로 적용 여부를 판단합니다.
- `liveTorqueParameters`에 보이는 학습 기준값과 코너에서 실제 사용되는 동적 유효값은 다를 수 있습니다. 조향감을 분석할 때는 둘을 구분해야 합니다.

## 6. 확인해야 할 로그

조향 변경 후에는 다음 항목을 함께 확인합니다.

- 토크 파라미터: `latAccelFactor`, `latAccelOffset`, `friction`, `totalBucketPoints`
- Live Torque 학습: `liveValid`, 필터링값, 좌우 버킷 분포, freeze 사유, base 값과 dynamic effective 값
- 명령 전송: `gmSteerCommandSent`, `gmSteerCommandGapMs`, `gmSteerCommandDeadlineLagMs`
- 전송 이상: `gmSteerCommandGapFault`, loopback counter
- 토크 제한: `gmSteerCommandTorque`, `gmSteerRequestedTorque`, `gmSteerTorqueLimited`
- 제어 상태: 운전자 개입, 포화 상태, 조향 오류, 속도 구간

## 7. 변경 원칙

1. 최대 토크 `300`, 전송 주기 `50 Hz`, 증가·감소 제한 `7/17`은 각각 따로 검증합니다.
2. 저속 응답을 높일 때는 운전자 개입과 토크 포화가 자연스럽게 유지되는지 확인합니다.
3. 고속 설정은 빠른 응답보다 곡률 변화의 연속성과 차선 유지 안정성을 우선합니다.
4. 변경 후 저속 코너, 직선 복귀, 고속 완만한 곡선, 운전자 덮어쓰기, 조향 오류 상황을 모두 시험합니다.
5. 실제 도로 시험 전에 단위 테스트, 리플레이 또는 폐쇄된 환경에서 먼저 검증합니다.

## 8. 관련 소스

- 토크 제어기: `selfdrive/controls/lib/latcontrol_torque.py`
- GM 조향 명령 적용: `selfdrive/car/gm/carcontroller.py`
- 50 Hz 스케줄러와 제한 추적: `selfdrive/car/gm/steer_scheduler.py`
- GM 조향 상수: `selfdrive/car/gm/values.py`
- Live Torque 학습기: `selfdrive/locationd/torqued.py`
- Live Torque 적용과 로그: `selfdrive/controls/controlsd.py`
