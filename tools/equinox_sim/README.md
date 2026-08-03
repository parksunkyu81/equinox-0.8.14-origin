# 이쿼녹스 Virtual Panda 벤치 시뮬레이터

이 시뮬레이터는 차량이나 물리적인 Panda 데이터 경로 없이 실제 GM
이쿼녹스용 `CarState`, `controlsd`, `CarController`, CAN 패커 및 온로드 UI를
실행합니다. 콤마 장치를 차량과 분리한 벤치 환경에서 사용하는 도구입니다.

가상 Panda 상태, 시동 상태 및 이쿼녹스 CAN 메시지를 100Hz로 발행합니다.
openpilot이 실제로 생성한 `sendcan` 페달·조향 명령을 디코딩하여 결정론적
차량 모델에 입력합니다. 시뮬레이터 모드에서만 `plannerd`를 직선 도로용
크루즈 주행계획으로 대체합니다.

시간에 민감한 `equinoxcan`은 CAN·sendcan·차량 모델만 100Hz로 처리합니다.
`equinoxservices`는 modelV2·주행계획·위치·레이더를 20Hz/4Hz로 별도 처리하여
모델 메시지 생성이 CAN 주기를 늦추지 않게 합니다. `recoverylogger`는 제어에
개입하지 않고 복구 이벤트 전후 데이터만 메모리에 수집합니다.

## 안전 범위

- 시작하기 전에 콤마 장치를 차량에서 완전히 분리하십시오.
- 실행 스크립트는 `--bench` 인자를 요구하며 `NOBOARD=1`을 설정합니다.
- 벤치 프로세스는 reader 퇴출을 방지하기 위해 `ZMQ=1`로 실행됩니다.
- 시뮬레이터 환경변수를 일반 부팅 서비스에 절대로 등록하지 마십시오.
- 이 시뮬레이터로는 Panda 펌웨어, 하네스 배선, ECU 명령 수용 여부 또는
  실제 엔진·변속기의 반응을 검증할 수 없습니다.

## 빌드

이 시뮬레이터는 일반 cereal 및 DBC 네이티브 확장 모듈을 사용하므로 먼저
저장소를 빌드해야 합니다.

```bash
cd /data/openpilot
rm -f prebuilt
scons -j$(nproc)
```

## 콤마 장치에서 실행

콤마 장치와 PC를 같은 Wi-Fi 네트워크에 연결합니다. 첫 번째 PC 터미널에서
콤마 장치에 SSH로 접속한 다음, 두 번째 manager가 실행되지 않도록 기존
openpilot을 중지하고 시뮬레이터를 실행합니다. EON/comma two처럼
`root@localhost` 프롬프트와 `comma` tmux 세션을 사용하는 장치는 다음과
같습니다.

```bash
ssh root@<콤마-장치-IP>
tmux kill-session -t comma
pgrep -af manager.py
cd /data/openpilot
bash tools/equinox_sim/launch.sh --bench
```

`pgrep` 결과가 비어 있어야 합니다. `manager.py`가 남아 있으면 다른
터미널에서 실행 중인 시뮬레이터를 `Ctrl+C`로 먼저 종료하십시오. manager를
두 개 동시에 실행하면 메시지 발행 충돌과 장치 프로세스 오류가 발생합니다.

systemd 서비스를 사용하는 장치만 `systemctl stop comma`를 사용합니다.
`root@localhost`는 이미 관리자 권한이므로 `sudo`를 사용하지 않습니다.

첫 번째 터미널은 실행 상태로 둡니다. PC에서 두 번째 터미널을 열어 다시
SSH로 접속한 다음, 콤마 장치의 온로드 화면을 보면서 가상 주행을 제어합니다.

```bash
ssh comma@<콤마-장치-IP>
cd /data/openpilot
python3 -m tools.equinox_sim.control status
python3 -m tools.equinox_sim.control target 100
python3 -m tools.equinox_sim.control fault on
python3 -m tools.equinox_sim.control recovery on
python3 -m tools.equinox_sim.control recovery off
python3 -m tools.equinox_sim.control production on
python3 -m tools.equinox_sim.control production off
python3 -m tools.equinox_sim.control fault off
python3 -m tools.equinox_sim.control brake on
python3 -m tools.equinox_sim.control brake off
python3 -m tools.equinox_sim.control reset
```

각 명령의 의미는 다음과 같습니다.

- `status`: 현재 속도, 목표속도, 페달 명령 및 복구 상태를 확인합니다.
- `target 100`: 가상 차량의 목표속도를 100km/h로 설정합니다.
- `fault on`: PID 가속 요구를 0으로 계속 강제하고 자동 복구를 잠시 막습니다.
  `pedalCommand: 0.0`과 가상 차량의 속도 하락을 먼저 확인할 수 있습니다.
- `recovery on`: 강제 복구를 한 번 실행한 뒤 장애 주입을 자동 해제하고 정상
  PID로 돌아갑니다. 복구 직후 `recoveryCount`는 한 번만 증가해야 합니다.
- `recovery off`: 장애를 유지한 채 복구를 다시 막습니다.
- `production on`: `accel = 0` 장애를 계속 유지하면서 실차와 동일한 0.30m/s
  속도오차 및 모든 안전 조건으로 복구합니다. 시뮬레이터용 조건 우회와 자동
  fault 해제를 사용하지 않습니다.
- `production off`: production-fidelity 장애 주입을 종료합니다.
- `fault off`: `accel = 0` 장애 주입을 해제합니다.
- `brake on` / `brake off`: 가상 브레이크 입력을 켜거나 끕니다.
- `reset`: 차량 속도, 이동거리 및 제어 상태를 초기화합니다.

목표속도가 가상 차량의 현재 속도보다 높은 상태에서 먼저 `fault on`을 실행하면
가속 요구가 0으로 유지되어 속도가 내려갑니다. `recovery on`은 시연용 1회
복구이며, 실차 조건을 그대로 검증하려면 `production on`을 사용합니다. 복구에
성공하면 다음 상태를 확인할 수 있습니다.

- `control status` 출력의 `recoveryActive: true`
- `pedalCommand`가 0.060 이상
- `loopHz`가 지속적으로 약 90~105 범위 (`110` 초과는 비정상)
- 가상 차량 속도 증가
- PEDAL 게이지 바로 위에 황색 복구 경고 표시

## Production-fidelity 시험

먼저 `status`의 `loopHz`가 90~105이고 통신 오류가 없는지 확인합니다. 현재
속도보다 목표속도를 충분히 높게 설정한 뒤 지속 장애 모드를 실행합니다.

```bash
python3 -m tools.equinox_sim.control target 120
python3 -m tools.equinox_sim.control production on
python3 -m tools.equinox_sim.control status
```

정상적인 연속 복구 구간에서는 `faultMode: 3`, `productionFidelity: true`,
`recoveryRawAccel <= 0.001`, `recoveryForcedAccel >= 0.36`,
`pedalCommand >= 0.060`이어야 합니다. 목표속도 도달로 복구 조건이 해제된 뒤
지속 장애로 다시 감속하면 복구가 새 이벤트로 재활성화될 수 있습니다.

시험을 끝낼 때는 반드시 장애를 해제합니다.

```bash
python3 -m tools.equinox_sim.control production off
```

## 복구 이벤트 로그

`recoverylogger`는 manager가 자동으로 시작합니다. 평상시에는 최근 5초를
메모리에만 보관합니다. ACC가 활성화된 주행 중 브레이크·운전자 가속 입력 없이
`abs(accel) <= 0.001`이 발생하면 속도오차, 플랜 유효성 및 플랜 소스와 관계없이
이전 5초와 이후 10초를 다음 위치에 JSONL로 저장합니다.

60초 저장 제한은 없습니다. 하나의 연속된 `accel = 0` 구간은 이벤트 한 건으로
기록하고, 값이 0에서 벗어났다가 다시 0이 되면 앞 이벤트의 이후 10초 수집과
겹치더라도 새로운 파일로 별도 기록합니다. 파일명과 metadata의
`triggerWallTime`은 저장 완료 시각이 아니라 실제 이벤트 시작 시각입니다.

```text
/data/media/0/pedal_recovery_logs/
```

최근 이벤트와 분석 결과는 다음 명령으로 확인합니다.

```bash
ls -lt /data/media/0/pedal_recovery_logs/ | head
python3 -m tools.equinox_sim.analyze_recovery \
  /data/media/0/pedal_recovery_logs/<이벤트파일>.jsonl
```

분석 결과는 소프트웨어 복구, CarController 페달 출력, checksum이 정상인
sendcan 요청, Panda의 성공 송신 receipt(`can`의 반환 source `0x80`), 차량 가속
반응을 분리해서 표시합니다. 첫 `accel = 0` 시점의 실차 복구 조건을 모두 검사하며,
실패한 조건은 `failedEligibilityGates`에 이름으로 출력합니다. `sendcan`만 성공하고
`pandaCanPathPassed`가 거짓이면 Panda 안전 필터·송신 단계까지 통과했다고 판정하지
않습니다. 실제 배선 이후 ECU 수신 여부는 차량의 `aEgo` 반응과 별도 버스 계측으로
최종 확인해야 합니다.

과거 전체 `rlog.bz2`가 있다면 파일을 별도 PC의 같은 저장소로 복사한 뒤 현재
controlsd로 재생한 새 로그와 분석 JSON을 한 번에 생성할 수 있습니다. process
replay는 테스트 Params를 초기화하므로 콤마 장치에서는 실행을 거부합니다.

```bash
python3 -m tools.equinox_sim.replay_recovery /복사한/경로/rlog.bz2
```

로그만으로 차량 fingerprint를 찾지 못할 때만 다음 인자를 추가합니다.

```bash
--fingerprint "CHEVROLET EQUINOX NO RADAR"
```

실차에서는 장애를 인위적으로 주입하지 말고, 기록기를 읽기 전용으로 사용하여
자연 발생 이벤트나 폐쇄 시험 이벤트를 확인하십시오.

시뮬레이터를 종료하려면 실행 중인 첫 번째 터미널에서 `Ctrl+C`를 누릅니다.
EON/comma two에서 일반 openpilot 동작을 다시 시작하려면 시뮬레이터 종료 후
장치를 재부팅하는 것이 가장 확실합니다.

```bash
reboot
```

systemd 방식 장치에서는 대신 `systemctl start comma`를 사용합니다.

## 오류 확인

`오픈파일럿 사용불가 - 장치 프로세스 동작 오류`가 표시되면 다음 명령으로
가장 최근 통신 진단을 확인합니다.

```bash
tail -n 3 /data/log/process_diagnostics.jsonl
```

출력의 `not_alive`, `invalid`, `not_freq_ok` 항목에 문제가 된 서비스 이름이
기록됩니다. 시뮬레이터 실행 터미널에 Python 예외가 출력되었다면 그 예외도
함께 확인해야 합니다.

`0x201 message checks failed` 또는 `virtual_panda lagging`이 반복된 이전 실행은
반드시 `Ctrl+C`로 완전히 종료한 뒤 수정된 코드로 다시 시작해야 합니다.
Ratekeeper의 누적 지연은 실행 중 자동으로 초기화되지 않습니다.

## 지원 범위와 제한사항

온로드 화면의 카메라 배경은 콤마 장치의 실제 카메라 영상입니다. 이
시뮬레이터는 직선 형태의 가상 주행계획과 차량/CAN 피드백을 제공하지만 3D
도로를 렌더링하지 않습니다. 3D 환경과 합성 카메라 영상이 필요하면 공식
openpilot CARLA/MetaDrive 브리지를 사용해야 합니다.
