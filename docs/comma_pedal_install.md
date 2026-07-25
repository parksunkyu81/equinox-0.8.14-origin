# 콤마 페달 펌웨어 설치 안내

이 문서는 Windows 노트북에서 콤마 페달을 STM32 USB DFU 방식으로 직접
플래싱하는 방법을 설명합니다.

> Panda와 CAN은 사용하지 않습니다.

## 1. 플래싱 전 준비

다음 상태를 모두 확인하세요.

관리자 PowerShell에서:
---------------------------------------------
usbipd list

목록에서 Panda 또는 STM32 장치의 BUSID를 확인합니다. 

예를 들어 4-4라면:
--------------------------------------------
usbipd bind --busid 4-4

usbipd attach --wsl --busid 4-4

usbipd list
-----------------------------------------------
- 콤마 페달을 차량 배선에서 완전히 분리
- 페달 보드를 `DFU/BOOT0` 모드로 설정
- 페달 보드의 USB/DFU 연결선을 노트북에 연결
- Docker Desktop 실행
- STM32 DFU 장치 ID: `0483:df11`

> 차량에 연결된 상태에서는 절대 플래싱하지 마세요.

## 2. PowerShell에서 작업 폴더 열기

PowerShell을 열고 다음 명령을 실행합니다.

```powershell
cd C:\py_workspace\equinox-0.8.14-origin
```

## 3. DFU 장치 연결 확인

다음 명령은 Docker 이미지를 준비하고 장치만 확인합니다. 펌웨어는 기록하지
않습니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\output\pedal-firmware-gm-low-impedance-0.8.14\flash_pedal_dfu_docker.ps1 -ListOnly
```

정상적으로 연결되면 다음 내용이 표시됩니다.

```text
0483:df11 STMicroelectronics STM32 BOOTLOADER
PEDAL_DFU_DEVICE_READY
```

Docker 이미지가 이미 만들어져 있다면 `-SkipImageBuild`를 함께 사용해도
됩니다.

## 4. 펌웨어 플래싱

Docker 이미지를 처음 만드는 경우:

```powershell
powershell -ExecutionPolicy Bypass -File .\output\pedal-firmware-gm-low-impedance-0.8.14\flash_pedal_dfu_docker.ps1
```

Docker 이미지가 이미 만들어져 있는 경우:

```powershell
powershell -ExecutionPolicy Bypass -File .\output\pedal-firmware-gm-low-impedance-0.8.14\flash_pedal_dfu_docker.ps1 -SkipImageBuild
```

다음 확인 문구가 나오면 정확하게 입력합니다.

```text
DFU FLASH
```

스크립트는 다음 순서로 기록합니다.

1. 펌웨어 크기와 SHA-256 검사
2. `pedal.bin.signed`를 `0x08004000`에 기록
3. `bootstub.pedal.bin`을 `0x08000000`에 기록
4. STM32 DFU 모드 종료

내부에서 실행되는 `dfu-util` 명령은 다음과 같습니다.

```bash
dfu-util -d 0483:df11 -a 0 -s 0x08004000 -D pedal.bin.signed
dfu-util -d 0483:df11 -a 0 -s 0x08000000:leave -D bootstub.pedal.bin
```

## 5. 완료 확인

정상적으로 완료되면 다음 메시지가 표시됩니다.

```text
PEDAL_DFU_FLASH_COMPLETE
Direct pedal USB DFU flash completed.
```

플래싱이 끝나면:

1. USB 전원을 분리합니다.
2. `BOOT0` 점퍼 또는 연결선을 정상 부팅 위치로 되돌립니다.
3. 페달 전원을 다시 연결합니다.
4. 차량 연결 전 입력·출력 전압을 정차 상태에서 확인합니다.

## 펌웨어 빌드와 플래싱의 차이

- ARM GCC/SCons: 소스 코드를 펌웨어 파일로 빌드
- `dfu-util`: 완성된 펌웨어 파일을 STM32에 기록

`dfu-util`은 펌웨어를 빌드하지 않습니다.

## 문제 해결

### `expected exactly one STM32 DFU device ... found 0`

- 페달이 `DFU/BOOT0` 모드인지 확인
- USB 데이터 케이블인지 확인
- `usbipd list`에서 STM32 장치 확인
- USB를 다시 연결한 후 WSL에 다시 연결

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

`usbipd bind`는 최초 한 번 관리자 PowerShell에서 실행합니다.

### Docker가 실행되지 않는 경우

Docker Desktop을 실행한 후 확인합니다.

```powershell
docker info
```

### 펌웨어 파일 검사만 실행

USB 장치에 기록하지 않고 파일만 검사합니다.

```powershell
powershell -ExecutionPolicy Bypass -File .\output\pedal-firmware-gm-low-impedance-0.8.14\flash_pedal_dfu_docker.ps1 -SkipImageBuild -VerifyOnly
```

### 랜선을 뽑은 다음 다시 연결후 513번 보여야 한다. 

```powershell
cd /data/openpilot
python3 selfdrive/debug/can_printer.py --bus 0 --max_msg 514

0201( 513)(...)
```
추가로 페달 인식 여부를 확인합니다.
```powershell
python3 -c 'from common.params import Params; from cereal import car; d=Params().get("CarParams"); print("enableGasInterceptor:", car.CarParams.from_bytes(d).enableGasInterceptor if d else "NO CarParams")'
```

정상 결과:
```powershell
enableGasInterceptor: True
``` 