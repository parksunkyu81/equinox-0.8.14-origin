# GM low-impedance comma pedal firmware

Target hardware:

- Original commaai/neo pedal PCB
- STM32F205
- 1 kOhm input resistor version
- GM vehicle pedal signals

Source:

- Workspace base commit: `b59d04cff8f26e5b7326878907138ff80c0020d3`
- Modified file: `panda/board/pedal/main.c`
- ADC correction: `((raw * 1545) / 1000) + 25`
- CAN protocol unchanged: command `0x200`, sensor `0x201`, 500 kbps, CRC-8 polynomial `0xD5`

Build:

- Docker base: Ubuntu 20.04
- ARM GCC: 9.2.1
- SCons: 3.1.2
- Debug signing certificate from this workspace

Artifacts:

- `pedal.bin.signed`: use for normal CAN flashing
- `pedal.bin`: unsigned build intermediate; do not use for normal CAN flashing
- `bootstub.pedal.bin`: use only for DFU recovery together with the signed application

SHA-256:

```text
685DD63784B4B8C5F930286082AB6C75B231CF3C300A63CE96B214B409679578  pedal.bin.signed
978876A401706D9E22E351B659D243583F368A9E13CD1D10A6CD997FAE5559C2  pedal.bin
6F93267F4488C2C47333640E6F7AEC64FBEE78F0BEBD1BEA0180FD7AACEE802E  bootstub.pedal.bin
```

## Standalone compilation on a Windows PC

This build does not connect to a Panda, USB device, CAN bus, or vehicle. It
copies the local `panda` source into a temporary Docker filesystem and compiles
it with ARM GCC 9.2.1 and SCons 3.1.2.

Run from PowerShell at the workspace root:

```powershell
powershell -ExecutionPolicy Bypass -File .\output\pedal-firmware-gm-low-impedance-0.8.14\build_pedal_docker.ps1
```

Each run writes fresh artifacts under:

```text
output\pedal-firmware-gm-low-impedance-0.8.14\compiled\yyyyMMdd-HHmmss
```

The first run downloads and builds the compiler image. To reuse an existing
image on later runs:

```powershell
powershell -ExecutionPolicy Bypass -File .\output\pedal-firmware-gm-low-impedance-0.8.14\build_pedal_docker.ps1 -SkipImageBuild
```

## Direct USB DFU flashing from Windows PowerShell

This method connects the PC directly to the pedal STM32 in hardware DFU/BOOT0
mode. It does not use a Panda or CAN.

Disconnect the pedal completely from the vehicle, connect only its USB/DFU
interface, and confirm that Windows/WSL exposes device `0483:df11`.

Check the DFU connection without writing:

```powershell
powershell -ExecutionPolicy Bypass -File .\output\pedal-firmware-gm-low-impedance-0.8.14\flash_pedal_dfu_docker.ps1 -ListOnly
```

Flash the firmware:

```powershell
powershell -ExecutionPolicy Bypass -File .\output\pedal-firmware-gm-low-impedance-0.8.14\flash_pedal_dfu_docker.ps1
```

Type `DFU FLASH` at the confirmation prompt. The script verifies both exact
SHA-256 values, writes `pedal.bin.signed` at `0x08004000`, then writes
`bootstub.pedal.bin` at `0x08000000` and leaves DFU mode.

## Docker CAN flashing from PowerShell

Connect the pedal CAN lines to the selected Panda bus and connect the Panda to
USB. Keep the vehicle in P with the parking brake applied and READY/engine OFF.

From the workspace root:

```powershell
powershell -ExecutionPolicy Bypass -File .\output\pedal-firmware-gm-low-impedance-0.8.14\flash_pedal_docker.ps1
```

The script verifies the exact firmware SHA-256, builds an isolated Docker
flasher, and asks you to type `FLASH` before it sends anything to the pedal.
The default is Panda CAN bus 0. Use `-Bus 1` if the pedal is connected to bus 1:

```powershell
powershell -ExecutionPolicy Bypass -File .\output\pedal-firmware-gm-low-impedance-0.8.14\flash_pedal_docker.ps1 -Bus 1
```

If more than one Panda is connected, select one with
`-PandaSerial "PANDA_SERIAL"`.

The Linux Docker engine must be able to see the Panda under `/dev/bus/usb`.
On Windows, check USB/IP with `usbipd list`; if the device is not visible,
attach it to WSL before running the script. Native Linux or the EON is the
simplest fallback when Docker Desktop cannot expose the USB device.

After flashing, the script waits for pedal CAN `0x201`. Before any road test,
verify input/output voltage pass-through with the vehicle stationary and not
READY.
