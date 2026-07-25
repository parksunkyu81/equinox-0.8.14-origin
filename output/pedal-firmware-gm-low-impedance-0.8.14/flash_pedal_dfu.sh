#!/usr/bin/env bash
set -euo pipefail

DFU_ID="0483:df11"
APP_IMAGE="/firmware/pedal.bin.signed"
BOOTSTUB_IMAGE="/firmware/bootstub.pedal.bin"
APP_SHA256="685dd63784b4b8c5f930286082ab6c75b231cf3c300a63ce96b214b409679578"
BOOTSTUB_SHA256="6f93267f4488c2c47333640e6f7aec64fbee78f0bebd1bea0180fd7aacee802e"
APP_SIZE="6508"
BOOTSTUB_SIZE="12216"

verify_file() {
  local path="$1"
  local expected_hash="$2"
  local expected_size="$3"
  local label="$4"

  if [[ ! -f "${path}" ]]; then
    echo "ERROR: missing ${label}: ${path}" >&2
    exit 3
  fi

  local actual_hash
  local actual_size
  actual_hash="$(sha256sum "${path}" | awk '{print $1}')"
  actual_size="$(wc -c < "${path}" | tr -d ' ')"

  if [[ "${actual_hash}" != "${expected_hash}" ]]; then
    echo "ERROR: ${label} SHA-256 mismatch." >&2
    echo "Expected: ${expected_hash}" >&2
    echo "Actual:   ${actual_hash}" >&2
    exit 4
  fi
  if [[ "${actual_size}" != "${expected_size}" ]]; then
    echo "ERROR: ${label} size mismatch." >&2
    echo "Expected: ${expected_size}" >&2
    echo "Actual:   ${actual_size}" >&2
    exit 4
  fi

  echo "${label}: ${actual_size} bytes, SHA-256 OK"
}

verify_all() {
  verify_file "${APP_IMAGE}" "${APP_SHA256}" "${APP_SIZE}" "GM pedal application"
  verify_file "${BOOTSTUB_IMAGE}" "${BOOTSTUB_SHA256}" "${BOOTSTUB_SIZE}" "pedal bootstub"
}

require_one_dfu_device() {
  local count
  count="$(lsusb -d "${DFU_ID}" | wc -l | tr -d ' ')"
  if [[ "${count}" -ne 1 ]]; then
    echo "ERROR: expected exactly one STM32 DFU device (${DFU_ID}), found ${count}." >&2
    echo "Put the pedal into hardware DFU/BOOT0 mode and expose its USB device to Docker." >&2
    exit 2
  fi
}

command="${1:-verify}"
case "${command}" in
  verify)
    verify_all
    echo "PEDAL_DFU_IMAGES_VERIFIED"
    ;;
  list)
    lsusb || true
    echo
    dfu-util -d "${DFU_ID}" -l 2>&1 || true
    require_one_dfu_device
    echo "PEDAL_DFU_DEVICE_READY"
    ;;
  flash)
    if [[ "${CONFIRM_PEDAL_DFU_FLASH:-}" != "YES" ]]; then
      echo "ERROR: flashing is locked." >&2
      exit 5
    fi

    verify_all
    require_one_dfu_device

    echo "Writing signed pedal application at 0x08004000..."
    dfu-util -d "${DFU_ID}" -a 0 -s 0x08004000 -D "${APP_IMAGE}"

    echo "Writing pedal bootstub at 0x08000000, then leaving DFU..."
    dfu-util -d "${DFU_ID}" -a 0 -s 0x08000000:leave -D "${BOOTSTUB_IMAGE}"
    echo "PEDAL_DFU_FLASH_COMPLETE"
    ;;
  *)
    echo "Usage: flash_pedal_dfu.sh {verify|list|flash}" >&2
    exit 64
    ;;
esac

