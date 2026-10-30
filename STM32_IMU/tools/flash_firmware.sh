#!/usr/bin/env bash
# ====================================================================
# Flashing script for STM32_IMU via ST-Link V2 (OpenOCD)
# ====================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"
ELF_FILE="${BUILD_DIR}/STM32_IMU.elf"

# Ensure local user toolchain is in PATH
export PATH="/home/rifai/.local/bin:$PATH"

OPENOCD_BIN="/home/rifai/.local/bin/openocd"
OPENOCD_SCRIPTS="/home/rifai/.arduino15/packages/esp32/tools/openocd-esp32/v0.12.0-esp32-20251215/share/openocd/scripts"

echo "===================================================================="
echo " [1/2] Building STM32_IMU Firmware..."
echo "===================================================================="
cmake -B "${BUILD_DIR}" -S "${PROJECT_DIR}" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="${PROJECT_DIR}/arm-none-eabi-toolchain.cmake"
cmake --build "${BUILD_DIR}"

if [ ! -f "${ELF_FILE}" ]; then
  echo "[ERROR] ELF file not found at ${ELF_FILE}"
  exit 1
fi

echo ""
echo "===================================================================="
echo " [2/2] Flashing STM32_IMU via ST-Link V2..."
echo "===================================================================="
"${OPENOCD_BIN}" -s "${OPENOCD_SCRIPTS}" \
  -f interface/stlink.cfg \
  -c "transport select swd; reset_config none; set FLASH_SIZE 0x20000" \
  -f target/stm32f1x.cfg \
  -c "program \"${ELF_FILE}\" verify exit"

ENTRY_POINT=$(arm-none-eabi-readelf -h "${ELF_FILE}" | grep "Entry point" | awk '{print $4}')
echo "[INFO] Starting target at entry point ${ENTRY_POINT} (SP: 0x20005000)..."
pyocd cmd -t stm32f103rc -c "reset halt" -c "wreg sp 0x20005000" -c "wreg pc ${ENTRY_POINT}" -c "continue"

echo ""
echo "===================================================================="
echo " [SUCCESS] Firmware Flashed, Verified, and Target is Running!"
echo " Telemetry streaming active on /dev/ttyUSB0 @ 115200 baud."
echo "===================================================================="
