#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# build.sh — CLI build script for ESP32-BME680-Air-Quality
#
# Wraps arduino-cli for compile, upload, monitor, and lint operations.
#
# Prerequisites:
#   brew install arduino-cli cppcheck
#   arduino-cli config init
#   arduino-cli config add board_manager.additional_urls \
#     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
#   arduino-cli core update-index
#   arduino-cli core install esp32:esp32
#   arduino-cli lib install "BSEC Software Library"
#   arduino-cli lib install "Adafruit BME680 Library"
#   arduino-cli lib install "Adafruit Unified Sensor"
#   arduino-cli lib install "Adafruit BusIO"
#   arduino-cli lib install "WiFiManager"
# ============================================================================

SKETCH="ESP32-BME680-Air-Quality.ino"
FQBN="esp32:esp32:featheresp32:LoopCore=1,EventsCore=1,PartitionScheme=default,CPUFreq=240,FlashFreq=80,FlashSize=4M,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
PORT="${PORT:-/dev/tty.usbserial-0160E5AB}"
BUILD_DIR="build"
BAUD=115200

usage() {
    echo "Usage: $0 {compile|upload|monitor|lint|test|all}"
    echo ""
    echo "Commands:"
    echo "  compile  — Compile the sketch"
    echo "  upload   — Compile and upload to the board"
    echo "  monitor  — Open serial monitor at ${BAUD} baud"
    echo "  lint     — Run cppcheck static analysis"
    echo "  test     — Run unit tests on host (requires g++)"
    echo "  all      — Compile, upload, and monitor"
    echo ""
    echo "Environment variables:"
    echo "  PORT     — Serial port (default: ${PORT})"
    exit 1
}

do_compile() {
    echo "==> Compiling ${SKETCH}..."
    arduino-cli compile \
        --fqbn "${FQBN}" \
        --output-dir "${BUILD_DIR}" \
        "${SKETCH}"
    echo "==> Compile complete."
}

do_upload() {
    do_compile
    echo "==> Uploading to ${PORT}..."
    arduino-cli upload \
        --fqbn "${FQBN}" \
        --port "${PORT}" \
        --input-dir "${BUILD_DIR}"
    echo "==> Upload complete."
}

do_monitor() {
    echo "==> Opening serial monitor on ${PORT} at ${BAUD} baud..."
    arduino-cli monitor \
        --port "${PORT}" \
        --config baudrate="${BAUD}"
}

do_lint() {
    echo "==> Running cppcheck on ${SKETCH}..."
    cppcheck \
        --enable=all \
        --std=c++11 \
        --language=c++ \
        --suppressions-list=cppcheck-suppressions.txt \
        --inline-suppr \
        "${SKETCH}"
    echo "==> Lint complete."
}

do_test() {
    echo "==> Running unit tests..."
    g++ -std=c++11 -Wall -Wextra -Werror -I. -o test_runner test/test_bme680.cpp && ./test_runner
    echo "==> Tests complete."
}

if [[ $# -eq 0 ]]; then
    usage
fi

case "$1" in
    compile)
        do_compile
        ;;
    upload)
        do_upload
        ;;
    monitor)
        do_monitor
        ;;
    lint)
        do_lint
        ;;
    test)
        do_test
        ;;
    all)
        do_upload
        do_monitor
        ;;
    *)
        usage
        ;;
esac
