#!/usr/bin/env bash
set -euo pipefail

# Build and convert to UF2 for Adafruit QT Py (SAMD21)
# Requirements: arduino-cli (https://arduino.github.io/arduino-cli/), uf2conv.py (from https://github.com/adafruit/uf2-samdx1 or tinyuf2)

SKETCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/qtpy_samd21"
BUILD_DIR="$(pwd)/build_qtpy_samd21"
FQBN="${FQBN:-adafruit:samd:qtpy_m0}"
BOOTLOADER_OFFSET="${BOOTLOADER_OFFSET:-0x2000}"

mkdir -p "$BUILD_DIR"

echo "Building sketch with arduino-cli (FQBN=$FQBN)..."
arduino-cli compile --fqbn "$FQBN" --build-path "$BUILD_DIR" "$SKETCH_DIR"

ELF=$(find "$BUILD_DIR" -type f -name '*.elf' | head -n1)
if [ -z "$ELF" ]; then
  echo "ERROR: compiled ELF not found in $BUILD_DIR" >&2
  exit 2
fi

if command -v uf2conv.py >/dev/null 2>&1; then
  UF2_OUT="$BUILD_DIR/$(basename "$SKETCH_DIR").uf2"
  echo "Converting $ELF -> $UF2_OUT (boot offset $BOOTLOADER_OFFSET)..."
  uf2conv.py "$ELF" -b "$BOOTLOADER_OFFSET" -c -o "$UF2_OUT"
  echo "Done. UF2: $UF2_OUT"
  echo "To flash: double-tap the QT Py to enter bootloader and copy the UF2 file onto the USB mass-storage device."
else
  echo "ERROR: uf2conv.py not found in PATH."
  echo "Please install uf2conv.py (see https://github.com/adafruit/uf2-samdx1 or https://github.com/Microsoft/uf2)")
  exit 3
fi
