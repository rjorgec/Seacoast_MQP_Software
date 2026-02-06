DRV8434S driver (minimal)

Overview
-------

This folder contains a minimal, portable C driver interface for a DRV8434S-like SPI motor driver. The code is intentionally tiny and hardware-agnostic: callers provide callbacks for SPI transfers, CS/reset control, and delays.

Files
-----
- `drv8434s.h` - public header with driver types and prototypes
- `drv8434s.c` - minimal implementation: init, reset, probe
- `example_qtpy_rp2040.c` - small example showing how to wire callbacks on a QT Py RP2040 (RP2040 SDK)
- `example_qtpy_samd21.cpp` - small example showing how to run on a QT Py (SAMD21) with Arduino / PlatformIO

Probe behavior
--------------
`drv8434s_probe()` asserts CS, sends two 0x00 bytes and checks if MISO returned something other than 0xFF (floating). This is a simple heuristic for detecting a connected device and may be replaced with register-specific reads for a production implementation.

For stronger checks, use the new helpers:

- `drv8434s_probe_cmd(dev, tx, tx_len, rx, rx_len)` — send a specific command and validate returned bytes (useful for device-specific opcodes).
- `drv8434s_read_register(dev, reg, out, out_len)` — performs a common `[reg][dummy...]` SPI transaction and returns true if any returned byte is not `0xFF`.

These helpers are provided because register-based reads are a much more reliable way to confirm device presence than detecting floating MISO bytes.

QT Py RP2040 example
--------------------
See `example_qtpy_rp2040.c` for a concrete example using the Raspberry Pi Pico SDK. The code is guarded by `#ifdef TEST_QTYPY_RP2040` and is provided as a test harness only.

QT Py (SAMD21) example
----------------------
See `example_qtpy_samd21.cpp` for an Arduino/PlatformIO-friendly example. The code has also been packaged as an Arduino sketch at `arduino/qtpy_samd21/DRV8434S_QTYPY_SAMD21.ino`.

If you want to build and produce a `.uf2` for Adafruit QT Py (SAMD21) using arm-none-eabi GCC under the hood, use the included helper script `arduino/build_qtpy_samd21.sh` which invokes `arduino-cli` and `uf2conv.py` to create a UF2 you can copy to the board. Adjust the `FQBN` or `BOOTLOADER_OFFSET` environment variables if needed.

Requirements: `arduino-cli` (to compile) and `uf2conv.py` (to convert ELF -> UF2). See `arduino/README.md` for step-by-step instructions.

License: MIT-style (add appropriate license header for your project).
