# Seacoast MQP — System-Level NLSpec Overview

## Version
0.1.0

## Date
2026-03-02

## Status
Active — governing implementation

---

## Purpose

This document is the top-level NLSpec for the Seacoast MQP Automated Mycelium Inoculator software system. It defines the system boundary, the relationship between component specs, and cross-cutting requirements that apply to all firmware.

This document, together with the component specs it references, is the authoritative source of truth. When code and spec conflict, the code must be updated.

---

## System Description

The Automated Mycelium Inoculator is an electromechanical system that dispenses measured quantities of mycelium spawn from a hopper bag into substrate bags. The system consists of two cooperating microcontrollers:

1. **ESP32-C6** — runs FreeRTOS + ESP-IDF. Hosts the LVGL touchscreen UI, the `pico_link` UART transport, and the `sys_sequence` FreeRTOS task which implements the whiteboard process flow (preferred orchestration path). The legacy ESP-side `dosing` state machine remains for the manual Dosing screen.
2. **Raspberry Pi Pico 2 (RP2350, ARM core)** — runs bare-metal C with a cooperative polling loop. Directly drives all actuators and sensors: DRV8263 H-bridge motor drivers (flaps in standard mode; hot wire + vacuum pump 2 in independent half-bridge mode), DRV8434S SPI stepper drivers on a 4-device SPI1 daisy-chain (agitator [device 0], rotational plenum [device 1], linear plenum [device 2], hotwire traverse [device 3]; turntable and indexer not yet connected to the daisy chain), HX711 load cell, vacuum pumps, and the closed-loop spawn dispensing algorithm.

The two processors communicate over a **UART link at 115200 baud** using COBS framing with CRC-16-CCITT integrity checking, carrying a structured binary protocol defined in `shared/proto/proto.h`.

---

## Component Spec Index

| Spec File | Scope | Depends On |
|-----------|-------|------------|
| `01-shared-protocol.nlspec.md` | Wire protocol, message IDs, payload structs, enumerations | — |
| `02-pico-firmware.nlspec.md` | Pico polling loop, subsystem state machines, drivers, spawn dosing, sensor reading | `01-shared-protocol` |
| `03-esp32-firmware.nlspec.md` | ESP32 UI screens, pico_link transport, motor_hal helpers, control task, dosing context | `01-shared-protocol` |

**Dependency is one-directional:** Both firmware specs depend on the shared protocol spec. Neither firmware spec reaches into the other's internals. All cross-processor interaction occurs exclusively through the message protocol.

---

## Cross-Cutting Requirements

### CR-1: All tuning parameters are `#define` constants with `#ifndef` guards

Every numeric constant that may require hardware calibration (step counts, timeouts, current thresholds, PWM duty cycles, flow rates, PID gains) must be defined as a preprocessor macro wrapped in `#ifndef` so it can be overridden at compile time without editing source files. These constants must be centralized:
- **Pico side:** `pico_fw/src/board_pins.h`
- **ESP32 side:** component-level header or `Kconfig` (implementation choice)

### CR-2: No magic numbers in logic code

All numeric literals in `.c` files that represent hardware parameters, protocol values, or behavioral thresholds must reference a named constant from the appropriate header. The only acceptable bare literals are `0`, `1`, `true`, `false`, and standard C idioms (`NULL`, `sizeof`).

### CR-3: Endianness

All multi-byte fields in the wire protocol are little-endian. Both RP2040 and ESP32-C6 are natively little-endian, so `__attribute__((packed))` structs may be used directly without byte-swapping.

### CR-4: Safety invariant

On any unrecoverable error, communication loss exceeding 5 seconds, or watchdog timeout, both processors must drive all actuators to a safe state:
- Flaps: close (or coast if close is impossible)
- Steppers: disable (coast)
- Hot wire: off
- Vacuum pumps: off

The specific timeout values and recovery behavior are defined in each component spec.

### CR-5: Logging

- **Pico:** `printf()` over USB CDC (stdio_usb enabled, stdio_uart disabled to avoid conflict with the protocol UART).
- **ESP32:** `ESP_LOGx()` macros with per-module tags matching the source file name.

All state machine transitions must be logged at `INFO` level. All errors must be logged at `WARN` or `ERROR` level.

---

## Intentional Ambiguity (System Level)

The following are explicitly left to implementer choice and are not specified:

- Internal variable naming conventions (follow existing codebase style)
- Internal function decomposition within a module (follow existing patterns)
- Specific LVGL widget styling (colors, fonts) beyond layout requirements
- Order of `#include` directives
- Comment formatting
- Git branching strategy

---

## Relationship to Existing Documents

- `docs/state_machine_design.md` is a **design document** that preceded this spec. Where this spec and that document conflict, **this spec governs**. The design document should be updated to match, or annotated as superseded.
- `docs/project_description_1.txt` captures **raw intent** from the project team. This spec is the resolved, authoritative version of that intent.