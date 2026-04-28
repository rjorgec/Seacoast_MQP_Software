# NLSpec: Shared Communication Protocol

## Version
0.1.2

## Depends On
Nothing (leaf spec)

---

## 1. Overview

This spec defines the binary protocol used for all communication between the ESP32-C6 and Raspberry Pi Pico 2 over UART. It governs the contents of `shared/proto/proto.h`, `shared/proto/cobs.c`, and `shared/proto/proto_crc.c`.

---

## 2. Transport Layer

### 2.1 Physical Layer

| Parameter | Value |
|-----------|-------|
| Interface | UART (hardware), full-duplex |
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |

### 2.2 Framing

Each frame on the wire is:

```
[COBS-encoded( proto_hdr_t | payload | CRC-16-CCITT )] 0x00
```

- **COBS encoding** eliminates all `0x00` bytes from the data, so `0x00` serves as an unambiguous frame delimiter.
- **CRC-16-CCITT** is computed over the concatenation of `proto_hdr_t` and `payload` bytes (before COBS encoding). Polynomial: `0x1021`, initial value: `0xFFFF`. The CRC is appended as 2 bytes in little-endian order.
- **Maximum COBS-decoded frame size:** `sizeof(proto_hdr_t) + PROTO_MAX_PAYLOAD + 2` = `6 + 128 + 2` = **136 bytes**.

### 2.3 Header Structure

```c
typedef struct __attribute__((packed)) {
    uint8_t  version;   // PROTO_VERSION = 1
    uint8_t  type;      // msg_type_t
    uint16_t seq;       // sequence number (little-endian)
    uint16_t len;       // payload length in bytes (little-endian)
} proto_hdr_t;
```

- `version`: must be `PROTO_VERSION` (1). Frames with a different version are NACKed with `NACK_BAD_VERSION`.
- `seq`: monotonically increasing per-sender. Used to correlate ACK/NACK responses to commands. Wraps at `0xFFFF` → `0x0000`.
- `len`: number of payload bytes following the header. Must be `<= PROTO_MAX_PAYLOAD` (128). If `len` does not match the expected payload size for the given `type`, the receiver NACKs with `NACK_BAD_LEN`.

### 2.4 Shared Protocol Defaults

The following compile-time protocol defaults are defined in `shared/proto/proto.h` and are the canonical source for both firmware projects unless explicitly overridden by build flags:

| Constant | Default | Purpose |
|----------|---------|---------|
| `PROTO_STEPPER_SOFT_TORQUE_LIMIT_DEFAULT` | 100 | Default soft torque-limit threshold for DRV8434S motion commands (`0` disables torque-limit stop) |

---

## 3. Message Type Table

### 3.1 Legacy Messages (unchanged, fully operational)

| ID | Constant | Direction | Payload | Description |
|----|----------|-----------|---------|-------------|
| 0x01 | `MSG_PING` | ESP → Pico | none (len=0) | Connectivity check; Pico responds with ACK |
| 0x10 | `MSG_MOTOR_DRV8263_START_MON` | ESP → Pico | `pl_drv8263_start_mon_t` (12 bytes) | Start DRV8263 motor with current monitoring |
| 0x11 | `MSG_MOTOR_DRV8263_STOP_MON` | ESP → Pico | none (len=0) | Stop DRV8263 motor and monitoring |
| 0x28 | `MSG_HX711_TARE` | ESP → Pico | none (len=0) | Zero the load cell |
| 0x29 | `MSG_HX711_MEASURE` | ESP → Pico | `pl_hx711_measure_t` (4 bytes) | Request a weight reading; Pico responds with ACK containing `pl_hx711_mass_t` |
| 0x30 | `MSG_CTRL_TARE` | ESP → Pico | none | Control-level tare command |
| 0x31 | `MSG_CTRL_START` | ESP → Pico | none | Control-level start |
| 0x32 | `MSG_CTRL_STOP` | ESP → Pico | none | Control-level stop / abort |
| 0x33 | `MSG_CTRL_PAUSE` | ESP → Pico | none | Control-level pause |

### 3.2 State-Based Command Messages (0x40–0x5F, ESP32 → Pico)

| ID | Constant | Payload Struct | Size | Description |
|----|----------|---------------|------|-------------|
| 0x40 | `MSG_FLAPS_OPEN` | none (len=0) | 0 | Open both hopper flaps; DRV8263 forward, current-drop auto-stop |
| 0x41 | `MSG_FLAPS_CLOSE` | none (len=0) | 0 | Close both hopper flaps; DRV8263 reverse, current-high auto-stop |
| 0x42 | `MSG_ARM_MOVE` | `pl_arm_move_t` | 1 | Move arm stepper to named absolute position |
| 0x43 | `MSG_RACK_MOVE` | `pl_rack_move_t` | 1 | Move rack stepper to named absolute position |
| 0x44 | `MSG_TURNTABLE_GOTO` | `pl_turntable_goto_t` | 1 | Move turntable to named angular position |
| 0x45 | `MSG_TURNTABLE_HOME` | none (len=0) | 0 | Home turntable via stall detection; zero position counter |
| 0x46 | `MSG_HOTWIRE_SET` | `pl_hotwire_set_t` | 1 | Enable/disable hot wire constant-current output |
| 0x47 | `MSG_VACUUM_SET` | `pl_vacuum_set_t` | 1 | Turn primary vacuum pump on/off |
| 0x48 | `MSG_VACUUM2_SET` | `pl_vacuum2_set_t` | 1 | Turn secondary vacuum pump on/off — drives DRV8263 independent half-bridge IN2. NOT mutually exclusive with hotwire (IN1); both can run simultaneously. |
| 0x49 | `MSG_DISPENSE_SPAWN` | `pl_innoculate_bag_t` | 7 | Start closed-loop spawn dosing on Pico |
| 0x4A | `MSG_SPAWN_STATUS` | `pl_spawn_status_t` | 16 | Pico → ESP unsolicited: dosing progress updates |
| 0x4B | `MSG_HOTWIRE_TRAVERSE` | `pl_hotwire_traverse_t` | 1 | Traverse hotwire stepper (STEPPER_DEV_HW_CARRIAGE, device 3); direction 0=cut, 1=return |
| 0x4C | `MSG_INDEXER_MOVE` | `pl_indexer_move_t` | 1 | Move bag depth/eject rack (STEPPER_DEV_INDEXER) to named position |
| 0x4D | `MSG_ARM_HOME` | none (len=0) | 0 | Sensorlessly home the rotary arm against its positive hard stop, then back off |
| 0x4E | `MSG_AGITATE` | `pl_agitate_t` | 1 | Trigger agitator knead cycle; optional home before kneading via `AGITATE_FLAG_DO_HOME` in flags byte |

### 3.3 Unsolicited Status Messages (0x60–0x6F, Pico → ESP32)

| ID | Constant | Payload Struct | Size | Description |
|----|----------|---------------|------|-------------|
| 0x60 | `MSG_MOTION_DONE` | `pl_motion_done_t` | 8 | Motion complete/fault notification for any subsystem |
| 0x61 | `MSG_VACUUM_STATUS` | `pl_vacuum_status_t` | 4 | Vacuum pump RPM and blocked/OK status |
| 0x62 | `MSG_ARM_SEAL_EVENT` | `pl_arm_seal_event_t` | 10 | Rotary-arm seal transition event (LOST/RESTORED) with RPM context |

### 3.4 Response Messages

| ID | Constant | Payload | Description |
|----|----------|---------|-------------|
| 0x80 | `MSG_ACK` | varies (0 or response payload) | Command accepted. Payload depends on original command (e.g., `MSG_HX711_MEASURE` ACK carries `pl_hx711_mass_t`). |
| 0x81 | `MSG_NACK` | `pl_nack_t` (1 byte) | Command rejected. `code` field contains `nack_code_t`. |

### 3.5 Acknowledgement Policy

Every ESP → Pico command receives an **immediate** `MSG_ACK` or `MSG_NACK` once the Pico's `dispatch_decoded()` validates the command and accepts the state machine transition. This ACK means "command received and transition initiated" — **not** "motion complete."

For actuator commands (`MSG_FLAPS_OPEN`, `MSG_ARM_MOVE`, etc.), a subsequent `MSG_MOTION_DONE` is sent **asynchronously** when the physical motion completes or faults.

For steady-state commands (`MSG_HOTWIRE_SET`, `MSG_VACUUM_SET`, `MSG_VACUUM2_SET`), the ACK is the only confirmation. No `MSG_MOTION_DONE` follows.

For `MSG_DISPENSE_SPAWN`, ACK confirms the dosing run has started. Progress is reported via periodic `MSG_SPAWN_STATUS` messages until completion.

---

## 4. Enumeration Types

### 4.1 NACK Codes (`nack_code_t`)

| Value | Name | Meaning |
|-------|------|---------|
| 1 | `NACK_BAD_FRAME` | COBS decode failed or frame too short |
| 2 | `NACK_BAD_CRC` | CRC-16 mismatch |
| 3 | `NACK_BAD_LEN` | Payload length does not match expected size for message type |
| 4 | `NACK_BAD_VERSION` | `proto_hdr_t.version` ≠ `PROTO_VERSION` |
| 5 | `NACK_UNKNOWN` | Unknown message type or generic rejection |

### 4.2 Subsystem IDs (`subsystem_id_t`)

| Value | Name | Hardware |
|-------|------|----------|
| 0 | `SUBSYS_FLAPS` | DRV8263 flap instance(s) |
| 1 | `SUBSYS_ARM` | DRV8434S device 1 (rotational plenum) |
| 2 | `SUBSYS_RACK` | DRV8434S device 2 (linear plenum) |
| 3 | `SUBSYS_TURNTABLE` | DRV8434S (platform) — not yet wired |
| 4 | `SUBSYS_HOTWIRE` | DRV8263 hotwire instance (IN1 independent half-bridge); traverse stepper is DRV8434S device 3 |
| 5 | `SUBSYS_VACUUM` | Vacuum pump 1 GPIO + RPM ISR |
| 6 | `SUBSYS_INDEXER` | DRV8434S (bag depth/eject rack) — not yet wired |
| 7 | `SUBSYS_AGITATOR` | DRV8434S device 0 (agitator eccentric arm) |

### 4.3 Motion Result Codes (`motion_result_t`)

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `MOTION_OK` | Reached target position or achieved target state |
| 1 | `MOTION_STALLED` | Stall detected before target (DRV8434S torque threshold or unexpected current drop) |
| 2 | `MOTION_TIMEOUT` | Motion did not complete within the subsystem's timeout deadline |
| 3 | `MOTION_FAULT` | Driver fault pin (nFAULT) asserted |
| 4 | `MOTION_SPI_FAULT` | SPI communication failure (not a physical stall); distinguishable from nFAULT for diagnostics |

### 4.4 Position Enumerations

**Arm (`arm_pos_t`):**

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `ARM_POS_PRESS` | Press against attachment point (stall = engagement confirmation) |
| 1 | `ARM_POS_1` | Intermediate position 1 |
| 2 | `ARM_POS_2` | Intermediate position 2 |
| 3 | `ARM_POS_HOME` | Home position (after homing backoff) |

**Rack (`rack_pos_t`):**

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `RACK_POS_HOME` | Drive to physical endstop via stall detection; zero counter |
| 1 | `RACK_POS_EXTEND` | Extended position for bag centering |
| 2 | `RACK_POS_PRESS` | Press position (beyond extend) |

**Turntable (`turntable_pos_t`):**

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `TURNTABLE_POS_INTAKE` | Intake position (default home / hard endstop) |
| 1 | `TURNTABLE_POS_TRASH` | Trash / intermediate position |
| 2 | `TURNTABLE_POS_EJECT` | Eject position — bag pushed out |

**Indexer (`indexer_pos_t`):**

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `INDEXER_POS_OPEN` | Fully retracted — bag can slide in freely |
| 1 | `INDEXER_POS_CENTER` | Extended to bag-centering position (holds bag while weighing) |
| 2 | `INDEXER_POS_EJECT` | Fully extended — pushes inoculated bag out of platform |

### 4.5 Vacuum Status (`vacuum_status_code_t`)

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `VACUUM_OK` | Pump running, vacuum detected (RPM in normal range) |
| 1 | `VACUUM_BLOCKED` | Pump running, RPM below threshold (seal lost or blockage) |
| 2 | `VACUUM_OFF` | Pump is off |

### 4.6 Spawn Dosing Status (`spawn_status_code_t`)

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `SPAWN_STATUS_RUNNING` | Dosing in progress, flow normal |
| 1 | `SPAWN_STATUS_DONE` | Target mass dispensed successfully |
| 2 | `SPAWN_STATUS_STALLED` | Flow stalled (bridge detected), attempting agitation |
| 3 | `SPAWN_STATUS_AGITATING` | Agitator is running to break a bridge |
| 4 | `SPAWN_STATUS_BAG_EMPTY` | Inoculant bag empty or insufficient spawn remaining |
| 5 | `SPAWN_STATUS_ERROR` | Unrecoverable error (sensor failure, driver fault) |
| 6 | `SPAWN_STATUS_FLOW_FAILURE` | Flaps fully open with no measurable flow |
| 7 | `SPAWN_STATUS_ABORTED` | Dose cancelled by `MSG_CTRL_STOP` |

### 4.7 Arm Seal Event (`arm_seal_event_t`)

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `ARM_SEAL_EVENT_LOST` | Seal loss edge detected |
| 1 | `ARM_SEAL_EVENT_RESTORED` | Seal restored edge detected |

### 4.8 Arm Seal Reason (`arm_seal_reason_t`)

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `ARM_SEAL_REASON_TRANSIENT` | Fast detach spike |
| 1 | `ARM_SEAL_REASON_STEADY` | Sustained +RPM shift (leak/unsealed) |
| 2 | `ARM_SEAL_REASON_STALE_TACH` | Tach pulses missing/stale while vacuum ON |
| 3 | `ARM_SEAL_REASON_UNKNOWN` | Fallback reason when unavailable |

### 4.9 Spawn Finish Mode (`spawn_finish_mode_t`)

Selects the end-of-dose strategy used by the Pico spawn dosing algorithm.

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `SPAWN_FINISH_MODE_A` | Early close + top-off pulses to correct undershoot |
| 1 | `SPAWN_FINISH_MODE_B` | Low-flow taper — switch to minimal-PWM nudges when near target |

---

## 5. Payload Struct Reference

All structs are C99, `__attribute__((packed))`, fixed-width types from `<stdint.h>`. Sizes listed are wire sizes.

| Struct | Size | Fields |
|--------|------|--------|
| `pl_drv8263_start_mon_t` | 12 | `dir:u8, _rsvd:u8, speed:u16, low_th:u16, high_th:u16, interval_ms:u32` |
| `pl_hx711_measure_t` | 4 | `interval_us:u32` |
| `pl_hx711_mass_t` | 8 | `mass_ug:i32, unit:u8, _rsvd[3]:u8` |
| `pl_nack_t` | 1 | `code:u8` |
| `pl_arm_move_t` | 1 | `position:u8` (arm_pos_t) |
| `pl_rack_move_t` | 1 | `position:u8` (rack_pos_t) |
| `pl_turntable_goto_t` | 1 | `position:u8` (turntable_pos_t) |
| `pl_hotwire_set_t` | 1 | `enable:u8` (1=on, 0=off) |
| `pl_vacuum_set_t` | 1 | `enable:u8` (1=on, 0=off) |
| `pl_vacuum2_set_t` | 1 | `enable:u8` (1=on, 0=off) |
| `pl_innoculate_bag_t` | 7 | `bag_mass:u16, spawn_mass:u16, innoc_percent:u16, bag_number:u8` |
| `pl_spawn_status_t` | 16 | `status:u8, retries:u8, bag_number:u16, target_ug:u32, disp_ug:u32, remain_ug:u32` |
| `pl_motion_done_t` | 8 | `subsystem:u8, result:u8, _rsvd[2]:u8, steps_done:i32` |
| `pl_vacuum_status_t` | 4 | `status:u8, _rsvd:u8, rpm:u16` |
| `pl_arm_seal_event_t` | 10 | `event:u8, reason:u8, rpm_baseline:u16, rpm_filt:u16, delta_rpm:i16, age_ms:u16` |
| `pl_hotwire_traverse_t` | 1 | `direction:u8` (0=cut/forward, 1=return) |
| `pl_indexer_move_t` | 1 | `position:u8` (indexer_pos_t) |
| `pl_agitate_t` | 1 | `flags:u8` (`AGITATE_FLAG_DO_HOME` = 0x01 to home agitator before kneading; 0 = knead only) |

---

## 6. Definition of Done

- [ ] `shared/proto/proto.h` contains all message IDs, enumerations, and payload structs listed above, with no omissions
- [ ] All payload struct sizes match the sizes in the table (verified with `_Static_assert`)
- [ ] `MSG_ARM_SEAL_EVENT` uses ID `0x62`; payload includes signed `delta_rpm` (`rpm_filt - rpm_baseline`) and sample `age_ms`
- [ ] Pico sends `MSG_ARM_SEAL_EVENT` only on LOST/RESTORED transitions (no repeated LOST spam while latched)
- [ ] `proto_crc16_ccitt()` computes CRC-16-CCITT (polynomial 0x1021, init 0xFFFF) correctly for at least 3 known test vectors
- [ ] COBS encode/decode round-trips correctly for payloads of size 0, 1, 64, and 128 bytes
- [ ] All `#define` constants use `#ifndef` guards
- [ ] `PROTO_STEPPER_SOFT_TORQUE_LIMIT_DEFAULT` (value: 100) is present in the shared protocol header and used as the fallback default on both Pico and ESP firmware
- [ ] The header compiles cleanly under both GCC (ARM, Pico SDK) and GCC (RISC-V, ESP-IDF) with `-Wall -Werror`
