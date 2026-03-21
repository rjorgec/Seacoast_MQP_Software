# Seacoast MQP Software — Developer Reference

**Project:** Automated Mycelium Inoculator  
**Repository:** `rjorgec/Seacoast_MQP_Software`  
**Platforms:** Raspberry Pi Pico 2 (RP2350, ARM core) + ESP32-C6 (ESP-IDF v5.5.x)

---

## Repository Structure

```
Seacoast_MQP_Software/
├── docs/                          ← This file + design documents
│   ├── README_software.md
│   ├── state_machine_design.md    ← Pico-side SM + protocol reference
│   ├── project_description_1.txt
│   └── specs/                     ← nlspec design files
├── shared/
│   └── proto/                     ← Wire protocol (shared between both firmwares)
│       ├── proto.h                ← Message types, enums, payload structs
│       ├── proto_crc.c            ← CRC-16-CCITT
│       └── cobs.c / cobs.h        ← Consistent Overhead Byte Stuffing
├── pico_fw/
│   ├── src/
│   │   ├── main.c                 ← Polling loop entry point
│   │   ├── uart_server.c/.h       ← Dispatch, state machines, subsystem handlers
│   │   ├── board_pins.h           ← ALL pin assignments + tuning constants
│   │   └── CMakeLists.txt
│   └── drivers/
│       ├── drv8263/               ← TI DRV8263 H-bridge driver (flaps + hot wire)
│       │   ├── drv8263.h
│       │   └── drv8263.c
│       ├── drv8434s/              ← TI DRV8434S SPI stepper driver (daisy chain)
│       │   ├── drv8434s.h
│       │   └── drv8434s.c
│       └── hx711/                 ← HX711 load cell (via extern/pico-scale)
├── lcd_uext_ili9341/              ← ESP32 firmware (IDF component project)
│   ├── main/
│   │   ├── app_main.c             ← Entry point + FreeRTOS init
│   │   ├── sys_sequence.c/.h      ← System-level state machine (NEW)
│   │   ├── ui_screens.c/.h        ← LVGL UI screens
│   │   ├── motor_hal_pico.c       ← motor_hal implementation via pico_link
│   │   ├── dosing.c/.h            ← LEGACY ESP32-side dosing (see dosing.h)
│   │   └── CMakeLists.txt
│   └── components/
│       ├── motor_hal/             ← Actuator command API (ESP32 side)
│       ├── pico_link/             ← UART+COBS+CRC framing to Pico
│       └── proto/                 ← ESP32 copy of shared/proto/proto.h
└── extern/
    └── pico-scale/                ← HX711 + scale submodule
```

---

## Build Instructions

### Pico Firmware (Pico SDK 2.2.0)

```bash
cd pico_fw/src
mkdir build && cd build
cmake .. -DPICO_BOARD=pico2
make -j4
# Flash: copy UART_TEST.uf2 to the Pico 2 USB mass storage
```

### ESP32 Firmware (ESP-IDF v5.5.x, ESP32-C6)

```bash
cd lcd_uext_ili9341
idf.py set-target esp32c6
idf.py build
idf.py flash monitor
```

---

## Pin Assignments

All Pico pin assignments are in [`pico_fw/src/board_pins.h`](../pico_fw/src/board_pins.h).

| GPIO | Function                                                    | Peripheral | Constant                |
|:----:|-------------------------------------------------------------|:----------:|-------------------------|
| 0    | UART0 TX → ESP32                                            | UART0      | `PICO_UART_TX_GPIO`     |
| 1    | UART0 RX ← ESP32                                            | UART0      | `PICO_UART_RX_GPIO`     |
| 10    | DRV8434S SCK (SPI1)                                         | SPI1       | `DRV8434S_SCK_GPIO`     |
| 11    | DRV8434S MOSI (SPI1)                                        | SPI1       | `DRV8434S_MOSI_GPIO`    |
| 12    | DRV8434S MISO (SPI1)                                        | SPI1       | `DRV8434S_MISO_GPIO`    |
| 13    | DRV8434S CS (SPI1)                                          | SPI1 GPIO  | `DRV8434S_CS_GPIO`      |
| 6    | Flap 1 DRV8263 IN1 (PWM)                                    | PWM3A      | `DRV8263_CTRL_A_GPIO`   |
| 7    | Flap 1 DRV8263 IN2 (PWM)                                    | PWM3B      | `DRV8263_CTRL_B_GPIO`   |
| 8    | Flap 2 DRV8263 IN1 (PWM)                                    | PWM4A      | `FLAP2_CTRL_A_PIN`      |
| 9    | Flap 2 DRV8263 IN2 (PWM)                                    | PWM4B      | `FLAP2_CTRL_B_PIN`      |
| 4   | Hot wire IN1 — DRV8263 independent half-bridge (PWM)        | GPIO out      | `HOTWIRE_PIN_IN1`       |
| 5   | Vacuum pump 2 IN2 — DRV8263 independent half-bridge (PWM)   | GPIO out      | `HOTWIRE_PIN_IN2`       |
| 2   | Vacuum pump 1 trigger (output)                              | GPIO out   | `VACUUM_TRIGGER_PIN`    |
| 3   | Vacuum pump 1 RPM sense (rising-edge interrupt)             | GPIO irq   | `VACUUM_RPM_SENSE_PIN`  |
| 14   | HX711 DATA                                                  | GPIO       | `HX711_DATA_GPIO`       |
| 15   | HX711 CLK                                                   | GPIO       | `HX711_CLK_GPIO`        |
| 27   | Flap 2 current sense ADC ch1                                | ADC1       | `FLAP2_ADC_SENSE_PIN`   |
| 28   | Flap 1 current sense ADC ch2                                | ADC2       | `DRV8263_SENSE_GPIO`    |

**PWM slices:** GP6/7 = slice 3 (flap 1), GP8/9 = slice 4 (flap 2), GP10/11 = slice 5 (hot wire + vacuum 2). No conflicts.  
**ADC channels:** GP27 = ADC ch1 (flap 2 sense), GP28 = ADC ch2 (flap 1 sense). No conflicts.  
**Hot wire ADC (GP26, ADC ch0):** Not populated — current regulation is done by external Rsense on DRV8263.

---

## DRV8434S Device Index Table

The DRV8434S stepper controllers are daisy-chained on SPI1.  
`DRV8434S_N_DEVICES` in `board_pins.h` must match the number of physically wired devices.

| Index | Constant                  | Subsystem                    | Status            |
|:-----:|---------------------------|------------------------------|:-----------------:|
| 0     | `STEPPER_DEV_ROT_ARM`     | Rotary suction arm           | Wired             |
| 1     | `STEPPER_DEV_LIN_ARM`     | Linear vacuum arm (rack)     | Wired             |
| 2     | `STEPPER_DEV_TURNTABLE`   | Turntable / platform         | Wired             |
| 3     | `STEPPER_DEV_AGITATOR`    | Agitator eccentric arm       | Not yet wired     |
| 4     | `STEPPER_DEV_HW_CARRIAGE` | Hot wire carriage traverse   | Not yet wired     |
| 5     | `STEPPER_DEV_INDEXER`     | Bag depth/eject rack         | Not yet wired     |

To add a device: uncomment the `#define` in `board_pins.h` and increment `DRV8434S_N_DEVICES`.

---

## Protocol Summary

Wire format: `[COBS-encoded( proto_hdr_t | payload | CRC-16-CCITT )] 0x00`

All multi-byte fields: **little-endian**.  
See [`shared/proto/proto.h`](../shared/proto/proto.h) for all message IDs and payload structs.

### Command Messages (ESP32 → Pico)

| ID   | Name                    | Payload Struct           | Description                         |
|:----:|-------------------------|--------------------------|-------------------------------------|
| 0x40 | `MSG_FLAPS_OPEN`        | _(none)_                 | Open both flaps                     |
| 0x41 | `MSG_FLAPS_CLOSE`       | _(none)_                 | Close both flaps                    |
| 0x42 | `MSG_ARM_MOVE`          | `pl_arm_move_t`          | Rotary arm to named position        |
| 0x43 | `MSG_RACK_MOVE`         | `pl_rack_move_t`         | Linear arm to named position        |
| 0x44 | `MSG_TURNTABLE_GOTO`    | `pl_turntable_goto_t`    | Turntable to named position         |
| 0x45 | `MSG_TURNTABLE_HOME`    | _(none)_                 | Home turntable, zero counter        |
| 0x46 | `MSG_HOTWIRE_SET`       | `pl_hotwire_set_t`       | Hot wire ON/OFF (DRV8263 IN1)       |
| 0x47 | `MSG_VACUUM_SET`        | `pl_vacuum_set_t`        | Vacuum pump 1 ON/OFF                |
| 0x48 | `MSG_VACUUM2_SET`       | `pl_vacuum2_set_t`       | Vacuum pump 2 ON/OFF (DRV8263 IN2)  |
| 0x49 | `MSG_DISPENSE_SPAWN`    | `pl_innoculate_bag_t`    | Start Pico-side closed-loop dosing  |
| 0x4B | `MSG_HOTWIRE_TRAVERSE`  | `pl_hotwire_traverse_t`  | Traverse hot wire carriage stepper  |
| 0x4C | `MSG_INDEXER_MOVE`      | `pl_indexer_move_t`      | Move bag depth/eject rack           |
| 0x4D | `MSG_ARM_HOME`          | _(none)_                 | Sensorlessly home arm, then back off |

### Unsolicited Status Messages (Pico → ESP32)

| ID   | Name                 | Payload Struct         | Description                        |
|:----:|----------------------|------------------------|------------------------------------|
| 0x4A | `MSG_SPAWN_STATUS`   | `pl_spawn_status_t`    | Dosing status updates              |
| 0x60 | `MSG_MOTION_DONE`    | `pl_motion_done_t`     | Motion complete notification       |
| 0x61 | `MSG_VACUUM_STATUS`  | `pl_vacuum_status_t`   | Vacuum pump RPM / blocked status   |

### Acknowledgement Policy

Every ESP→Pico command receives an immediate `MSG_ACK` (0x80) or `MSG_NACK` (0x81) from `dispatch_decoded()`. `MSG_MOTION_DONE` arrives later, asynchronously, when the physical motion completes or faults.

---

## UI Screen Map

```
Home Screen
├── Automated Functions Screen  (sys_sequence.c — supervised inoculation)
├── Scale Screen                (HX711 weight readout + tare)
├── Operations Screen           (manual actuator control + status display)
└── Dosing Screen               (LEGACY — manual spawn dosing for isolated testing)
```

### Automated Functions Screen buttons

Setup/Load | Start | Abort

- **Setup/Load** — `sys_sequence_send_cmd(SYS_CMD_SETUP_LOAD)`: moves all subsystems to loading positions, waits for Start.
- **Start** — `sys_sequence_send_cmd(SYS_CMD_START)`: begins the full inoculation cycle.
- **Abort** — `sys_sequence_send_cmd(SYS_CMD_ABORT)`: safe-stops all actuators and returns the sequence to `SYS_IDLE`, immediately re-enabling manual controls. _(Note: an earlier implementation incorrectly targeted the legacy control task queue; the correct target is the sequence task queue.)_

### Dosing Screen buttons

Start Dose | Abort

- **Start Dose** — `pico_link_send_rpc(MSG_DISPENSE_SPAWN, …)`: starts Pico-side closed-loop spawn dispensing.
- **Abort** — `pico_link_send_rpc(MSG_CTRL_STOP, …)`: sends `MSG_CTRL_STOP` to the Pico to abort the spawn state machine, stop the dosing timer, and fast-close flaps. Closing flaps alone via `motor_flap_close()` is insufficient — the Pico spawn SM continues running until `MSG_CTRL_STOP` is received.

### Operations Screen buttons

Flap Open/Close | Arm Home/Press/Pos1/Pos2 | Rack Home/Extend/Press |  
Turntable Pos A/B/C/D | HotWire ON/OFF | Vacuum ON/OFF | Turntable Home

---

## Tuning Guide

All hardware calibration constants are in [`pico_fw/src/board_pins.h`](../pico_fw/src/board_pins.h) with `#ifndef` guards for compile-time override.

| Constant                     | Default  | Description                                    |
|------------------------------|:--------:|------------------------------------------------|
| `HX711_REF_UNIT`             | -165     | HX711 slope (calibrate with known weight)      |
| `HX711_ZERO_OFFSET`          | 8130430  | HX711 zero offset (tare empty platform)        |
| `FLAP_OPEN_SPEED_PWM`        | 2500     | 12-bit flap open PWM duty                      |
| `FLAP_OPEN_CURRENT_DROP_TH`  | 50       | ADC counts — open-circuit endpoint             |
| `FLAP_CLOSE_SPEED_PWM`       | 3000     | 12-bit flap close PWM duty                     |
| `FLAP_CLOSE_TORQUE_TH`       | 145      | ADC counts — torque/stall threshold            |
| `FLAP_MOTION_TIMEOUT_MS`     | 15000    | Max ms for flap motion before FAULT            |
| `ARM_STEPS_PRESS`            | -3500    | Steps from physical arm home hard-stop to press position |
| `ARM_STEPS_POS1`             | -500     | Steps from physical arm home hard-stop to pos 1 |
| `ARM_STEPS_POS2`             | -100     | Steps from physical arm home hard-stop to pos 2 |
| `ARM_HOME_SEARCH_STEPS`      | 5000     | Positive-direction search distance for arm sensorless homing |
| `ARM_HOME_BACKOFF_STEPS`     | 100      | Fixed release distance after the arm hits home |
| `ARM_HOME_TORQUE_LIMIT`      | 300      | Torque threshold used to detect the arm hard stop |
| `RACK_STEPS_EXTEND`          | 800      | Steps from home to extend position             |
| `TURNTABLE_STEPS_B`          | 400      | Steps from home to accept/bag position         |
| `HOTWIRE_ENABLE_DUTY`        | 4095     | Full on; current set by external Rsense        |
| `SPAWN_FLOW_NOFLOW_UG`       | 500000   | Min µg/window to count as flowing              |
| `SPAWN_MAX_RETRIES`          | 100      | Agitation retries before SPAWN_STATUS_BAG_EMPTY|
| `DRV8434S_SPI_WATCHDOG_INTERVAL_MS` | 2000 | Idle SPI health check period             |
| `VACUUM_RPM_BLOCKED_THRESHOLD` | 400   | RPM below which pump is reported BLOCKED       |

Override constants at compile time:

```bash
cmake .. -DPICO_BOARD=pico2 -DARM_STEPS_PRESS=600 -DRACK_STEPS_EXTEND=950
```

For the rotary arm, `MSG_ARM_HOME` defines the physical reference: home is the positive hard stop, logical zero is that stall point, and the firmware then releases to `-ARM_HOME_BACKOFF_STEPS`. Retune the named arm positions as zero-or-negative values from that reference.

---

## Bring-Up Order

1. Verify `MSG_PING` round-trip over UART
2. Test `MSG_FLAPS_OPEN` / `MSG_FLAPS_CLOSE` + `MSG_MOTION_DONE`
3. `MSG_TURNTABLE_HOME` → verify step counter zeroes
4. `MSG_TURNTABLE_GOTO` A→B→C→D
5. `MSG_RACK_MOVE` HOME (homing via stall), then EXTEND/PRESS
6. `MSG_ARM_HOME` from several starting positions; verify the arm hits the hard stop and backs off to `-ARM_HOME_BACKOFF_STEPS`
7. `MSG_ARM_MOVE` PRESS, then POS1/POS2
8. `MSG_HOTWIRE_SET` ON/OFF — verify DRV8263 IN1 output
9. `MSG_VACUUM_SET` ON/OFF — verify RPM telemetry `MSG_VACUUM_STATUS`
10. Integrate Operations screen, verify all buttons including `Arm Home`
11. Calibrate all `board_pins.h` position constants
12. Test `MSG_DISPENSE_SPAWN` end-to-end with `MSG_SPAWN_STATUS`
13. Run sys_sequence.c Sequence screen through a full bag cycle
