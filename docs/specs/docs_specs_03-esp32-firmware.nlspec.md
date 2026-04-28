# NLSpec: ESP32-C6 Firmware

## Version
0.1.3

## Depends On
`01-shared-protocol.nlspec.md`

---

## 1. Overview

The ESP32-C6 firmware runs FreeRTOS under ESP-IDF v5.5.x. It is the user-facing controller: it drives the touchscreen UI, sends commands to the Pico, receives status updates, and runs the high-level orchestration logic.

### 1.1 Source File Map

| Directory / File | Responsibility |
|-----------------|---------------|
| `lcd_uext_ili9341/main/app_main.c` | Entry point: init pico_link, display, UI, control task, touch |
| `lcd_uext_ili9341/main/ui_screens.c` | All LVGL screen definitions (Home, Operations, Dosing) |
| `lcd_uext_ili9341/main/ui_screens.h` | Screen function declarations |
| `lcd_uext_ili9341/main/ui.c` / `ui.h` | LVGL init, display registration |
| `lcd_uext_ili9341/main/display.c` / `display.h` | SPI bus + ILI9341 panel init |
| `lcd_uext_ili9341/main/touch_ns2009.c/h` | I2C resistive touch driver |
| `lcd_uext_ili9341/main/touch_cal.c/h` | Touch calibration NVS persistence |
| `lcd_uext_ili9341/main/sys_sequence.c/h` | **Preferred** system-level inoculation state machine (FreeRTOS task, whiteboard process flow) |
| `lcd_uext_ili9341/main/control.c/h` | Control task: command queue, legacy dosing orchestration |
| `lcd_uext_ili9341/main/dosing.c/h` | ESP-side dosing state machine (**LEGACY** — see dosing.h; preferred path is sys_sequence.c) |
| `lcd_uext_ili9341/main/flap.c/h` | Flap convenience functions (wraps motor_hal) |
| `lcd_uext_ili9341/main/loadcell.c/h` | Load cell reading via pico_link RPC |
| `lcd_uext_ili9341/main/recipes.c/h` | SPIFFS recipe storage (future) |
| `lcd_uext_ili9341/components/pico_link/` | UART transport component (COBS + CRC, FreeRTOS task) |
| `lcd_uext_ili9341/components/motor_hal/` | Motor HAL: `motor_hal_pico.c` wraps pico_link for actuator commands |
| `lcd_uext_ili9341/components/proto/` | Shared protocol header (symlinked or copied from `shared/proto/`) |

### 1.2 Execution Model

ESP-IDF FreeRTOS with the following tasks:

| Task | Priority | Stack | Responsibility |
|------|----------|-------|---------------|
| LVGL port task | (managed by `esp_lvgl_port`) | — | Renders UI, handles touch input |
| pico_link RX task | (managed by `pico_link`) | — | Reads UART bytes, decodes COBS frames, dispatches to callback |
| sys_seq | 5 | 4096 | **Preferred** system-level state machine (whiteboard process flow, FreeRTOS task) |
| control_task | 10 | 4096 | Legacy: polls command queue, ticks ESP-side dosing state machine |
| main (app_main) | — | — | Init only; enters idle loop (`vTaskDelay`) after setup |

---

## 2. Power Rail & Startup Sequence

### 2.1 12 V Motor Driver Power Control (GPIO10 Relay)

GPIO10 is configured as a push-pull digital output, driven LOW at reset. It drives the coil of a normally-open relay whose contact connects the 12 V supply to all motor driver ICs (DRV8424s and DRV8263). The drivers are unpowered until the ESP asserts GPIO10 HIGH.

| GPIO | Direction | Default | Effect when HIGH |
|------|-----------|---------|-----------------|
| 10 | Output | LOW (relay open) | Relay closes → 12 V applied to DRV8424s + DRV8263 |

The ESP asserts GPIO10 only after LVGL, the display, and the pico_link transport are fully initialised. This guarantees the Pico has already configured its GPIO pin modes and set safe output defaults before the motor drivers are powered.

### 2.2 Startup Sequence

The three-phase power-on order prevents any motor driver input from floating or glitching to an active state during the 12 V ramp:

| Phase | Actor | Action |
|-------|-------|--------|
| 1 | Pico | Configures all GPIO pin modes; drives IN2 (vacuum pump 2) LOW to hold pump off before 12 V arrives |
| 2 | ESP32 | Initialises pico_link transport, display (ILI9341 + NS2009), and LVGL UI; then asserts GPIO10 HIGH to close the relay and apply 12 V to motor drivers |
| 3 | Pico | Detects controller power-on and performs DRV8424 / DRV8263 SPI chain initialisation |

Phase 1 must complete before Phase 2 asserts GPIO10. In practice this is guaranteed by the Pico's boot being faster than the ESP's full UI init; no explicit handshake is required, but the design relies on this ordering.

---

## 3. pico_link Transport Component

### 3.1 Initialization

```c
pico_link_cfg_t link = {
    .uart_num  = UART_NUM_1,
    .tx_gpio   = 5,
    .rx_gpio   = 4,
    .baud      = 115200,
    .on_rx     = pico_rx_cb,
};
ESP_ERROR_CHECK(pico_link_init(&link));
```

### 3.2 Send API

| Function | Behavior |
|----------|----------|
| `pico_link_send(type, payload, len, NULL)` | Fire-and-forget: encode and transmit, no ACK wait |
| `pico_link_send_rpc(type, payload, len, timeout_ms, &nack_code)` | Blocking RPC: send, wait for ACK/NACK up to `timeout_ms`. Returns `ESP_OK` on ACK, `ESP_ERR_*` on NACK or timeout. `nack_code` is populated on NACK. |

### 3.3 Receive Callback

`pico_rx_cb(type, seq, payload, len)` is called from the pico_link RX task for every decoded frame from the Pico. The callback in `app_main.c` dispatches based on `type`:

| Message Type | Handler |
|-------------|---------|
| `MSG_MOTION_DONE` | `sys_sequence_notify_motion_done()` — notify sys_seq task; `ui_ops_on_motion_done()` — update Operations screen status |
| `MSG_VACUUM_STATUS` | `ui_ops_on_vacuum_status()` — update vacuum status label |
| `MSG_ARM_SEAL_EVENT` | `sys_sequence_notify_arm_seal_event()` — notify bag-open retry logic; `ui_ops_on_arm_seal_event()` — operator visibility |
| `MSG_SPAWN_STATUS` | `sys_sequence_notify_spawn_status()` — notify sys_seq task (preferred path); `ui_dosing_on_spawn_status()` — update Dosing screen (legacy path) |
| All others | `ui_screens_pico_rx_handler()` — weight display, generic status |

---

## 4. motor_hal Component

The `motor_hal` component provides ESP-side convenience functions that compose `pico_link_send()` calls with the correct message types and payloads. These are the functions UI button callbacks and the control task call.

| Function | Message Sent | Payload |
|----------|-------------|---------|
| `motor_flap_open()` | `MSG_FLAPS_OPEN` | none |
| `motor_flap_close()` | `MSG_FLAPS_CLOSE` | none |
| `motor_arm_move(arm_pos_t pos)` | `MSG_ARM_MOVE` | `pl_arm_move_t{position=pos}` |
| `motor_arm_home(void)` | `MSG_ARM_HOME` | none |
| `motor_rack_move(rack_pos_t pos)` | `MSG_RACK_MOVE` | `pl_rack_move_t{position=pos}` |
| `motor_turntable_goto(turntable_pos_t pos)` | `MSG_TURNTABLE_GOTO` | `pl_turntable_goto_t{position=pos}` |
| `motor_turntable_home()` | `MSG_TURNTABLE_HOME` | none |
| `motor_hotwire_set(bool enable)` | `MSG_HOTWIRE_SET` | `pl_hotwire_set_t{enable}` |
| `motor_vacuum_set(bool enable)` | `MSG_VACUUM_SET` | `pl_vacuum_set_t{enable}` |
| `motor_vacuum2_set(bool enable)` | `MSG_VACUUM2_SET` | `pl_vacuum2_set_t{enable}` |
| `motor_hotwire_traverse(bool cut)` | `MSG_HOTWIRE_TRAVERSE` | `pl_hotwire_traverse_t{direction}` |
| `motor_indexer_move(uint8_t position)` | `MSG_INDEXER_MOVE` | `pl_indexer_move_t{position}` |
| `motor_agitate(void)` | `MSG_AGITATE` | `pl_agitate_t{flags=0}` (knead only, no home) |
| `motor_agitate_home(void)` | `MSG_AGITATE` | `pl_agitate_t{flags=AGITATE_FLAG_DO_HOME}` (home then knead) |
| `motor_linact_start_monitor_dir(dir, speed, low, high, interval)` | `MSG_MOTOR_DRV8263_START_MON` | `pl_drv8263_start_mon_t` (legacy) |
| `motor_linact_stop_monitor()` | `MSG_MOTOR_DRV8263_STOP_MON` | none (legacy) |

`motor_arm_home()` is explicit/operator-invoked only. Boot still allows arm moves before the first home for backward compatibility, but after an arm `MOTION_STALLED`, `MOTION_TIMEOUT`, `MOTION_FAULT`, or `MOTION_SPI_FAULT`, later `motor_arm_move()` requests are NACKed until `motor_arm_home()` succeeds.

All `motor_*` functions return `esp_err_t`. They are non-blocking (`pico_link_send`, not `_rpc`) except APIs that explicitly use RPC semantics in implementation.

---

## 5. UI Screens (LVGL)

### 5.1 Display Hardware

| Parameter | Value |
|-----------|-------|
| Controller | ILI9341 |
| Interface | SPI2 (MOSI=GP18, CLK=GP19, CS=GP21, DC=GP20) |
| Resolution | 320 × 240 pixels (landscape) |
| Color format | RGB565 |
| Touch | NS2009 resistive, I2C (SDA=GP6, SCL=GP7, addr=0x48) |

### 5.2 Screen Map

```
Home Screen
├── Automated Functions Screen  (sys_sequence.c — supervised inoculation)
├── Operations Screen           (direct actuator control — manual testing)
├── Dosing Screen               (LEGACY manual spawn dispensing — isolated testing)
└── Scale Screen                (HX711 weight readout + tare)
```

### 5.3 Home Screen (`ui_show_home()`)

**Layout:** Status label at top, four navigation buttons (300 × 46 px each, 4 px gap).

| Widget | Label | Callback | Action |
|--------|-------|----------|--------|
| Label | "IDLE • Seacoast Inoculator" | — | `lbl_status` updated by `ui_status_set()` |
| Button | "Automated Functions" | `on_auto_page()` | Navigate to Automated Functions screen |
| Button | "Scale" | `on_scale_page()` | Navigate to Scale screen |
| Button | "Operations" | `on_ops_page()` | Navigate to Operations screen (blocked while sequence active) |
| Button | "Dosing" | `on_dose()` | Navigate to Dosing screen (blocked while sequence active) |

### 5.4 Operations Screen (`ui_show_operations()`)

**Layout:** 320 × 240, landscape, 10px padding. Title "Operations" top-left, "Home" nav button top-right. Rows of actuator buttons, status bar at bottom.

| Section | Button Label | Callback | Message |
|---------|-------------|----------|---------|
| FLAPS | "Flap Open" | `on_flap_open()` | `motor_flap_open()` |
| FLAPS | "Flap Close" | `on_flap_close()` | `motor_flap_close()` |
| ARM | "Arm Home" | `on_arm_home()` | `motor_arm_home()` |
| ARM | "Arm Press" | `on_arm_press()` | `motor_arm_move(ARM_POS_PRESS)` |
| ARM | "Arm Pos 1" | `on_arm_pos1()` | `motor_arm_move(ARM_POS_1)` |
| ARM | "Arm Pos 2" | `on_arm_pos2()` | `motor_arm_move(ARM_POS_2)` |
| HOT WIRE | "Hotwire Home" | `on_hotwire_home()` | `motor_hotwire_move(HOTWIRE_POS_HOME)` |
| HOT WIRE | "Hot Extend" | `on_hotwire_extend()` | `motor_hotwire_move(HOTWIRE_POS_EXTEND)` |
| HOT WIRE | "Hot Press" | `on_hotwire_retrect()` | `motor_hotwire_move(HOTWIRE_POS_RETRACT)` |
| HOT WIRE | "HotWire ON" | `on_hotwire_on()` | `motor_hotwire_set(true)` |
| HOT WIRE | "HotWire OFF" | `on_hotwire_off()` | `motor_hotwire_set(false)` |
| VACUUM | "Vacuum ON" | `on_vacuum_on()` | `motor_vacuum_set(true)` |
| VACUUM | "Vacuum OFF" | `on_vacuum_off()` | `motor_vacuum_set(false)` |
| VACUUM2 | "Vacuum2 ON" | `on_vacuum2_on()` | `motor_vacuum2_set(true)` |
| VACUUM2 | "Vacuum2 OFF" | `on_vacuum2_off()` | `motor_vacuum2_set(false)` |
| CALIBRATE | "Home All" | `on_home()` | `motor_home_ all()` |

**Status feedback:** When `MSG_MOTION_DONE` is received, `ui_ops_on_motion_done()` updates the status label with subsystem name and result (e.g., "ARM: OK" or "FLAPS: TIMEOUT"). When `MSG_VACUUM_STATUS` is received, `ui_ops_on_vacuum_status()` updates the vacuum status label (e.g., "Vacuum: OK 1200 RPM" or "Vacuum: BLOCKED 350 RPM"). If the arm faults or stalls during a normal move, the operator must use the `"Arm Home"` button before later arm-position buttons will be accepted again.

### 5.5 Automated Functions Screen (`ui_show_auto()`)

**Purpose:** Supervise the full inoculation sequence via `sys_sequence.c`. Disabled while sequence is not active.

**Layout:** Title "Automated Functions", status label, three action buttons.

| Widget | Label | Callback | Action |
|--------|-------|----------|--------|
| Button | "Setup / Load" | `on_setup_load()` | `sys_sequence_send_cmd(SYS_CMD_SETUP_LOAD)` |
| Button | "Start" | `on_seq_start()` | `sys_sequence_send_cmd(SYS_CMD_START)` |
| Button | "Abort" | `on_seq_abort()` | `sys_sequence_send_cmd(SYS_CMD_ABORT)` → sequence transitions to `SYS_IDLE`; all actuators safe-stopped |

> **Abort routing note:** `on_seq_abort()` sends `SYS_CMD_ABORT` to the sequence task queue via `sys_sequence_send_cmd()`. The sequence task's main loop checks for this command at the top of every iteration and calls `safe_stop_all()` before transitioning back to `SYS_IDLE`, immediately re-enabling manual controls. (An earlier implementation incorrectly sent `CTRL_CMD_STOP` to the legacy control task queue, which had no effect on the sequence state machine.)

#### 5.5.1 `sys_sequence` State Table (`sys_state_t`)

| Value | State | Description |
|-------|-------|-------------|
| 0 | `SYS_IDLE` | Inactive; manual controls enabled |
| 1 | `SYS_SETUP_LOAD` | Load spawn bag, home/position subsystems |
| 2 | `SYS_CUTTING_TIP` | Heat hot wire and traverse carriage to cut bag tip |
| 3 | `SYS_ROTATING_TO_ACCEPT` | Move turntable to `TURNTABLE_POS_INTAKE` |
| 4 | `SYS_INTAKE_WAITING` | Wait for substrate bag weight signal |
| 5 | `SYS_INTAKE_WEIGHING` | Measure bag mass via `MSG_HX711_MEASURE` |
| 6 | `SYS_OPENING_BAG` | Multi-step bag opening: arm press, vacuum on, rack extend |
| 7 | `SYS_OPEN_RECOVERING` | Seal-loss recovery: backoff / re-press / re-open retry |
| 8 | `SYS_INOCULATING` | Closed-loop spawn dosing via `MSG_DISPENSE_SPAWN` |
| 9 | `SYS_POST_DOSE` | Close flaps, return arm and rack to neutral |
| 10 | `SYS_EJECTING` | Move turntable to `TURNTABLE_POS_EJECT`, push bag out |
| 11 | `SYS_ROTATING_TO_INTAKE` | Return turntable to intake position for next bag |
| 12 | `SYS_SPAWN_EMPTY` | Prompt operator to replace spawn bag |
| 13 | `SYS_CONTINUE_RESTART` | Rotate to `TURNTABLE_POS_TRASH`, open flaps, restart cycle |
| 14 | `SYS_ERROR` | Safe-stop all actuators; operator intervention required |
| 15 | `SYS_ESTOP` | Emergency stop |

### 5.6 Dosing Screen (`ui_show_dosing()`)

**Purpose:** Isolated manual spawn dispensing for testing purposes (LEGACY path — see `dosing.h`). Accessible from Home Screen; blocked while a sequence is active.

**Layout:**
- Inoculation percentage selector (buttons, range 1–500 in tenths of percent, default: 200 = 20.0%)
- Bag number display (auto-incrementing)
- "Start Dose" button → `pico_link_send_rpc(MSG_DISPENSE_SPAWN, …)`
- "Abort" button → `pico_link_send_rpc(MSG_CTRL_STOP, …)` — sends `MSG_CTRL_STOP` to the Pico, which aborts the Pico-side spawn state machine, stops the dosing timer, and fast-closes the flaps. Closing flaps alone via `motor_flap_close()` is insufficient because the Pico spawn SM continues running until it receives `MSG_CTRL_STOP`.
- Status label — updated by `ui_dosing_on_spawn_status()`:

| `spawn_status_code_t` | Display Text | Color |
|----------------------|--------------|-------|
| `SPAWN_STATUS_RUNNING` | "Dosing: Xg / Yg" | White |
| `SPAWN_STATUS_DONE` | "DONE — Xg dispensed" | Green |
| `SPAWN_STATUS_STALLED` | "STALLED — agitating…" | Yellow |
| `SPAWN_STATUS_AGITATING` | "Agitating (retry N)" | Yellow |
| `SPAWN_STATUS_BAG_EMPTY` | "BAG EMPTY" | Red |
| `SPAWN_STATUS_ERROR` | "ERROR" | Red |
| `SPAWN_STATUS_FLOW_FAILURE` | "NO FLOW — check bag" | Red |
| `SPAWN_STATUS_ABORTED` | "ABORTED" | Orange |

---

## 6. Control Task

### 6.1 Command Queue

The control task receives `ctrl_cmd_t` structs via a FreeRTOS queue (depth: 8).

| Command | Constant | Behavior |
|---------|----------|----------|
| Tare | `CTRL_CMD_TARE` | `loadcell_tare()` → sends `MSG_HX711_TARE` |
| Start | `CTRL_CMD_START` | `dosing_start(&s_dose, target_g)` — starts ESP-side dosing loop |
| Stop | `CTRL_CMD_STOP` | `dosing_abort(&s_dose)` — closes flaps, sends `MSG_CTRL_STOP` |
| Pause | `CTRL_CMD_PAUSE` | `dosing_abort(&s_dose)` — same as stop for now |
| Clean | `CTRL_CMD_CLEAN` | Future — not implemented |
| Home | `CTRL_CMD_HOME` | Future — not implemented |

### 6.2 ESP-Side Dosing (Legacy)

The `dosing.c` module implements an ESP-side dosing state machine that reads the load cell via `loadcell_read_g()` and commands flap opening via `flap_set_opening()`. This is a **legacy path** used when the control task's START command is invoked. The Pico-side `MSG_DISPENSE_SPAWN` path (triggered from the Dosing screen) is the **preferred path** for new development.

**Dosing States:** `IDLE`, `PRIME`, `FAST`, `TAPER`, `FINAL`, `DONE`, `ABORTED`, `ERROR`

**Defaults:**

| Parameter | Default | Unit |
|-----------|---------|------|
| `taper_start_g` | 8.0 | grams before target to begin tapering |
| `final_band_g` | 2.0 | grams before target to enter final slow mode |
| `overshoot_g` | 0.7 | grams over target before hard stop |
| `fast_open` | 1.0 | fraction (0–1) flap opening during FAST |
| `taper_open` | 0.40 | fraction during TAPER |
| `final_open` | 0.15 | fraction during FINAL |
| `min_step_ms` | 50 | ms minimum between actuator commands |
| `ema_alpha` | 0.25 | EMA smoothing factor for weight readings |

**The control task ticks the dosing state machine every 20 ms** (`pdMS_TO_TICKS(20)`).

---

## 7. Async Status Handling

The ESP32 receives unsolicited messages from the Pico that must be routed to the correct UI handler:

| Message | Routing |
|---------|---------|
| `MSG_MOTION_DONE` | Extract `pl_motion_done_t`, call `ui_ops_on_motion_done()` |
| `MSG_VACUUM_STATUS` | Extract `pl_vacuum_status_t`, call `ui_ops_on_vacuum_status()` |
| `MSG_ARM_SEAL_EVENT` | Extract `pl_arm_seal_event_t`, call `ui_ops_on_arm_seal_event()`, and notify `sys_sequence_notify_arm_seal_event()` |
| `MSG_SPAWN_STATUS` | Extract `pl_spawn_status_t`, call `ui_dosing_on_spawn_status()` |
| `MSG_HX711_MEASURE` ACK | Extract `pl_hx711_mass_t`, update weight label via `ui_screens_pico_rx_handler()` |

## 7.1 Bag-Opening Recovery on Arm Seal Loss

When `MSG_ARM_SEAL_EVENT(LOST)` arrives during bag opening:
- transition to `SYS_OPEN_RECOVERING`,
- execute deterministic retry sequence (backoff/re-press/re-open),
- retry at most `ARM_OPEN_RETRY_MAX` (default 3),
- if still failing, transition to `SYS_ERROR` and safe-stop.

`MSG_ARM_SEAL_EVENT(RESTORED)` is surfaced to the UI and clears retry-latched context on the Pico side.

All UI updates from the RX callback must acquire the LVGL lock (`lvgl_port_lock(0)`) before modifying widgets and release it (`lvgl_port_unlock()`) after.

---

## 8. Intentional Ambiguity

The following are explicitly left to implementer choice:

- LVGL widget colors, font sizes, and exact pixel positioning (the layout in Section 4 is schematic, not pixel-exact)
- Internal structure of `pico_link` FreeRTOS task (buffer sizes, ring buffer implementation)
- Whether recipes use SPIFFS, NVS, or another persistence mechanism
- Touch calibration procedure UX
- The specific EMA alpha and dosing thresholds (defaults are provided; they are starting points for hardware calibration)

---

## 9. Definition of Done

### 9.1 Transport
- [ ] `pico_link_init()` establishes UART communication at 115200 baud
- [ ] `pico_link_send()` successfully transmits and receives ACK for MSG_PING
- [ ] `pico_link_send_rpc()` blocks and returns ESP_OK on ACK, ESP_ERR_TIMEOUT on timeout

### 9.2 UI Screens
- [ ] Home screen displays all buttons listed in Section 4.3
- [ ] Operations screen displays all actuator buttons listed in Section 4.4
- [ ] Operations screen updates status label on MSG_MOTION_DONE receipt
- [ ] Operations screen updates vacuum status on MSG_VACUUM_STATUS receipt
- [ ] Operations/auto UI surfaces `MSG_ARM_SEAL_EVENT` with reason (transient/steady/stale tach)
- [ ] Dosing screen allows setting inoculation percentage and starting a dose
- [ ] Dosing screen displays spawn status for all 8 status codes per Section 4.5

### 9.3 Sequence Recovery (Bag Opening)
- [ ] `MSG_ARM_SEAL_EVENT(LOST)` during bag opening transitions to `SYS_OPEN_RECOVERING`
- [ ] Sequence retries bag opening at most `ARM_OPEN_RETRY_MAX` times
- [ ] Exceeded retries transitions to `SYS_ERROR` with safe-stop

### 9.4 motor_hal
- [ ] All motor_hal functions listed in Section 3 compile and send the correct message type with correct payload

### 9.5 Control Task
- [ ] control_task processes CTRL_CMD_START, CTRL_CMD_STOP, CTRL_CMD_PAUSE, CTRL_CMD_TARE
- [ ] dosing_tick() is called at 20 ms intervals while running
- [ ] dosing_abort() closes flaps and sends MSG_CTRL_STOP

### 9.6 Power Sequencing
- [ ] GPIO10 configured as output, default LOW at boot
- [ ] GPIO10 asserted HIGH only after pico_link, display, and UI init complete
- [ ] Relay de-asserted (GPIO10 LOW) during any safe-stop or shutdown path

### 9.7 Build
- [ ] Firmware compiles cleanly with ESP-IDF v5.5.x targeting ESP32-C6
- [ ] LVGL v9.4.x via esp_lvgl_port v2.7.x
- [ ] All ESP_LOGx tags match source file names
- [ ] No compiler warnings with default ESP-IDF warning level
