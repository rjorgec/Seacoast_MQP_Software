# NLSpec: ESP32-C6 Firmware

## Version
0.1.1

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

## 2. pico_link Transport Component

### 2.1 Initialization

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

### 2.2 Send API

| Function | Behavior |
|----------|----------|
| `pico_link_send(type, payload, len, NULL)` | Fire-and-forget: encode and transmit, no ACK wait |
| `pico_link_send_rpc(type, payload, len, timeout_ms, &nack_code)` | Blocking RPC: send, wait for ACK/NACK up to `timeout_ms`. Returns `ESP_OK` on ACK, `ESP_ERR_*` on NACK or timeout. `nack_code` is populated on NACK. |

### 2.3 Receive Callback

`pico_rx_cb(type, seq, payload, len)` is called from the pico_link RX task for every decoded frame from the Pico. The callback in `app_main.c` dispatches based on `type`:

| Message Type | Handler |
|-------------|---------|
| `MSG_MOTION_DONE` | `sys_sequence_notify_motion_done()` — notify sys_seq task; `ui_ops_on_motion_done()` — update Operations screen status |
| `MSG_VACUUM_STATUS` | `ui_ops_on_vacuum_status()` — update vacuum status label |
| `MSG_SPAWN_STATUS` | `sys_sequence_notify_spawn_status()` — notify sys_seq task (preferred path); `ui_dosing_on_spawn_status()` — update Dosing screen (legacy path) |
| All others | `ui_screens_pico_rx_handler()` — weight display, generic status |

---

## 3. motor_hal Component

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
| `motor_linact_start_monitor_dir(dir, speed, low, high, interval)` | `MSG_MOTOR_DRV8263_START_MON` | `pl_drv8263_start_mon_t` (legacy) |
| `motor_linact_stop_monitor()` | `MSG_MOTOR_DRV8263_STOP_MON` | none (legacy) |
| `motor_stepper_enable(bool en)` | `MSG_MOTOR_STEPPER_ENABLE` | `pl_stepper_enable_t{enable}` (legacy/raw stepper API) |
| `motor_stepper_step(motor_dir_t dir, uint32_t steps, uint32_t step_delay_us)` | `MSG_MOTOR_STEPPER_STEPJOB` | `pl_stepper_stepjob_t{dir,steps,step_delay_us,torque_limit}` (legacy/raw stepper API) |

For `motor_stepper_step()`, the fallback `torque_limit` value is sourced from `PROTO_STEPPER_SOFT_TORQUE_LIMIT_DEFAULT` in the shared protocol header (via `STEPPER_SOFT_TORQUE_LIMIT`), keeping ESP and Pico default behavior aligned.

`motor_arm_home()` is explicit/operator-invoked only. Boot still allows arm moves before the first home for backward compatibility, but after an arm `MOTION_STALLED`, `MOTION_TIMEOUT`, `MOTION_FAULT`, or `MOTION_SPI_FAULT`, later `motor_arm_move()` requests are NACKed until `motor_arm_home()` succeeds.

All `motor_*` functions return `esp_err_t`. They are non-blocking (`pico_link_send`, not `_rpc`) except APIs that explicitly use RPC semantics in implementation.

---

## 4. UI Screens (LVGL)

### 4.1 Display Hardware

| Parameter | Value |
|-----------|-------|
| Controller | ILI9341 |
| Interface | SPI2 (MOSI=GP18, CLK=GP19, CS=GP21, DC=GP20) |
| Resolution | 320 × 240 pixels (landscape) |
| Color format | RGB565 |
| Touch | NS2009 resistive, I2C (SDA=GP6, SCL=GP7, addr=0x48) |

### 4.2 Screen Map

```
Home Screen
├── Operations Screen   (direct actuator control — manual testing)
├── Dosing Screen       (LEGACY manual spawn dispensing — see dosing.h)
├── Sequence Screen     (NEW — sys_sequence.c whiteboard process flow)
└── (Recipes — future, not specified here)
```

### 4.3 Home Screen (`ui_show_home()`)

**Layout:** Button grid with status labels at bottom.

| Widget | Label | Callback | Action |
|--------|-------|----------|--------|
| Button | "START" | `on_start()` | Send `CTRL_CMD_START` to control queue |
| Button | "PAUSE" | `on_pause()` | Send `CTRL_CMD_PAUSE` to control queue |
| Button | "STOP" | `on_stop()` | Send `CTRL_CMD_STOP` to control queue |
| Button | "TARE" | `on_tare()` | `pico_link_send_rpc(MSG_HX711_TARE, …)` |
| Button | "WEIGHT" | `on_read_weight()` | `pico_link_send(MSG_HX711_MEASURE, …)` |
| Button | "DOSE" | `on_dose()` | Navigate to Dosing screen |
| Button | "OPERATIONS" | `on_ops_page()` | Navigate to Operations screen |
| Label | "Weight: -- g" | — | Updated on `MSG_HX711_MEASURE` response |
| Label | (status bar) | — | Updated by `ui_status_set()` |

### 4.4 Operations Screen (`ui_show_operations()`)

**Layout:** 320 × 240, landscape, 10px padding. Title "Operations" top-left, "Home" nav button top-right. Rows of actuator buttons, status bar at bottom.

| Section | Button Label | Callback | Message |
|---------|-------------|----------|---------|
| FLAPS | "Flap Open" | `on_flap_open()` | `motor_flap_open()` |
| FLAPS | "Flap Close" | `on_flap_close()` | `motor_flap_close()` |
| ARM | "Arm Home" | `on_arm_home()` | `motor_arm_home()` |
| ARM | "Arm Press" | `on_arm_press()` | `motor_arm_move(ARM_POS_PRESS)` |
| ARM | "Arm Pos 1" | `on_arm_pos1()` | `motor_arm_move(ARM_POS_1)` |
| ARM | "Arm Pos 2" | `on_arm_pos2()` | `motor_arm_move(ARM_POS_2)` |
| RACK | "Rack Home" | `on_rack_home()` | `motor_rack_move(RACK_POS_HOME)` |
| RACK | "Rack Extend" | `on_rack_extend()` | `motor_rack_move(RACK_POS_EXTEND)` |
| RACK | "Rack Press" | `on_rack_press()` | `motor_rack_move(RACK_POS_PRESS)` |
| TURNTABLE | "Pos A" – "Pos D" | `on_tt_a()` – `on_tt_d()` | `motor_turntable_goto(POS_A..D)` |
| HOT WIRE | "HotWire ON" | `on_hotwire_on()` | `motor_hotwire_set(true)` |
| HOT WIRE | "HotWire OFF" | `on_hotwire_off()` | `motor_hotwire_set(false)` |
| VACUUM | "Vacuum ON" | `on_vacuum_on()` | `motor_vacuum_set(true)` |
| VACUUM | "Vacuum OFF" | `on_vacuum_off()` | `motor_vacuum_set(false)` |
| CALIBRATE | "Turntable Home" | `on_tt_home()` | `motor_turntable_home()` |

**Status feedback:** When `MSG_MOTION_DONE` is received, `ui_ops_on_motion_done()` updates the status label with subsystem name and result (e.g., "ARM: OK" or "FLAPS: TIMEOUT"). When `MSG_VACUUM_STATUS` is received, `ui_ops_on_vacuum_status()` updates the vacuum status label (e.g., "Vacuum: OK 1200 RPM" or "Vacuum: BLOCKED 350 RPM"). If the arm faults or stalls during a normal move, the operator must use the `"Arm Home"` button before later arm-position buttons will be accepted again.

### 4.5 Dosing Screen (`ui_show_dosing()`)

**Purpose:** Configure and launch Pico-side spawn dosing, display real-time progress.

**Layout:**
- Inoculation percentage selector (spinner or buttons, range 1–500 in tenths of percent, default: 100 = 10.0%)
- Bag number display (auto-incrementing)
- "Start Dose" button → `pico_link_send_rpc(MSG_DISPENSE_SPAWN, …)`
- "Abort" button → `motor_flap_close()` (forces flap close, Pico dose timer detects and terminates)
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

## 5. Control Task

### 5.1 Command Queue

The control task receives `ctrl_cmd_t` structs via a FreeRTOS queue (depth: 8).

| Command | Constant | Behavior |
|---------|----------|----------|
| Tare | `CTRL_CMD_TARE` | `loadcell_tare()` → sends `MSG_HX711_TARE` |
| Start | `CTRL_CMD_START` | `dosing_start(&s_dose, target_g)` — starts ESP-side dosing loop |
| Stop | `CTRL_CMD_STOP` | `dosing_abort(&s_dose)` — closes flaps, sends `MSG_CTRL_STOP` |
| Pause | `CTRL_CMD_PAUSE` | `dosing_abort(&s_dose)` — same as stop for now |
| Clean | `CTRL_CMD_CLEAN` | Future — not implemented |
| Home | `CTRL_CMD_HOME` | Future — not implemented |

### 5.2 ESP-Side Dosing (Legacy)

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

## 6. Async Status Handling

The ESP32 receives unsolicited messages from the Pico that must be routed to the correct UI handler:

| Message | Routing |
|---------|---------|
| `MSG_MOTION_DONE` | Extract `pl_motion_done_t`, call `ui_ops_on_motion_done()` |
| `MSG_VACUUM_STATUS` | Extract `pl_vacuum_status_t`, call `ui_ops_on_vacuum_status()` |
| `MSG_SPAWN_STATUS` | Extract `pl_spawn_status_t`, call `ui_dosing_on_spawn_status()` |
| `MSG_HX711_MEASURE` ACK | Extract `pl_hx711_mass_t`, update weight label via `ui_screens_pico_rx_handler()` |

All UI updates from the RX callback must acquire the LVGL lock (`lvgl_port_lock(0)`) before modifying widgets and release it (`lvgl_port_unlock()`) after.

---

## 7. Intentional Ambiguity

The following are explicitly left to implementer choice:

- LVGL widget colors, font sizes, and exact pixel positioning (the layout in Section 4 is schematic, not pixel-exact)
- Internal structure of `pico_link` FreeRTOS task (buffer sizes, ring buffer implementation)
- Whether recipes use SPIFFS, NVS, or another persistence mechanism
- Touch calibration procedure UX
- The specific EMA alpha and dosing thresholds (defaults are provided; they are starting points for hardware calibration)

---

## 8. Definition of Done

### 8.1 Transport
- [ ] `pico_link_init()` establishes UART communication at 115200 baud
- [ ] `pico_link_send()` successfully transmits and receives ACK for MSG_PING
- [ ] `pico_link_send_rpc()` blocks and returns ESP_OK on ACK, ESP_ERR_TIMEOUT on timeout

### 8.2 UI Screens
- [ ] Home screen displays all buttons listed in Section 4.3
- [ ] Operations screen displays all actuator buttons listed in Section 4.4
- [ ] Operations screen updates status label on MSG_MOTION_DONE receipt
- [ ] Operations screen updates vacuum status on MSG_VACUUM_STATUS receipt
- [ ] Dosing screen allows setting inoculation percentage and starting a dose
- [ ] Dosing screen displays spawn status for all 8 status codes per Section 4.5

### 8.3 motor_hal
- [ ] All motor_hal functions listed in Section 3 compile and send the correct message type with correct payload

### 8.4 Control Task
- [ ] control_task processes CTRL_CMD_START, CTRL_CMD_STOP, CTRL_CMD_PAUSE, CTRL_CMD_TARE
- [ ] dosing_tick() is called at 20 ms intervals while running
- [ ] dosing_abort() closes flaps and sends MSG_CTRL_STOP

### 8.5 Build
- [ ] Firmware compiles cleanly with ESP-IDF v5.5.x targeting ESP32-C6
- [ ] LVGL v9.4.x via esp_lvgl_port v2.7.x
- [ ] All ESP_LOGx tags match source file names
- [ ] No compiler warnings with default ESP-IDF warning level
