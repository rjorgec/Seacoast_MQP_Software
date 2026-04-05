# NLSpec: Raspberry Pi Pico 2 Firmware

## Version
0.1.2

## Depends On
`01-shared-protocol.nlspec.md`

---

## 1. Overview

The Pico firmware runs bare-metal C on the RP2040 (ARM core of the Pico 2). It is the sole controller of all physical actuators and sensors. It receives commands from the ESP32 over UART, executes them via hardware drivers, and reports results back.

### 1.1 Source File Map

| File | Responsibility |
|------|---------------|
| `pico_fw/src/main.c` | Entry point; calls `uart_server_init()`, runs polling loop calling `uart_server_poll()` |
| `pico_fw/src/uart_server.c` | UART RX/TX, COBS framing, message dispatch, all subsystem state machines, spawn dosing algorithm |
| `pico_fw/src/uart_server.h` | Public API: `uart_server_init()`, `uart_server_poll()` |
| `pico_fw/src/board_pins.h` | All pin assignments and tuning constants |
| `pico_fw/drivers/drv8263/drv8263.c` | DRV8263 H-bridge driver (PWM + current monitoring) |
| `pico_fw/drivers/drv8434s/drv8434s.c` | DRV8434S SPI stepper driver (daisy-chain) |
| `shared/proto/proto.h` | Protocol definitions (shared with ESP32) |
| `shared/proto/cobs.c` | COBS encode/decode |
| `shared/proto/proto_crc.c` | CRC-16-CCITT |

**Stepper torque default source:** The fallback soft torque-limit threshold is defined once in `shared/proto/proto.h` as `PROTO_STEPPER_SOFT_TORQUE_LIMIT_DEFAULT` and consumed by Pico (`STEPPER_SOFT_TORQUE_LIMIT` fallback in `uart_server.c`).

### 1.2 Execution Model

The Pico runs a **cooperative polling loop** in `main()`:

```
while (true) {
    uart_server_poll();   // process incoming bytes, dispatch messages, tick state machines
    sleep_ms(5);          // yield ~5 ms between iterations
}
```

There is no RTOS. All subsystem state machines are polled every loop iteration via tick functions. Stepper pulse generation uses the Pico SDK `repeating_timer` at per-axis `step_delay_us` intervals.

**Boot sequence:**
1. `stdio_init_all()` — enables USB CDC for debug printf
2. `sleep_ms(10000)` — 10-second boot delay for USB enumeration
3. `uart_server_init()` — initializes UART, GPIO, ADC, SPI, DRV8263 instances, DRV8434S daisy chain, HX711 load cell
4. Enter polling loop

---

## 2. Hardware Subsystems

### 2.1 Flap Actuators (DRV8263 H-Bridge)

**Hardware:** Two linear actuators driven by one or two DRV8263 H-bridge ICs with current sense ADC.

**Behavior:**
- **Open (MSG_FLAPS_OPEN):** Drive motor forward at `FLAP_OPEN_SPEED` (default: 4095 = full 12-bit PWM). Enable current monitoring. When ADC reading drops below `FLAP_OPEN_LOW_TH` (default: 30), the actuator has reached end-of-travel (internal limit switch opened). Auto-stop and report `MOTION_OK` or `MOTION_CURRENT_STOP`.
- **Close (MSG_FLAPS_CLOSE):** Drive motor reverse at `FLAP_CLOSE_SPEED` (default: 4095). Enable current monitoring. When ADC reading exceeds `FLAP_CLOSE_HIGH_TH` (default: 300), the actuator has reached mechanical stop. Auto-stop and report.
- **Timeout:** If neither threshold triggers within `FLAP_TIMEOUT_MS` (default: 8000 ms), enter FAULT state and report `MOTION_TIMEOUT`.

**Flap State Machine States:** `IDLE`, `OPENING`, `OPEN`, `CLOSING`, `CLOSED`, `FAULT`

**Transition Table:**

| From | Event | To | Side Effect |
|------|-------|-----|-------------|
| IDLE | MSG_FLAPS_OPEN | OPENING | Start DRV8263 FWD; start timeout; ACK |
| IDLE | MSG_FLAPS_CLOSE | CLOSING | Start DRV8263 REV; start timeout; ACK |
| OPENING | Current drops below low threshold | OPEN | Stop motor; send MOTION_DONE(FLAPS, OK) |
| OPENING | Timeout | FAULT | Stop motor; send MOTION_DONE(FLAPS, TIMEOUT) |
| OPENING | MSG_FLAPS_CLOSE | CLOSING | Reverse motor; reset timer; ACK |
| OPEN | MSG_FLAPS_CLOSE | CLOSING | Start REV; start timer; ACK |
| OPEN | MSG_FLAPS_OPEN | OPEN | No-op; send MOTION_DONE(FLAPS, OK); ACK |
| CLOSING | Current exceeds high threshold | CLOSED | Stop motor; send MOTION_DONE(FLAPS, OK) |
| CLOSING | Timeout | FAULT | Stop motor; send MOTION_DONE(FLAPS, TIMEOUT) |
| CLOSING | MSG_FLAPS_OPEN | OPENING | Reverse motor; reset timer; ACK |
| CLOSED | MSG_FLAPS_OPEN | OPENING | Start FWD; start timer; ACK |
| CLOSED | MSG_FLAPS_CLOSE | CLOSED | No-op; send MOTION_DONE(FLAPS, OK); ACK |
| FAULT | MSG_FLAPS_OPEN | OPENING | Clear fault; start FWD; ACK |
| FAULT | MSG_FLAPS_CLOSE | CLOSING | Clear fault; start REV; ACK |

**Second flap driver (`s_drv8263_flap2`):** If present and initialized (`s_drv8263_flap2_ready == true`), both flap DRV8263 instances are commanded in parallel for every flap state transition. Both must complete before MOTION_DONE is sent.

### 2.2 Arm Stepper (DRV8434S Device 0)

**Hardware:** DRV8434S on SPI1 daisy-chain, device index 0.

**Position tracking:** `s_arm_pos_steps` (int32_t), maintained by the Pico. Boot still initializes it to `0` for backward compatibility, but the trusted physical reference is established only by `MSG_ARM_HOME`.

**Behavior:** On `MSG_ARM_MOVE(position)`, compute delta from `s_arm_pos_steps` to target steps and start a DRV8434S motion. If `s_arm_rehome_required == true`, the Pico NACKs `MSG_ARM_MOVE` until `MSG_ARM_HOME` succeeds. Named positions map to step counts from the arm's physical home hard-stop:

| Position | Constant | Default Steps |
|----------|----------|--------------|
| ARM_POS_PRESS | `ARM_STEPS_PRESS` | -3500 |
| ARM_POS_1 | `ARM_STEPS_POS1` | -500 |
| ARM_POS_2 | `ARM_STEPS_POS2` | -100 |

**Press semantics:** A torque-stop while moving to `ARM_POS_PRESS` is treated as a successful press instead of a fault. The Pico keeps the tracked arm position at the actual achieved steps and does not require re-home just because the press hit resistance as intended.

**Vacuum-assisted retry:** If vacuum RPM telemetry is valid and the vacuum is ON when `ARM_POS_PRESS` starts, the Pico captures the pre-press RPM, waits `ARM_PRESS_RETRY_VERIFY_TIMEOUT_MS` after the press completes, and checks for an absolute RPM change of at least `ARM_PRESS_RETRY_RPM_DELTA` (default now 10 RPM to match observed scale). If no change is detected, the arm backs off by `ARM_PRESS_RETRY_BACKOFF_STEPS` and retries the press, up to `ARM_PRESS_RETRY_MAX_RETRIES`. If all retries are exhausted without an RPM response, the Pico reports `MOTION_TIMEOUT` but does not force re-home.

**Sensorless homing:** `MSG_ARM_HOME` is explicit/manual. The Pico requires the DRV8434S motion engine to be idle, then seeks in the positive direction by `ARM_HOME_SEARCH_STEPS` using `ARM_HOME_TORQUE_LIMIT`, `ARM_HOME_TORQUE_BLANK_STEPS`, `ARM_HOME_TORQUE_SAMPLE_DIV`, and `ARM_HOME_STEP_DELAY_US`. A torque-stop during the seek is treated as the home hard-stop (`s_arm_pos_steps = 0`), after which the firmware runs a fixed backoff of `ARM_HOME_BACKOFF_STEPS` and reports `MSG_MOTION_DONE(SUBSYS_ARM, MOTION_OK, -ARM_HOME_BACKOFF_STEPS)`.

**Homing failures:** If the seek consumes all `ARM_HOME_SEARCH_STEPS` without a torque-stop, the Pico reports `MOTION_TIMEOUT` and leaves `s_arm_rehome_required = true`. SPI faults, driver faults, and timeouts during seek or backoff also leave the arm requiring re-home.

**Timeouts:** Normal arm moves use `ARM_MOTION_TIMEOUT_MS`; each homing phase uses `ARM_HOME_TIMEOUT_MS`.

**Soft torque-limit default:** For state-based stepper moves (`MSG_ARM_MOVE`, `MSG_RACK_MOVE`, `MSG_TURNTABLE_GOTO`, etc.), Pico uses `STEPPER_SOFT_TORQUE_LIMIT`; if not overridden, this resolves to `PROTO_STEPPER_SOFT_TORQUE_LIMIT_DEFAULT` from the shared protocol header.

**Arm state flags:** `s_arm_homing_phase` (`IDLE`, `SEEK`, `BACKOFF`), `s_arm_homed`, and `s_arm_rehome_required`.

**Transition Table:**

| From | Event | To | Side Effect |
|------|-------|-----|-------------|
| IDLE / AT_* | MSG_ARM_MOVE(pos) | MOVING | Compute delta; start step job; ACK |
| IDLE / AT_* / REHOME_REQUIRED | MSG_ARM_HOME | HOMING_SEEK | Require idle DRV8434S engine; start positive seek; ACK |
| REHOME_REQUIRED | MSG_ARM_MOVE(pos) | REHOME_REQUIRED | NACK until `MSG_ARM_HOME` succeeds |
| MOVING | Step job complete | AT_PRESS / AT_POS1 / AT_POS2 | Update s_arm_pos_steps; send MOTION_DONE(ARM, OK, steps) |
| MOVING | Stall while target is `ARM_POS_PRESS` | AT_PRESS | Update tracked steps; report `MOTION_OK`; do not require re-home |
| MOVING / AT_PRESS | No RPM change after press verify window | PRESS_RETRY_BACKOFF or AT_PRESS | Back off and retry until `ARM_PRESS_RETRY_MAX_RETRIES` is exhausted; then report `MOTION_TIMEOUT` without forcing re-home |
| MOVING | Stall / timeout / driver fault / SPI fault on any other arm move | REHOME_REQUIRED | Update tracked steps, send terminal MOTION_DONE, require re-home |
| MOVING | MSG_ARM_MOVE(new_pos) | MOVING | Cancel current; start new job; ACK |
| HOMING_SEEK | Torque-stop detected | HOMING_BACKOFF | Set `s_arm_pos_steps = 0`; start fixed backoff |
| HOMING_SEEK | Search distance exhausted | REHOME_REQUIRED | Send MOTION_DONE(ARM, TIMEOUT, steps) |
| HOMING_BACKOFF | Step job complete | AT_HOME_RELEASED | Send MOTION_DONE(ARM, OK, `-ARM_HOME_BACKOFF_STEPS`) |
| HOMING_SEEK / HOMING_BACKOFF | SPI / driver fault / timeout | REHOME_REQUIRED | Send matching MOTION_DONE; reject later `MSG_ARM_MOVE` |

### 2.3 Rack Stepper (DRV8434S Device 1)

**Hardware:** DRV8434S on SPI0 daisy-chain, device index 1.

**Position tracking:** `s_rack_pos_steps` (int32_t). Zeroed on successful home.

**Homing:** `RACK_POS_HOME` drives full reverse until stall detection triggers at the physical endstop. On stall, set `s_rack_pos_steps = 0`.

| Position | Constant | Default Steps |
|----------|----------|--------------|
| RACK_POS_HOME | — | Drive to stall; zero counter |
| RACK_POS_EXTEND | `RACK_STEPS_EXTEND` | 6000 |
| RACK_POS_PRESS | `RACK_STEPS_PRESS` | 7500 |

**Timeouts:** `RACK_HOME_TIMEOUT_MS` (default: 15000), `RACK_MOVE_TIMEOUT_MS` (default: 12000).

**Rack State Machine States:** `IDLE`, `HOMING`, `MOVING`, `AT_HOME`, `AT_EXTEND`, `AT_PRESS`, `FAULT`

### 2.4 Turntable Stepper (DRV8434S Device 2)

**Hardware:** DRV8434S on SPI0 daisy-chain, device index 2.

**Position tracking:** `s_turntable_pos_steps` (int32_t). Zeroed on home.

**Boot requirement:** The turntable starts in `UNCALIBRATED` state. It **must** receive `MSG_TURNTABLE_HOME` before any `MSG_TURNTABLE_GOTO` is valid. Commands received in `UNCALIBRATED` state are NACKed.

| Position | Constant | Default Steps |
|----------|----------|--------------|
| TURNTABLE_POS_A | `TURNTABLE_STEPS_A` | 0 |
| TURNTABLE_POS_B | `TURNTABLE_STEPS_B` | 1250 |
| TURNTABLE_POS_C | `TURNTABLE_STEPS_C` | 2500 |
| TURNTABLE_POS_D | `TURNTABLE_STEPS_D` | 3750 |

**Timeouts:** `TURNTABLE_HOME_TIMEOUT_MS` (default: 20000), `TURNTABLE_MOVE_TIMEOUT_MS` (default: 15000).

**Turntable State Machine States:** `UNCALIBRATED`, `HOMING`, `MOVING`, `AT_A`, `AT_B`, `AT_C`, `AT_D`, `FAULT`

**Critical rules:**
- FAULT state requires re-home (`MSG_TURNTABLE_HOME`) before any GOTO is accepted
- MOVING + MSG_TURNTABLE_HOME = cancel current job, begin re-home

### 2.5 Hot Wire (DRV8263 Hotwire Instance — Independent Half-Bridge Mode)

**Hardware:** DRV8263 on GP6 (IN1, PWM) / GP7 (IN2, PWM), current sense on GP26 (ADC ch0, **unused — see below**).

**Configuration: Independent H-bridge mode.** The DRV8263 is wired in independent half-bridge mode:
- **IN1 (GP6, `HOTWIRE_PIN_IN1`)** — drives the nichrome hot wire. Current regulation handled by an external Rsense resistor and the DRV8263's internal regulation loop. No software current control needed.
- **IN2 (GP7, `HOTWIRE_PIN_IN2`)** — drives vacuum pump 2 independently. Can run simultaneously with IN1.

**Key difference from H-bridge mode:** `drv8263_set_in1()` and `drv8263_set_in2()` are used instead of `drv8263_set_motor_control()`, so each half-bridge is controlled independently without clearing the other.

**Constant on/off:** Full duty cycle (`HOTWIRE_ENABLE_DUTY = 4095`). No PWM ramping needed — current is set by Rsense.

**ADC sense pin:** GP26 is initialized by the driver but readings are **not used for control** — current is hardware-regulated.

**NOT mutually exclusive with vacuum2.** Both IN1 (hot wire) and IN2 (vacuum pump 2) can be active simultaneously.

**No MSG_MOTION_DONE** is sent for the hot wire — it is a steady-state output.

**Hot Wire States:** `OFF`, `ON`, `FAULT`

### 2.6 Vacuum Pumps

**Primary vacuum pump:**
- Trigger: GPIO output (`VACUUM_TRIGGER_GPIO`, default: GP2). HIGH = on, LOW = off.
- RPM sense: GPIO input (`VACUUM_RPM_SENSE_GPIO`, default: GP3), rising-edge interrupt.
- RPM calculation: count rising edges over `VACUUM_RPM_SAMPLE_INTERVAL_MS` (default: 100 ms), convert using `VACUUM_PULSES_PER_REV` (default: 2).
- Blocked detection: if RPM < `VACUUM_RPM_BLOCKED_THRESHOLD` (default: 400) while pump is ON, report `VACUUM_BLOCKED`.
- Periodic status: send `MSG_VACUUM_STATUS` every `VACUUM_PERIODIC_STATUS_MS` (default: 5000 ms) while pump is on, and immediately on any OK ↔ BLOCKED transition.

**Secondary vacuum pump (MSG_VACUUM2_SET):**
- Uses the hotwire DRV8263 independent half-bridge IN2 (GP7 driven via `drv8263_set_in2()`).
- Simple on/off, no RPM monitoring.
- **Not mutually exclusive with hotwire (IN1).** Both can run simultaneously.

**Vacuum States:** `OFF`, `ON_OK`, `ON_BLOCKED`

### 2.8 Hot Wire Traverse Stepper (DRV8434S Device 4 — `STEPPER_DEV_HW_CARRIAGE`)

**Status: Not yet wired.** Defined but guarded with `#ifdef STEPPER_DEV_HW_CARRIAGE` in `uart_server.c`.

**Hardware:** DRV8434S on SPI0 daisy-chain, device index 4 (when wired). Drives the linear carriage that traverses the nichrome wire through the crimp point of the spawn bag tip.

**Behavior on `MSG_HOTWIRE_TRAVERSE(direction=0)`:** Move carriage forward `HOTWIRE_TRAVERSE_STEPS` (default: -4000, full-step no subdivision) at `HOTWIRE_TRAVERSE_STEP_DELAY_US` (default: 2 µs/step). This cuts the tip quickly. 0=full-step mode, no microstep subdivision (DRV8434S_CTRL3_MICROSTEP_MODE=0).

**Behavior on `MSG_HOTWIRE_TRAVERSE(direction=1)`:** Move carriage in reverse `HOTWIRE_TRAVERSE_RETRACE_STEPS` (default: 4000) and zero position on successful return. Timeout is bounded by traversal time + `HOTWIRE_TIMEOUT_GUARD_MS` (default: 8000 ms).

**Activation:** To uncomment `STEPPER_DEV_HW_CARRIAGE` in `board_pins.h` and increment `DRV8434S_N_DEVICES` to include this device.

### 2.9 Indexer / Bag Depth Rack (DRV8434S Device 5 — `STEPPER_DEV_INDEXER`)

**Status: Not yet wired.** Defined but guarded with `#ifdef STEPPER_DEV_INDEXER` in `uart_server.c`.

**Hardware:** DRV8434S on SPI0 daisy-chain, device index 5 (when wired). Drives the bag depth/eject rack that centers the incoming substrate bag and pushes out the inoculated bag.

**Position tracking:** `s_indexer_pos_steps` (int32_t). Updated on each move.

| Position | Constant | Default Steps | Purpose |
|----------|----------|--------------|---------|
| `INDEXER_POS_OPEN` | — | 0 | Retracted; bag can slide in |
| `INDEXER_POS_CENTER` | `INDEXER_STEPS_CENTER` | 3000 | Holds bag centered for weighing/opening |
| `INDEXER_POS_EJECT` | `INDEXER_STEPS_EJECT` | 8000 | Fully extended; pushes inoculated bag out |

**Activation:** Uncomment `STEPPER_DEV_INDEXER` in `board_pins.h` and increment `DRV8434S_N_DEVICES`.

### 2.7 Load Cell (HX711)

**Hardware:** HX711 ADC on GP14 (DATA) / GP15 (CLK). Uses `pico-scale` library (submodule in `extern/`).

**Behavior:**
- `MSG_HX711_TARE`: Zero the scale. ACK on success.
- `MSG_HX711_MEASURE`: Read weight. ACK payload is `pl_hx711_mass_t` with mass in micrograms.
- Weight is also read internally during spawn dosing (see Section 3).

---

## 3. Spawn Dosing Algorithm (Closed-Loop on Pico)

### 3.1 Overview

When `MSG_DISPENSE_SPAWN` is received, the Pico runs a closed-loop dosing algorithm that:
1. Opens the hopper flaps
2. Reads the load cell continuously
3. Controls flap position to regulate spawn flow rate
4. Detects and recovers from material bridging
5. Closes flaps when target mass is dispensed
6. Reports progress via `MSG_SPAWN_STATUS`

### 3.2 Initiation

**Trigger:** `MSG_DISPENSE_SPAWN` with `pl_innoculate_bag_t` payload.

**Precondition:** HX711 must be initialized and ready (`s_hx711_ready == true`). If not, NACK with `NACK_UNKNOWN`.

**If a prior dose is active:** Stop the prior dose (stop timer, close flaps), then start the new one.

**Target calculation:**
```
target_ug = (start_mass_ug × innoc_percent) / 1000
```
Where `innoc_percent` is in tenths of a percent (e.g., 250 = 25.0%).

### 3.3 Timer-Driven Control Loop

The dosing algorithm runs on a `repeating_timer` at `SPAWN_TIMER_PERIOD_MS` (default: 50 ms) intervals. Each tick:

1. **Read the load cell** — single-shot reading via `pico-scale`.
2. **Compute dispensed mass** — `dispensed_ug = current_mass_ug - start_mass_ug`.
3. **Flow detection** — over a window of `SPAWN_FLOW_WINDOW_MS` (default: 100 ms), if dispensed mass has not increased, flow has stalled.
4. **Control action** — based on current dosing state (see 3.4).
5. **Report status** — send `MSG_SPAWN_STATUS` with current state, dispensed mass, target, retries.

### 3.4 Dosing State Machine

| State | Entry Condition | Behavior | Exit Condition |
|-------|----------------|----------|----------------|
| STARTUP_OPEN | Dose begins | Drive flaps open at full PWM. Wait for first weight change. | Weight change detected → RUNNING |
| RUNNING | Flow detected | Flaps held at dosing PWM. Monitor flow rate. | `dispensed_ug >= target_ug` → DONE. No flow for window → STALLED. |
| STALLED | No flow detected for `SPAWN_FLOW_WINDOW_MS` | Close flaps. Activate agitator (eccentric arm stepper) gently. | After agitation period → re-open flaps → RUNNING (retry). Max retries exceeded → BAG_EMPTY. |
| AGITATING | Bridge detected | Agitator kneads bag side. Flaps closed. | Agitation time elapsed → re-open → RUNNING. |
| DONE | Target mass reached | Close flaps. Stop timer. | Send SPAWN_STATUS(DONE). Terminal. |
| BAG_EMPTY | Retries exhausted or insufficient spawn | Close flaps. Stop timer. | Send SPAWN_STATUS(BAG_EMPTY). Terminal. |
| ERROR | Sensor failure, driver fault | Close flaps. Stop timer. | Send SPAWN_STATUS(ERROR). Terminal. |
| ABORTED | MSG_CTRL_STOP received | Close flaps. Stop timer. | Send SPAWN_STATUS(ABORTED). Terminal. |

### 3.5 Flap Nudging

During RUNNING state, if flow rate drops but is not zero, the algorithm applies short position-nudge pulses to the flap motors to maintain flow without fully re-opening. A nudge drives the motor for a brief pulse (`nudge_until` timestamp), then stops and waits for the next timer tick to evaluate.

### 3.6 Defaults and Boundaries

| Parameter | Default | Unit | Notes |
|-----------|---------|------|-------|
| `SPAWN_TIMER_PERIOD_MS` | 50 | ms | Dosing loop tick rate |
| `SPAWN_FLOW_WINDOW_MS` | 100 | ms | Window for flow detection |
| `SPAWN_SCALE_READ_SAMPLES` | 1 | — | Single-shot for speed |
| Max agitation retries | 3 | — | Per dose attempt. Configurable via `s_spawn.max_retries`. |

---

## 4. Polling Loop Tick Functions

The main polling loop must call the following on every iteration:

```c
flap_sm_tick();                // polls DRV8263 monitoring_enabled flag, timeout
vacuum_sm_tick();              // reads RPM counter every VACUUM_RPM_SAMPLE_INTERVAL_MS
arm_seal_monitor_tick();       // rotary-arm seal monitor (EMA, baseline, LOST/RESTORED edges)
stepper_completion_tick();     // polls drv8434s_motion active flag for arm/rack/turntable, timeout
stepper_spi_watchdog_tick();   // reads FAULT register ~2s while idle; logs/clears faults
```

These functions are called from within `uart_server_poll()` (or directly in the main loop — implementation choice).

## 4.1 Rotary Arm Seal Monitor (Vacuum RPM, Pico-side)

Purpose: detect suction-cup seal loss during bag opening on the Pico (no high-rate UART telemetry).

Monitor state machine:
- `SEAL_MON_DISABLED`
- `SEAL_MON_BASELINING`
- `SEAL_MON_SEALED_OK`
- `SEAL_MON_LOST_LATCHED`

Key behavior:
- Starts baselining after a press command/move sequence while vacuum is ON.
- Computes `rpm_filt` using EMA (`ARM_SEAL_EMA_ALPHA_X1000`) and sealed baseline via time-window average.
- Declares LOST on either:
  - transient trigger (`delta >= ARM_SEAL_TRANSIENT_DELTA_RPM` for `ARM_SEAL_TRANSIENT_DEBOUNCE_MS`)
  - steady trigger (`delta >= ARM_SEAL_STEADY_DELTA_RPM` for `ARM_SEAL_STEADY_HOLD_MS`)
  - stale tach while opening-related arm motion (`ARM_SEAL_TACH_STALE_MS`)
- On LOST:
  - cancels active arm motion immediately,
  - keeps stepper outputs enabled (hold position),
  - sends one `MSG_ARM_SEAL_EVENT(LOST, reason, …)`,
  - sends `MSG_MOTION_DONE(SUBSYS_ARM, MOTION_STALLED, steps_done)`.
- LOST is latched; repeated LOST messages are suppressed until recovery.
- RESTORED is sent once after commanded retry press + re-baseline + restore debounce (`ARM_SEAL_RESTORE_DEBOUNCE_MS`).

Tunables in `board_pins.h` (`#ifndef` guarded defaults):
- `ARM_SEAL_EMA_ALPHA_X1000` = 250
- `ARM_SEAL_BASELINE_WINDOW_MS` = 300
- `ARM_SEAL_BASELINE_MIN_SAMPLES` = 3
- `ARM_SEAL_BASELINE_TIMEOUT_MS` = 1000
- `ARM_SEAL_TRANSIENT_DELTA_RPM` = 60
- `ARM_SEAL_TRANSIENT_DEBOUNCE_MS` = 80
- `ARM_SEAL_STEADY_DELTA_RPM` = 15
- `ARM_SEAL_STEADY_HOLD_MS` = 800
- `ARM_SEAL_TACH_STALE_MS` = 200
- `ARM_SEAL_RESTORE_DEBOUNCE_MS` = 200

---

## 5. Pin Assignment Table

All pin assignments live in `pico_fw/src/board_pins.h` with `#ifndef` guards.

| GPIO | Function | Peripheral | Constant |
|:----:|----------|:----------:|----------|
| 0 | UART TX → ESP32 | UART0 | `PICO_UART_TX_GPIO` |
| 1 | UART RX ← ESP32 | UART0 | `PICO_UART_RX_GPIO` |
| 10 | DRV8434S SCK | SPI1 | `DRV8434S_SCK_GPIO` |
| 11 | DRV8434S MOSI | SPI1 | `DRV8434S_MOSI_GPIO` |
| 12 | DRV8434S MISO | SPI1 | `DRV8434S_MISO_GPIO` |
| 13 | DRV8434S CS | SPI1 GPIO | `DRV8434S_CS_GPIO` |
| 6 | Flap 1 DRV8263 IN1 (PWM) | PWM3A | `DRV8263_CTRL_A_GPIO` |
| 7 | Flap 1 DRV8263 IN2 (PWM) | PWM3B | `DRV8263_CTRL_B_GPIO` |
| 8 | Flap 2 DRV8263 IN1 (PWM) | PWM4A | `FLAP2_CTRL_A_PIN` |
| 9 | Flap 2 DRV8263 IN2 (PWM) | PWM4B | `FLAP2_CTRL_B_PIN` |
| 4 | Hot wire IN1 — DRV8263 independent half-bridge (PWM) | GPIO out | `HOTWIRE_PIN_IN1` |
| 5 | Vacuum pump 2 IN2 — DRV8263 independent half-bridge (PWM) | GPIO out | `HOTWIRE_PIN_IN2` |
| 2 | Vacuum pump 1 trigger (output) | GPIO out | `VACUUM_TRIGGER_PIN` |
| 3 | Vacuum pump 1 RPM sense (rising-edge interrupt) | GPIO IRQ | `VACUUM_RPM_SENSE_PIN` |
| 14 | HX711 DATA | GPIO | `HX711_DATA_GPIO` |
| 15 | HX711 CLK | GPIO | `HX711_CLK_GPIO` |
| 27 | Flap 2 current sense ADC ch1 | ADC1 | `FLAP2_ADC_SENSE_PIN` |
| 28 | Flap 1 current sense ADC ch2 | ADC2 | `DRV8263_SENSE_GPIO` |

**PWM slices:** GP6/7 = slice 3 (flap 1), GP8/9 = slice 4 (flap 2), GP10/11 = slice 5 (hot wire + vacuum 2). No conflicts.  
**ADC channels:** GP27 = ADC ch1 (flap 2 sense), GP28 = ADC ch2 (flap 1 sense). No conflicts.  
**Hot wire ADC (GP26, ADC ch0):** Not populated — current regulation is done by external Rsense on DRV8263.

---

## 6. Definition of Done

### 6.1 Subsystem State Machines
- [ ] Flap state machine implements all transitions in Section 2.1 table
- [ ] Arm state machine implements all transitions in Section 2.2 table
- [ ] Rotary arm seal monitor emits exactly one LOST and one RESTORED edge per incident/recovery cycle
- [ ] Seal-loss cancellation sends both `MSG_ARM_SEAL_EVENT(LOST, …)` and `MSG_MOTION_DONE(SUBSYS_ARM, MOTION_STALLED, steps_done)`
- [ ] Rack state machine implements all transitions including homing via stall
- [ ] Turntable state machine enforces UNCALIBRATED → HOME before GOTO
- [ ] Hot wire uses DRV8263 independent half-bridge mode (drv8263_set_in1/in2); IN1 and IN2 can be active simultaneously
- [ ] Vacuum state machine reports RPM status periodically and on transitions
- [ ] All state machines send correct MSG_MOTION_DONE on terminal states

### 6.2 Protocol Compliance
- [ ] Every ESP → Pico command receives ACK or NACK within one polling iteration
- [ ] Unknown message types receive NACK(NACK_UNKNOWN)
- [ ] Payload length mismatches receive NACK(NACK_BAD_LEN)
- [ ] CRC failures receive NACK(NACK_BAD_CRC)

### 6.3 Spawn Dosing
- [ ] MSG_DISPENSE_SPAWN initiates closed-loop dosing with correct target calculation
- [ ] Dosing reports progress via MSG_SPAWN_STATUS at each timer tick
- [ ] Bridge detection triggers agitation and retry
- [ ] MSG_CTRL_STOP aborts an active dose and closes flaps
- [ ] On completion, flaps are closed and SPAWN_STATUS_DONE is sent

### 6.4 Safety
- [ ] All actuators drive to safe state on unrecoverable error
- [ ] All tuning parameters are `#define` with `#ifndef` guards in `board_pins.h`
- [ ] No magic numbers in state machine logic
- [ ] All state transitions are logged via printf

### 6.5 Build
- [ ] Firmware compiles cleanly with Pico SDK 2.2.0, `pico2` board target
- [ ] USB CDC stdio enabled, UART stdio disabled
- [ ] Output: `.uf2` file for flashing
