# Spawn Dosing Flap Controller — Specification

**File:** `pico_fw/src/uart_server.c`  
**Config:** `pico_fw/src/board_pins.h` (all tunables under "Spawn dosing control loop")  
**Protocol:** `shared/proto/proto.h` — `pl_innoculate_bag_t`, `spawn_finish_mode_t`

---

## 1. State Machine

```
IDLE
  │
  ├─(do_home flag set)──► HOMING ──────────────────────────────────────────────────►─┐
  │                         │ endpoint detected or HOME_TIMEOUT                        │
  └────────────────────────►─┘                                                        │
                            ▼                                                          │
                          PRIME (open slowly, wait for flow)                          │
                            │                                                          │
                    flow detected (≥ SPAWN_STARTUP_FLOW_DETECT_UG)                   │
                            │                                                          │
                          DOSE_MAIN  ◄───────────────────────────────────────────────-┘
                            │
              ┌─────────────┤─────────────────────────────┐
              │  Finish A   │                 Finish B     │
              │  (default)  │                              │
              │             │                              │
    remain ≤ trigger_ug     │         remain ≤ LOWFLOW_THRESHOLD_UG
    (early-close trigger)   │                              │
              │             │                 FINISH_B_LOWFLOW
              │             │                 (low-PWM nudge)
              │             │                              │
              │             │ remain ≤ CLOSE_THRESHOLD_UG │
      FINISH_A_CLOSE        │                              │
        (fast close)        │                    FAST_CLOSE (from Finish B)
              │             │
        CLOSE_CONFIRM ◄──── dispensed_ug ≥ target_ug (direct close)
              │
    ┌─────────┴─────────┐
    │undershoot > tol?  │
    │                   │
   YES                  NO
    │                   │
  FINISH_A_TOPOFF     DONE
  (tiny open pulses)
    │
  (within tol or
   max pulses)
    │
   DONE

Any state ──► FAULT    (flow failure, bag empty, unexpected state)
Any state ──► ABORTED  (MSG_CTRL_STOP)
```

### State descriptions

| State | Description |
|-------|-------------|
| `SPAWN_SM_IDLE` | Inactive; no dose in progress |
| `SPAWN_SM_HOMING` | Drives flaps to fully-closed endpoint; waits for `FLAP_SM_CLOSED` or timeout |
| `SPAWN_SM_PRIME` | Opens flaps slowly; waits for scale delta ≥ `SPAWN_STARTUP_FLOW_DETECT_UG` |
| `SPAWN_SM_DOSE_MAIN` | Closed-loop EMA-filtered nudge control toward target |
| `SPAWN_SM_FINISH_A_CLOSE` | Finish A: issues fast close when overshoot risk crosses threshold |
| `SPAWN_SM_CLOSE_CONFIRM` | Waits for `FLAP_SM_CLOSED` or `SPAWN_CLOSE_CONFIRM_TIMEOUT_MS` |
| `SPAWN_SM_FINISH_A_TOPOFF` | Finish A: tiny open pulses to correct undershoot |
| `SPAWN_SM_FINISH_B_LOWFLOW` | Finish B: minimal nudges at low-flow setpoint |
| `SPAWN_SM_FAST_CLOSE` | Issues fast close at `SPAWN_FAST_CLOSE_PWM`; proceeds to CLOSE_CONFIRM |
| `SPAWN_SM_DONE` | Dose complete; timer stopped |
| `SPAWN_SM_FAULT` | Flow failure or bag-empty condition |
| `SPAWN_SM_ABORTED` | Dose cancelled by `MSG_CTRL_STOP` |

---

## 2. Finish Mode A vs B

### Finish Mode A — Close-Early + Top-Off  *(default, `flags = 0x00`)*

**Goal:** Prevent overshoot by closing flaps before the target is reached,
using predicted in-flight mass to decide when to close.

**Trigger condition:**
```
remaining_ug ≤ (ema_flow_ug × latency_ticks) + SPAWN_CLOSE_EARLY_MARGIN_UG
```
where `latency_ticks = SPAWN_CLOSE_LATENCY_MS / SPAWN_TIMER_PERIOD_MS`.

**After close:**
- Waits for `FLAP_SM_CLOSED` (or `SPAWN_CLOSE_CONFIRM_TIMEOUT_MS`)
- If `target − dispensed > SPAWN_TOPOFF_TOLERANCE_UG`: enters top-off
  - Issues up to `SPAWN_TOPOFF_MAX_PULSES` open pulses of `SPAWN_TOPOFF_PULSE_MS` each
  - Settles `SPAWN_TOPOFF_SETTLE_MS` between pulses to let scale stabilise

**Tuning:**
- Reduce `SPAWN_CLOSE_EARLY_MARGIN_UG` to close later (more mass, higher overshoot risk)
- Increase `SPAWN_CLOSE_LATENCY_MS` to account for slower hardware
- Reduce `SPAWN_TOPOFF_PULSE_MS` for finer top-off granularity

---

### Finish Mode B — Low-Flow Taper  *(set `SPAWN_FLAG_FINISH_MODE_B` in `flags`)*

**Goal:** Reduce flow rate near target to slow accumulation, minimising overshoot
without requiring the close-early prediction.

**Stages:**
1. `remaining_ug ≤ SPAWN_LOWFLOW_THRESHOLD_UG` → enter `FINISH_B_LOWFLOW`
   - Nudge duration reduced to `SPAWN_LOWFLOW_NUDGE_MS` (shorter than normal)
   - Closed-loop continues with EMA; desired rate = `SPAWN_TICK_MIN_UG`
2. `remaining_ug ≤ SPAWN_CLOSE_THRESHOLD_UG` → issue fast close

**Tuning:**
- `SPAWN_LOWFLOW_THRESHOLD_UG`: how far from target to enter taper (default 5 g)
- `SPAWN_LOWFLOW_NUDGE_MS`: shorter = less flow per tick (default 80 ms)
- `SPAWN_CLOSE_THRESHOLD_UG`: how close to target before final close (default 0.5 g)

---

## 3. Parameter Reference

All parameters are `#define` constants in `pico_fw/src/board_pins.h`.
Override via `-D` compiler flags without editing the file.

### Timing

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `SPAWN_TIMER_PERIOD_MS` | 50 ms | 20–100 ms | Control loop period |
| `SPAWN_FLOW_WINDOW_MS` | 100 ms | 50–500 ms | No-flow detection window |

### Flow / detection

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPAWN_STARTUP_FLOW_DETECT_UG` | 1 000 000 µg | Min per-tick delta to confirm prime flow |
| `SPAWN_FLOW_NOFLOW_UG` | 500 000 µg | Min µg/window to avoid agitation |
| `SPAWN_FLOW_MIN_UG` | 1 000 000 µg/window | Low-end target flow rate |
| `SPAWN_FLOW_MAX_UG` | 5 000 000 µg/window | High-end target flow rate |
| `SPAWN_MAX_RETRIES` | 100 | Agitation retries before bag-empty fault |
| `SPAWN_AGITATE_MS` | 2 000 ms | Agitation hold-off duration |

### EMA filtering

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `SPAWN_EMA_ALPHA_X1000` | 250 | 50–500 | EMA smoothing factor ×1000 (α ≈ 0.25) |
| `SPAWN_FLOW_SPIKE_CLAMP_UG` | 10 000 000 µg | — | Per-tick delta clamp (anti-glitch) |

Higher `SPAWN_EMA_ALPHA_X1000` = faster response, more noise.
Lower = smoother signal, slower response.

### Nudge control

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPAWN_TICK_DEADBAND` | 50 000 µg | ±deadband; no nudge within this band |
| `SPAWN_NUDGE_OPEN_MS` | 300 ms | Open pulse duration |
| `SPAWN_NUDGE_CLOSE_MS` | 200 ms | Close pulse duration |
| `SPAWN_MAX_OPEN_NUDGES` | 8 | Max consecutive open nudges before forced hold |

### Homing

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPAWN_HOME_TIMEOUT_MS` | 6 000 ms | Max time to wait for homing endpoint detect |

### Close confirmation

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPAWN_CLOSE_CONFIRM_TIMEOUT_MS` | 6 000 ms | Timeout for close endpoint detection |
| `SPAWN_FAST_CLOSE_PWM` | 4095 | PWM for fast close (max 4095) |

### Finish A

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPAWN_CLOSE_LATENCY_MS` | 800 ms | Expected time from close command to flap seated |
| `SPAWN_CLOSE_EARLY_MARGIN_UG` | 500 000 µg | Safety margin added to predicted in-flight mass |
| `SPAWN_TOPOFF_TOLERANCE_UG` | 1 000 000 µg | Undershoot below which top-off is skipped |
| `SPAWN_TOPOFF_PULSE_MS` | 80 ms | Duration of each top-off open pulse |
| `SPAWN_TOPOFF_MAX_PULSES` | 5 | Max top-off pulses before declaring done |
| `SPAWN_TOPOFF_SETTLE_MS` | 200 ms | Settle time after each top-off pulse |

### Finish B

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SPAWN_LOWFLOW_THRESHOLD_UG` | 5 000 000 µg | Remaining mass to enter low-flow taper |
| `SPAWN_LOWFLOW_NUDGE_MS` | 80 ms | Reduced nudge duration in low-flow phase |
| `SPAWN_CLOSE_THRESHOLD_UG` | 500 000 µg | Remaining mass to issue final close from B |

---

## 4. Protocol — Selecting Finish Mode

The `flags` byte in `pl_innoculate_bag_t` (byte 7) selects behaviour:

```c
/* In shared/proto/proto.h */
#define SPAWN_FLAG_FINISH_MODE_B  (1u << 0)  /* 0 = Finish A (default), 1 = Finish B */
#define SPAWN_FLAG_DO_HOME        (1u << 1)  /* 1 = home flaps before priming         */
```

**Example payloads (ESP32 side):**

```c
// Finish A, no homing (legacy / default)
pl.flags = 0x00;

// Finish B, no homing
pl.flags = SPAWN_FLAG_FINISH_MODE_B;

// Finish A + homing
pl.flags = SPAWN_FLAG_DO_HOME;

// Finish B + homing
pl.flags = SPAWN_FLAG_FINISH_MODE_B | SPAWN_FLAG_DO_HOME;
```

Legacy 7-byte payloads (without the `flags` field) are accepted and treated as `flags = 0x00`.

---

## 5. Logging

Every state transition is logged:
```
Spawn SM: <prev> → <next>
```

Per-tick log line (every 50 ms during dosing):
```
Spawn[<state>]: mass=<g>g disp=<ug>ug remain=<ug>ug ema_flow=<ug>ug/tick
```

Nudge commands:
```
Spawn DOSE_MAIN: open nudge <ms>ms (error=<ug>ug, ema=<ug>ug, consec=<n>)
Spawn DOSE_MAIN: close nudge <ms>ms (error=<ug>ug, ema=<ug>ug)
```

Close confirmation:
```
Spawn CLOSE_CONFIRM: closed in <ms>ms (flap_state=<n>)
Spawn CLOSE_CONFIRM: timeout after <ms>ms (flap_state=<n>)
```

Abort:
```
Spawn ABORT: stopped at state=<n> disp=<ug>ug target=<ug>ug
```

---

## 6. Acceptance Criteria

1. **Dosing starts from closed:** state begins in PRIME (or HOMING→PRIME) with flaps
   driving open under undercurrent monitoring.

2. **Flow detection:** first tick with `tick_delta ≥ SPAWN_STARTUP_FLOW_DETECT_UG` exits
   PRIME and enters DOSE_MAIN; if flap reaches fully-open before flow, `FLOW_FAILURE` is reported.

3. **Finish mode selectable:** `SPAWN_FLAG_FINISH_MODE_B` in `flags` enables Mode B at
   runtime with no firmware rebuild.

4. **Overshoot reduced vs prior behaviour:** Mode A closes before target using predicted
   in-flight mass; Mode B tapers flow rate; both mechanisms target < 5 g overshoot
   (vs prior 10–15 g).  Validate from post-run logs: `disp=` vs `target=`.

5. **ABORT is immediate:** `MSG_CTRL_STOP` cancels the timer, stops motors, and issues
   fast close within the same dispatch cycle.

6. **Parameters centralised:** all tunables are in `board_pins.h` under the dosing
   section; no magic numbers in `uart_server.c`.

7. **Logs support debugging:** state transitions, flow estimate, remaining mass, nudge
   commands, close confirmation events are all printed.

---

## 7. A/B Test Procedure

1. Run 5 bags with `flags = 0x00` (Finish A); record `disp_ug` vs `target_ug` from
   `MSG_SPAWN_STATUS` responses.
2. Run 5 bags with `flags = SPAWN_FLAG_FINISH_MODE_B` (Finish B); record same.
3. Compare overshoot (mean and max) between modes.
4. Adjust `SPAWN_CLOSE_LATENCY_MS` or `SPAWN_LOWFLOW_THRESHOLD_UG` and repeat.
