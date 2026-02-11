DRV8434S driver
================

Overview
--------

Portable C driver for the TI DRV8434S SPI stepper-motor driver.  The code is
hardware-agnostic: callers provide callbacks for SPI transfers, CS/reset
control, and delays.  A complete RP2350 (Raspberry Pi Pico 2) demo is
included.

Files
-----
- `drv8434s.h` — public header: register map, bit-field defines, driver types
  and prototypes
- `drv8434s.c` — driver implementation: init, reset, probe, register R/W,
  motor control helpers, fault/torque diagnostics
- `main.c`     — test of stepper motor control written for Raspberry Pi Pico 2
  (RP2350, ARM core)
- `CMakeLists.txt` — Pico SDK build configuration
- `pico_sdk_import.cmake` — Pico SDK locator (standard)
- `arduino/`   — Arduino port for Adafruit QT Py (SAMD21)

Register Map
------------

| Addr | Name  | Type | Description                              |
|------|-------|------|------------------------------------------|
| 0x00 | FAULT | R    | Fault status (clear on read)             |
| 0x01 | DIAG1 | R    | OCP per half-bridge                      |
| 0x02 | DIAG2 | R    | Open-load / stall detail                 |
| 0x03 | CTRL1 | RW   | EN_OUT, microstep mode                   |
| 0x04 | CTRL2 | RW   | Decay mode, TRQ DAC (current scale)      |
| 0x05 | CTRL3 | RW   | CLR_FLT, LOCK, TOFF                      |
| 0x06 | CTRL4 | RW   | SPI step/dir, step mode, open-load       |
| 0x07 | CTRL5 | RW   | Stall detection threshold                |
| 0x08 | CTRL6 | RW   | Stall count, open-load count             |
| 0x09 | CTRL7 | R    | TRQ_COUNT (instantaneous torque reading)  |

Driver API
----------

### Minimal layer (portable)

```c
bool drv8434s_init(drv8434s_t *dev, const drv8434s_config_t *cfg);
bool drv8434s_probe(drv8434s_t *dev);
void drv8434s_reset(drv8434s_t *dev, unsigned reset_ms);
bool drv8434s_probe_cmd(dev, tx, tx_len, rx, rx_len);
bool drv8434s_read_register(dev, reg, out, out_len);
```

### Register-level

```c
bool drv8434s_write_register(drv8434s_t *dev, uint8_t reg, uint8_t value);
bool drv8434s_reg_read(drv8434s_t *dev, uint8_t reg, uint8_t *value);
bool drv8434s_reg_modify(drv8434s_t *dev, uint8_t reg, uint8_t mask, uint8_t value);
```

### Motor control

```c
bool drv8434s_enable(drv8434s_t *dev);
bool drv8434s_disable(drv8434s_t *dev);
bool drv8434s_set_microstep(drv8434s_t *dev, drv8434s_microstep_t mode);
bool drv8434s_set_torque(drv8434s_t *dev, uint8_t trq);   // 0–15
bool drv8434s_set_decay(drv8434s_t *dev, drv8434s_decay_t decay);
bool drv8434s_set_spi_step_mode(drv8434s_t *dev);
bool drv8434s_set_spi_dir(drv8434s_t *dev, bool reverse);
bool drv8434s_spi_step(drv8434s_t *dev);
```

### Diagnostics

```c
bool drv8434s_read_fault(drv8434s_t *dev, uint8_t *faults);
bool drv8434s_clear_faults(drv8434s_t *dev);
bool drv8434s_read_torque_count(drv8434s_t *dev, uint8_t *trq_count);
bool drv8434s_read_all_regs(drv8434s_t *dev, uint8_t regs[10]);
```

SPI protocol notes
------------------

The DRV8434S uses a **16-bit SPI frame** (CPOL=0, CPHA=1, MSB-first).  Each
CS assertion must clock exactly 16 continuous SCLK cycles — no gap between
the command byte and data byte.

**Critical:** On the RP2350, `spi_set_format` must be configured for **16-bit
frame size** (not 8-bit).  Using 8-bit frames causes the SPI peripheral to
insert a small inter-byte pause that resets the DRV8434S shift register,
producing a **loopback echo** where SDO mirrors SDI instead of returning
device data.  The platform callback in `main.c` uses
`spi_write16_read16_blocking()` to ensure gapless 16-bit transfers.

SDI frame layout (16 bits, MSB first):

```
Bit:  15   14   13  12  11  10   9    8    7   6   5   4   3   2   1   0
      DC    W   A4  A3  A2  A1  A0   DC   D7  D6  D5  D4  D3  D2  D1  D0
```

- Bit 15: don't care (set to 0)
- Bit 14: W — 1 = Read, 0 = Write
- Bits 13:9: 5-bit register address (A4:A0)
- Bit 8: don't care (set to 0)
- Bits 7:0: 8-bit data (D7:D0)

| SPI parameter | Value              |
|---------------|--------------------|
| CPOL          | 0 (idle low)       |
| CPHA          | 1 (Mode 1)        |
| Frame size    | 16 bits            |
| Bit order     | MSB first          |
| Max clock     | 10 MHz             |
| W bit (14)    | 1 = Read, 0 = Write |
| Address       | bits 13:9 (5-bit, byte 0 bits 5:1) |

Timing guards in the driver:
- **CS hold** (5 µs) — enforced after every CS deassert via `delay_us` callback
- **Register settle** (2 µs) — after every register write
- **Step edge** (5 µs) — explicit 0→1 transition on SPI_STEP to guarantee
  the device sees a rising edge

Probe behaviour
---------------

`drv8434s_probe()` sends a read command for register 0x00 (FAULT) and
validates the response.  It rejects two failure modes:

1. **Loopback echo** — if `rx == tx`, MISO is echoing MOSI (wrong SPI frame
   size or physical wiring error).
2. **Floating MISO** — if `rx == {0xFF, 0xFF}`, no device is driving SDO.

For stronger checks, use:

- `drv8434s_probe_cmd(dev, tx, tx_len, rx, rx_len)` — send a specific
  command and validate returned bytes.
- `drv8434s_read_register(dev, reg, out, out_len)` — performs a common
  `[reg][dummy…]` SPI transaction.

Test Usage (`main.c`)
---------------------

The RP2350 demo in `main.c` exercises the full driver API:

1. **SPI & GPIO initialisation** — configures SPI0 at 1 MHz (Mode 1),
   CS/RST/nFAULT pins.
2. **Device probe & register dump** — probes the bus and prints all 10
   registers in a formatted table.
3. **Motor configuration** — sets SPI step/direction mode, 1/16
   micro-stepping, torque DAC at 67 %, smart-tune dynamic decay, and enables
   outputs.
4. **Non-blocking motor spin** — uses `repeating_timer` to issue SPI steps
   at a configurable rate (default 500 µs per step) without blocking the main
   loop.
5. **Software torque limit** — a second repeating timer reads the TRQ_COUNT
   register every 10 ms.  If the instantaneous torque exceeds a configurable
   threshold the motor is stopped, torque is reduced, and the motor
   is restarted.
6. **Fault reporting** — the nFAULT pin triggers a GPIO interrupt that
   immediately halts the motor.  The main loop reads and prints the FAULT
   register, clears faults, and re-enables the driver.
7. **Periodic status & direction reversal** — every 2 s the main loop prints
   step count, TRQ_COUNT, and fault status.  Every 20 s the motor reverses
   direction.

### Pin wiring (defaults)

| Signal | GPIO | Function        |
|--------|------|-----------------|
| SCK    | 18   | SPI0 clock      |
| MOSI   | 19   | SPI0 TX         |
| MISO   | 16   | SPI0 RX         |
| CS     | 17   | Chip select     |
| RST    | 20   | Device reset    |
| nFAULT | 21   | Fault indicator |

### Build

```bash
cd drv8434s
mkdir build && cd build
cmake ..
make -j$(nproc)
```

Flash the resulting `drv8434s_stepper_demo.uf2` via USB boot mode.

### Configuration

Edit the `#define` block near the top of `main.c`:

| Define                    | Default | Description                          |
|---------------------------|---------|--------------------------------------|
| `MOTOR_MICROSTEP`         | 1/16    | Microstep resolution                 |
| `MOTOR_TORQUE`            | 10      | TRQ DAC (0–15)                       |
| `MOTOR_DECAY`             | smart   | Decay mode enum                      |
| `MOTOR_STEP_INTERVAL_US`  | 500     | Step period in µs (controls speed)   |
| `MOTOR_TORQUE_LIMIT`      | 200     | TRQ_COUNT threshold (0–255)          |
| `TORQUE_CHECK_INTERVAL_MS`| 10      | Torque polling interval              |

License: MIT-style (add appropriate license header for your project).
