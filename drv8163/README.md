# DRV8163 SPI Driver

C driver for Texas Instruments DRV8163 Three-Phase Smart Gate Driver with integrated current shunt amplifiers.

**Reference:** TI DRV8163-Q1 Datasheet SLVSD06A  
**Device URL:** https://www.ti.com/lit/ds/symlink/drv8163-q1.pdf

## Features

- Complete register-level access via SPI interface
- Helper functions for common operations (PWM mode, gate drive, CSA config, OCP)
- Fault detection and clearing
- Register caching for effa# DRV8163 SPI Driver

C driver for Texas Instruments DRV8163 H-Bridge Motor Driver with integrated protection and diagnostics.

**Reference:** TI DRV8163-Q1 Datasheet SLVSH07A  
**Device URL:** https://www.ti.com/lit/ds/symlink/drv8163-q1.pdf

## Features

- Complete register-level access via SPI interface
- Helper functions for common H-bridge operations
- Comprehensive fault detection and status monitoring
- Register caching for efficient read-modify-write operations
- Platform-agnostic with HAL abstraction

## SPI Configuration Requirements

**Per datasheet Section 7.6:**

- **SPI Mode:** Mode 1 (CPOL=0, CPHA=1)
- **Clock Frequency:** Maximum 10 MHz
- **Frame Format:** 16-bit, MSB first
- **Bit Ordering:** 
  - Bit 15: R/W̅ (1=Read, 0=Write)
  - Bits 14-8: Address (7 bits, only 0x00-0x09 used)
  - Bits 7-0: Data (8 bits)
- **CS Timing:** 400 ns minimum between transfers (t_DCSS)

## File Structure

```
drv8163.h           - Header with register definitions and API
drv8163.c           - Driver implementation
drv8163_example.c   - Usage examples and integration guide
```

## Register Map

| Address | Register Name | Access | Description                                |
|---------|---------------|--------|--------------------------------------------|
| 0x00    | DEVICE_ID     | R      | Device ID and revision                     |
| 0x01    | FAULT         | R      | Fault status flags                         |
| 0x02    | STATUS1       | R      | OCP and diagnostic status                  |
| 0x03    | STATUS2       | R      | Temperature and driver status              |
| 0x04    | COMMAND       | R/W    | Clear faults, register lock control        |
| 0x05    | SPI_IN        | R/W    | SPI input control                          |
| 0x06    | CONFIG1       | R/W    | Retry modes, OLA, SSC, OV settings         |
| 0x07    | CONFIG2       | R/W    | Current trip, ISEL, diagnostic config      |
| 0x08    | CONFIG3       | R/W    | Slew rate, dead time, blanking time        |
| 0x09    | CONFIG4       | R/W    | Input selection, OCP selection, OLA filter |

## API Usage

### 1. Initialization

```c
#include "drv8163.h"

drv8163_handle_t drv;

// Platform-specific SPI transfer function
int my_spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len) {
    // Implement full-duplex SPI transfer
    // Return 0 on success, non-zero on error
}

// Optional CS control
void my_cs_assert(void) { /* Pull CS low */ }
void my_cs_deassert(void) { /* Pull CS high */ }

// Initialize driver (reads all registers to populate cache)
drv8163_status_t status = drv8163_init(&drv, my_spi_transfer, 
                                       my_cs_assert, my_cs_deassert);

// Read device ID
uint8_t device_id, revision;
drv8163_read_device_id(&drv, &device_id, &revision);
```

### 2. Configuration

```c
// Set output slew rate (0=slowest, 3=fastest)
drv8163_set_slew_rate(&drv, 2);  // Medium slew rate

// Set overcurrent trip threshold (0-7, see datasheet Table 7-19)
drv8163_set_trip_current(&drv, 4);  // Mid-range threshold

// Set current sense selection (0-3, see datasheet)
drv8163_set_current_sense(&drv, 1);

// Enable overcurrent retry mode (false=latch, true=retry)
drv8163_set_ocp_retry(&drv, true);

// Enable open load detection
drv8163_set_open_load_detect(&drv, true);
```

### 3. Fault Handling

```c
uint8_t faults;
drv8163_status_t status = drv8163_read_faults(&drv, &faults);

if (status == DRV8163_ERR_FAULT) {
    // Check individual fault bits
    if (faults & DRV8163_FAULT_OLA) {
        // Open load condition detected
    }
    if (faults & DRV8163_FAULT_TSD) {
        // Thermal shutdown - device too hot
    }
    if (faults & DRV8163_FAULT_OCP) {
        // Overcurrent protection triggered
    }
    if (faults & DRV8163_FAULT_VMUV) {
        // Motor supply undervoltage
    }
    if (faults & DRV8163_FAULT_VMOV) {
        // Motor supply overvoltage
    }
    
    // Clear latched faults
    drv8163_clear_faults(&drv);
}
```

### 4. Direct Register Access

```c
// Read any register
uint8_t data;
drv8163_read_reg(&drv, DRV8163_REG_STATUS1, &data);

// Write any writable register (0x04-0x09)
drv8163_write_reg(&drv, DRV8163_REG_CONFIG1, 0x1F);

// Modify specific bits without affecting others
drv8163_modify_reg(&drv, DRV8163_REG_CONFIG3, 
                   DRV8163_CONFIG3_TBLK, DRV8163_CONFIG3_TBLK);
```

## Fault Register Bits (0x01)

| Bit | Name  | Description |
|-----|-------|-------------|
| 0   | OLA   | Open load condition (if enabled) |
| 1   | TSD   | Thermal shutdown |
| 2   | OCP   | Overcurrent protection |
| 3   | VMUV  | Motor supply undervoltage |
| 4   | VMOV  | Motor supply overvoltage |
| 5   | FAULT | Latched fault indicator |
| 6   | POR   | Power-on reset occurred |
| 7   | ERR   | SPI communication error |

## Configuration Options

### Slew Rate (CONFIG3 register, bits 2:1)

| Setting | Slew Rate |
|---------|-----------|
| 0       | Slowest   |
| 1       | Slow      |
| 2       | Fast      |
| 3       | Fastest   |

### Trip Current Threshold (CONFIG2 register, bits 2:0)

See datasheet Table 7-19 for specific current values based on sense resistor.

### Current Sense Selection (CONFIG2 register, bits 4:3)

| ISEL | Description |
|------|-------------|
| 0    | Reserved    |
| 1    | 1x gain     |
| 2    | 2x gain     |
| 3    | 4x gain     |

## H-Bridge Control

The DRV8163 provides H-bridge control through:
- **Hardware inputs:** IN1, IN2 pins (or SPI control if enabled)
- **Output drivers:** OUT1, OUT2 with integrated MOSFETs
- **Current sensing:** Integrated current sense amplifier

**Control modes:**
- Independent control via IN1/IN2 pins
- SPI-controlled operation via SPI_IN register
- Diagnostic feedback via STATUS registers

## Platform Integration Checklist

1. **SPI peripheral setup:**
   - Mode 1 (CPOL=0, CPHA=1)
   - ≤10 MHz clock
   - 8-bit or 16-bit transfer size
   - MSB first
   
2. **GPIO for nSCS (CS):**
   - Active low
   - Assert before transfer, deassert after
   - 400 ns minimum between transfers
   
3. **Optional GPIOs for monitoring:**
   - nFAULT (input): Goes low on fault condition
   - nSLEEP (output): Sleep mode control
   
4. **Power supply considerations:**
   - VM: 4.5V to 30V motor supply
   - VCC: 3.0V to 5.5V logic supply
   - Bypass capacitors per datasheet Section 8.2

## Error Handling

Return codes:
- `DRV8163_OK` (0): Success
- `DRV8163_ERR_INVALID_PARAM` (-1): NULL pointer or out-of-range parameter
- `DRV8163_ERR_SPI_TIMEOUT` (-2): SPI communication failure
- `DRV8163_ERR_NOT_INIT` (-3): Handle not initialized
- `DRV8163_ERR_FAULT` (-4): Device fault detected

## Thread Safety

This driver is **not inherently thread-safe**. If used in a multi-threaded environment:

1. Protect each handle with a mutex
2. Or ensure only one thread accesses each handle
3. Do not share handles between ISRs and main code without synchronization

## Hardware Requirements

- DRV8163 or DRV8163-Q1 device
- SPI-capable microcontroller
- Pull-up resistor on nFAULT pin (optional, if monitoring faults via interrupt)
- Current sense resistor (if using current feedback)
- Motor supply bypass capacitors

## References

1. TI DRV8163-Q1 Datasheet (SLVSH07A)
2. Device product page: https://www.ti.com/product/DRV8163-Q1

## License

This driver implementation is provided as example code. Consult your project's license requirements.icient read-modify-write operations
- Platform-agnostic with HAL abstraction

## SPI Configuration Requirements

**Per datasheet Section 7.6:**

- **SPI Mode:** Mode 1 (CPOL=0, CPHA=1)
- **Clock Frequency:** Maximum 10 MHz
- **Frame Format:** 16-bit, MSB first
- **Bit Ordering:** 
  - Bit 15: R/W̅ (1=Read, 0=Write)
  - Bits 14-8: Address (7 bits, only 0x00-0x06 used)
  - Bits 7-0: Data (8 bits)
- **CS Timing:** 400 ns minimum between transfers (t_DCSS, Table 6.5)

## File Structure

```
drv8163.h           - Header with register definitions and API
drv8163.c           - Driver implementation
drv8163_example.c   - Usage examples and integration guide
```

## Register Map

| Address | Register Name       | Access | Description                           |
|---------|---------------------|--------|---------------------------------------|
| 0x00    | Fault Status 1      | R      | VDS/OTSD/UVLO fault flags             |
| 0x01    | Fault Status 2      | R      | GDF/OCP/OTW fault flags               |
| 0x02    | Driver Control      | R/W    | PWM mode, coast, brake, clear fault   |
| 0x03    | Gate Drive HS       | R/W    | High-side gate drive current settings |
| 0x04    | Gate Drive LS       | R/W    | Low-side gate drive current settings  |
| 0x05    | OCP Control         | R/W    | VDS OCP threshold and mode            |
| 0x06    | CSA Control         | R/W    | Current sense amp gain and config     |

## API Usage

### 1. Initialization

```c
#include "drv8163.h"

drv8163_handle_t drv;

// Platform-specific SPI transfer function
int my_spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len) {
    // Implement full-duplex SPI transfer
    // Return 0 on success, non-zero on error
}

// Optional CS control
void my_cs_assert(void) { /* Pull CS low */ }
void my_cs_deassert(void) { /* Pull CS high */ }

// Initialize driver (reads all registers to populate cache)
drv8163_status_t status = drv8163_init(&drv, my_spi_transfer, 
                                       my_cs_assert, my_cs_deassert);
```

### 2. Configuration

```c
// Set PWM mode (6x, 3x, 1x, or independent)
drv8163_set_pwm_mode(&drv, DRV8163_DRV_PWM_MODE_3X);

// Configure gate drive currents
// Parameters: handle, is_high_side, source_current (0-15), sink_current (0-15)
// See datasheet Tables 7-7 and 7-8 for current vs. setting
drv8163_set_gate_drive(&drv, true, 0x05, 0x08);   // HS: ~100mA src, ~200mA sink
drv8163_set_gate_drive(&drv, false, 0x05, 0x08);  // LS: ~100mA src, ~200mA sink

// Set CSA gain (5, 10, 20, or 40 V/V)
drv8163_set_csa_gain(&drv, DRV8163_CSA_GAIN_10);

// Configure overcurrent protection
// Mode: LATCH, RETRY, REPORT, or IGNORE
// VDS level: 0-15 (see Table 7-11, 0.06V to 2.0V)
drv8163_set_ocp(&drv, DRV8163_OCP_OCP_MODE_RETRY, 0x08);  // 0.6V threshold
```

### 3. Fault Handling

```c
uint8_t fault1, fault2;
drv8163_status_t status = drv8163_read_faults(&drv, &fault1, &fault2);

if (status == DRV8163_ERR_FAULT) {
    // Check individual fault bits
    if (fault1 & DRV8163_FAULT1_VDS_HA) {
        // High-side A overcurrent
    }
    if (fault1 & DRV8163_FAULT1_OTSD) {
        // Overtemperature shutdown - requires cooling before clearing
    }
    
    // Clear latched faults
    drv8163_clear_faults(&drv);
}
```

### 4. Direct Register Access

```c
// Read any register
uint8_t data;
drv8163_read_reg(&drv, DRV8163_REG_DRIVER_CTRL, &data);

// Write any register (except read-only fault status registers)
drv8163_write_reg(&drv, DRV8163_REG_DRIVER_CTRL, 0x12);

// Modify specific bits without affecting others
drv8163_modify_reg(&drv, DRV8163_REG_DRIVER_CTRL, 
                   DRV8163_DRV_COAST, DRV8163_DRV_COAST);
```

## PWM Modes (Register 0x02, Bits 1-0)

| Mode | Setting | Description |
|------|---------|-------------|
| 6x   | 0b00    | Six independent PWM inputs (INHA, INLA, INHB, INLB, INHC, INLC) |
| 3x   | 0b01    | Three PWM inputs (INH controls high-side, INL controls low-side) |
| 1x   | 0b10    | Single PWM + DIR input (INHA=PWM, INLA=DIR, automatic deadtime) |
| IND  | 0b11    | Independent half-bridge control |

## Gate Drive Current Settings

**Source current (IDRIVEP):** 10 mA to 1000 mA (4-bit value, see datasheet Table 7-8)  
**Sink current (IDRIVEN):** 20 mA to 2000 mA (4-bit value, see datasheet Table 7-7)

Common settings:
- Light load: 0x03 source, 0x05 sink (~50mA/100mA)
- Typical: 0x05 source, 0x08 sink (~100mA/200mA)
- Heavy load: 0x08 source, 0x0B sink (~200mA/400mA)

## Current Sense Amplifier Gains

| Gain | Setting | Shunt Voltage for 1A | Use Case |
|------|---------|----------------------|----------|
| 5 V/V | 0b00 | 5 mV | High current (>10A) |
| 10 V/V | 0b01 | 10 mV | Medium current (5-10A) |
| 20 V/V | 0b10 | 20 mV | Low current (2-5A) |
| 40 V/V | 0b11 | 40 mV | Very low current (<2A) |

**Note:** Shunt voltage = (Motor current) × (Shunt resistance) × (Gain)

## OCP VDS Threshold Levels

Settings 0x00-0x0F correspond to 0.06V to 2.0V in ~0.13V steps (datasheet Table 7-11).

Common thresholds:
- 0x04: 0.3V (sensitive, low-current applications)
- 0x08: 0.6V (typical)
- 0x0C: 1.0V (higher tolerance)

## Platform Integration Checklist

1. **SPI peripheral setup:**
   - Mode 1 (CPOL=0, CPHA=1)
   - ≤10 MHz clock
   - 8-bit or 16-bit transfer size
   - MSB first
   
2. **GPIO for nSCS (CS):**
   - Active low
   - Assert before transfer, deassert after
   - 400 ns minimum between transfers
   
3. **Optional GPIOs for monitoring:**
   - nFAULT (input): Goes low on fault condition
   - EN (output): Enable pin (active high)
   
4. **Power supply considerations:**
   - VM: 4.4V to 35V motor supply
   - AVDD/DVDD: 3.3V logic supply
   - Bypass capacitors per datasheet Section 8.2

## Error Handling

Return codes:
- `DRV8163_OK` (0): Success
- `DRV8163_ERR_INVALID_PARAM` (-1): NULL pointer or out-of-range parameter
- `DRV8163_ERR_SPI_TIMEOUT` (-2): SPI communication failure
- `DRV8163_ERR_NOT_INIT` (-3): Handle not initialized
- `DRV8163_ERR_FAULT` (-4): Device fault detected

## Thread Safety

This driver is **not inherently thread-safe**. If used in a multi-threaded environment:

1. Protect each handle with a mutex
2. Or ensure only one thread accesses each handle
3. Do not share handles between ISRs and main code without synchronization

## Hardware Requirements

- DRV8163 or DRV8163-Q1 device
- SPI-capable microcontroller
- Pull-up resistors on nFAULT pin (optional, if monitoring faults via interrupt)
- External gate resistors (if needed, per application)

## References

1. TI DRV8163-Q1 Datasheet (SLVSD06A)
2. TI Application Note: "Three-Phase Smart Gate Driver Design Considerations" (SLVAES0)
3. Device product page: https://www.ti.com/product/DRV8163-Q1

## License

This driver implementation is provided as example code. Consult your project's license requirements.
