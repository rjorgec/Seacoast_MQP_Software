

For a write command ( $W0 = 0$ ), the response word on the SDO pin is the data currently in the register being written to.

For a read command ( $W0 = 1$ ), the response word is the data currently in the register being read.

**Table 7-13. SDI Input Data Word Format**

|     | R/W | ADDRESS |     |     |     |    | DON'T CARE | DATA |    |    |    |    |    |    |    |
|-----|-----|---------|-----|-----|-----|----|------------|------|----|----|----|----|----|----|----|
| B15 | B14 | B13     | B12 | B11 | B10 | B9 | B8         | B7   | B6 | B5 | B4 | B3 | B2 | B1 | B0 |
| 0   | W0  | A4      | A3  | A2  | A1  | A0 | X          | D7   | D6 | D5 | D4 | D3 | D2 | D1 | D0 |

**Table 7-14. SDO Output Data Word Format**

| STATUS |     |      |      |     |     |    |    | REPORT |    |    |    |    |    |    |    |
|--------|-----|------|------|-----|-----|----|----|--------|----|----|----|----|----|----|----|
| B15    | B14 | B13  | B12  | B11 | B10 | B9 | B8 | B7     | B6 | B5 | B4 | B3 | B2 | B1 | B0 |
| 1      | 1   | UVLO | CPUV | OCP | STL | TF | OL | D7     | D6 | D5 | D4 | D3 | D2 | D1 | D0 |

#### 7.5.1.2 SPI for a Single Target Device

The SPI is used to set device configurations, operating parameters, and read out diagnostic information. The SPI operates in target mode. The SPI input-data (SDI) word consists of a 16-bit word, with 8 bits command and 8 bits of data. The SPI output data (SDO) word consists of 8 bits of status register with fault status indication and 8 bits of register data. [Figure 7-23](#) shows the data sequence between the MCU and the SPI target driver.

![Diagram of an SPI transaction between an MCU and a device. It shows three signals: nSCS (chip select), SDI (input data), and SDO (output data). The nSCS signal is active low, shown as a pulse. The SDI signal shows a 16-bit word with an 8-bit command (A1) followed by 8-bit data (D1). The SDO signal shows an 8-bit status word (S1) followed by 8-bit register data (R1).](187bba66c887c745c512add37a577c5e_img.jpg)

Diagram of an SPI transaction between an MCU and a device. It shows three signals: nSCS (chip select), SDI (input data), and SDO (output data). The nSCS signal is active low, shown as a pulse. The SDI signal shows a 16-bit word with an 8-bit command (A1) followed by 8-bit data (D1). The SDO signal shows an 8-bit status word (S1) followed by 8-bit register data (R1).

**Figure 7-23. SPI Transaction Between MCU and the device**

A valid frame must meet the following conditions:

- The SCLK pin must be low when the nSCS pin goes low and when the nSCS pin goes high.
- The nSCS pin should be taken high for at least 500 ns between frames.
- When the nSCS pin is asserted high, any signals at the SCLK and SDI pins are ignored, and the SDO pin is in the high-impedance state (Hi-Z).
- Full 16 SCLK cycles must occur.
- Data is captured on the falling edge of the clock and data is driven on the rising edge of the clock.
- The most-significant bit (MSB) is shifted in and out first.
- If the data word sent to SDI pin is less than 16 bits or more than 16 bits, a frame error occurs and the data word is ignored.
- For a write command, the existing data in the register being written to is shifted out on the SDO pin following the 8-bit command data.

#### 7.5.1.3 SPI for Multiple Target Devices in Daisy Chain Configuration

The DRV8434S device can be connected in a daisy chain configuration to keep GPIO ports available when multiple devices are communicating to the same MCU. [Figure 7-24](#) shows the topology when three devices are connected in series.

![Figure 7-24: Three DRV8434S Devices Connected in Daisy Chain. A Master MCU is connected to three DRV8434S devices in a daisy chain. The Master's M-SDO pin connects to the SDI1 pin of the first DRV8434S (1). The SDO1/SDI2 pin of DRV8434S (1) connects to the SDI1 pin of DRV8434S (2). The SDO1/SDI2 pin of DRV8434S (2) connects to the SDI1 pin of DRV8434S (3). The SDO3 pin of DRV8434S (3) connects back to the Master's M-SDI pin. All three devices share the Master's M-nSCS, M-SCLK, and M-SDI signals.](47a7beddcb8a1b7abdca746967e32bb4_img.jpg)

Figure 7-24: Three DRV8434S Devices Connected in Daisy Chain. A Master MCU is connected to three DRV8434S devices in a daisy chain. The Master's M-SDO pin connects to the SDI1 pin of the first DRV8434S (1). The SDO1/SDI2 pin of DRV8434S (1) connects to the SDI1 pin of DRV8434S (2). The SDO1/SDI2 pin of DRV8434S (2) connects to the SDI1 pin of DRV8434S (3). The SDO3 pin of DRV8434S (3) connects back to the Master's M-SDI pin. All three devices share the Master's M-nSCS, M-SCLK, and M-SDI signals.

**Figure 7-24. Three DRV8434S Devices Connected in Daisy Chain**

The first device in the chain receives data from the MCU in the following format for 3-device configuration: 2 bytes of header (HDRx) followed by 3 bytes of address (Ax) followed by 3 bytes of data (Dx).

![Figure 7-25: SPI Frame With Three Devices. A timing diagram showing the SPI frame for three devices. The nSCS signal is active low. The SDI1 line shows the Master's transmission: HDR1, HDR2, A3, A2, A1, D3, D2, D1. The SDO1/SDI2 line shows the first device's response: S1, HDR1, HDR2, A3, A2, R1, D3, D2. The SDO2/SDI3 line shows the second device's response: S2, S1, HDR1, HDR2, A3, R2, R1, D3. The SDO3 line shows the third device's response: S3, S2, S1, HDR1, HDR2, R3, R2, R1. Annotations indicate 'Status response here' for S1, 'Reads executed here' for R1, R2, R3, and 'Writes executed here' for D1, D2, D3. Arrows point to 'All Address bytes reach destination' and 'All Data bytes reach destination'.](4ff00a489baaa34a0c01070de1d613db_img.jpg)

Figure 7-25: SPI Frame With Three Devices. A timing diagram showing the SPI frame for three devices. The nSCS signal is active low. The SDI1 line shows the Master's transmission: HDR1, HDR2, A3, A2, A1, D3, D2, D1. The SDO1/SDI2 line shows the first device's response: S1, HDR1, HDR2, A3, A2, R1, D3, D2. The SDO2/SDI3 line shows the second device's response: S2, S1, HDR1, HDR2, A3, R2, R1, D3. The SDO3 line shows the third device's response: S3, S2, S1, HDR1, HDR2, R3, R2, R1. Annotations indicate 'Status response here' for S1, 'Reads executed here' for R1, R2, R3, and 'Writes executed here' for D1, D2, D3. Arrows point to 'All Address bytes reach destination' and 'All Data bytes reach destination'.

**Figure 7-25. SPI Frame With Three Devices**

After the data has been transmitted through the chain, the MCU receives the data string in the following format for 3-device configuration: 3 bytes of status (Sx) followed by 2 bytes of header followed by 3 bytes of report (Rx).

![Figure 7-26: SPI Data Sequence for Three Devices. A timing diagram showing the SPI data sequence for the MCU. The nSCS signal is active low. The SDI line shows the Master's transmission: HDR1, HDR2, A3, A2, A1, D3, D2, D1. The SDO line shows the MCU's reception: S3, S2, S1, HDR1, HDR2, R3, R2, R1.](f1091147d93cee4dfa88498610e395a7_img.jpg)

Figure 7-26: SPI Data Sequence for Three Devices. A timing diagram showing the SPI data sequence for the MCU. The nSCS signal is active low. The SDI line shows the Master's transmission: HDR1, HDR2, A3, A2, A1, D3, D2, D1. The SDO line shows the MCU's reception: S3, S2, S1, HDR1, HDR2, R3, R2, R1.

**Figure 7-26. SPI Data Sequence for Three Devices**

The header bytes contain information of the number of devices connected in the chain, and a global clear fault command that will clear the fault registers of all the devices on the rising edge of the chip select (nSCS) signal. Header values N5 through N0 are 6 bits dedicated to show the number of devices in the chain. Up to 63 devices can be connected in series for each daisy chain connection.

The 5 LSBs of the HDR2 register are don't care bits that can be used by the MCU to determine integrity of the daisy chain connection. Header bytes must start with 1 and 0 for the two MSBs.

![Figure 7-27: Header Bytes. Shows two 8-bit registers, HDR 1 and HDR 2, with bit-level details.](329c96049bb432e9c2cbda4e224a0c9c_img.jpg)

**HDR 1**

|   |   |    |    |    |    |    |    |
|---|---|----|----|----|----|----|----|
| 1 | 0 | N5 | N4 | N3 | N2 | N1 | N0 |
|---|---|----|----|----|----|----|----|

**HDR 2**

|   |   |     |   |   |   |   |   |
|---|---|-----|---|---|---|---|---|
| 1 | 0 | CLR | x | x | x | x | x |
|---|---|-----|---|---|---|---|---|

Annotations:

- Arrow from N5 to N0 points to: No. of devices in the chain (up to  $2^6 - 1 = 63$ )
- Arrow from CLR points to: 1 = global FAULT clear, 0 = don't care
- Arrow from the x bits points to: Don't care

Figure 7-27: Header Bytes. Shows two 8-bit registers, HDR 1 and HDR 2, with bit-level details.

**Figure 7-27. Header Bytes**

The status byte provides information about the fault status register for each device in the daisy chain so that the MCU does not have to initiate a read command to read the fault status from any particular device. This keeps additional read commands for the MCU and makes the system more efficient to determine fault conditions flagged in a device. Status bytes must start with 1 and 1 for the two MSBs.

![Figure 7-28: Contents of Header, Status, Address, and Data Bytes for DRV8434S. Shows four 8-bit registers with bit-level details.](107da2e3495b2f24352c9e3b26ec4841_img.jpg)

**Header Bytes (HDRx)**

|   |   |     |    |    |    |    |    |
|---|---|-----|----|----|----|----|----|
| 1 | 0 | N5  | N4 | N3 | N2 | N1 | N0 |
| 1 | 0 | CLR | X  | X  | X  | X  | X  |

**Status Byte (Sx)**

|   |   |      |      |     |     |    |    |
|---|---|------|------|-----|-----|----|----|
| 1 | 1 | UVLO | CPUV | OCP | STL | TF | OL |
|---|---|------|------|-----|-----|----|----|

**Address Byte (Ax)**

|   |     |    |    |    |    |    |   |
|---|-----|----|----|----|----|----|---|
| 0 | R/W | A4 | A3 | A2 | A1 | A0 | X |
|---|-----|----|----|----|----|----|---|

**Data Byte (Dx)**

|    |    |    |    |    |    |    |    |
|----|----|----|----|----|----|----|----|
| D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0 |
|----|----|----|----|----|----|----|----|

Figure 7-28: Contents of Header, Status, Address, and Data Bytes for DRV8434S. Shows four 8-bit registers with bit-level details.

**Figure 7-28. Contents of Header, Status, Address, and Data Bytes for DRV8434S**

When data passes through a device, it determines the position of itself in the chain by counting the number of status bytes it receives followed by the first header byte. For example, in this 3-device configuration, device 2 in the chain receives two status bytes before receiving the HDR1 byte which is then followed by the HDR2 byte.

From the two status bytes, the data can determine that its position is second in the chain. From the HDR2 byte, the data can determine how many devices are connected in the chain. In this way, the data only loads the relevant address and data byte in its buffer and bypasses the other bits. This protocol allows for faster communication without adding latency to the system for up to 63 devices in the chain.

The address and data bytes remain the same with respect to a 1-device connection. The report bytes (R1 through R3), as shown in [Figure 7-26](#), are the content of the register being accessed.

![Figure 7-29: SPI Transaction timing diagram. The diagram shows six signal lines: nSCS (active low), SCLK (clock), SDI (Serial Data Input), SDO (Serial Data Output), Capture Point, and Propagate Point. The SDI line shows data frames: X MSB, X LSB, Z MSB, Z LSB. The SDO line shows data frames: Z MSB, Z LSB, X MSB, X LSB. The nSCS line shows active low periods for each transaction. The Capture Point and Propagate Point lines show vertical markers indicating the start and end of each transaction.](1316cc5a3c69067473f110271212db3b_img.jpg)

Figure 7-29: SPI Transaction timing diagram. The diagram shows six signal lines: nSCS (active low), SCLK (clock), SDI (Serial Data Input), SDO (Serial Data Output), Capture Point, and Propagate Point. The SDI line shows data frames: X MSB, X LSB, Z MSB, Z LSB. The SDO line shows data frames: Z MSB, Z LSB, X MSB, X LSB. The nSCS line shows active low periods for each transaction. The Capture Point and Propagate Point lines show vertical markers indicating the start and end of each transaction.

**Figure 7-29. SPI Transaction**

#### **7.5.1.4 SPI for Multiple Target Devices in Parallel Configuration**

![Figure 7-30: Three DRV8434S devices connected in parallel configuration. A Microcontroller block on the left has pins M-CS1, M-CS2, M-CS3, M-CLK, M-SDO, and M-SDI. The M-CLK pin connects to the SCLK pin of all three DRV8434S devices (labeled (1), (2), and (3)). The M-SDO pin connects to the SDI1, SDI2, and SDI3 pins of the three devices. The SDO1, SDO2, and SDO3 pins of the devices connect to the M-SDI pin of the Microcontroller. Each device has its own nSCS pin connected to M-CS1, M-CS2, and M-CS3 respectively.](34b047489058d6400b412cd0ae2334ba_img.jpg)

Figure 7-30: Three DRV8434S devices connected in parallel configuration. A Microcontroller block on the left has pins M-CS1, M-CS2, M-CS3, M-CLK, M-SDO, and M-SDI. The M-CLK pin connects to the SCLK pin of all three DRV8434S devices (labeled (1), (2), and (3)). The M-SDO pin connects to the SDI1, SDI2, and SDI3 pins of the three devices. The SDO1, SDO2, and SDO3 pins of the devices connect to the M-SDI pin of the Microcontroller. Each device has its own nSCS pin connected to M-CS1, M-CS2, and M-CS3 respectively.

**Figure 7-30. Three DRV8434S Devices Connected in Parallel Configuration**

## 7.6 Register Maps

Table 7-15 lists the memory-mapped registers for the DRV8434S device. All register addresses not listed in Table 7-15 should be considered as reserved locations and the register contents must not be modified.

**Table 7-15. Memory Map**

| Register Name | 7               | 6          | 5         | 4          | 3                    | 2           | 1         | 0         | Access Type | Address |      |      |      |
|---------------|-----------------|------------|-----------|------------|----------------------|-------------|-----------|-----------|-------------|---------|------|------|------|
| FAULT Status  | FAULT           | SPI_ERROR  | UVLO      | CPUV       | OCP                  | STL         | TF        | OL        | R           | 0x00    |      |      |      |
| DIAG Status 1 | OCP_LS2_B       | OCP_HS2_B  | OCP_LS1_B | OCP_HS1_B  | OCP_LS2_A            | OCP_HS2_A   | OCP_LS1_A | OCP_HS1_A | R           | 0x01    |      |      |      |
| DIAG Status 2 | RSVD            | OTW        | OTS       | STL_LRN_OK | STALL                | RSVD        | OL_B      | OL_A      | R           | 0x02    |      |      |      |
| CTRL1         | TRQ_DAC [3:0]   |            |           |            | RSVD                 |             | OL_MODE   |           | RSVD        | RW      | 0x03 |      |      |
| CTRL2         | EN_OUT          | RSVD       |           | TOFF [1:0] |                      | DECAY [2:0] |           |           |             | RW      | 0x04 |      |      |
| CTRL3         | DIR             | STEP       | SPI_DIR   | SPI_STEP   | MICROSTEP_MODE [3:0] |             |           |           |             |         | RW   | 0x05 |      |
| CTRL4         | CLR_FLT         | LOCK [2:0] |           |            | EN_OL                | OCP_MODE    | OTSD_MODE | OTW_REP   | RW          | 0x06    |      |      |      |
| CTRL5         | RSVD            |            | STL_LRN   | EN_STL     | STL_REP              | RSVD        |           |           |             |         | RW   | 0x07 |      |
| CTRL6         | STALL_TH [7:0]  |            |           |            | RSVD                 |             |           |           |             |         |      | RW   | 0x08 |
| CTRL7         | RC_RIPPLE[1:0]  |            | EN_SSC    | TRQ_SCALE  | STALL_TH[11:8]       |             |           |           |             |         | RW   | 0x09 |      |
| CTRL8         | TRQ_COUNT [7:0] |            |           |            | RSVD                 |             |           |           |             |         |      | R    | 0x0A |
| CTRL9         | REV_ID[3:0]     |            |           |            | TRQ_COUNT[11:8]      |             |           |           |             |         | R    | 0x0B |      |

Complex bit access types are encoded to fit into small table cells. Table 7-16 shows the codes that are used for access types in this section.

**Table 7-16. Access Type Codes**

| Access Type            | Code | Description                            |
|------------------------|------|----------------------------------------|
| Read Type              |      |                                        |
| R                      | R    | Read                                   |
| Write Type             |      |                                        |
| W                      | W    | Write                                  |
| Reset or Default Value |      |                                        |
| -n                     |      | Value after reset or the default value |

### 7.6.1 Status Registers

The status registers are used to reporting warning and fault conditions. Status registers are read-only registers

[Table 7-17](#) lists the memory-mapped registers for the status registers. All register offset addresses not listed in [Table 7-17](#) should be considered as reserved locations and the register contents should not be modified.

**Table 7-17. Status Registers Summary Table**

| Address | Register Name | Section                       |
|---------|---------------|-------------------------------|
| 0x00    | FAULT status  | <a href="#">Section 7.6.2</a> |
| 0x01    | DIAG status 1 | <a href="#">Section 7.6.3</a> |
| 0x02    | DIAG status 2 | <a href="#">Section 7.6.4</a> |

### 7.6.2 FAULT Status Register Name (address = 0x00)

FAULT status is shown in [Figure 7-31](#) and described in [Table 7-18](#).

Read-only

**Figure 7-31. FAULT Status Register**

| 7     | 6         | 5    | 4    | 3    | 2    | 1    | 0    |
|-------|-----------|------|------|------|------|------|------|
| FAULT | SPI_ERROR | UVLO | CPUV | OCP  | STL  | TF   | OL   |
| R-0b  | R-0b      | R-0b | R-0b | R-0b | R-0b | R-0b | R-0b |

**Table 7-18. FAULT Status Register Field Descriptions**

| Bit | Field     | Type | Default | Description                                                                                                                                                                                                                                                                                                                                  |
|-----|-----------|------|---------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7   | FAULT     | R    | 0b      | When nFAULT pin is at 1, FAULT bit is 0. When nFAULT pin is at 0, FAULT bit is 1.                                                                                                                                                                                                                                                            |
| 6   | SPI_ERROR | R    | 0b      | Indicates SPI protocol errors, such as more SCLK pulses than are required or SCLK is absent even though nSCS is low. Becomes high in fault and the nFAULT pin is driven low. Normal operation resumes when the protocol error is removed and a clear faults command has been issued either through the CLR_FLT bit or an nSLEEP reset pulse. |
| 5   | UVLO      | R    | 0b      | Indicates an supply undervoltage lockout fault condition.                                                                                                                                                                                                                                                                                    |
| 4   | CPUV      | R    | 0b      | Indicates charge pump undervoltage fault condition.                                                                                                                                                                                                                                                                                          |
| 3   | OCP       | R    | 0b      | Indicates overcurrent fault condition                                                                                                                                                                                                                                                                                                        |
| 2   | STL       | R    | 0b      | Indicates motor stall condition.                                                                                                                                                                                                                                                                                                             |
| 1   | TF        | R    | 0b      | Logic OR of the overtemperature warning and overtemperature shutdown.                                                                                                                                                                                                                                                                        |
| 0   | OL        | R    | 0b      | Indicates open-load condition.                                                                                                                                                                                                                                                                                                               |

### 7.6.3 DIAG Status 1 (address = 0x01)

DIAG Status 1 is shown in [Figure 7-32](#) and described in [Table 7-19](#).

Read-only

**Figure 7-32. DIAG Status 1 Register**

| 7         | 6         | 5         | 4         | 3         | 2         | 1         | 0         |
|-----------|-----------|-----------|-----------|-----------|-----------|-----------|-----------|
| OCP_LS2_B | OCP_HS2_B | OCP_LS1_B | OCP_HS1_B | OCP_LS2_A | OCP_HS2_A | OCP_LS1_A | OCP_HS1_A |
| R-0b      | R-0b      | R-0b      | R-0b      | R-0b      | R-0b      | R-0b      | R-0b      |

**Table 7-19. DIAG Status 1 Register Field Descriptions**

| Bit | Field     | Type | Default | Description                                                               |
|-----|-----------|------|---------|---------------------------------------------------------------------------|
| 7   | OCP_LS2_B | R    | 0b      | Indicates overcurrent fault on the low-side FET of half bridge 2 in BOUT  |
| 6   | OCP_HS2_B | R    | 0b      | Indicates overcurrent fault on the high-side FET of half bridge 2 in BOUT |

**Table 7-19. DIAG Status 1 Register Field Descriptions (continued)**

| Bit | Field     | Type | Default | Description                                                               |
|-----|-----------|------|---------|---------------------------------------------------------------------------|
| 5   | OCP_LS1_B | R    | 0b      | Indicates overcurrent fault on the low-side FET of half bridge 1 in BOUT  |
| 4   | OCP_HS1_B | R    | 0b      | Indicates overcurrent fault on the high-side FET of half bridge 1 in BOUT |
| 3   | OCP_LS2_A | R    | 0b      | Indicates overcurrent fault on the low-side FET of half bridge 2 in AOUT  |
| 2   | OCP_HS2_A | R    | 0b      | Indicates overcurrent fault on the high-side FET of half bridge 2 in AOUT |
| 1   | OCP_LS1_A | R    | 0b      | Indicates overcurrent fault on the low-side FET of half bridge 1 in AOUT  |
| 0   | OCP_HS1_A | R    | 0b      | Indicates overcurrent fault on the high-side FET of half bridge 1 in AOUT |

### 7.6.4 DIAG Status 2 (address = 0x02)

DIAG Status 2 is shown in [Figure 7-33](#) and described in [Table 7-20](#).

Read-only

**Figure 7-33. DIAG Status 2 Register**

| 7    | 6    | 5    | 4          | 3     | 2    | 1    | 0    |
|------|------|------|------------|-------|------|------|------|
| RSVD | OTW  | OTS  | STL_LRN_OK | STALL | RSVD | OL_B | OL_A |
| R-0b | R-0b | R-0b | R-0b       | R-0b  | R-0b | R-0b | R-0b |

**Table 7-20. DIAG Status 2 Register Field Descriptions**

| Bit | Field      | Type | Default | Description                                      |
|-----|------------|------|---------|--------------------------------------------------|
| 7   | RSVD       | R    | 0b      | Resvred.                                         |
| 6   | OTW        | R    | 0b      | Indicates overtemperature warning.               |
| 5   | OTS        | R    | 0b      | Indicates overtemperature shutdown.              |
| 4   | STL_LRN_OK | R    | 0b      | Indicates stall detection learning is successful |
| 3   | STALL      | R    | 0b      | Indicates motor stall condition                  |
| 2   | RSVD       | R    | 0b      | Reserved.                                        |
| 1   | OL_B       | R    | 0b      | Indicates open-load detection on BOUT            |
| 0   | OL_A       | R    | 0b      | Indicates open-load detection on AOUT            |

### 7.6.5 Control Registers

The IC control registers are used to configure the device. Status registers are read and write capable.

[Table 7-21](#) lists the memory-mapped registers for the control registers. All register offset addresses not listed in [Table 7-21](#) should be considered as reserved locations and the register contents should not be modified.

**Table 7-21. Control Registers Summary Table**

| Address | Register Name | Section                        |
|---------|---------------|--------------------------------|
| 0x03    | CTRL1         | <a href="#">Section 7.6.6</a>  |
| 0x04    | CTRL2         | <a href="#">Section 7.6.7</a>  |
| 0x05    | CTRL3         | <a href="#">Section 7.6.8</a>  |
| 0x06    | CTRL4         | <a href="#">Section 7.6.9</a>  |
| 0x07    | CTRL5         | <a href="#">Section 7.6.10</a> |
| 0x08    | CTRL6         | <a href="#">Section 7.6.11</a> |
| 0x09    | CTRL7         | <a href="#">Section 7.6.12</a> |
| 0x0A    | CTRL8         | <a href="#">Section 7.6.13</a> |
| 0x0B    | CTRL9         |                                |

#### 7.6.6 CTRL1 Control Register (address = 0x03)

CTRL1 control is shown in [Figure 7-34](#) and described in [Table 7-22](#).

Read/Write

**Figure 7-34. CTRL1 Control Register**

| 7             | 6 | 5 | 4 | 3      | 2 | 1       | 0     |
|---------------|---|---|---|--------|---|---------|-------|
| TRQ_DAC [3:0] |   |   |   | RSVD   |   | OL_MODE | RSVD  |
| RW-0000b      |   |   |   | RW-00b |   | RW-0b   | RW-0b |

**Table 7-22. CTRL1 Control Register Field Descriptions**

| Bit | Field         | Type | Default | Description                                                                                                                                                                                                                                                                         |
|-----|---------------|------|---------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7-4 | TRQ_DAC [3:0] | R/W  | 0000b   | <b>0000b = 100%</b><br>0001b = 93.75%<br>0010b = 87.5%<br>0011b = 81.25%<br>0100b = 75%<br>0101b = 68.75%<br>0110b = 62.5%<br>0111b = 56.25%<br>1000b = 50%<br>1001b = 43.75%<br>1010b = 37.5%<br>1011b = 31.25%<br>1100b = 25%<br>1101b = 18.75%<br>1110b = 12.5%<br>1111b = 6.25% |
| 3-2 | RSVD          | R/W  | 00b     | Reserved                                                                                                                                                                                                                                                                            |
| 1   | OL_MODE       | R/W  | 0b      | <b>0b = nFAULT is released after latched OL fault is cleared using CLR_FLT bit or nSLEEP reset pulse</b><br>1b = nFAULT is released immediately after OL fault condition is removed                                                                                                 |
| 0   | RSVD          | R/W  | 0b      | Reserved                                                                                                                                                                                                                                                                            |

#### 7.6.7 CTRL2 Control Register (address = 0x04)

CTRL2 is shown in [Figure 7-35](#) and described in [Table 7-23](#).

Read/Write

**Figure 7-35. CTRL2 Control Register**

| 7      | 6      | 5 | 4          | 3 | 2           | 1 | 0 |
|--------|--------|---|------------|---|-------------|---|---|
| EN_OUT | RSVD   |   | TOFF [1:0] |   | DECAY [2:0] |   |   |
| R/W-0b | RW-00b |   | RW-01b     |   | RW-111b     |   |   |

**Table 7-23. CTRL2 Control Register Field Descriptions**

| Bit | Field      | Type | Default | Description                                                                                  |
|-----|------------|------|---------|----------------------------------------------------------------------------------------------|
| 7   | EN_OUT     | R/W  | 0b      | Write '0' to disable all outputs.                                                            |
| 6-5 | RSVD       | R/W  | 00b     | Reserved                                                                                     |
| 4-3 | TOFF [1:0] | R/W  | 01b     | 00b = 7 $\mu$ s<br><b>01b = 16 <math>\mu</math>s</b><br>10b = 24 $\mu$ s<br>11b = 32 $\mu$ s |

**Table 7-23. CTRL2 Control Register Field Descriptions (continued)**

| Bit | Field       | Type | Default | Description                                                                                                                                                                                                                                                                                                                                                                |
|-----|-------------|------|---------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2-0 | DECAY [2:0] | R/W  | 111b    | 000b = Increasing SLOW, decreasing SLOW<br>001b = Increasing SLOW, decreasing MIXED 30%<br>010b = Increasing SLOW, decreasing MIXED 60%<br>011b = Increasing SLOW, decreasing FAST<br>100b = Increasing MIXED 30%, decreasing MIXED 30%<br>101b = Increasing MIXED 60%, decreasing MIXED 60%<br>110b = Smart tune Dynamic Decay<br><b>111b = Smart tune Ripple Control</b> |

#### 7.6.8 CTRL3 Control Register (address = 0x05)

CTRL3 is shown in [Figure 7-36](#) and described in [Table 7-24](#).

Read/Write

**Figure 7-36. CTRL3 Control Register**

| 7      | 6      | 5       | 4        | 3                    | 2 | 1 | 0 |
|--------|--------|---------|----------|----------------------|---|---|---|
| DIR    | STEP   | SPI_DIR | SPI_STEP | MICROSTEP_MODE [3:0] |   |   |   |
| R/W-0b | R/W-0b | R/W-0b  | R/W-0b   | R/W-0110b            |   |   |   |

**Table 7-24. CTRL3 Control Register Field Descriptions**

| Bit | Field                | Type | Default | Description                                                                                                                                                                                                                                                                                                                                                     |
|-----|----------------------|------|---------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7   | DIR                  | R/W  | 0b      | Direction input. Logic '1' sets the direction of stepping, when SPI_DIR = 1.                                                                                                                                                                                                                                                                                    |
| 6   | STEP                 | R/W  | 0b      | Step input. Logic '1' causes the indexer to advance one step, when SPI_STEP = 1. This bit is self-clearing, automatically becomes '0' after writing '1'.                                                                                                                                                                                                        |
| 5   | SPI_DIR              | R/W  | 0b      | <b>0b = Outputs follow input pin for DIR</b><br>1b = Outputs follow SPI registers DIR                                                                                                                                                                                                                                                                           |
| 4   | SPI_STEP             | R/W  | 0b      | <b>0b = Outputs follow input pin for STEP</b><br>1b = Outputs follow SPI registers STEP                                                                                                                                                                                                                                                                         |
| 3-0 | MICROSTEP_MODE [3:0] | R/W  | 0110b   | <b>0000b = Full step (2-phase excitation) with 100% current</b><br>0001b = Full step (2-phase excitation) with 71% current<br>0010b = Non-circular 1/2 step<br>0011b = 1/2 step<br>0100b = 1/4 step<br>0101b = 1/8 step<br>0110b = 1/16 step<br>0111b = 1/32 step<br>1000b = 1/64 step<br>1001b = 1/128 step<br>1010b = 1/256 step<br>1011b to 1111b = Reserved |

#### 7.6.9 CTRL4 Control Register (address = 0x06)

CTRL4 is shown in [Figure 7-37](#) and described in [Table 7-25](#).

Read/Write

**Figure 7-37. CTRL4 Control Register**

| 7       | 6          | 5 | 4 | 3      | 2        | 1         | 0       |
|---------|------------|---|---|--------|----------|-----------|---------|
| CLR_FLT | LOCK [2:0] |   |   | EN_OL  | OCP_MODE | OTSD_MODE | OTW_REP |
| R/W-0b  | R/W-011b   |   |   | R/W-0b | R/W-0b   | R/W-0b    | R/W-0b  |

**Table 7-25. CTRL4 Control Register Field Descriptions**

| Bit | Field      | Type | Default | Description                                                                                                                                                                                                                                                                                                           |
|-----|------------|------|---------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7   | CLR_FLT    | R/W  | 0b      | Write '1' to this bit to clear all latched fault bits. This bit automatically resets after being written.                                                                                                                                                                                                             |
| 6-4 | LOCK [2:0] | R/W  | 011b    | Write 110b to lock the settings by ignoring further register writes except to these bits and address 0x06h bit 7 (CLR_FLT). Writing any sequence other than 110b has no effect when unlocked.<br>Write 011b to this register to unlock all registers. Writing any sequence other than 011b has no effect when locked. |
| 3   | EN_OL      | R/W  | 0b      | Write '1' to enable open load detection                                                                                                                                                                                                                                                                               |
| 2   | OCP_MODE   | R/W  | 0b      | <b>0b = Overcurrent condition causes a latched fault</b><br>1b = Overcurrent condition causes an automatic retrying fault                                                                                                                                                                                             |
| 1   | OTSD_MODE  | R/W  | 0b      | <b>0b = Overtemperature condition will cause latched fault</b><br>1b = Overtemperature condition will cause automatic recovery fault                                                                                                                                                                                  |
| 0   | TW_REP     | R/W  | 0b      | <b>0b = Overtemperature or undertemperature warning is not reported on the nFAULT line</b><br>1b = Overtemperature or undertemperature warning is reported on the nFAULT line                                                                                                                                         |

### 7.6.10 CTRL5 Control Register (address = 0x07)

CTRL5 control is shown in [Figure 7-38](#) and described in [Table 7-26](#).

Read/Write

**Figure 7-38. CTRL5 Control Register**

| 7       | 6 | 5       | 4      | 3       | 2        | 1 | 0 |
|---------|---|---------|--------|---------|----------|---|---|
| RSVD    |   | STL_LRN | EN_STL | STL_REP | RSVD     |   |   |
| R/W-00b |   | R/W-0b  | R/W-0b | R/W-1b  | R/W-000b |   |   |

**Table 7-26. CTRL5 Control Register Field Descriptions**

| Bit | Field   | Type | Default | Description                                                                                                                            |
|-----|---------|------|---------|----------------------------------------------------------------------------------------------------------------------------------------|
| 7-6 | RSVD    | R/W  | 00b     | Reserved. Should always be '00'.                                                                                                       |
| 5   | STL_LRN | R/W  | 0b      | Write '1' to learn stall count for stall detection. This bit automatically returns to '0' when the stall learning process is complete. |
| 4   | EN_STL  | R/W  | 0b      | <b>0b = Stall detection is disabled</b><br>1b = Stall detection is enabled                                                             |
| 3   | STL_REP | R/W  | 1b      | 0b = Stall detection is not reported on nFAULT<br><b>1b = Stall detection is reported on nFAULT</b>                                    |
| 2-0 | RSVD    | R/W  | 000b    | Reserved. Should always be '000'.                                                                                                      |

### 7.6.11 CTRL6 Control Register (address = 0x08)

CTRL6 is shown in [Figure 7-39](#) and described in [Table 7-27](#).

Read/Write

**Figure 7-39. CTRL6 Control Register**

| 7              | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|----------------|---|---|---|---|---|---|---|
| STALL_TH [7:0] |   |   |   |   |   |   |   |
| R/W-00000011b  |   |   |   |   |   |   |   |