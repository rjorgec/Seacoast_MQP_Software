/**
 * @file drv8163.h
 * @brief DRV8163 Three-Phase BLDC Motor Driver SPI Interface Driver
 *
 * Based on: TI DRV8163-Q1 Datasheet (SLVSD06A)
 * SPI Mode: Mode 1 (CPOL=0, CPHA=1)
 * Max SPI Clock: 10 MHz
 * Frame Format: 16-bit (1-bit R/W, 7-bit addr, 8-bit data)
 */

#ifndef DRV8163_H
#define DRV8163_H

#include <stdint.h>
#include <stdbool.h>

/* Register Addresses (Table 7-16 of datasheet) */
#define DRV8163_REG_DEVICE_ID 0x00
#define DRV8163_REG_FAULT 0x01
#define DRV8163_REG_STATUS1 0x02
#define DRV8163_REG_STATUS2 0x03
#define DRV8163_REG_COMMAND 0x04
#define DRV8163_REG_SPI_IN 0x05
#define DRV8163_REG_CONFIG1 0x06
#define DRV8163_REG_CONFIG2 0x07
#define DRV8163_REG_CONFIG3 0x08
#define DRV8163_REG_CONFIG4 0x09

/* SPI Frame Control Bits */
#define DRV8163_SPI_READ 0x8000 /* R/W bit: 1=Read, 0=Write */
#define DRV8163_ADDR_SHIFT 8
#define DRV8163_DATA_MASK 0x00FF

/* Device ID Register (0x00) - Read Only */
#define DRV8163_DEVICE_ID_REV_ID_MASK 0x0003
#define DRV8163_DEVICE_ID_DEV_ID_MASK 0x07F8
#define DRV8163_DEVICE_ID_DEV_ID_SHIFT 3

/* Fault Register (0x01) - Read Only */
#define DRV8163_FAULT_OLA (1 << 0)
#define DRV8163_FAULT_TSD (1 << 1)
#define DRV8163_FAULT_OCP (1 << 2)
#define DRV8163_FAULT_VMUV (1 << 3)
#define DRV8163_FAULT_VMOV (1 << 4)
#define DRV8163_FAULT_FAULT (1 << 5)
#define DRV8163_FAULT_POR (1 << 6)
#define DRV8163_FAULT_ERR (1 << 7)

/* Status1 Register (0x02) - Read Only */
#define DRV8163_STATUS1_OCP_L (1 << 0)
#define DRV8163_STATUS1_OCP_H_SHIFT 1
#define DRV8163_STATUS1_OCP_H_MASK (0x03 << 1)
#define DRV8163_STATUS1_OCP_L_SHIFT 3
#define DRV8163_STATUS1_OCP_L_MASK (0x03 << 3)
#define DRV8163_STATUS1_OCP_H5 (1 << 5)
#define DRV8163_STATUS1_ACTIVE (1 << 5)
#define DRV8163_STATUS1_ITRIP_CMP (1 << 6)
#define DRV8163_STATUS1_OLA (0x03 << 6)

/* Status2 Register (0x03) - Read Only */
#define DRV8163_STATUS2_OLP_CMP (1 << 0)
#define DRV8163_STATUS2_RSVD (1 << 1)
#define DRV8163_STATUS2_ACTIVE (1 << 4)
#define DRV8163_STATUS2_OTW (1 << 5)
#define DRV8163_STATUS2_RSVD2 (1 << 6)
#define DRV8163_STATUS2_DRV_STAT (1 << 7)

/* Command Register (0x04) - R/W */
#define DRV8163_CMD_REG_LOCK_MASK 0x0003
#define DRV8163_CMD_RSVD_SHIFT 2
#define DRV8163_CMD_RSVD_MASK (0x03 << 2)
#define DRV8163_CMD_SPI_IN_LOCK_SHIFT 4
#define DRV8163_CMD_SPI_IN_LOCK_MASK (0x03 << 4)
#define DRV8163_CMD_OTP (1 << 6)
#define DRV8163_CMD_CLR_FLT (1 << 7)

/* SPI_IN Register (0x05) - R/W */
#define DRV8163_SPI_IN_S_IN (1 << 0)
#define DRV8163_SPI_IN_RSVD (1 << 1)
#define DRV8163_SPI_IN_RSVD2 (1 << 2)
#define DRV8163_SPI_IN_S_PWMOFF (1 << 3)
#define DRV8163_SPI_IN_RSVD3_MASK (0x0F << 4)

/* Config1 Register (0x06) - R/W */
#define DRV8163_CONFIG1_OLA_RTRY (1 << 0)
#define DRV8163_CONFIG1_OV_RTRY (1 << 1)
#define DRV8163_CONFIG1_TSD_RTRY (1 << 2)
#define DRV8163_CONFIG1_OCP_RTRY (1 << 3)
#define DRV8163_CONFIG1_SSC_DIS (1 << 4)
#define DRV8163_CONFIG1_OVSEL (1 << 5)
#define DRV8163_CONFIG1_OTW_SEL (1 << 6)
#define DRV8163_CONFIG1_EN_OLA (1 << 7)

/* Config2 Register (0x07) - R/W */
#define DRV8163_CONFIG2_S_ITRIP_MASK 0x0007
#define DRV8163_CONFIG2_ISEL_MASK (0x03 << 3)
#define DRV8163_CONFIG2_ISEL_SHIFT 3
#define DRV8163_CONFIG2_S_DIAG_MASK (0x03 << 5)
#define DRV8163_CONFIG2_S_DIAG_SHIFT 5
#define DRV8163_CONFIG2_RSVD (1 << 7)

/* Config3 Register (0x08) - R/W */
#define DRV8163_CONFIG3_RSVD (1 << 0)
#define DRV8163_CONFIG3_SR_MASK (0x03 << 1)
#define DRV8163_CONFIG3_SR_SHIFT 1
#define DRV8163_CONFIG3_TBLK (1 << 3)
#define DRV8163_CONFIG3_RSVD2 (1 << 4)
#define DRV8163_CONFIG3_TOFF_MASK (0x03 << 5)
#define DRV8163_CONFIG3_TOFF_SHIFT 5

/* Config4 Register (0x09) - R/W */
#define DRV8163_CONFIG4_IN_SEL (1 << 0)
#define DRV8163_CONFIG4_RSVD (1 << 1)
#define DRV8163_CONFIG4_DRV_SEL (1 << 2)
#define DRV8163_CONFIG4_OCP_SEL_MASK (0x03 << 3)
#define DRV8163_CONFIG4_OCP_SEL_SHIFT 3
#define DRV8163_CONFIG4_OLA_FLTR (1 << 5)
#define DRV8163_CONFIG4_TOCP (1 << 6)
#define DRV8163_CONFIG4_OTW_REP (1 << 7)

/* Driver Instance Structure */
typedef struct
{
    /* User-provided SPI transfer function pointer */
    int (*spi_transfer)(uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

    /* Chip select control function pointer (optional) */
    void (*cs_assert)(void);
    void (*cs_deassert)(void);

    /* Cached register values for read-modify-write operations */
    uint8_t reg_cache[10]; /* 10 registers: 0x00-0x09 */

    /* Status flags */
    bool initialized;
} drv8163_handle_t;

/* Return codes */
typedef enum
{
    DRV8163_OK = 0,
    DRV8163_ERR_INVALID_PARAM = -1,
    DRV8163_ERR_SPI_TIMEOUT = -2,
    DRV8163_ERR_NOT_INIT = -3,
    DRV8163_ERR_FAULT = -4
} drv8163_status_t;

/* Function Prototypes */

/**
 * @brief Initialize DRV8163 driver instance
 * @param handle Pointer to driver handle structure
 * @param spi_transfer Function pointer to SPI transfer function
 * @param cs_assert Optional CS assert function (can be NULL)
 * @param cs_deassert Optional CS deassert function (can be NULL)
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_init(drv8163_handle_t *handle,
                              int (*spi_transfer)(uint8_t *, uint8_t *, uint16_t),
                              void (*cs_assert)(void),
                              void (*cs_deassert)(void));

/**
 * @brief Read a register from DRV8163
 * @param handle Pointer to driver handle
 * @param reg_addr Register address (0x00-0x06)
 * @param data Pointer to store read data
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_read_reg(drv8163_handle_t *handle, uint8_t reg_addr, uint8_t *data);

/**
 * @brief Write a register to DRV8163
 * @param handle Pointer to driver handle
 * @param reg_addr Register address (0x02-0x06, read-only regs will fail)
 * @param data Data to write
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_write_reg(drv8163_handle_t *handle, uint8_t reg_addr, uint8_t data);

/**
 * @brief Modify specific bits in a register (read-modify-write)
 * @param handle Pointer to driver handle
 * @param reg_addr Register address
 * @param mask Bit mask for bits to modify
 * @param value New value for masked bits
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_modify_reg(drv8163_handle_t *handle, uint8_t reg_addr,
                                    uint8_t mask, uint8_t value);

/**
 * @brief Read fault status register
 * @param handle Pointer to driver handle
 * @param faults Pointer to store fault register value
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_read_faults(drv8163_handle_t *handle, uint8_t *faults);

/**
 * @brief Clear fault conditions
 * @param handle Pointer to driver handle
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_clear_faults(drv8163_handle_t *handle);

/**
 * @brief Read device ID and revision
 * @param handle Pointer to driver handle
 * @param device_id Pointer to store device ID (6-bit value)
 * @param revision Pointer to store revision ID (2-bit value)
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_read_device_id(drv8163_handle_t *handle, uint8_t *device_id, uint8_t *revision);

/**
 * @brief Set output slew rate
 * @param handle Pointer to driver handle
 * @param slew_rate Slew rate setting (0-3, see datasheet)
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_set_slew_rate(drv8163_handle_t *handle, uint8_t slew_rate);

/**
 * @brief Set trip current threshold
 * @param handle Pointer to driver handle
 * @param trip_current Trip current setting (0-7, see datasheet)
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_set_trip_current(drv8163_handle_t *handle, uint8_t trip_current);

/**
 * @brief Set current sense selection
 * @param handle Pointer to driver handle
 * @param isel Current sense selection (0-3, see datasheet)
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_set_current_sense(drv8163_handle_t *handle, uint8_t isel);

/**
 * @brief Enable/disable overcurrent retry
 * @param handle Pointer to driver handle
 * @param enable True to enable retry, false to latch
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_set_ocp_retry(drv8163_handle_t *handle, bool enable);

/**
 * @brief Enable/disable open load detection
 * @param handle Pointer to driver handle
 * @param enable True to enable, false to disable
 * @return DRV8163_OK on success, error code otherwise
 */
drv8163_status_t drv8163_set_open_load_detect(drv8163_handle_t *handle, bool enable);

#endif /* DRV8163_H */
