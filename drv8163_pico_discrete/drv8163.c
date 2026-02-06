/**
 * @file drv8163.c
 * @brief DRV8163 Three-Phase BLDC Motor Driver SPI Interface Implementation
 *
 * Based on: TI DRV8163-Q1 Datasheet (SLVSD06A)
 * SPI Protocol: 16-bit frames, MSB first
 * Bit 15: R/W (1=read, 0=write)
 * Bits 14-8: Address (7 bits)
 * Bits 7-0: Data (8 bits)
 */

#include "drv8163.h"
#include <string.h>

/* Internal helper functions */

/**
 * @brief Execute SPI transaction with CS control
 */
static drv8163_status_t drv8163_spi_transaction(drv8163_handle_t *handle,
                                                uint16_t tx_frame,
                                                uint16_t *rx_frame)
{
    if (!handle || !handle->spi_transfer)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    uint8_t tx_data[2];
    uint8_t rx_data[2];

    /* Convert to big-endian (MSB first) */
    tx_data[0] = (uint8_t)(tx_frame >> 8);
    tx_data[1] = (uint8_t)(tx_frame & 0xFF);

    /* Assert CS if function provided */
    if (handle->cs_assert)
    {
        handle->cs_assert();
    }

    /* Execute SPI transfer */
    int result = handle->spi_transfer(tx_data, rx_data, 2);

    /* Deassert CS if function provided */
    if (handle->cs_deassert)
    {
        handle->cs_deassert();
    }

    if (result != 0)
    {
        return DRV8163_ERR_SPI_TIMEOUT;
    }

    /* Convert received data from big-endian */
    if (rx_frame)
    {
        *rx_frame = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    }

    return DRV8163_OK;
}

/* Public API Implementation */

drv8163_status_t drv8163_init(drv8163_handle_t *handle,
                              int (*spi_transfer)(uint8_t *, uint8_t *, uint16_t),
                              void (*cs_assert)(void),
                              void (*cs_deassert)(void))
{
    if (!handle || !spi_transfer)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    /* Initialize handle structure */
    memset(handle, 0, sizeof(drv8163_handle_t));
    handle->spi_transfer = spi_transfer;
    handle->cs_assert = cs_assert;
    handle->cs_deassert = cs_deassert;

    /* Read all registers to populate cache */
    for (uint8_t addr = 0; addr <= DRV8163_REG_CONFIG4; addr++)
    {
        uint8_t data;
        drv8163_status_t status = drv8163_read_reg(handle, addr, &data);
        if (status != DRV8163_OK)
        {
            return status;
        }
        handle->reg_cache[addr] = data;
    }

    handle->initialized = true;
    return DRV8163_OK;
}

drv8163_status_t drv8163_read_reg(drv8163_handle_t *handle, uint8_t reg_addr, uint8_t *data)
{
    if (!handle || !data)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    if (reg_addr > DRV8163_REG_CONFIG4)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    /* Construct read frame: R/W=1, address, data=0x00 */
    uint16_t tx_frame = DRV8163_SPI_READ | ((uint16_t)reg_addr << DRV8163_ADDR_SHIFT);
    uint16_t rx_frame = 0;

    drv8163_status_t status = drv8163_spi_transaction(handle, tx_frame, &rx_frame);
    if (status != DRV8163_OK)
    {
        return status;
    }

    /* Extract data byte from response */
    *data = (uint8_t)(rx_frame & DRV8163_DATA_MASK);

    /* Update cache if initialized */
    if (handle->initialized)
    {
        handle->reg_cache[reg_addr] = *data;
    }

    return DRV8163_OK;
}

drv8163_status_t drv8163_write_reg(drv8163_handle_t *handle, uint8_t reg_addr, uint8_t data)
{
    if (!handle)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    /* Registers 0x00-0x03 are read-only */
    if (reg_addr < DRV8163_REG_COMMAND || reg_addr > DRV8163_REG_CONFIG4)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    /* Construct write frame: R/W=0, address, data */
    uint16_t tx_frame = ((uint16_t)reg_addr << DRV8163_ADDR_SHIFT) | data;

    drv8163_status_t status = drv8163_spi_transaction(handle, tx_frame, NULL);
    if (status != DRV8163_OK)
    {
        return status;
    }

    /* Update cache if initialized */
    if (handle->initialized)
    {
        handle->reg_cache[reg_addr] = data;
    }

    return DRV8163_OK;
}

drv8163_status_t drv8163_modify_reg(drv8163_handle_t *handle, uint8_t reg_addr,
                                    uint8_t mask, uint8_t value)
{
    if (!handle)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    if (!handle->initialized)
    {
        return DRV8163_ERR_NOT_INIT;
    }

    /* Read current value from cache */
    uint8_t current_val = handle->reg_cache[reg_addr];

    /* Modify bits */
    uint8_t new_val = (current_val & ~mask) | (value & mask);

    /* Write modified value */
    return drv8163_write_reg(handle, reg_addr, new_val);
}

drv8163_status_t drv8163_read_faults(drv8163_handle_t *handle, uint8_t *faults)
{
    if (!handle || !faults)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    /* Read Fault register */
    drv8163_status_t status = drv8163_read_reg(handle, DRV8163_REG_FAULT, faults);
    if (status != DRV8163_OK)
    {
        return status;
    }

    /* Check if any fault is present */
    if (*faults != 0)
    {
        return DRV8163_ERR_FAULT;
    }

    return DRV8163_OK;
}

drv8163_status_t drv8163_clear_faults(drv8163_handle_t *handle)
{
    if (!handle)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    /* Set CLR_FLT bit in Command register */
    return drv8163_modify_reg(handle,
                              DRV8163_REG_COMMAND,
                              DRV8163_CMD_CLR_FLT,
                              DRV8163_CMD_CLR_FLT);
}

drv8163_status_t drv8163_read_device_id(drv8163_handle_t *handle, uint8_t *device_id, uint8_t *revision)
{
    if (!handle || !device_id || !revision)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    uint8_t data;
    drv8163_status_t status = drv8163_read_reg(handle, DRV8163_REG_DEVICE_ID, &data);
    if (status != DRV8163_OK)
    {
        return status;
    }

    *revision = data & DRV8163_DEVICE_ID_REV_ID_MASK;
    *device_id = (data & DRV8163_DEVICE_ID_DEV_ID_MASK) >> DRV8163_DEVICE_ID_DEV_ID_SHIFT;

    return DRV8163_OK;
}

drv8163_status_t drv8163_set_slew_rate(drv8163_handle_t *handle, uint8_t slew_rate)
{
    if (!handle)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    if (slew_rate > 3)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    return drv8163_modify_reg(handle,
                              DRV8163_REG_CONFIG3,
                              DRV8163_CONFIG3_SR_MASK,
                              slew_rate << DRV8163_CONFIG3_SR_SHIFT);
}

drv8163_status_t drv8163_set_trip_current(drv8163_handle_t *handle, uint8_t trip_current)
{
    if (!handle)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    if (trip_current > 7)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    return drv8163_modify_reg(handle,
                              DRV8163_REG_CONFIG2,
                              DRV8163_CONFIG2_S_ITRIP_MASK,
                              trip_current);
}

drv8163_status_t drv8163_set_current_sense(drv8163_handle_t *handle, uint8_t isel)
{
    if (!handle)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    if (isel > 3)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    return drv8163_modify_reg(handle,
                              DRV8163_REG_CONFIG2,
                              DRV8163_CONFIG2_ISEL_MASK,
                              isel << DRV8163_CONFIG2_ISEL_SHIFT);
}

drv8163_status_t drv8163_set_ocp_retry(drv8163_handle_t *handle, bool enable)
{
    if (!handle)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    return drv8163_modify_reg(handle,
                              DRV8163_REG_CONFIG1,
                              DRV8163_CONFIG1_OCP_RTRY,
                              enable ? DRV8163_CONFIG1_OCP_RTRY : 0);
}

drv8163_status_t drv8163_set_open_load_detect(drv8163_handle_t *handle, bool enable)
{
    if (!handle)
    {
        return DRV8163_ERR_INVALID_PARAM;
    }

    return drv8163_modify_reg(handle,
                              DRV8163_REG_CONFIG1,
                              DRV8163_CONFIG1_EN_OLA,
                              enable ? DRV8163_CONFIG1_EN_OLA : 0);
}