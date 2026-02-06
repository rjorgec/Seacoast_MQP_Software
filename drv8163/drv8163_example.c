/**
 * @file drv8163_example.c
 * @brief Example usage of DRV8163 H-Bridge motor driver
 *
 * Demonstrates initialization, configuration, and fault handling
 * Assumes STM32 HAL or similar platform with SPI peripheral
 */

#include "drv8163.h"
#include <stdio.h>

/* Platform-specific includes would go here, e.g.:
 * #include "stm32f4xx_hal.h"
 * extern SPI_HandleTypeDef hspi1;
 */

/* Example SPI transfer function - must be implemented per platform */
int platform_spi_transfer(uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
    /* Example for STM32 HAL:
     * HAL_StatusTypeDef status;
     * status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, len, 100);
     * return (status == HAL_OK) ? 0 : -1;
     */

    /* Placeholder - implement based on your platform */
    return 0;
}

/* Example CS control functions - must be implemented per platform */
void platform_cs_assert(void)
{
    /* Example for STM32 HAL:
     * HAL_GPIO_WritePin(DRV8163_CS_GPIO_Port, DRV8163_CS_Pin, GPIO_PIN_RESET);
     */
}

void platform_cs_deassert(void)
{
    /* Example for STM32 HAL:
     * HAL_GPIO_WritePin(DRV8163_CS_GPIO_Port, DRV8163_CS_Pin, GPIO_PIN_SET);
     */
}

/* Example: Basic initialization and configuration */
int drv8163_example_init(void)
{
    drv8163_handle_t drv_handle;
    drv8163_status_t status;

    /* Initialize driver */
    status = drv8163_init(&drv_handle,
                          platform_spi_transfer,
                          platform_cs_assert,
                          platform_cs_deassert);

    if (status != DRV8163_OK)
    {
        printf("DRV8163 initialization failed: %d\n", status);
        return -1;
    }

    /* Read and verify device ID */
    uint8_t device_id, revision;
    status = drv8163_read_device_id(&drv_handle, &device_id, &revision);
    if (status != DRV8163_OK)
    {
        printf("Failed to read device ID: %d\n", status);
        return -1;
    }
    printf("DRV8163 Device ID: 0x%02X, Revision: %d\n", device_id, revision);

    /* Configure slew rate (2 = fast) */
    status = drv8163_set_slew_rate(&drv_handle, 2);
    if (status != DRV8163_OK)
    {
        printf("Failed to set slew rate: %d\n", status);
        return -1;
    }

    /* Set overcurrent trip threshold */
    status = drv8163_set_trip_current(&drv_handle, 4);
    if (status != DRV8163_OK)
    {
        printf("Failed to set trip current: %d\n", status);
        return -1;
    }

    /* Configure current sense selection */
    status = drv8163_set_current_sense(&drv_handle, 1);
    if (status != DRV8163_OK)
    {
        printf("Failed to set current sense: %d\n", status);
        return -1;
    }

    /* Enable overcurrent retry mode */
    status = drv8163_set_ocp_retry(&drv_handle, true);
    if (status != DRV8163_OK)
    {
        printf("Failed to set OCP retry: %d\n", status);
        return -1;
    }

    /* Enable open load detection */
    status = drv8163_set_open_load_detect(&drv_handle, true);
    if (status != DRV8163_OK)
    {
        printf("Failed to enable open load detect: %d\n", status);
        return -1;
    }

    printf("DRV8163 initialized successfully\n");
    return 0;
}

/* Example: Fault checking and clearing */
int drv8163_example_check_faults(drv8163_handle_t *handle)
{
    uint8_t faults;
    drv8163_status_t status;

    /* Read fault status */
    status = drv8163_read_faults(handle, &faults);

    if (status == DRV8163_ERR_FAULT)
    {
        printf("DRV8163 Fault detected! Fault register: 0x%02X\n", faults);

        /* Decode fault bits */
        if (faults & DRV8163_FAULT_OLA)
        {
            printf("  - OLA: Open load condition\n");
        }
        if (faults & DRV8163_FAULT_TSD)
        {
            printf("  - TSD: Thermal shutdown (device too hot!)\n");
        }
        if (faults & DRV8163_FAULT_OCP)
        {
            printf("  - OCP: Overcurrent protection triggered\n");
        }
        if (faults & DRV8163_FAULT_VMUV)
        {
            printf("  - VMUV: Motor supply undervoltage\n");
        }
        if (faults & DRV8163_FAULT_VMOV)
        {
            printf("  - VMOV: Motor supply overvoltage\n");
        }
        if (faults & DRV8163_FAULT_POR)
        {
            printf("  - POR: Power-on reset occurred\n");
        }
        if (faults & DRV8163_FAULT_ERR)
        {
            printf("  - ERR: SPI communication error\n");
        }

        /* Clear faults if safe to do so */
        printf("Clearing faults...\n");
        status = drv8163_clear_faults(handle);
        if (status != DRV8163_OK)
        {
            printf("Failed to clear faults: %d\n", status);
            return -1;
        }

        return 1; /* Fault occurred but cleared */
    }
    else if (status == DRV8163_OK)
    {
        printf("No faults detected\n");
        return 0;
    }
    else
    {
        printf("Failed to read faults: %d\n", status);
        return -1;
    }
}

/* Example: Read status registers for diagnostics */
int drv8163_example_read_status(drv8163_handle_t *handle)
{
    uint8_t status1, status2;
    drv8163_status_t status;

    /* Read STATUS1 register */
    status = drv8163_read_reg(handle, DRV8163_REG_STATUS1, &status1);
    if (status != DRV8163_OK)
    {
        printf("Failed to read STATUS1: %d\n", status);
        return -1;
    }

    /* Read STATUS2 register */
    status = drv8163_read_reg(handle, DRV8163_REG_STATUS2, &status2);
    if (status != DRV8163_OK)
    {
        printf("Failed to read STATUS2: %d\n", status);
        return -1;
    }

    printf("STATUS1: 0x%02X, STATUS2: 0x%02X\n", status1, status2);

    /* Check if driver is active */
    if (status2 & DRV8163_STATUS2_ACTIVE)
    {
        printf("  Driver is ACTIVE\n");
    }

    /* Check for overtemperature warning */
    if (status2 & DRV8163_STATUS2_OTW)
    {
        printf("  WARNING: Overtemperature detected\n");
    }

    return 0;
}

/* Example: Advanced configuration via direct register access */
int drv8163_example_advanced_config(drv8163_handle_t *handle)
{
    drv8163_status_t status;

    /* Example: Set dead time and blanking time in CONFIG3 */
    uint8_t config3_val = 0;
    config3_val |= (2 << DRV8163_CONFIG3_SR_SHIFT);   // Slew rate = 2
    config3_val |= DRV8163_CONFIG3_TBLK;              // Enable blanking time
    config3_val |= (1 << DRV8163_CONFIG3_TOFF_SHIFT); // Off-time setting

    status = drv8163_write_reg(handle, DRV8163_REG_CONFIG3, config3_val);
    if (status != DRV8163_OK)
    {
        printf("Failed to write CONFIG3: %d\n", status);
        return -1;
    }

    /* Example: Read back to verify */
    uint8_t readback;
    status = drv8163_read_reg(handle, DRV8163_REG_CONFIG3, &readback);
    if (status != DRV8163_OK)
    {
        printf("Failed to read CONFIG3: %d\n", status);
        return -1;
    }

    printf("CONFIG3 Register: 0x%02X\n", readback);

    return 0;
}

/* Example: Motor control with H-bridge */
void drv8163_example_motor_control(void)
{
    static drv8163_handle_t drv_handle;
    static bool initialized = false;

    /* One-time initialization */
    if (!initialized)
    {
        if (drv8163_example_init() == 0)
        {
            initialized = true;
        }
        else
        {
            return; /* Initialization failed */
        }
    }

    /* Periodic fault checking (e.g., every 100ms) */
    drv8163_example_check_faults(&drv_handle);

    /* Motor control via hardware pins IN1, IN2:
     *
     * Forward:  IN1=HIGH, IN2=LOW
     * Reverse:  IN1=LOW,  IN2=HIGH
     * Brake:    IN1=HIGH, IN2=HIGH
     * Coast:    IN1=LOW,  IN2=LOW
     *
     * These pins are controlled by MCU GPIO, not SPI
     */

    /* Alternatively, use SPI_IN register for software control:
     * drv8163_write_reg(&drv_handle, DRV8163_REG_SPI_IN, ...);
     */

    /* Current sensing can be read via ADC connected to IPROPI pin */
    /* Motor speed control via PWM to IN1/IN2 (hardware PWM from MCU timer) */
}

/* Example: Emergency stop on fault interrupt */
void drv8163_fault_isr_handler(void)
{
    /* This would be called from nFAULT pin interrupt */
    static drv8163_handle_t drv_handle;

    /* Immediately stop motor (set IN1=LOW, IN2=LOW via GPIO) */
    /* Read fault status to determine cause */
    uint8_t faults;
    drv8163_read_faults(&drv_handle, &faults);

    /* Log fault for later analysis */
    printf("FAULT ISR: 0x%02X\n", faults);

    /* Set flag for main loop to handle */
}