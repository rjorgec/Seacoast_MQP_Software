#ifndef DRV8163_PICO_H
#define DRV8163_PICO_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"

#include "drv8163.h"

// --- Pin Definitions (Adjust these to your specific hardware connections) ---

// SPI Pins
#define DRV8163_SPI_PORT spi0
#define DRV8163_SCK_PIN 18
#define DRV8163_MOSI_PIN 19
#define DRV8163_MISO_PIN 16
#define DRV8163_CS_PIN 17

// GPIO Control Pins for Motor H-Bridge (IN1, IN2)
#define DRV8163_IN1_PIN 20
#define DRV8163_IN2_PIN 21

// nSLEEP Pin (Device Enable)
#define DRV8163_NSLEEP_PIN 22

// nFAULT Pin (Fault Output, active low)
#define DRV8163_NFAULT_PIN 26 // Example, can be any available GPIO

// IPROPI Pin (Current Sense Output, analog)
#define DRV8163_IPROPI_ADC_PIN 27    // ADC1 input
#define DRV8163_IPROPI_ADC_CHANNEL 1 // Corresponds to GPIO27

// ITRIP Pin (Current Regulation Threshold, PMW simulated DAC)
#define DRV8163_ITRIP_PIN 28 // Example, can be any available GPIO

// --- Motor Control Enumerations ---
typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_BRAKE
} motor_state_t;

// --- Pico-specific DRV8163 handle extension ---
// This could be used to store Pico-specific data if needed, but for now,
// we'll pass the base drv8163_handle_t and manage Pico resources separately.

// --- Function Prototypes for Pico-specific Hardware Interface ---

/**
 * @brief Initializes the Pico's SPI peripheral for DRV8163 communication.
 * @param baud_rate SPI baud rate in Hz.
 */
void drv8163_pico_spi_init(uint32_t baud_rate);

/**
 * @brief Pico-specific SPI transfer function for DRV8163.
 *        This function matches the signature required by drv8163_init().
 * @param tx_data Pointer to transmit data buffer.
 * @param rx_data Pointer to receive data buffer.
 * @param len Number of bytes to transfer.
 * @return 0 on success, -1 on failure.
 */
int drv8163_pico_spi_transfer(uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

/**
 * @brief Pico-specific CS assert function for DRV8163.
 */
void drv8163_pico_cs_assert(void);

/**
 * @brief Pico-specific CS deassert function for DRV8163.
 */
void drv8163_pico_cs_deassert(void);

/**
 * @brief Initializes the Pico's GPIO pins for DRV8163 motor control and nSLEEP.
 */
void drv8163_pico_gpio_init(void);

/**
 * @brief Sets the motor state (STOP, FORWARD, REVERSE, BRAKE) using IN1/IN2 GPIOs.
 * @param state Desired motor state.
 */
void drv8163_set_motor_state(motor_state_t state);

/**
 * @brief Initializes the Pico's PWM peripheral for motor speed control.
 * @param pwm_freq Desired PWM frequency in Hz.
 */
void drv8163_pico_pwm_init(uint32_t pwm_freq);

/**
 * @brief Sets the PWM duty cycle for motor speed control.
 *        Assumes PWM is configured on one of the IN pins (e.g., IN1).
 * @param duty_cycle Duty cycle value (0-255 for 8-bit resolution, or 0-4096 for 1-bit).
 *                   This example uses 16-bit for consistency with `pwm_set_chan_level`.
 */
void drv8163_set_pwm_duty_cycle(uint16_t duty_cycle);

/**
 * @brief Sets motor control with direction and speed in a single call.
 *        Handles both forward and reverse operation with speed control.
 * @param state Motor state (MOTOR_FORWARD, MOTOR_REVERSE, MOTOR_STOP, MOTOR_BRAKE).
 * @param speed Speed value (0-65535 for 16-bit PWM resolution). Only used for MOTOR_FORWARD and MOTOR_REVERSE.
 *              For MOTOR_STOP and MOTOR_BRAKE, this parameter is ignored.
 */
void drv8163_set_motor_control(motor_state_t state, uint16_t speed);

/**
 * @brief Initializes the Pico's PWM for ITRIP pin to simulate DAC output.
 * @param dac_freq Desired PWM frequency for ITRIP in Hz.
 */
void drv8163_pico_itrip_pwm_init(uint32_t dac_freq);

/**
 * @brief Writes a value to the ITRIP pin using PWM to simulate a DAC output.
 * @param itrip_value Value to set on ITRIP pin (0-255 for 8-bit resolution).
 */
void drv8163_set_itrip_pwm(uint8_t itrip_value);

/**
 * @brief Initializes the Pico's ADC for reading the IPROPI pin.
 */
void drv8163_pico_adc_init(void);

/**
 * @brief Reads the analog value from the IPROPI pin via ADC.
 * @return Raw 12-bit ADC value.
 */
uint16_t drv8163_read_iprobi_raw_adc(void);

/**
 * @brief Configures the nFAULT pin as an input with an interrupt.
 * @param handler Function pointer to the ISR to be called on nFAULT.
 */
void drv8163_pico_nfaul_interrupt_init(void);

/**
 * @brief Enables the DRV8163 by asserting the nSLEEP pin.
 */
void drv8163_enable_driver(void);

/**
 * @brief Disables the DRV8163 by deasserting the nSLEEP pin.
 */
void drv8163_disable_driver(void);

/**
 * @brief Initializes a non-blocking current monitor task that reverses motor direction
 *        if current is too high or too low.
 * @param speed Motor speed (0-4096 for PWM level to maintain).
 * @param low_threshold ADC threshold for low current condition.
 * @param high_threshold ADC threshold for high current condition.
 * @param check_interval_ms How often to check current in milliseconds.
 * @return true if task started successfully, false otherwise.
 */
bool drv8163_start_current_monitor_task(uint16_t speed, uint16_t low_threshold,
                                        uint16_t high_threshold, uint32_t check_interval_ms);

/**
 * @brief Stops the non-blocking current monitor task.
 */
void drv8163_stop_current_monitor_task(void);

#endif // DRV8163_PICO_H
