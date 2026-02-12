/**
 * @file drv8163.h
 * @brief DRV8163 H-Bridge Motor Driver Interface
 *
 * Hardware variant: 2 PWM control pins + 1 analog current sense pin
 * Features:
 * - H-bridge control with forward/reverse/brake/coast modes
 * - PWM speed control
 * - Analog current sensing via ADC
 * - Interrupt-based overcurrent/undercurrent detection
 */

#ifndef DRV8163_H
#define DRV8163_H

#include <stdint.h>
#include <stdbool.h>

/* Motor Control States */
typedef enum
{
    DRV8163_MOTOR_STOP = 0, // Coast (both outputs low)
    DRV8163_MOTOR_FORWARD,  // Forward direction
    DRV8163_MOTOR_REVERSE,  // Reverse direction
    DRV8163_MOTOR_BRAKE     // Active brake (both outputs high)
} drv8163_motor_state_t;

/* Current Monitoring Status */
typedef enum
{
    DRV8163_CURRENT_OK = 0,
    DRV8163_CURRENT_OVERCURRENT,
    DRV8163_CURRENT_UNDERCURRENT
} drv8163_current_status_t;

/* Rolling Average Structure*/
#define ADC_AVERAGES 10
typedef struct
{
    uint16_t buffer[ADC_AVERAGES]; // buffer
    uint8_t head;                  // buffer head
    uint16_t sum;                  // sum
    uint16_t average;              // average
} drv8163_adc_buffer;

/* Callback function types */
typedef void (*drv8163_delay_ms_t)(uint32_t ms);
typedef void (*drv8163_delay_us_t)(uint32_t us);
typedef void (*drv8163_current_callback_t)(drv8163_current_status_t status, uint16_t current_adc);

/* Driver Configuration */
typedef struct
{
    void *user_ctx;                        // User context passed to callbacks
    uint8_t ctrl_a_pin;                    // H-bridge control A pin (GPIO number)
    uint8_t ctrl_b_pin;                    // H-bridge control B pin (GPIO number)
    uint8_t sense_pin;                     // Analog current sense pin (GPIO number)
    uint8_t sense_adc_channel;             // ADC channel for sense pin
    uint16_t current_high_threshold;       // ADC value for overcurrent detection
    uint16_t current_low_threshold;        // ADC value for undercurrent detection
    uint32_t pwm_frequency_hz;             // PWM frequency for motor control
    uint32_t current_check_interval_ms;    // How often to check current (ms)
    uint32_t startup_blanking_ms;          // Ignore overcurrent for this duration after motor start
    drv8163_current_callback_t current_cb; // Current event callback (optional)
    drv8163_delay_ms_t delay_ms;           // Millisecond delay function (optional)
} drv8163_config_t;

/* Driver Instance */
typedef struct
{
    drv8163_config_t config;                 // Configuration
    drv8163_motor_state_t current_state;     // Current motor state
    uint16_t current_speed;                  // Current speed (0-4095 for 12-bit PWM)
    bool monitoring_enabled;                 // Current monitoring active
    drv8163_current_status_t current_status; // Last current status
    drv8163_adc_buffer last_current_adc;     // Last ADC reading
    void *platform_data;                     // Platform-specific data
} drv8163_t;

/* Function Prototypes */

/**
 * @brief Initialize the DRV8163 driver
 * @param dev Pointer to driver instance
 * @param cfg Pointer to configuration structure
 * @return true if initialization successful, false otherwise
 */
bool drv8163_init(drv8163_t *dev, const drv8163_config_t *cfg);

/**
 * @brief Set motor state (stop, forward, reverse, brake)
 * @param dev Pointer to driver instance
 * @param state Desired motor state
 * @return true if successful, false otherwise
 */
bool drv8163_set_motor_state(drv8163_t *dev, drv8163_motor_state_t state);

/**
 * @brief Set motor speed via PWM duty cycle
 * @param dev Pointer to driver instance
 * @param speed Speed value (0-4095 for 12-bit PWM resolution)
 * @return true if successful, false otherwise
 */
bool drv8163_set_motor_speed(drv8163_t *dev, uint16_t speed);

/**
 * @brief Set motor state and speed in one call
 * @param dev Pointer to driver instance
 * @param state Motor state
 * @param speed Speed value (0-4095 for 12-bit PWM)
 * @return true if successful, false otherwise
 */
bool drv8163_set_motor_control(drv8163_t *dev, drv8163_motor_state_t state, uint16_t speed);

/**
 * @brief Read current sense ADC value
 * @param dev Pointer to driver instance
 * @return 12-bit ADC value representing motor current
 */
uint16_t drv8163_read_current(drv8163_t *dev);

/**
 * @brief Start current monitoring with interrupt-based checking
 * @param dev Pointer to driver instance
 * @return true if monitoring started, false otherwise
 */
bool drv8163_start_current_monitoring(drv8163_t *dev);

/**
 * @brief Stop current monitoring
 * @param dev Pointer to driver instance
 */
void drv8163_stop_current_monitoring(drv8163_t *dev);

/**
 * @brief Get current monitoring status
 * @param dev Pointer to driver instance
 * @return Current status (OK, overcurrent, undercurrent)
 */
drv8163_current_status_t drv8163_get_current_status(drv8163_t *dev);

/**
 * @brief Update current thresholds dynamically
 * @param dev Pointer to driver instance
 * @param low_threshold New low threshold ADC value
 * @param high_threshold New high threshold ADC value
 */
void drv8163_set_current_thresholds(drv8163_t *dev, uint16_t low_threshold, uint16_t high_threshold);

/**
 * @brief Emergency stop - immediately stops motor
 * @param dev Pointer to driver instance
 */
void drv8163_emergency_stop(drv8163_t *dev);

#endif /* DRV8163_H */
