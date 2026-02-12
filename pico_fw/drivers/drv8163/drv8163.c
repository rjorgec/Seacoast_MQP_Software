/**
 * @file drv8163.c
 * @brief DRV8163 H-Bridge Motor Driver Implementation
 *
 * Raspberry Pi Pico implementation for simplified hardware variant
 */

#include "drv8163.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Platform-specific data structure */
typedef struct
{
    uint pwm_slice;            // PWM slice number
    uint pwm_chan_a;           // PWM channel for ctrl_a
    uint pwm_chan_b;           // PWM channel for ctrl_b
    repeating_timer_t timer;   // Timer for current monitoring
    uint32_t motor_start_time; // Timestamp when motor started (for blanking period)
    bool in_blanking_period;   // True if in startup blanking period
} drv8163_platform_t;

/* Forward declarations for internal functions */
static void drv8163_apply_motor_control(drv8163_t *dev);
static bool drv8163_current_monitor_callback(repeating_timer_t *rt);

/**
 * @brief Initialize the DRV8163 driver
 */
bool drv8163_init(drv8163_t *dev, const drv8163_config_t *cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    // Clear device structure
    memset(dev, 0, sizeof(drv8163_t));

    // Copy configuration
    memcpy(&dev->config, cfg, sizeof(drv8163_config_t));

    // Allocate platform data
    drv8163_platform_t *pdata = malloc(sizeof(drv8163_platform_t));
    if (!pdata)
    {
        return false;
    }
    memset(pdata, 0, sizeof(drv8163_platform_t));
    dev->platform_data = pdata;

    // Initialize PWM for motor control
    gpio_set_function(cfg->ctrl_a_pin, GPIO_FUNC_PWM);
    gpio_set_function(cfg->ctrl_b_pin, GPIO_FUNC_PWM);

    // Get PWM slice and channels
    pdata->pwm_slice = pwm_gpio_to_slice_num(cfg->ctrl_a_pin);
    pdata->pwm_chan_a = pwm_gpio_to_channel(cfg->ctrl_a_pin);
    pdata->pwm_chan_b = pwm_gpio_to_channel(cfg->ctrl_b_pin);

    // Configure PWM: 12-bit resolution (0-4095)
    pwm_set_wrap(pdata->pwm_slice, 4095);

    // Calculate clock divider for desired frequency
    float divider = (float)clock_get_hz(clk_sys) / (cfg->pwm_frequency_hz * 4096.0f);
    pwm_set_clkdiv(pdata->pwm_slice, divider);

    // Initialize both channels to 0
    pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, 0);
    pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, 0);

    // Enable PWM
    pwm_set_enabled(pdata->pwm_slice, true);

    // Initialize ADC for current sensing
    adc_init();
    adc_gpio_init(cfg->sense_pin);
    adc_select_input(cfg->sense_adc_channel);

    // Set initial state
    dev->current_state = DRV8163_MOTOR_STOP;
    dev->current_speed = 0;
    dev->monitoring_enabled = false;
    dev->current_status = DRV8163_CURRENT_OK;
    dev->last_current_adc.sum = drv8163_read_current(dev) * ADC_AVERAGES;

    printf("DRV8163: Initialized on pins A=%d, B=%d, Sense=%d (ADC%d)\n",
           cfg->ctrl_a_pin, cfg->ctrl_b_pin, cfg->sense_pin, cfg->sense_adc_channel);
    printf("DRV8163: PWM freq=%u Hz, Current thresholds: %u - %u\n",
           cfg->pwm_frequency_hz, cfg->current_low_threshold, cfg->current_high_threshold);

    return true;
}

/**
 * @brief Set motor state (stop, forward, reverse, brake)
 */
bool drv8163_set_motor_state(drv8163_t *dev, drv8163_motor_state_t state)
{
    if (!dev)
    {
        return false;
    }

    dev->current_state = state;
    drv8163_apply_motor_control(dev);
    return true;
}

/**
 * @brief Set motor speed via PWM duty cycle
 */
bool drv8163_set_motor_speed(drv8163_t *dev, uint16_t speed)
{
    if (!dev)
    {
        return false;
    }

    // Clamp speed to valid range (0-4095)
    if (speed > 4095)
    {
        speed = 4095;
    }

    dev->current_speed = speed;
    drv8163_apply_motor_control(dev);
    return true;
}

/**
 * @brief Set motor state and speed in one call
 */
bool drv8163_set_motor_control(drv8163_t *dev, drv8163_motor_state_t state, uint16_t speed)
{
    if (!dev)
    {
        return false;
    }

    // Clamp speed to valid range
    if (speed > 4095)
    {
        speed = 4095;
    }

    dev->current_state = state;
    dev->current_speed = speed;
    drv8163_apply_motor_control(dev);
    return true;
}

/**
 * @brief Internal function to apply motor control to hardware
 */
static void drv8163_apply_motor_control(drv8163_t *dev)
{
    drv8163_platform_t *pdata = (drv8163_platform_t *)dev->platform_data;

    switch (dev->current_state)
    {
    case DRV8163_MOTOR_STOP:
        // Coast: both outputs low
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, 0);
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, 0);
        break;

    case DRV8163_MOTOR_FORWARD:
        // Forward: A=PWM, B=0
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, dev->current_speed);
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, 0);
        break;

    case DRV8163_MOTOR_REVERSE:
        // Reverse: A=0, B=PWM
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, 0);
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, dev->current_speed);
        break;

    case DRV8163_MOTOR_BRAKE:
        // Active brake: both outputs high (full duty cycle)
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, 4095);
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, 4095);
        break;
    }
}

/**
 * @brief Read current sense ADC value
 */
uint16_t drv8163_read_current(drv8163_t *dev)
{
    if (!dev)
    {
        return 0;
    }

    adc_select_input(dev->config.sense_adc_channel);
    return adc_read();
}

/**
 * @brief Timer callback for current monitoring
 */
static bool drv8163_current_monitor_callback(repeating_timer_t *rt)
{
    drv8163_t *dev = (drv8163_t *)rt->user_data;
    drv8163_platform_t *pdata = (drv8163_platform_t *)dev->platform_data;

    if (!dev || !dev->monitoring_enabled)
    {
        return false; // Stop timer
    }

    // Read current
    uint16_t current_adc = drv8163_read_current(dev);
    dev->last_current_adc.sum = dev->last_current_adc.sum - dev->last_current_adc.buffer[dev->last_current_adc.head];
    dev->last_current_adc.sum = dev->last_current_adc.sum + current_adc;
    dev->last_current_adc.buffer[dev->last_current_adc.head] = current_adc;
    dev->last_current_adc.head = (dev->last_current_adc.head + 1) % ADC_AVERAGES;
    dev->last_current_adc.average = dev->last_current_adc.sum / ADC_AVERAGES;

    // Read time
    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Check thresholds
    drv8163_current_status_t new_status = DRV8163_CURRENT_OK;
    if (current_time > (pdata->motor_start_time + dev->config.startup_blanking_ms))
    {
        if (dev->last_current_adc.average > dev->config.current_high_threshold)
        {
            new_status = DRV8163_CURRENT_OVERCURRENT;
        }
        else if (dev->last_current_adc.average < dev->config.current_low_threshold)
        {
            new_status = DRV8163_CURRENT_UNDERCURRENT;
        }
    }

    // Update status and call callback if status changed
    if (new_status != dev->current_status)
    {
        dev->current_status = new_status;

        if (dev->config.current_cb)
        {
            dev->config.current_cb(new_status, current_adc);
        }

        // Print status change
        const char *status_str[] = {"OK", "OVERCURRENT", "UNDERCURRENT"};
        printf("DRV8163: Current status changed to %s (ADC=%u)\n",
               status_str[new_status], current_adc);
    }

    return true; // Continue monitoring
}

/**
 * @brief Start current monitoring with interrupt-based checking
 */
bool drv8163_start_current_monitoring(drv8163_t *dev)
{
    if (!dev || dev->monitoring_enabled)
    {
        return false;
    }

    drv8163_platform_t *pdata = (drv8163_platform_t *)dev->platform_data;

    dev->monitoring_enabled = true;
    dev->current_status = DRV8163_CURRENT_OK;

    // Initialize blanking period
    pdata->motor_start_time = to_ms_since_boot(get_absolute_time());
    pdata->in_blanking_period = true;

    // Start repeating timer (interval in negative ms)
    bool success = add_repeating_timer_ms(
        -(int32_t)dev->config.current_check_interval_ms,
        drv8163_current_monitor_callback,
        dev,
        &pdata->timer);

    if (success)
    {
        printf("DRV8163: Current monitoring started (interval=%u ms, blanking=%u ms)\n",
               dev->config.current_check_interval_ms, dev->config.startup_blanking_ms);
    }
    else
    {
        dev->monitoring_enabled = false;
        printf("DRV8163: Failed to start current monitoring\n");
    }

    return success;
}

/**
 * @brief Stop current monitoring
 */
void drv8163_stop_current_monitoring(drv8163_t *dev)
{
    if (!dev || !dev->monitoring_enabled)
    {
        return;
    }

    drv8163_platform_t *pdata = (drv8163_platform_t *)dev->platform_data;

    dev->monitoring_enabled = false;
    cancel_repeating_timer(&pdata->timer);

    printf("DRV8163: Current monitoring stopped\n");
}

/**
 * @brief Get current monitoring status
 */
drv8163_current_status_t drv8163_get_current_status(drv8163_t *dev)
{
    if (!dev)
    {
        return DRV8163_CURRENT_OK;
    }

    return dev->current_status;
}

/**
 * @brief Update current thresholds dynamically
 */
void drv8163_set_current_thresholds(drv8163_t *dev, uint16_t low_threshold, uint16_t high_threshold)
{
    if (!dev)
    {
        return;
    }

    dev->config.current_low_threshold = low_threshold;
    dev->config.current_high_threshold = high_threshold;

    printf("DRV8163: Current thresholds updated: %u - %u\n", low_threshold, high_threshold);
}

/**
 * @brief Emergency stop - immediately stops motor
 */
void drv8163_emergency_stop(drv8163_t *dev)
{
    if (!dev)
    {
        return;
    }

    // Stop current monitoring
    if (dev->monitoring_enabled)
    {
        drv8163_stop_current_monitoring(dev);
    }

    // Apply brake
    dev->current_state = DRV8163_MOTOR_BRAKE;
    dev->current_speed = 0;
    drv8163_apply_motor_control(dev);

    printf("DRV8163: EMERGENCY STOP activated\n");
}