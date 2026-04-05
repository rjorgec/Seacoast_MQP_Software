/**
 * @file drv8263.c
 * @brief TI DRV8263 H-Bridge Motor Driver Implementation
 *
 * Raspberry Pi Pico (RP2040/RP2350) implementation.
 * Supports standard H-bridge mode for flap linear actuators and
 * independent half-bridge mode for the hot wire (IN1) and vacuum
 * pump 2 (IN2) — both outputs can be driven simultaneously.
 *
 * In independent half-bridge mode the ADC sense pin is unused;
 * current regulation is handled by an external Rsense resistor and
 * the DRV8263's internal regulation loop.
 */

#include "drv8263.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── Platform-specific data ─────────────────────────────────────────────────── */

typedef struct
{
    uint pwm_slice;            /* PWM slice number */
    uint pwm_chan_a;           /* PWM channel for ctrl_a (IN1) */
    uint pwm_chan_b;           /* PWM channel for ctrl_b (IN2) */
    repeating_timer_t timer;   /* Timer for current monitoring */
    uint32_t motor_start_time; /* Timestamp when motor started (for blanking period) */
    bool in_blanking_period;   /* True if in startup blanking period */
} drv8263_platform_t;

/* ── Forward declarations ───────────────────────────────────────────────────── */

static void drv8263_apply_motor_control(drv8263_t *dev);
static bool drv8263_current_monitor_callback(repeating_timer_t *rt);

/* ── Public API ─────────────────────────────────────────────────────────────── */

/**
 * @brief Initialize the DRV8263 driver.
 */
bool drv8263_init(drv8263_t *dev, const drv8263_config_t *cfg)
{
    if (!dev || !cfg)
    {
        return false;
    }

    /* Clear device structure */
    memset(dev, 0, sizeof(drv8263_t));

    /* Copy configuration */
    memcpy(&dev->config, cfg, sizeof(drv8263_config_t));

    /* Allocate platform data */
    drv8263_platform_t *pdata = malloc(sizeof(drv8263_platform_t));
    if (!pdata)
    {
        return false;
    }
    memset(pdata, 0, sizeof(drv8263_platform_t));
    dev->platform_data = pdata;

    /* Initialize PWM for motor control */
    gpio_set_function(cfg->ctrl_a_pin, GPIO_FUNC_PWM);
    gpio_set_function(cfg->ctrl_b_pin, GPIO_FUNC_PWM);

    /* Get PWM slice and channels */
    pdata->pwm_slice = pwm_gpio_to_slice_num(cfg->ctrl_a_pin);
    pdata->pwm_chan_a = pwm_gpio_to_channel(cfg->ctrl_a_pin);
    pdata->pwm_chan_b = pwm_gpio_to_channel(cfg->ctrl_b_pin);

    /* Configure PWM: 12-bit resolution (0–4095) */
    pwm_set_wrap(pdata->pwm_slice, 4095);

    /* Calculate clock divider for desired frequency */
    float divider = (float)clock_get_hz(clk_sys) / (cfg->pwm_frequency_hz * 4096.0f);
    pwm_set_clkdiv(pdata->pwm_slice, divider);

    /* Initialize both channels to 0 (coast) */
    pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, 0);
    pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, 0);

    /* Enable PWM */
    pwm_set_enabled(pdata->pwm_slice, true);

    /* Initialize ADC for current sensing.
     * NOTE: In independent H-bridge mode (hot wire / vacuum pump 2 instance)
     * the ADC sense pin is unused — current is set by an external Rsense
     * resistor and regulated internally by the DRV8263. The adc_gpio_init()
     * call is harmless but the readings are not acted upon. */
    adc_init();
    adc_gpio_init(cfg->sense_pin);
    adc_select_input(cfg->sense_adc_channel);

    /* Set initial state */
    dev->current_state = DRV8263_MOTOR_STOP;
    dev->current_speed = 0;
    dev->monitoring_enabled = false;
    dev->current_status = DRV8263_CURRENT_OK;
    dev->last_current_adc.sum = drv8263_read_current(dev) * ADC_AVERAGES;
    dev->last_current_adc.average = dev->last_current_adc.sum / ADC_AVERAGES;

    #ifdef ACTUATOR_DEBUG
    printf("DRV8263: Initialized on pins A=%d, B=%d, Sense=%d (ADC%d)\n",
           cfg->ctrl_a_pin, cfg->ctrl_b_pin, cfg->sense_pin, cfg->sense_adc_channel);
    printf("DRV8263: PWM freq=%u Hz, Current thresholds: %u - %u\n",
           cfg->pwm_frequency_hz, cfg->current_low_threshold, cfg->current_high_threshold);
    printf("DRV8263: ADC val=%u\n", dev->last_current_adc.average);
    #endif

    return true;
}

/**
 * @brief Set motor state (stop, forward, reverse, brake).
 */
bool drv8263_set_motor_state(drv8263_t *dev, drv8263_motor_state_t state)
{
    if (!dev)
    {
        return false;
    }

    dev->current_state = state;
    drv8263_apply_motor_control(dev);
    return true;
}

/**
 * @brief Set motor speed via PWM duty cycle.
 */
bool drv8263_set_motor_speed(drv8263_t *dev, uint16_t speed)
{
    if (!dev)
    {
        return false;
    }

    if (speed > 4095u)
    {
        speed = 4095u;
    }

    dev->current_speed = speed;
    drv8263_apply_motor_control(dev);
    return true;
}

/**
 * @brief Set motor state and speed in one call.
 */
bool drv8263_set_motor_control(drv8263_t *dev, drv8263_motor_state_t state, uint16_t speed)
{
    if (!dev)
    {
        return false;
    }

    if (speed > 4095u)
    {
        speed = 4095u;
    }

    dev->current_state = state;
    dev->current_speed = speed;
    drv8263_apply_motor_control(dev);
    return true;
}

/**
 * @brief Internal: apply the current state/speed to hardware PWM channels.
 */
static void drv8263_apply_motor_control(drv8263_t *dev)
{
    drv8263_platform_t *pdata = (drv8263_platform_t *)dev->platform_data;

    switch (dev->current_state)
    {
    case DRV8263_MOTOR_STOP:
        /* Coast: both outputs low */
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, 0);
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, 0);
        break;

    case DRV8263_MOTOR_FORWARD:
        /* Forward / IN1 active: A=PWM, B=0 */
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, dev->current_speed);
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, 0);
        break;

    case DRV8263_MOTOR_REVERSE:
        /* Reverse / IN2 active: A=0, B=PWM */
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, 0);
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, dev->current_speed);
        break;

    case DRV8263_MOTOR_BRAKE:
        /* Active brake: both outputs high */
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, 4095);
        pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, 4095);
        break;
    }
}

/**
 * @brief Read current sense ADC value.
 *
 * NOTE: In independent H-bridge mode (hot wire / vacuum2 instance) the
 * current is regulated by external Rsense — this reading is not used
 * for control purposes.
 */
uint16_t drv8263_read_current(drv8263_t *dev)
{
    if (!dev)
    {
        return 0;
    }

    adc_select_input(dev->config.sense_adc_channel);
    return adc_read();
}

/**
 * @brief Repeating-timer callback for current monitoring.
 */
static bool drv8263_current_monitor_callback(repeating_timer_t *rt)
{
    drv8263_t *dev = (drv8263_t *)rt->user_data;
    drv8263_platform_t *pdata = (drv8263_platform_t *)dev->platform_data;

    if (!dev || !dev->monitoring_enabled)
    {
        return false; /* Stop timer */
    }

    /* Read and update rolling average */
    uint16_t current_adc = drv8263_read_current(dev);
    dev->last_current_adc.sum =
        dev->last_current_adc.sum - dev->last_current_adc.buffer[dev->last_current_adc.head];
    dev->last_current_adc.sum += current_adc;
    dev->last_current_adc.buffer[dev->last_current_adc.head] = current_adc;
    dev->last_current_adc.head = (dev->last_current_adc.head + 1) % ADC_AVERAGES;
    dev->last_current_adc.average = dev->last_current_adc.sum / ADC_AVERAGES;

    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    /* Evaluate thresholds (after startup blanking) */
    drv8263_current_status_t new_status = DRV8263_CURRENT_OK;
    if (current_time > (pdata->motor_start_time + dev->config.startup_blanking_ms))
    {
        if (dev->last_current_adc.average > dev->config.current_high_threshold)
        {
            new_status = DRV8263_CURRENT_OVERCURRENT;
        }
        else if (dev->last_current_adc.average < dev->config.current_low_threshold)
        {
            new_status = DRV8263_CURRENT_UNDERCURRENT;
        }
    }

    /* Notify on status change */
    if (new_status != dev->current_status)
    {
        dev->current_status = new_status;

        if (dev->config.current_cb)
        {
            dev->config.current_cb(new_status, current_adc);
        }

        const char *status_str[] = {"OK", "OVERCURRENT", "UNDERCURRENT"};
        #ifdef ACTUATOR_DEBUG
        printf("DRV8263: Current status changed to %s (ADC=%u)\n",
               status_str[new_status], current_adc);
        #endif
    }

    return true; /* Continue monitoring */
}

/**
 * @brief Start current monitoring with repeating-timer-based checking.
 */
bool drv8263_start_current_monitoring(drv8263_t *dev)
{
    if (!dev || dev->monitoring_enabled)
    {
        return false;
    }

    drv8263_platform_t *pdata = (drv8263_platform_t *)dev->platform_data;

    dev->monitoring_enabled = true;
    dev->current_status = DRV8263_CURRENT_OK;

    /* Pre-seed the rolling ADC average with the current reading so that
     * stale values from a previous session do not contaminate the new
     * session's threshold comparisons. */
    {
        uint16_t init_val = drv8263_read_current(dev);
        for (int i = 0; i < ADC_AVERAGES; ++i)
        {
            dev->last_current_adc.buffer[i] = init_val;
        }
        dev->last_current_adc.sum = (uint16_t)(init_val * ADC_AVERAGES);
        dev->last_current_adc.average = init_val;
        dev->last_current_adc.head = 0;
    }

    /* Record start time for blanking period */
    pdata->motor_start_time = to_ms_since_boot(get_absolute_time());
    pdata->in_blanking_period = true;

    bool success = add_repeating_timer_ms(
        -(int32_t)dev->config.current_check_interval_ms,
        drv8263_current_monitor_callback,
        dev,
        &pdata->timer);

    if (success)
    {

        #ifdef ACTUATOR_DEBUG
        printf("DRV8263: Current monitoring started (interval=%u ms, blanking=%u ms)\n",
               dev->config.current_check_interval_ms, dev->config.startup_blanking_ms);
        #endif
    }
    else
    {
        dev->monitoring_enabled = false;
        #ifdef ACTUATOR_DEBUG
        printf("DRV8263: Failed to start current monitoring\n");
        #endif
    }

    return success;
}

/**
 * @brief Stop current monitoring.
 */
void drv8263_stop_current_monitoring(drv8263_t *dev)
{
    if (!dev || !dev->monitoring_enabled)
    {
        return;
    }

    drv8263_platform_t *pdata = (drv8263_platform_t *)dev->platform_data;

    dev->monitoring_enabled = false;
    cancel_repeating_timer(&pdata->timer);

    #ifdef ACTUATOR_DEBUG
    printf("DRV8263: Current monitoring stopped\n");
    #endif
}

/**
 * @brief Get current monitoring status.
 */
drv8263_current_status_t drv8263_get_current_status(drv8263_t *dev)
{
    if (!dev)
    {
        return DRV8263_CURRENT_OK;
    }

    return dev->current_status;
}

/**
 * @brief Update current thresholds dynamically.
 */
void drv8263_set_current_thresholds(drv8263_t *dev, uint16_t low_threshold, uint16_t high_threshold)
{
    if (!dev)
    {
        return;
    }

    dev->config.current_low_threshold = low_threshold;
    dev->config.current_high_threshold = high_threshold;

    #ifdef ACTUATOR_DEBUG
    printf("DRV8263: Current thresholds updated: %u - %u\n", low_threshold, high_threshold);
    #endif
}

/**
 * @brief Drive IN1 (ctrl_a_pin) independently without affecting IN2.
 *
 * For use when the DRV8263 is wired in independent H-bridge mode:
 *   IN1 → hot wire (current set by external Rsense, DRV8263 internal regulation)
 *   IN2 → vacuum pump 2
 * Both half-bridges can be active simultaneously.
 */
bool drv8263_set_in1(drv8263_t *dev, uint16_t duty)
{
    if (!dev)
    {
        return false;
    }

    if (duty > 4095u)
    {
        duty = 4095u;
    }

    drv8263_platform_t *pdata = (drv8263_platform_t *)dev->platform_data;
    pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_a, duty);
    return true;
}

/**
 * @brief Drive IN2 (ctrl_b_pin) independently without affecting IN1.
 */
bool drv8263_set_in2(drv8263_t *dev, uint16_t duty)
{
    if (!dev)
    {
        return false;
    }

    if (duty > 4095u)
    {
        duty = 4095u;
    }

    drv8263_platform_t *pdata = (drv8263_platform_t *)dev->platform_data;
    pwm_set_chan_level(pdata->pwm_slice, pdata->pwm_chan_b, duty);
    return true;
}

/**
 * @brief Emergency stop — immediately coasts motor and stops monitoring.
 */
void drv8263_emergency_stop(drv8263_t *dev)
{
    if (!dev)
    {
        return;
    }

    if (dev->monitoring_enabled)
    {
        drv8263_stop_current_monitoring(dev);
    }

    dev->current_state = DRV8263_MOTOR_STOP;
    dev->current_speed = 0;
    drv8263_apply_motor_control(dev);

    #ifdef ACTUATOR_DEBUG
    printf("DRV8263: EMERGENCY STOP activated\n");
    #endif
}
