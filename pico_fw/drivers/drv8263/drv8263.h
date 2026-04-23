/**
 * @file drv8263.h
 * @brief TI DRV8263 H-Bridge Motor Driver Interface
 *
 * Hardware variant: 2 PWM control pins + 1 analog current sense pin.
 * Supports standard H-bridge mode (flap actuators) and independent
 * half-bridge mode (hot wire IN1 + vacuum pump 2 IN2 — simultaneous
 * operation permitted; current regulation via external Rsense).
 *
 * Features:
 * - H-bridge control with forward/reverse/brake/coast modes
 * - PWM speed control (12-bit, 0-4095)
 * - Analog current sensing via ADC (optional — unused in independent mode)
 * - Repeating-timer-based overcurrent/undercurrent detection
 */

#ifndef DRV8263_H
#define DRV8263_H

#include <stdint.h>
#include <stdbool.h>

/* Motor Control States */
typedef enum
{
    DRV8263_MOTOR_STOP = 0, /* Coast (both outputs low) */
    DRV8263_MOTOR_FORWARD,  /* Forward direction (IN1 = PWM, IN2 = 0) */
    DRV8263_MOTOR_REVERSE,  /* Reverse direction (IN1 = 0, IN2 = PWM) */
    DRV8263_MOTOR_BRAKE     /* Active brake (both outputs high) */
} drv8263_motor_state_t;

/* Current Monitoring Status */
typedef enum
{
    DRV8263_CURRENT_OK = 0,
    DRV8263_CURRENT_OVERCURRENT,
    DRV8263_CURRENT_UNDERCURRENT
} drv8263_current_status_t;

/* Rolling Average Structure */
#define ADC_AVERAGES 10
typedef struct
{
    uint16_t buffer[ADC_AVERAGES]; /* circular buffer of ADC readings */
    uint8_t head;                  /* write index */
    uint16_t sum;                  /* running sum */
    uint16_t average;              /* current rolling average */
} drv8263_adc_buffer;

/* Callback function types */
typedef void (*drv8263_delay_ms_t)(uint32_t ms);
typedef void (*drv8263_delay_us_t)(uint32_t us);
typedef void (*drv8263_current_callback_t)(drv8263_current_status_t status, uint16_t current_adc);

/* Driver Configuration */
typedef struct
{
    void *user_ctx;                        /* User context passed to callbacks */
    uint8_t ctrl_a_pin;                    /* H-bridge control A pin (GPIO number, IN1) */
    uint8_t ctrl_b_pin;                    /* H-bridge control B pin (GPIO number, IN2) */
    uint8_t sense_pin;                     /* Analog current sense pin (GPIO number) */
    uint8_t sense_adc_channel;             /* ADC channel for sense pin */
    uint16_t current_high_threshold;       /* ADC value for overcurrent detection */
    uint16_t current_low_threshold;        /* ADC value for undercurrent detection */
    uint32_t pwm_frequency_hz;             /* PWM frequency for motor control */
    uint32_t current_check_interval_ms;    /* How often to check current (ms) */
    uint32_t startup_blanking_ms;          /* Ignore overcurrent for this duration after motor start */
    drv8263_current_callback_t current_cb; /* Current event callback (optional, may be NULL) */
    drv8263_delay_ms_t delay_ms;           /* Millisecond delay function (optional, may be NULL) */
    uint16_t initial_duty_a;               /* PWM level applied to IN1 before slice enable (0-4095) */
    uint16_t initial_duty_b;               /* PWM level applied to IN2 before slice enable (0-4095) */
} drv8263_config_t;

/* Driver Instance */
typedef struct
{
    drv8263_config_t config;                 /* Configuration (writable — thresholds may change at runtime) */
    drv8263_motor_state_t current_state;     /* Current motor state */
    uint16_t current_speed;                  /* Current speed (0-4095 for 12-bit PWM) */
    bool monitoring_enabled;                 /* Current monitoring active */
    drv8263_current_status_t current_status; /* Last current status */
    drv8263_adc_buffer last_current_adc;     /* Rolling ADC average */
    void *platform_data;                     /* Platform-specific data (opaque) */
} drv8263_t;

/* ── Function Prototypes ──────────────────────────────────────────────────── */

/**
 * @brief Initialize the DRV8263 driver.
 * @param dev Pointer to driver instance (zeroed by this call)
 * @param cfg Pointer to configuration structure (copied internally)
 * @return true if initialization successful, false otherwise
 */
bool drv8263_init(drv8263_t *dev, const drv8263_config_t *cfg);

/**
 * @brief Set motor state (stop, forward, reverse, brake).
 * @param dev   Pointer to driver instance
 * @param state Desired motor state
 * @return true if successful, false otherwise
 */
bool drv8263_set_motor_state(drv8263_t *dev, drv8263_motor_state_t state);

/**
 * @brief Set motor speed via PWM duty cycle.
 * @param dev   Pointer to driver instance
 * @param speed Speed value (0-4095 for 12-bit PWM resolution)
 * @return true if successful, false otherwise
 */
bool drv8263_set_motor_speed(drv8263_t *dev, uint16_t speed);

/**
 * @brief Set motor state and speed in one call.
 * @param dev   Pointer to driver instance
 * @param state Motor state
 * @param speed Speed value (0-4095 for 12-bit PWM)
 * @return true if successful, false otherwise
 */
bool drv8263_set_motor_control(drv8263_t *dev, drv8263_motor_state_t state, uint16_t speed);

/**
 * @brief Read current sense ADC value.
 * @param dev Pointer to driver instance
 * @return 12-bit ADC value representing motor current
 *         (ADC sense unused in independent H-bridge mode — current set by external Rsense)
 */
uint16_t drv8263_read_current(drv8263_t *dev);

/**
 * @brief Start current monitoring with repeating-timer-based checking.
 * @param dev Pointer to driver instance
 * @return true if monitoring started, false otherwise
 */
bool drv8263_start_current_monitoring(drv8263_t *dev);

/**
 * @brief Stop current monitoring.
 * @param dev Pointer to driver instance
 */
void drv8263_stop_current_monitoring(drv8263_t *dev);

/**
 * @brief Get current monitoring status.
 * @param dev Pointer to driver instance
 * @return Current status (OK, overcurrent, undercurrent)
 */
drv8263_current_status_t drv8263_get_current_status(drv8263_t *dev);

/**
 * @brief Update current thresholds dynamically (safe to call while monitoring).
 * @param dev            Pointer to driver instance
 * @param low_threshold  New low threshold ADC value
 * @param high_threshold New high threshold ADC value
 */
void drv8263_set_current_thresholds(drv8263_t *dev, uint16_t low_threshold, uint16_t high_threshold);

/**
 * @brief Emergency stop — immediately coasts motor and stops monitoring.
 * @param dev Pointer to driver instance
 */
void drv8263_emergency_stop(drv8263_t *dev);

/* ── Independent half-bridge outputs (DRV8263 independent mode) ───────────── */

/**
 * @brief Drive IN1 (ctrl_a_pin) independently without affecting IN2.
 *
 * Used when the DRV8263 is configured in independent H-bridge mode:
 *   - IN1 drives the hot wire (current regulated by external Rsense)
 *   - IN2 drives vacuum pump 2
 * Both outputs can be active simultaneously.
 *
 * @param dev  Pointer to driver instance
 * @param duty 12-bit PWM duty cycle (0 = off, 4095 = full on)
 * @return true if successful, false otherwise
 */
bool drv8263_set_in1(drv8263_t *dev, uint16_t duty);

/**
 * @brief Drive IN2 (ctrl_b_pin) independently without affecting IN1.
 *
 * @param dev  Pointer to driver instance
 * @param duty 12-bit PWM duty cycle (0 = off, 4095 = full on)
 * @return true if successful, false otherwise
 */
bool drv8263_set_in2(drv8263_t *dev, uint16_t duty);

#endif /* DRV8263_H */
