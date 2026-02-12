/**
 * @file main.c
 * @brief DRV8163 Test Program
 *
 * Tests H-bridge motor control with:
 * - Timed motor operation (forward/reverse)
 * - Overcurrent detection and protection
 * - Undercurrent detection (motor stall/disconnect)
 * - Startup blanking period to ignore inrush current
 * - Automatic emergency stop on fault conditions
 */

#include "drv8163.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Pin Definitions - Adjust these to match your hardware
#define MOTOR_CTRL_A_PIN 20  // GPIO20 - PWM channel for H-bridge A
#define MOTOR_CTRL_B_PIN 21  // GPIO21 - PWM channel for H-bridge B
#define MOTOR_SENSE_PIN 27   // GPIO27 - Analog current sense (ADC1)
#define MOTOR_SENSE_ADC_CH 1 // ADC channel 1

// Current Thresholds (12-bit ADC: 0-4095)
// These values should be calibrated for your specific motor and load
#define CURRENT_LOW_THRESHOLD 20   // Below this = undercurrent (motor stall/disconnect)
#define CURRENT_HIGH_THRESHOLD 220 // Above this = overcurrent (overload)

// Motor Control Parameters
#define MOTOR_PWM_FREQUENCY 20000 // 20 kHz PWM frequency
#define MOTOR_SPEED_LOW 1024      // 25% duty cycle
#define MOTOR_SPEED_MEDIUM 2048   // 50% duty cycle
#define MOTOR_SPEED_HIGH 3072     // 75% duty cycle
#define CURRENT_CHECK_INTERVAL 50 // Check current every 50ms
#define STARTUP_BLANKING_MS 500   // Ignore current for 500ms after motor start

// Test Parameters
#define TEST_RUN_DURATION_MS 5000   // Run each direction for 5 seconds
#define TEST_PAUSE_DURATION_MS 1000 // Pause between tests

// Global variables
static drv8163_t motor_driver;
static volatile bool emergency_stop_triggered = false;
static volatile bool endstop_reached = false;
static volatile uint32_t fault_count = 0;

/**
 * @brief Current monitoring callback
 * Called when current goes out of acceptable range
 */
void current_event_callback(drv8163_current_status_t status, uint16_t current_adc)
{
    fault_count++;

    switch (status)
    {
    case DRV8163_CURRENT_OVERCURRENT:
        printf("*** OVERCURRENT DETECTED! ADC=%u (threshold=%u) ***\n",
               current_adc, CURRENT_HIGH_THRESHOLD);
        emergency_stop_triggered = true;
        drv8163_emergency_stop(&motor_driver);
        break;

    case DRV8163_CURRENT_UNDERCURRENT:
        printf("*** UNDERCURRENT DETECTED! ADC=%u (threshold=%u) ***\n",
               current_adc, CURRENT_LOW_THRESHOLD);
        printf("*** End of travel reached ***\n");
        endstop_reached = true;
        break;

    case DRV8163_CURRENT_OK:
        printf("--- Current returned to normal range: ADC=%u ---\n", current_adc);
        break;
    }
}

/**
 * @brief Print current status
 */
void print_current_status(void)
{
    uint16_t current = drv8163_read_current(&motor_driver);
    drv8163_current_status_t status = drv8163_get_current_status(&motor_driver);

    const char *status_str[] = {"OK", "OVERCURRENT", "UNDERCURRENT"};
    printf("Current: ADC=%u, Status=%s\n", current, status_str[status]);
}

/**
 * @brief Run motor for specified duration with current monitoring
 */
bool run_motor_test(drv8163_motor_state_t direction, uint16_t speed, uint32_t duration_ms)
{
    const char *dir_str[] = {"STOP", "FORWARD", "REVERSE", "BRAKE"};

    printf("\n=== Starting Motor Test: %s, Speed=%u, Duration=%u ms ===\n",
           dir_str[direction], speed, duration_ms);

    // Reset emergency stop flag
    emergency_stop_triggered = false;
    endstop_reached = false;

    // Set motor control
    drv8163_set_motor_control(&motor_driver, direction, speed);

    // Start current monitoring
    if (!drv8163_start_current_monitoring(&motor_driver))
    {
        printf("ERROR: Failed to start current monitoring\n");
        return false;
    }

    // Run for specified duration, checking for faults
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = start_time;

    while ((to_ms_since_boot(get_absolute_time()) - start_time) < duration_ms)
    {
        // Check for emergency stop
        if (emergency_stop_triggered)
        {
            printf("\n!!! Test ABORTED due to emergency stop !!!\n");
            drv8163_stop_current_monitoring(&motor_driver);
            return false;
        }
        if (endstop_reached)
        {
            printf("\n!!! Test skipped due to end of travel !!!\n");
            break;
        }

        // Print status every 500ms
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - last_print) >= 500)
        {
            print_current_status();
            last_print = now;
        }

        sleep_ms(10);
    }

    // Stop monitoring and motor
    drv8163_stop_current_monitoring(&motor_driver);
    drv8163_set_motor_control(&motor_driver, DRV8163_MOTOR_STOP, 0);

    printf("=== Test Complete: %s ===\n\n", emergency_stop_triggered ? "FAILED" : "PASSED");
    return !emergency_stop_triggered;
}

/**
 * @brief Main program
 */
int main(void)
{
    // Initialize stdio
    stdio_init_all();
    sleep_ms(5000); // Wait for USB serial connection

    printf("\n");
    printf("========================================\n");
    printf("  DRV8163 Motor Driver Test Program\n");
    printf("========================================\n");
    printf("Hardware Configuration:\n");
    printf("  Control A Pin: GPIO%d\n", MOTOR_CTRL_A_PIN);
    printf("  Control B Pin: GPIO%d\n", MOTOR_CTRL_B_PIN);
    printf("  Sense Pin:     GPIO%d (ADC%d)\n", MOTOR_SENSE_PIN, MOTOR_SENSE_ADC_CH);
    printf("  PWM Frequency: %u Hz\n", MOTOR_PWM_FREQUENCY);
    printf("\nCurrent Thresholds:\n");
    printf("  Low:  %u ADC counts\n", CURRENT_LOW_THRESHOLD);
    printf("  High: %u ADC counts\n", CURRENT_HIGH_THRESHOLD);
    printf("  Startup Blanking: %u ms\n", STARTUP_BLANKING_MS);
    printf("========================================\n\n");

    // Configure driver
    drv8163_config_t config = {
        .user_ctx = NULL,
        .ctrl_a_pin = MOTOR_CTRL_A_PIN,
        .ctrl_b_pin = MOTOR_CTRL_B_PIN,
        .sense_pin = MOTOR_SENSE_PIN,
        .sense_adc_channel = MOTOR_SENSE_ADC_CH,
        .current_high_threshold = CURRENT_HIGH_THRESHOLD,
        .current_low_threshold = CURRENT_LOW_THRESHOLD,
        .pwm_frequency_hz = MOTOR_PWM_FREQUENCY,
        .current_check_interval_ms = CURRENT_CHECK_INTERVAL,
        .startup_blanking_ms = STARTUP_BLANKING_MS,
        .current_cb = current_event_callback,
        .delay_ms = sleep_ms};

    // Initialize driver
    if (!drv8163_init(&motor_driver, &config))
    {
        printf("ERROR: Failed to initialize DRV8163 driver\n");
        return -1;
    }

    printf("Driver initialized successfully!\n\n");
    sleep_ms(1000);

    // Test sequence
    uint32_t test_number = 1;
    bool all_tests_passed = true;

    while (true)
    {
        printf("\n");
        printf("========================================\n");
        printf("  Test Cycle %u\n", test_number);
        printf("========================================\n");

        // Test 1: Forward at low speed
        printf("\n--- Test 1: Forward at LOW speed ---\n");
        if (!run_motor_test(DRV8163_MOTOR_FORWARD, MOTOR_SPEED_LOW, TEST_RUN_DURATION_MS))
        {
            all_tests_passed = false;
            break;
        }
        sleep_ms(TEST_PAUSE_DURATION_MS);

        // Test 2: Forward at medium speed
        printf("\n--- Test 2: Forward at MEDIUM speed ---\n");
        if (!run_motor_test(DRV8163_MOTOR_FORWARD, MOTOR_SPEED_MEDIUM, TEST_RUN_DURATION_MS))
        {
            all_tests_passed = false;
            break;
        }
        sleep_ms(TEST_PAUSE_DURATION_MS);

        // Test 3: Forward at high speed
        printf("\n--- Test 3: Forward at HIGH speed ---\n");
        if (!run_motor_test(DRV8163_MOTOR_FORWARD, MOTOR_SPEED_HIGH, TEST_RUN_DURATION_MS))
        {
            all_tests_passed = false;
            break;
        }
        sleep_ms(TEST_PAUSE_DURATION_MS);

        // Test 4: Reverse at medium speed
        printf("\n--- Test 4: Reverse at HIGH speed ---\n");
        if (!run_motor_test(DRV8163_MOTOR_REVERSE, MOTOR_SPEED_HIGH, TEST_RUN_DURATION_MS))
        {
            all_tests_passed = false;
            break;
        }
        sleep_ms(TEST_PAUSE_DURATION_MS);

        // Test 5: Brake test
        printf("\n--- Test 5: Active BRAKE ---\n");
        drv8163_set_motor_control(&motor_driver, DRV8163_MOTOR_BRAKE, 0);
        printf("Brake applied for 2 seconds\n");
        sleep_ms(2000);
        drv8163_set_motor_control(&motor_driver, DRV8163_MOTOR_STOP, 0);

        printf("\n");
        printf("========================================\n");
        printf("  Test Cycle %u Complete\n", test_number);
        printf("  Fault Count: %u\n", fault_count);
        printf("========================================\n");

        test_number++;
        sleep_ms(3000); // Pause between cycles
    }

    // If we broke out due to fault, wait here
    if (!all_tests_passed)
    {
        printf("\n");
        printf("========================================\n");
        printf("  TEST SEQUENCE HALTED DUE TO FAULT\n");
        printf("  Total Faults: %u\n", fault_count);
        printf("========================================\n");
        printf("\nMotor is stopped. System halted.\n");

        // Ensure motor is stopped
        drv8163_set_motor_control(&motor_driver, DRV8163_MOTOR_STOP, 0);

        // Blink LED to indicate fault state
        const uint LED_PIN = PICO_DEFAULT_LED_PIN;
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);

        while (true)
        {
            gpio_put(LED_PIN, 1);
            sleep_ms(200);
            gpio_put(LED_PIN, 0);
            sleep_ms(200);
        }
    }

    return 0;
}
