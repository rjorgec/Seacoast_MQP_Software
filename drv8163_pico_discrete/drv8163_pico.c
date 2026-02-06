s #include "drv8163_pico.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

    // SPI Handle for DRV8163
    static drv8163_handle_t global_drv8163_handle; // Global handle for ISR access
static spi_inst_t *spi_drv8163 = DRV8163_SPI_PORT;

// PWM slice and channel for motor speed control
static uint slice_num;
static uint channel_in1; // Or channel_in2, depending on PWM scheme

// Constants for current sensing
#define DRV8163_R_IPROPI_OHMS 1000.0f // Example: 1kOhm external resistor on IPROPI
#define DRV8163_A_IPROPI_GAIN 202e-6f // Current mirror gain: 202 uA/A
#define PICO_ADC_VOLTAGE_REF 3.3f     // Pico ADC reference voltage
#define PICO_ADC_MAX_VALUE 4095.0f    // 12-bit ADC max value

// --- Pico-specific SPI functions ---
void drv8163_pico_spi_init(uint32_t baud_rate)
{
    // Initialize SPI pins
    gpio_set_function(DRV8163_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(DRV8163_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(DRV8163_MOSI_PIN, GPIO_FUNC_SPI);

    // Initialize CS pin as GPIO and set high (inactive)
    gpio_init(DRV8163_CS_PIN);
    gpio_set_dir(DRV8163_CS_PIN, GPIO_OUT);
    gpio_put(DRV8163_CS_PIN, 1);

    // Initialize SPI peripheral
    spi_init(spi_drv8163, baud_rate);
    spi_set_format(spi_drv8163, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST); // DRV8163 uses Mode 1 (CPOL=0, CPHA=1), 16-bit frames
}

int drv8163_pico_spi_transfer(uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
    // The drv8163 driver expects 16-bit frames, but spi_write_read_blocking operates on bytes.
    // The len parameter here should be 2 bytes for a 16-bit frame.
    if (len != 2)
        return -1; // Expecting 2 bytes for a 16-bit frame

    int num_bytes = spi_write_read_blocking(spi_drv8163, tx_data, rx_data, len);
    return (num_bytes == len) ? 0 : -1;
}

void drv8163_pico_cs_assert(void)
{
    gpio_put(DRV8163_CS_PIN, 0);
}

void drv8163_pico_cs_deassert(void)
{
    gpio_put(DRV8163_CS_PIN, 1);
}

// --- Pico-specific GPIO and Motor Control functions ---
void drv8163_pico_gpio_init(void)
{
    // Initialize IN1 and IN2 as GPIO outputs
    gpio_init(DRV8163_IN1_PIN);
    gpio_set_dir(DRV8163_IN1_PIN, GPIO_OUT);
    gpio_init(DRV8163_IN2_PIN);
    gpio_set_dir(DRV8163_IN2_PIN, GPIO_OUT);

    // Initialize nSLEEP as GPIO output and set low to keep driver disabled initially
    gpio_init(DRV8163_NSLEEP_PIN);
    gpio_set_dir(DRV8163_NSLEEP_PIN, GPIO_OUT);
    gpio_put(DRV8163_NSLEEP_PIN, 0); // Start disabled

    // Initialize nFAULT as GPIO input
    gpio_init(DRV8163_NFAULT_PIN);
    gpio_set_dir(DRV8163_NFAULT_PIN, GPIO_IN);
    gpio_pull_up(DRV8163_NFAULT_PIN); // nFAULT is open-drain, needs pull-up
}

void drv8163_enable_driver(void)
{
    gpio_put(DRV8163_NSLEEP_PIN, 1);
}

void drv8163_disable_driver(void)
{
    gpio_put(DRV8163_NSLEEP_PIN, 0);
}

void drv8163_set_motor_state(motor_state_t state)
{
    switch (state)
    {
    case MOTOR_STOP:
        gpio_put(DRV8163_IN1_PIN, 0);
        gpio_put(DRV8163_IN2_PIN, 0); // Coast
        break;
    case MOTOR_FORWARD:
        gpio_put(DRV8163_IN1_PIN, 1);
        gpio_put(DRV8163_IN2_PIN, 0);
        break;
    case MOTOR_REVERSE:
        gpio_put(DRV8163_IN1_PIN, 0);
        gpio_put(DRV8163_IN2_PIN, 1);
        break;
    case MOTOR_BRAKE:
        gpio_put(DRV8163_IN1_PIN, 1);
        gpio_put(DRV8163_IN2_PIN, 1); // Brake (low-side short)
        break;
    }
}

// --- Pico-specific PWM functions ---
void drv8163_pico_pwm_init(uint32_t pwm_freq)
{
    // Set IN1 (or IN2) to PWM function
    gpio_set_function(DRV8163_IN1_PIN, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the GPIO pin
    slice_num = pwm_gpio_to_slice_num(DRV8163_IN1_PIN);
    channel_in1 = pwm_gpio_to_channel(DRV8163_IN1_PIN);

    // Set PWM clock divider
    pwm_set_wrap(slice_num, 65535);                                       // 16-bit resolution
    float divider = (float)clock_get_hz(clk_sys) / (pwm_freq * 65536.0f); // Calculate divider for 16-bit resolution
    pwm_set_clkdiv(slice_num, divider);
    pwm_set_enabled(slice_num, true);

    // Initially set duty cycle to 0
    pwm_set_chan_level(slice_num, channel_in1, 0);
}

void drv8163_set_pwm_duty_cycle(uint16_t duty_cycle)
{
    pwm_set_chan_level(slice_num, channel_in1, duty_cycle);
}

// --- ITRIP PWM (simulated DAC) ---
void drv8163_pico_itrip_pwm_init(uint32_t dac_freq)
{
    gpio_set_function(DRV8163_ITRIP_PIN, GPIO_FUNC_PWM);
    uint itrap_slice = pwm_gpio_to_slice_num(DRV8163_ITRIP_PIN);
    uint itrap_chan = pwm_gpio_to_channel(DRV8163_ITRIP_PIN);
    pwm_set_wrap(itrap_slice, 255); // 8-bit resolution
    float divider = (float)clock_get_hz(clk_sys) / (dac_freq * 256.0f);
    pwm_set_clkdiv(itrap_slice, divider);
    pwm_set_enabled(itrap_slice, true);
    pwm_set_chan_level(itrap_slice, itrap_chan, 0);
}

void drv8163_set_itrip_pwm(uint8_t itrip_value)
{
    uint itrap_slice = pwm_gpio_to_slice_num(DRV8163_ITRIP_PIN);
    uint itrap_chan = pwm_gpio_to_channel(DRV8163_ITRIP_PIN);
    pwm_set_chan_level(itrap_slice, itrap_chan, itrip_value);
}

// --- Pico-specific ADC functions ---
void drv8163_pico_adc_init(void)
{
    adc_init();
    adc_gpio_init(DRV8163_IPROPI_ADC_PIN); // Set up ADC input pin
    adc_select_input(DRV8163_IPROPI_ADC_CHANNEL);
}

uint16_t drv8163_read_iprobi_raw_adc(void)
{
    return adc_read();
}

// --- nFAULT Interrupt Handler ---
static void drv8163_nfault_isr(uint gpio, uint32_t events)
{
    if (gpio == DRV8163_NFAULT_PIN && (events & GPIO_IRQ_EDGE_FALL))
    {
        uint8_t faults = 0;
        drv8163_status_t status = drv8163_read_faults(&global_drv8163_handle, &faults);
        if (status == DRV8163_OK)
        {
            printf("DRV8163 FAULT detected! Register: 0x%02X\n", faults);
            // Implement emergency stop here, e.g.:
            drv8163_set_motor_state(MOTOR_STOP);
            drv8163_disable_driver();
            // Clearing the faults here immediately might lead to re-triggering if the fault is persistent.
            // It might be better to clear it in the main loop after some delay or user intervention.
            // drv8163_clear_faults(&global_drv8163_handle);
        }
        else if (status == DRV8163_ERR_FAULT)
        {
            printf("DRV8163 FAULT detected! Register: 0x%02X\n", faults);
            // Faults were read, and a fault is present. Perform emergency stop.
            drv8163_set_motor_state(MOTOR_STOP);
            drv8163_disable_driver();
        }
        else
        {
            printf("Failed to read DRV8163 faults during ISR: %d\n", status);
        }
    }
}

void drv8163_pico_nfaul_interrupt_init()
{
    gpio_set_irq_enabled_with_callback(DRV8163_NFAULT_PIN, GPIO_IRQ_EDGE_FALL, true, drv8163_nfault_isr);
}
