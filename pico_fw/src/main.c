#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "board_pins.h"

static volatile uint32_t s_vacuum_pulse_count = 0u;
static volatile uint32_t s_last_pulse_time_us = 0u;
static volatile uint32_t s_last_period_us = 0u;
static volatile uint64_t s_period_sum_us = 0u;
static volatile uint32_t s_period_samples = 0u;

#define VACUUM_RPM_TIMEOUT_MS 500u
#define VACUUM_RPM_DEBOUNCE_US 200u

static void vacuum_rpm_isr(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == (uint)VACUUM_RPM_SENSE_PIN)
    {
        uint32_t now_us = time_us_32();
        uint32_t dt_us = now_us - s_last_pulse_time_us;

        if ((s_last_pulse_time_us != 0u) && (dt_us < VACUUM_RPM_DEBOUNCE_US))
        {
            return;
        }

        if (s_last_pulse_time_us != 0u)
        {
            s_last_period_us = dt_us;
            s_period_sum_us += dt_us;
            ++s_period_samples;
        }

        s_last_pulse_time_us = now_us;
        ++s_vacuum_pulse_count;
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("Vacuum pump 1 sketch boot\n");
    printf("Trigger pin=%d, RPM pin=%d, sample=%d ms, pulses/rev=%d\n",
           VACUUM_TRIGGER_PIN,
           VACUUM_RPM_SENSE_PIN,
           VACUUM_RPM_SAMPLE_MS,
           VACUUM_PULSES_PER_REV);

    gpio_init((uint)VACUUM_TRIGGER_PIN);
    gpio_set_dir((uint)VACUUM_TRIGGER_PIN, GPIO_OUT);
    gpio_put((uint)VACUUM_TRIGGER_PIN, 0u);

    gpio_init((uint)VACUUM_RPM_SENSE_PIN);
    gpio_set_dir((uint)VACUUM_RPM_SENSE_PIN, GPIO_IN);
    gpio_pull_down((uint)VACUUM_RPM_SENSE_PIN);
    gpio_set_irq_enabled_with_callback((uint)VACUUM_RPM_SENSE_PIN,
                                       GPIO_IRQ_EDGE_RISE,
                                       true,
                                       vacuum_rpm_isr);

    sleep_ms(250);
    gpio_put((uint)VACUUM_TRIGGER_PIN, 1u);
    printf("Vacuum pump 1 ON\n");

    absolute_time_t next_print = make_timeout_time_ms(VACUUM_RPM_SAMPLE_MS);

    while (true)
    {
        sleep_until(next_print);
        next_print = delayed_by_ms(next_print, VACUUM_RPM_SAMPLE_MS);

        uint32_t irq_state = save_and_disable_interrupts();
        uint32_t pulses = s_vacuum_pulse_count;
        uint32_t last_pulse_time_us = s_last_pulse_time_us;
        uint32_t last_period_us = s_last_period_us;
        uint64_t period_sum_us = s_period_sum_us;
        uint32_t period_samples = s_period_samples;
        s_period_sum_us = 0u;
        s_period_samples = 0u;
        restore_interrupts(irq_state);

        uint32_t rpm = 0u;
        uint32_t age_ms = UINT32_MAX;

        if (last_pulse_time_us != 0u)
        {
            age_ms = to_ms_since_boot(get_absolute_time()) - (last_pulse_time_us / 1000u);
        }

        if ((last_pulse_time_us != 0u) && (age_ms < VACUUM_RPM_TIMEOUT_MS))
        {
            uint32_t avg_period_us = 0u;
            if (period_samples > 0u)
            {
                avg_period_us = (uint32_t)(period_sum_us / period_samples);
            }
            else if (last_period_us > 0u)
            {
                avg_period_us = last_period_us;
            }

            if (avg_period_us > 0u)
            {
                rpm = (uint32_t)(60000000ull /
                                 ((uint64_t)avg_period_us * (uint64_t)VACUUM_PULSES_PER_REV));
            }
        }

        s_vacuum_pulse_count = 0u;

        printf("vacuum1 pulses=%lu rpm=%lu period_us=%lu age_ms=%lu\n",
               (unsigned long)pulses,
               (unsigned long)rpm,
               (unsigned long)last_period_us,
               (unsigned long)age_ms);
    }
}
