#pragma once
#include <stdbool.h>
#include <stdint.h>

bool stepper8434s_init(void);                   // init SPI + drv8434s context + IRQ
bool stepper8434s_set_enabled(bool en);          // enable/disable outputs
bool stepper8434s_start_job(bool dir, uint32_t steps, uint32_t step_delay_us);
void stepper8434s_stop(void);
void stepper8434s_tick(void);                   // call frequently (non-blocking)
bool stepper8434s_faulted(void);                // latched nFAULT
