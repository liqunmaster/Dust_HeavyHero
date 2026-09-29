#pragma once

#include <stddef.h>
#include <stdint.h>

#define BSP_PWM_MAX_PULSES 24U

#ifdef __cplusplus
extern "C" {
#endif

int bsp_pwm_init(uint32_t period_ns);
int bsp_pwm_write(const uint16_t *high_ns, size_t count);

#ifdef __cplusplus
}
#endif
