#ifndef MC_PORT_H
#define MC_PORT_H

/** @file mc_port.h @brief Port abstraction hooks for platform-specific driver integration. */

#include "mc_port_adc.h"
#include "mc_types.h"

/** @brief Platform callback hooks invoked by the motor control library. */
typedef struct
{
    void (*pwm_apply)(const void *cmd); /**< Apply PWM command (platform-specific). */
    void (*adc_trigger_apply)(const mc_adc_trigger_plan_t *plan); /**< Apply ADC trigger plan. */
    void (*fault_signal)(uint32_t fault_code); /**< Signal a fault condition. */
    uint32_t (*get_time_us)(void); /**< Get current time in microseconds. */
} mc_port_hooks_t;

#endif /* MC_PORT_H */
