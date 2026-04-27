#ifndef MC_PORT_PWM_H
#define MC_PORT_PWM_H

/** @file mc_port_pwm.h @brief PWM command and phase-mode types. */

#include "mc_types.h"

/** @brief PWM phase output mode. */
typedef enum
{
    MC_PWM_PHASE_OFF = 0, /**< Phase output off (high-impedance). */
    MC_PWM_PHASE_HIGH, /**< Phase output forced high. */
    MC_PWM_PHASE_LOW, /**< Phase output forced low. */
    MC_PWM_PHASE_PWM /**< Phase output switching at PWM frequency. */
} mc_pwm_phase_mode_t;

/** @brief PWM command structure for one PWM period. */
typedef struct
{
    mc_f32_t duty_a; /**< Duty cycle for phase A [0..1]. */
    mc_f32_t duty_b; /**< Duty cycle for phase B [0..1]. */
    mc_f32_t duty_c; /**< Duty cycle for phase C [0..1]. */
    mc_f32_t common_mode_shift; /**< Common-mode injection value. */
    mc_pwm_phase_mode_t phase_mode_a; /**< Output mode for phase A. */
    mc_pwm_phase_mode_t phase_mode_b; /**< Output mode for phase B. */
    mc_pwm_phase_mode_t phase_mode_c; /**< Output mode for phase C. */
    uint8_t sector; /**< Space-vector sector index. */
    uint8_t valid; /**< Non-zero if the command is valid for application. */
} mc_pwm_cmd_t;

#endif /* MC_PORT_PWM_H */
