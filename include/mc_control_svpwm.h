#ifndef MC_CONTROL_SVPWM_H
#define MC_CONTROL_SVPWM_H
/** @file mc_control_svpwm.h @brief Space-vector PWM modulation types and API */

#include "mc_port_pwm.h"
#include "mc_transform.h"

/** @brief SVPWM modulator configuration */
typedef struct
{
    mc_f32_t duty_min;         /**< Minimum duty-cycle clamp */
    mc_f32_t duty_max;         /**< Maximum duty-cycle clamp */
    mc_f32_t modulation_limit; /**< Modulation index limit (typically sqrt(3)/2) */
} mc_svpwm_cfg_t;

/**
 * @brief Run SVPWM modulation to generate 3-phase PWM commands
 * @param voltage_ab  Alpha-beta voltage vector
 * @param cfg         SVPWM configuration
 * @param pwm_cmd     Output PWM command (3-phase duty cycles)
 */
void mc_svpwm_run(const mc_alphabeta_t *voltage_ab, const mc_svpwm_cfg_t *cfg, mc_pwm_cmd_t *pwm_cmd);

/**
 * @brief Run SVPWM from a Q31 alpha-beta vector
 * @param voltage_ab_q31 Alpha-beta voltage vector in Q31
 * @param cfg SVPWM configuration
 * @param pwm_cmd Output PWM command
 */
void mc_svpwm_q31_run(const mc_alphabeta_q31_t *voltage_ab_q31, const mc_svpwm_cfg_t *cfg, mc_pwm_cmd_t *pwm_cmd);

#endif /* MC_CONTROL_SVPWM_H */
