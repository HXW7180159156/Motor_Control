/**
 * @file mc_control_pi.c
 * @brief PI (Proportional-Integral) controller implementation
 */

#include "mc_constants.h"
#include "mc_control_pi.h"
#include "mc_math.h"

/**
 * @brief Clamp a Q31 value between minimum and maximum bounds
 * @param value Input Q31 value to clamp
 * @param min_value Lower bound
 * @param max_value Upper bound
 * @return Clamped Q31 value within [min_value, max_value]
 */
static mc_q31_t mc_pi_q31_clamp(mc_q31_t value, mc_q31_t min_value, mc_q31_t max_value)
{
    return mc_math_clamp_q31(value, min_value, max_value);
}

/**
 * @brief Initialise PI controller with configuration
 * @param pi Pointer to PI controller structure
 * @param cfg Pointer to PI configuration (gains, limits)
 */
void mc_pi_init(mc_pi_t *pi, const mc_pi_cfg_t *cfg)
{
    if ((pi == NULL) || (cfg == NULL))
    {
        return;
    }

    pi->cfg = *cfg;
    pi->integral = 0.0F;
    pi->output = 0.0F;
}

/**
 * @brief Reset PI controller state (integral and output to zero)
 * @param pi Pointer to PI controller structure
 */
void mc_pi_reset(mc_pi_t *pi)
{
    if (pi == NULL)
    {
        return;
    }

    pi->integral = 0.0F;
    pi->output = 0.0F;
}

/**
 * @brief Execute one PI controller step
 * @param pi Pointer to PI controller structure
 * @param error Error signal (setpoint - feedback)
 * @param dt_s Time step in seconds
 * @return Controller output after integral clamping and output limiting
 */
mc_f32_t mc_pi_run(mc_pi_t *pi, mc_f32_t error, mc_f32_t dt_s)
{
    mc_f32_t proportional;
    mc_f32_t output;

    if (pi == NULL)
    {
        return 0.0F;
    }

    pi->integral += error * pi->cfg.ki * dt_s;
    pi->integral = mc_math_clamp_f32(pi->integral, pi->cfg.integral_min, pi->cfg.integral_max);

    proportional = error * pi->cfg.kp;
    output = proportional + pi->integral;
    output = mc_math_clamp_f32(output, pi->cfg.output_min, pi->cfg.output_max);

    pi->output = output;
    return output;
}

/**
 * @brief Initialise Q31 PI controller with configuration
 * @param pi Pointer to Q31 PI controller structure
 * @param cfg Pointer to Q31 PI configuration (gains, limits)
 */
void mc_pi_q31_init(mc_pi_q31_t *pi, const mc_pi_q31_cfg_t *cfg)
{
    if ((pi == NULL) || (cfg == NULL))
    {
        return;
    }

    pi->cfg = *cfg;
    pi->integral = 0;
    pi->output = 0;
}

/**
 * @brief Reset Q31 PI controller state (integral and output to zero)
 * @param pi Pointer to Q31 PI controller structure
 */
void mc_pi_q31_reset(mc_pi_q31_t *pi)
{
    if (pi == NULL)
    {
        return;
    }

    pi->integral = 0;
    pi->output = 0;
}

/**
 * @brief Execute one Q31 PI controller step
 * @param pi Pointer to Q31 PI controller structure
 * @param error Error signal in Q31 (setpoint - feedback)
 * @param dt_q31 Q31 sample interval
 * @return Controller output after integral clamping and output limiting
 */
mc_q31_t mc_pi_q31_run(mc_pi_q31_t *pi, mc_q31_t error, mc_q31_t dt_q31)
{
    mc_q31_t proportional;
    mc_q31_t output;
    mc_q31_t integral_delta;

    if (pi == NULL)
    {
        return 0;
    }

    integral_delta = mc_q31_mul(mc_q31_mul(error, pi->cfg.ki), dt_q31);
    pi->integral = mc_q31_add_sat(pi->integral, integral_delta);
    pi->integral = mc_pi_q31_clamp(pi->integral, pi->cfg.integral_min, pi->cfg.integral_max);

    proportional = mc_q31_mul(error, pi->cfg.kp);
    pi->output = output;
    return output;
}

/* ================================================================
 *  Auto-Tune Utilities
 * ================================================================ */

mc_status_t mc_auto_tune_current_pi(mc_f32_t rs_ohm,
                                     mc_f32_t ld_h,
                                     mc_f32_t lq_h,
                                     uint32_t pwm_freq_hz,
                                     mc_f32_t bandwidth_divider,
                                     mc_f32_t voltage_limit,
                                     mc_f32_t current_limit,
                                     mc_pi_cfg_t *id_pi_cfg,
                                     mc_pi_cfg_t *iq_pi_cfg)
{
    mc_f32_t omega_c;
    mc_f32_t kp_d;
    mc_f32_t ki_d;
    mc_f32_t kp_q;
    mc_f32_t ki_q;
    mc_f32_t integral_limit;

    if ((id_pi_cfg == NULL) || (iq_pi_cfg == NULL) ||
        (rs_ohm <= 0.0F) || (ld_h <= 0.0F) || (lq_h <= 0.0F) ||
        (pwm_freq_hz == 0U) || (bandwidth_divider < 1.0F) ||
        (voltage_limit <= 0.0F))
    {
        return MC_STATUS_INVALID_ARG;
    }

    omega_c = MC_TWO_PI * (mc_f32_t)pwm_freq_hz / bandwidth_divider;

    kp_d = omega_c * ld_h;
    ki_d = omega_c * rs_ohm;
    kp_q = omega_c * lq_h;
    ki_q = omega_c * rs_ohm;

    integral_limit = voltage_limit * MC_1SHUNT_HALF / rs_ohm;
    if (integral_limit > current_limit)
    {
        integral_limit = current_limit;
    }
    if (integral_limit <= 0.0F)
    {
        integral_limit = 1.0F;
    }

    *id_pi_cfg = (mc_pi_cfg_t){kp_d, ki_d, -integral_limit, integral_limit, -voltage_limit, voltage_limit};
    *iq_pi_cfg = (mc_pi_cfg_t){kp_q, ki_q, -integral_limit, integral_limit, -voltage_limit, voltage_limit};

    return MC_STATUS_OK;
}

mc_status_t mc_auto_tune_speed_pi(mc_f32_t ld_h,
                                   uint32_t pwm_freq_hz,
                                   mc_f32_t bandwidth_divider,
                                   mc_f32_t speed_ratio,
                                   mc_f32_t iq_limit,
                                   mc_pi_cfg_t *speed_pi_cfg)
{
    mc_f32_t omega_c;
    mc_f32_t omega_s;
    mc_f32_t kp;
    mc_f32_t ki;

    if ((speed_pi_cfg == NULL) || (ld_h <= 0.0F) ||
        (pwm_freq_hz == 0U) || (bandwidth_divider < 1.0F) ||
        (speed_ratio < 1.0F) || (iq_limit <= 0.0F))
    {
        return MC_STATUS_INVALID_ARG;
    }

    omega_c = MC_TWO_PI * (mc_f32_t)pwm_freq_hz / bandwidth_divider;
    omega_s = omega_c / speed_ratio;

    kp = omega_s * ld_h * MC_1SHUNT_HALF;
    ki = omega_s * kp * 0.25F;

    *speed_pi_cfg = (mc_pi_cfg_t){kp, ki, -iq_limit, iq_limit, -iq_limit, iq_limit};

    return MC_STATUS_OK;
}
