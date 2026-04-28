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
    output = mc_q31_add_sat(proportional, pi->integral);
    output = mc_pi_q31_clamp(output, pi->cfg.output_min, pi->cfg.output_max);

    pi->output = output;
    return output;
}
