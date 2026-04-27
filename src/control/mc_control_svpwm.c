/**
 * @file mc_control_svpwm.c
 * @brief Space Vector Pulse Width Modulation (SVPWM) implementation
 */

#include "mc_control_svpwm.h"

#include "mc_math.h"

#include <math.h>

/**
 * @brief Clamp a value between minimum and maximum bounds
 * @param value Input value to clamp
 * @param min_value Lower bound
 * @param max_value Upper bound
 * @return Clamped value within [min_value, max_value]
 */
static mc_f32_t mc_svpwm_clamp(mc_f32_t value, mc_f32_t min_value, mc_f32_t max_value)
{
    mc_f32_t result = value;

    if (result < min_value)
    {
        result = min_value;
    }
    else if (result > max_value)
    {
        result = max_value;
    }

    return result;
}

/**
 * @brief Detect the sector of the voltage vector in alpha-beta frame
 * @param voltage_ab Voltage vector in stationary reference frame
 * @return Sector number (1, 3, 4, or 6)
 */
static uint8_t mc_svpwm_detect_sector(const mc_alphabeta_t *voltage_ab)
{
    uint8_t sector = 0U;

    if (voltage_ab->beta >= 0.0F)
    {
        sector = (voltage_ab->alpha >= 0.0F) ? 1U : 3U;
    }
    else
    {
        sector = (voltage_ab->alpha >= 0.0F) ? 6U : 4U;
    }

    return sector;
}

/**
 * @brief Limit voltage vector magnitude to modulation limit
 * @param cfg SVPWM configuration (modulation_limit)
 * @param voltage_ab Voltage vector to limit (modified in-place)
 */
static void mc_svpwm_limit_vector(const mc_svpwm_cfg_t *cfg, mc_alphabeta_t *voltage_ab)
{
    mc_f32_t modulation_limit;
    mc_f32_t magnitude;

    if ((cfg == NULL) || (voltage_ab == NULL))
    {
        return;
    }

    modulation_limit = cfg->modulation_limit;
    if (modulation_limit <= 0.0F)
    {
        return;
    }

    magnitude = sqrtf((voltage_ab->alpha * voltage_ab->alpha) + (voltage_ab->beta * voltage_ab->beta));
    if (magnitude > modulation_limit)
    {
        mc_f32_t scale = modulation_limit / magnitude;
        voltage_ab->alpha *= scale;
        voltage_ab->beta *= scale;
    }
}

/**
 * @brief Execute SVPWM to generate PWM duty commands from voltage vector
 * @param voltage_ab Desired voltage vector in alpha-beta frame
 * @param cfg SVPWM configuration (modulation limit, duty limits)
 * @param pwm_cmd Output PWM command (duty cycles, sector, valid flag)
 */
void mc_svpwm_run(const mc_alphabeta_t *voltage_ab, const mc_svpwm_cfg_t *cfg, mc_pwm_cmd_t *pwm_cmd)
{
    mc_alphabeta_t limited_voltage_ab;
    mc_f32_t va;
    mc_f32_t vb;
    mc_f32_t vc;
    mc_f32_t vmax;
    mc_f32_t vmin;
    mc_f32_t v_offset;
    mc_f32_t common_mode_shift;

    if ((voltage_ab == NULL) || (cfg == NULL) || (pwm_cmd == NULL))
    {
        return;
    }

    limited_voltage_ab = *voltage_ab;
    mc_svpwm_limit_vector(cfg, &limited_voltage_ab);

    va = limited_voltage_ab.alpha;
    vb = (-0.5F * limited_voltage_ab.alpha) + (0.8660254038F * limited_voltage_ab.beta);
    vc = (-0.5F * limited_voltage_ab.alpha) - (0.8660254038F * limited_voltage_ab.beta);

    vmax = va;
    if (vb > vmax)
    {
        vmax = vb;
    }
    if (vc > vmax)
    {
        vmax = vc;
    }

    vmin = va;
    if (vb < vmin)
    {
        vmin = vb;
    }
    if (vc < vmin)
    {
        vmin = vc;
    }

    v_offset = 0.5F * (vmax + vmin);
    common_mode_shift = pwm_cmd->common_mode_shift;

    pwm_cmd->duty_a = mc_svpwm_clamp((va - v_offset) + 0.5F + common_mode_shift, cfg->duty_min, cfg->duty_max);
    pwm_cmd->duty_b = mc_svpwm_clamp((vb - v_offset) + 0.5F + common_mode_shift, cfg->duty_min, cfg->duty_max);
    pwm_cmd->duty_c = mc_svpwm_clamp((vc - v_offset) + 0.5F + common_mode_shift, cfg->duty_min, cfg->duty_max);
    pwm_cmd->phase_mode_a = MC_PWM_PHASE_PWM;
    pwm_cmd->phase_mode_b = MC_PWM_PHASE_PWM;
    pwm_cmd->phase_mode_c = MC_PWM_PHASE_PWM;
    pwm_cmd->sector = mc_svpwm_detect_sector(&limited_voltage_ab);
    pwm_cmd->valid = 1U;
}

/**
 * @brief Execute SVPWM from a Q31 alpha-beta voltage vector
 * @param voltage_ab_q31 Desired voltage vector in Q31 alpha-beta frame
 * @param cfg SVPWM configuration (modulation limit, duty limits)
 * @param pwm_cmd Output PWM command (duty cycles, sector, valid flag)
 */
void mc_svpwm_q31_run(const mc_alphabeta_q31_t *voltage_ab_q31, const mc_svpwm_cfg_t *cfg, mc_pwm_cmd_t *pwm_cmd)
{
    mc_alphabeta_t voltage_ab;

    if ((voltage_ab_q31 == NULL) || (cfg == NULL) || (pwm_cmd == NULL))
    {
        return;
    }

    voltage_ab.alpha = mc_q31_to_f32(voltage_ab_q31->alpha);
    voltage_ab.beta = mc_q31_to_f32(voltage_ab_q31->beta);
    mc_svpwm_run(&voltage_ab, cfg, pwm_cmd);
}
