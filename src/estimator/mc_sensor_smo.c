/** @file mc_sensor_smo.c @brief Sliding Mode Observer (SMO) for PMSM sensorless control */

#include "mc_constants.h"
#include "mc_math.h"
#include "mc_sensor_smo.h"

#include <math.h>

#define MC_SMO_OPEN_LOOP_LOCK_SAMPLES (2U)
#define MC_SMO_PLL_LOCK_SAMPLES (2U)
#define MC_SMO_PLL_UNLOCK_SAMPLES (2U)
#define MC_SMO_COUNTER_MAX (255U)
#define MC_SMO_OPEN_LOOP_HANDOFF_ANGLE_ERR_RAD (0.5F)
#define MC_SMO_PLL_LOCK_ANGLE_ERR_RAD (0.35F)

/**
 * @brief Sliding mode sign function returning ±k_slide
 * @param error Current estimation error (i_hat - i_measured)
 * @param k_slide Sliding mode gain
 * @return +k_slide or -k_slide
 */
static mc_f32_t mc_smo_sign(mc_f32_t error, mc_f32_t k_slide)
{
    if (error >= 0.0F)
    {
        return k_slide;
    }
    return -k_slide;
}

/**
 * @brief Compute the inv_Ls term once to avoid repeated division
 * @param ls_h Inductance [H]
 * @return 1/Ls, or 1.0F if Ls <= 0
 */
static mc_f32_t mc_smo_inv_Ls(mc_f32_t ls_h)
{
    if (ls_h > MC_EPSILON_F)
    {
        return 1.0F / ls_h;
    }
    return 1.0F;
}

mc_status_t mc_smo_init(mc_smo_state_t *state, const mc_smo_cfg_t *cfg)
{
    if ((state == NULL) || (cfg == NULL) || (cfg->pole_pairs <= 0.0F) ||
        (cfg->ls_h < 0.0F) || (cfg->k_slide <= 0.0F) ||
        (cfg->lpf_alpha <= 0.0F) || (cfg->lpf_alpha > 1.0F) ||
        (cfg->min_bemf < 0.0F) || (cfg->pll_kp < 0.0F) || (cfg->pll_ki < 0.0F) ||
        (cfg->lock_bemf <= cfg->min_bemf) ||
        (cfg->startup_speed_rad_s < 0.0F) || (cfg->startup_accel_rad_s2 < 0.0F) ||
        (cfg->open_loop_voltage_start < 0.0F) || (cfg->open_loop_voltage_max < 0.0F) ||
        (cfg->open_loop_voltage_max < cfg->open_loop_voltage_start))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *state = (mc_smo_state_t){0};
    state->cfg = *cfg;
    state->open_loop_active = MC_TRUE;

    return MC_STATUS_OK;
}

mc_status_t mc_smo_update(mc_smo_state_t *state,
                          const mc_alphabeta_t *voltage_ab,
                          const mc_alphabeta_t *current_ab,
                          mc_f32_t dt_s,
                          uint32_t timestamp_us)
{
    mc_f32_t inv_ls;
    mc_f32_t i_err_alpha;
    mc_f32_t i_err_beta;
    mc_f32_t di_hat_alpha;
    mc_f32_t di_hat_beta;
    mc_f32_t measured_angle_rad;
    mc_f32_t angle_error;

    if ((state == NULL) || (voltage_ab == NULL) || (current_ab == NULL) || (dt_s <= 0.0F))
    {
        return MC_STATUS_INVALID_ARG;
    }

    inv_ls = mc_smo_inv_Ls(state->cfg.ls_h);

    /* --- open-loop startup --- */
    if (state->open_loop_active != MC_FALSE)
    {
        state->pll_speed_rad_s += state->cfg.startup_accel_rad_s2 * dt_s;
        if (state->pll_speed_rad_s > state->cfg.startup_speed_rad_s)
        {
            state->pll_speed_rad_s = state->cfg.startup_speed_rad_s;
        }
        state->elec_angle_rad = mc_math_wrap_angle_rad(
            state->elec_angle_rad + (state->pll_speed_rad_s * dt_s));

        if (state->cfg.startup_speed_rad_s > 0.0F)
        {
            mc_f32_t speed_ratio = state->pll_speed_rad_s / state->cfg.startup_speed_rad_s;
            state->open_loop_voltage = state->cfg.open_loop_voltage_start +
                (state->cfg.open_loop_voltage_max - state->cfg.open_loop_voltage_start) * speed_ratio;
        }
        if (state->open_loop_voltage < state->cfg.open_loop_voltage_start)
        {
            state->open_loop_voltage = state->cfg.open_loop_voltage_start;
        }
    }

    /* --- SMO current observer step --- */
    i_err_alpha = state->i_hat_ab.alpha - current_ab->alpha;
    i_err_beta  = state->i_hat_ab.beta  - current_ab->beta;

    state->z_ab.alpha = mc_smo_sign(i_err_alpha, state->cfg.k_slide);
    state->z_ab.beta  = mc_smo_sign(i_err_beta,  state->cfg.k_slide);

    di_hat_alpha = (voltage_ab->alpha - (state->cfg.rs_ohm * state->i_hat_ab.alpha) + state->z_ab.alpha) * inv_ls;
    di_hat_beta  = (voltage_ab->beta  - (state->cfg.rs_ohm * state->i_hat_ab.beta)  + state->z_ab.beta)  * inv_ls;

    state->i_hat_ab.alpha += di_hat_alpha * dt_s;
    state->i_hat_ab.beta  += di_hat_beta  * dt_s;

    /* NaN/Inf guard on estimated current */
    if ((!isfinite(state->i_hat_ab.alpha)) || (!isfinite(state->i_hat_ab.beta)))
    {
        state->i_hat_ab = *current_ab;
        state->observer_valid = MC_FALSE;
        state->pll_locked = MC_FALSE;
        state->last_timestamp_us = timestamp_us;
        return MC_STATUS_OK;
    }

    /* --- BEMF extraction via LPF --- */
    state->bemf_ab.alpha = mc_math_lpf_f32(state->bemf_ab.alpha, state->z_ab.alpha, state->cfg.lpf_alpha);
    state->bemf_ab.beta  = mc_math_lpf_f32(state->bemf_ab.beta,  state->z_ab.beta,  state->cfg.lpf_alpha);
    state->bemf_magnitude = sqrtf((state->bemf_ab.alpha * state->bemf_ab.alpha) +
                                   (state->bemf_ab.beta  * state->bemf_ab.beta));

    /* --- validity check --- */
    if (state->bemf_magnitude < state->cfg.min_bemf)
    {
        state->observer_valid = MC_FALSE;
        state->pll_locked = MC_FALSE;
        state->open_loop_lock_counter = 0U;
        state->pll_lock_counter = 0U;
        state->pll_unlock_counter = 0U;
        if (state->open_loop_active == MC_FALSE)
        {
            state->pll_speed_rad_s = 0.0F;
            state->pll_integrator = 0.0F;
        }
        state->mech_speed_rpm = 0.0F;
        state->last_timestamp_us = timestamp_us;
        return MC_STATUS_OK;
    }

    state->observer_valid = MC_TRUE;

    /* --- PLL: angle + speed tracking --- */
    measured_angle_rad = mc_math_wrap_angle_rad(
        atan2f(state->bemf_ab.beta, state->bemf_ab.alpha) - MC_PI_OVER_2);
    angle_error = mc_math_wrap_angle_rad(measured_angle_rad - state->elec_angle_rad);

    if (state->open_loop_active != MC_FALSE)
    {
        /* open-loop handoff check */
        if ((state->bemf_magnitude >= state->cfg.lock_bemf) &&
            (fabsf(angle_error) < MC_SMO_OPEN_LOOP_HANDOFF_ANGLE_ERR_RAD))
        {
            if (state->open_loop_lock_counter < MC_SMO_COUNTER_MAX)
            {
                state->open_loop_lock_counter++;
            }
        }
        else
        {
            state->open_loop_lock_counter = 0U;
        }

        if (state->open_loop_lock_counter >= MC_SMO_OPEN_LOOP_LOCK_SAMPLES)
        {
            state->open_loop_lock_counter = 0U;
            state->open_loop_active = MC_FALSE;
            state->pll_locked = MC_TRUE;
            state->pll_lock_counter = MC_SMO_PLL_LOCK_SAMPLES;
            state->pll_unlock_counter = 0U;
            state->pll_integrator = state->pll_speed_rad_s;
            state->open_loop_voltage = 0.0F;
        }
    }
    else
    {
        /* closed-loop PLL */
        state->pll_integrator += state->cfg.pll_ki * angle_error * dt_s;
        state->pll_speed_rad_s = state->pll_integrator + (state->cfg.pll_kp * angle_error);
        state->elec_angle_rad = mc_math_wrap_angle_rad(
            state->elec_angle_rad + (state->pll_speed_rad_s * dt_s));

        /* lock/unlock debounce */
        if (fabsf(angle_error) < MC_SMO_PLL_LOCK_ANGLE_ERR_RAD)
        {
            if (state->pll_lock_counter < MC_SMO_COUNTER_MAX)
            {
                state->pll_lock_counter++;
            }
            state->pll_unlock_counter = 0U;
            if (state->pll_lock_counter >= MC_SMO_PLL_LOCK_SAMPLES)
            {
                state->pll_locked = MC_TRUE;
            }
        }
        else
        {
            state->pll_lock_counter = 0U;
            if (state->pll_unlock_counter < MC_SMO_COUNTER_MAX)
            {
                state->pll_unlock_counter++;
            }
            if (state->pll_unlock_counter >= MC_SMO_PLL_UNLOCK_SAMPLES)
            {
                state->pll_locked = MC_FALSE;
            }
        }
    }

    state->mech_speed_rpm = (state->pll_speed_rad_s * MC_RAD_S_TO_RPM) / state->cfg.pole_pairs;
    state->last_timestamp_us = timestamp_us;

    return MC_STATUS_OK;
}
