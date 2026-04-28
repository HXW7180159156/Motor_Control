/** @file mc_sensor_sensorless.c @brief Sensorless observer (BEMF + PLL) implementation */

#include "mc_constants.h"
#include "mc_sensor_sensorless.h"
#include "mc_math.h"

#include <math.h>

/**
 * @brief Initialise sensorless observer state with configuration
 * @param state [out] Sensorless observer state structure
 * @param cfg Sensorless observer configuration parameters
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
mc_status_t mc_sensorless_init(mc_sensorless_state_t *state, const mc_sensorless_cfg_t *cfg)
{
    if ((state == NULL) || (cfg == NULL) || (cfg->pole_pairs <= 0.0F) || (cfg->ls_h < 0.0F) ||
        (cfg->bemf_filter_alpha <= 0.0F) || (cfg->bemf_filter_alpha > 1.0F) || (cfg->min_bemf < 0.0F) ||
        (cfg->pll_kp < 0.0F) || (cfg->pll_ki < 0.0F) || (cfg->lock_bemf <= cfg->min_bemf) ||
        (cfg->startup_speed_rad_s < 0.0F) || (cfg->startup_accel_rad_s2 < 0.0F) ||
        (cfg->open_loop_voltage_start < 0.0F) || (cfg->open_loop_voltage_max < 0.0F) ||
        (cfg->open_loop_voltage_max < cfg->open_loop_voltage_start))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *state = (mc_sensorless_state_t){0};
    state->cfg = *cfg;
    state->open_loop_active = MC_TRUE;

    return MC_STATUS_OK;
}

/**
 * @brief Update sensorless observer with voltage and current measurements
 * @param state Sensorless observer state structure (in/out)
 * @param voltage_ab Alpha-beta voltage vector
 * @param current_ab Alpha-beta current vector
 * @param dt_s Time step in seconds
 * @param timestamp_us Microsecond timestamp of the update
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
mc_status_t mc_sensorless_update(mc_sensorless_state_t *state,
                                 const mc_alphabeta_t *voltage_ab,
                                 const mc_alphabeta_t *current_ab,
                                 mc_f32_t dt_s,
                                 uint32_t timestamp_us)
{
    mc_alphabeta_t raw_bemf;
    mc_alphabeta_t di_dt_ab;
    mc_f32_t measured_angle_rad;
    mc_f32_t angle_error;

    if ((state == NULL) || (voltage_ab == NULL) || (current_ab == NULL) || (dt_s <= 0.0F))
    {
        return MC_STATUS_INVALID_ARG;
    }

    if (state->open_loop_active != MC_FALSE)
    {
        state->pll_speed_rad_s += state->cfg.startup_accel_rad_s2 * dt_s;
        if (state->pll_speed_rad_s > state->cfg.startup_speed_rad_s)
        {
            state->pll_speed_rad_s = state->cfg.startup_speed_rad_s;
        }
        state->elec_angle_rad = mc_math_wrap_angle_rad(state->elec_angle_rad + (state->pll_speed_rad_s * dt_s));

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

    di_dt_ab.alpha = (current_ab->alpha - state->last_current_ab.alpha) / dt_s;
    di_dt_ab.beta = (current_ab->beta - state->last_current_ab.beta) / dt_s;
    raw_bemf.alpha = voltage_ab->alpha - (state->cfg.rs_ohm * current_ab->alpha) - (state->cfg.ls_h * di_dt_ab.alpha);
    raw_bemf.beta = voltage_ab->beta - (state->cfg.rs_ohm * current_ab->beta) - (state->cfg.ls_h * di_dt_ab.beta);

    if ((!isfinite(raw_bemf.alpha)) || (!isfinite(raw_bemf.beta)))
    {
        state->observer_valid = MC_FALSE;
        state->pll_locked = MC_FALSE;
        state->last_current_ab = *current_ab;
        state->last_timestamp_us = timestamp_us;
        return MC_STATUS_OK;
    }

    state->bemf_ab.alpha = mc_math_lpf_f32(state->bemf_ab.alpha, raw_bemf.alpha, state->cfg.bemf_filter_alpha);
    state->bemf_ab.beta = mc_math_lpf_f32(state->bemf_ab.beta, raw_bemf.beta, state->cfg.bemf_filter_alpha);
    state->bemf_magnitude = sqrtf((state->bemf_ab.alpha * state->bemf_ab.alpha) +
                                  (state->bemf_ab.beta * state->bemf_ab.beta));

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
        state->last_current_ab = *current_ab;
        state->last_timestamp_us = timestamp_us;
        return MC_STATUS_OK;
    }

    state->observer_valid = MC_TRUE;
    measured_angle_rad = mc_math_wrap_angle_rad(atan2f(state->bemf_ab.beta, state->bemf_ab.alpha) - MC_PI_OVER_2);
    angle_error = mc_math_wrap_angle_rad(measured_angle_rad - state->elec_angle_rad);
    state->pll_integrator += state->cfg.pll_ki * angle_error * dt_s;
    if (state->open_loop_active != MC_FALSE)
    {
        if ((state->bemf_magnitude >= state->cfg.lock_bemf) &&
            (fabsf(angle_error) < MC_SENSORLESS_OPEN_LOOP_HANDOFF_ANGLE_ERR_RAD))
        {
            if (state->open_loop_lock_counter < MC_SENSORLESS_COUNTER_MAX)
            {
                state->open_loop_lock_counter++;
            }
        }
        else
        {
            state->open_loop_lock_counter = 0U;
        }

        if (state->open_loop_lock_counter >= MC_SENSORLESS_OPEN_LOOP_LOCK_SAMPLES)
        {
            state->open_loop_lock_counter = 0U;
            state->open_loop_active = MC_FALSE;
            state->pll_locked = MC_TRUE;
            state->pll_lock_counter = MC_SENSORLESS_PLL_LOCK_SAMPLES;
            state->pll_unlock_counter = 0U;
            state->pll_integrator = state->pll_speed_rad_s;
            state->open_loop_voltage = 0.0F;
        }
    }
    else
    {
        state->pll_speed_rad_s = state->pll_integrator + (state->cfg.pll_kp * angle_error);
        state->elec_angle_rad = mc_math_wrap_angle_rad(state->elec_angle_rad + (state->pll_speed_rad_s * dt_s));
        if (fabsf(angle_error) < MC_SENSORLESS_PLL_LOCK_ANGLE_ERR_RAD)
        {
            if (state->pll_lock_counter < MC_SENSORLESS_COUNTER_MAX)
            {
                state->pll_lock_counter++;
            }
            state->pll_unlock_counter = 0U;
            if (state->pll_lock_counter >= MC_SENSORLESS_PLL_LOCK_SAMPLES)
            {
                state->pll_locked = MC_TRUE;
            }
        }
        else
        {
            state->pll_lock_counter = 0U;
            if (state->pll_unlock_counter < MC_SENSORLESS_COUNTER_MAX)
            {
                state->pll_unlock_counter++;
            }
            if (state->pll_unlock_counter >= MC_SENSORLESS_PLL_UNLOCK_SAMPLES)
            {
                state->pll_locked = MC_FALSE;
            }
        }
    }

    state->mech_speed_rpm = (state->pll_speed_rad_s * MC_RAD_S_TO_RPM) / state->cfg.pole_pairs;
    state->last_current_ab = *current_ab;
    state->last_timestamp_us = timestamp_us;

    return MC_STATUS_OK;
}
