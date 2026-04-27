/** @file mc_sensor_resolver.c @brief Resolver sensor interface */

#include "mc_sensor_resolver.h"
#include "mc_math.h"

#include <math.h>

/**
 * @brief Initialise resolver state with configuration
 * @param state [out] Resolver state structure
 * @param cfg Resolver configuration parameters
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
mc_status_t mc_resolver_init(mc_resolver_state_t *state, const mc_resolver_cfg_t *cfg)
{
    if ((state == NULL) || (cfg == NULL) || (cfg->pole_pairs <= 0.0F) || (cfg->signal_scale <= 0.0F))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *state = (mc_resolver_state_t){0};
    state->cfg = *cfg;

    return MC_STATUS_OK;
}

/**
 * @brief Get resolver excitation command parameters
 * @param state Resolver state structure
 * @param amplitude [out] Excitation signal amplitude
 * @param frequency_hz [out] Excitation signal frequency in Hz
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
mc_status_t mc_resolver_get_excitation_command(const mc_resolver_state_t *state, int16_t *amplitude, uint16_t *frequency_hz)
{
    if ((state == NULL) || (amplitude == NULL) || (frequency_hz == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *amplitude = state->cfg.excitation_amplitude;
    *frequency_hz = state->cfg.excitation_frequency_hz;

    return MC_STATUS_OK;
}

/**
 * @brief Update resolver state with new raw sin/cos readings
 * @param state Resolver state structure (in/out)
 * @param raw Raw resolver sin/cos ADC values
 * @param timestamp_us Microsecond timestamp of the reading
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
mc_status_t mc_resolver_update(mc_resolver_state_t *state, const mc_resolver_raw_t *raw, uint32_t timestamp_us)
{
    mc_f32_t sin_value;
    mc_f32_t cos_value;
    mc_f32_t mech_angle_rad;

    if ((state == NULL) || (raw == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    sin_value = ((mc_f32_t)raw->sin_raw) * state->cfg.signal_scale;
    cos_value = ((mc_f32_t)raw->cos_raw) * state->cfg.signal_scale;

    state->sin_component = sin_value;
    state->cos_component = cos_value;
    state->signal_amplitude = sqrtf((sin_value * sin_value) + (cos_value * cos_value));
    state->signal_valid = (state->signal_amplitude >= state->cfg.min_signal_amplitude) ? MC_TRUE : MC_FALSE;

    if (state->signal_valid != MC_FALSE)
    {
        mech_angle_rad = atan2f(sin_value, cos_value);
        state->elec_angle_rad = mc_math_wrap_angle_rad(mech_angle_rad * state->cfg.pole_pairs);

        if ((state->last_timestamp_us != 0U) && (timestamp_us > state->last_timestamp_us))
        {
            mc_f32_t delta_us = (mc_f32_t)(timestamp_us - state->last_timestamp_us);
            mc_f32_t delta_mech_angle = mc_math_wrap_angle_rad(mech_angle_rad - state->last_mech_angle_rad);
            mc_f32_t mech_rev_per_sec = delta_mech_angle / 6.2831853072F;
            state->mech_speed_rpm = (mech_rev_per_sec * 1000000.0F * 60.0F) / delta_us;
        }

        state->last_mech_angle_rad = mech_angle_rad;
    }
    else
    {
        state->mech_speed_rpm = 0.0F;
    }

    state->last_timestamp_us = timestamp_us;

    return MC_STATUS_OK;
}
