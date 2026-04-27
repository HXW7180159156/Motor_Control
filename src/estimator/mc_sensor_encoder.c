/** @file mc_sensor_encoder.c @brief Quadrature encoder sensor interface */

#include "mc_sensor_encoder.h"
#include "mc_math.h"

/**
 * @brief Initialize encoder state with configuration
 * @param state [out] Encoder state structure
 * @param cfg Encoder configuration parameters
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
mc_status_t mc_encoder_init(mc_encoder_state_t *state, const mc_encoder_cfg_t *cfg)
{
    if ((state == NULL) || (cfg == NULL) || (cfg->counts_per_rev == 0U) || (cfg->pole_pairs <= 0.0F))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *state = (mc_encoder_state_t){0};
    state->cfg = *cfg;

    return MC_STATUS_OK;
}

/**
 * @brief Update encoder state with new count and timestamp
 * @param state Encoder state structure (in/out)
 * @param encoder_count Raw encoder position count
 * @param timestamp_us Microsecond timestamp of the reading
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
mc_status_t mc_encoder_update(mc_encoder_state_t *state, uint32_t encoder_count, uint32_t timestamp_us)
{
    mc_f32_t mech_angle_rad;

    if (state == NULL)
    {
        return MC_STATUS_INVALID_ARG;
    }

    mech_angle_rad = (((mc_f32_t)(encoder_count % state->cfg.counts_per_rev)) * 6.2831853072F) /
                     ((mc_f32_t)state->cfg.counts_per_rev);
    state->elec_angle_rad = mc_math_wrap_angle_rad(mech_angle_rad * state->cfg.pole_pairs);

    if ((state->last_timestamp_us != 0U) && (timestamp_us > state->last_timestamp_us))
    {
        int32_t delta_count = (int32_t)encoder_count - (int32_t)state->last_count;
        mc_f32_t delta_us = (mc_f32_t)(timestamp_us - state->last_timestamp_us);
        mc_f32_t mech_rev_per_sec = (((mc_f32_t)delta_count) * 1000000.0F) /
                                    (((mc_f32_t)state->cfg.counts_per_rev) * delta_us);
        state->mech_speed_rpm = mech_rev_per_sec * 60.0F;
    }

    state->last_count = encoder_count;
    state->last_timestamp_us = timestamp_us;

    return MC_STATUS_OK;
}
