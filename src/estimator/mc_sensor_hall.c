/** @file mc_sensor_hall.c @brief Hall effect sensor interface */

#include "mc_sensor_hall.h"

/**
 * @brief Find index of a Hall code in the configured sequence
 * @param cfg Hall sensor configuration
 * @param hall_code Current Hall effect sensor code
 * @param index [out] Index in the hall_code_sequence array
 * @return MC_STATUS_OK if found, MC_STATUS_INVALID_ARG if not found
 */
static mc_status_t mc_hall_find_index(const mc_hall_cfg_t *cfg, uint8_t hall_code, uint8_t *index)
{
    uint8_t idx;

    if ((cfg == NULL) || (index == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    for (idx = 0U; idx < 6U; ++idx)
    {
        if (cfg->hall_code_sequence[idx] == hall_code)
        {
            *index = idx;
            return MC_STATUS_OK;
        }
    }

    return MC_STATUS_INVALID_ARG;
}

/**
 * @brief Initialize Hall sensor state with configuration
 * @param state [out] Hall sensor state structure
 * @param cfg Hall sensor configuration parameters
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
mc_status_t mc_hall_init(mc_hall_state_t *state, const mc_hall_cfg_t *cfg)
{
    if ((state == NULL) || (cfg == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *state = (mc_hall_state_t){0};
    state->cfg = *cfg;

    return MC_STATUS_OK;
}

/**
 * @brief Update Hall sensor state with new reading
 * @param state Hall sensor state structure (in/out)
 * @param hall_code Current Hall effect sensor code
 * @param timestamp_us Microsecond timestamp of the reading
 * @param pole_pairs Number of motor pole pairs
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
mc_status_t mc_hall_update(mc_hall_state_t *state, uint8_t hall_code, uint32_t timestamp_us, mc_f32_t pole_pairs)
{
    uint8_t index;
    mc_status_t status;

    if ((state == NULL) || (pole_pairs <= 0.0F))
    {
        return MC_STATUS_INVALID_ARG;
    }

    status = mc_hall_find_index(&state->cfg, hall_code, &index);
    if (status != MC_STATUS_OK)
    {
        return status;
    }

    state->elec_angle_rad = state->cfg.elec_angle_table_rad[index];

    if ((state->last_transition_us != 0U) && (timestamp_us > state->last_transition_us) && (hall_code != state->last_hall_code))
    {
        mc_f32_t delta_us = (mc_f32_t)(timestamp_us - state->last_transition_us);
        mc_f32_t elec_rev_per_sec = 1000000.0F / (delta_us * 6.0F);
        mc_f32_t mech_rev_per_sec = elec_rev_per_sec / pole_pairs;
        state->mech_speed_rpm = mech_rev_per_sec * 60.0F;
    }

    if (hall_code != state->last_hall_code)
    {
        state->last_transition_us = timestamp_us;
        state->last_hall_code = hall_code;
    }

    return MC_STATUS_OK;
}
