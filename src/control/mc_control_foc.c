/**
 * @file mc_control_foc.c
 * @brief Field-Oriented Control (FOC) high-level controller
 */

#include "mc_control_foc.h"

/**
 * @brief Clamp a value between minimum and maximum bounds
 * @param value Input value to clamp
 * @param min_value Lower bound
 * @param max_value Upper bound
 * @return Clamped value within [min_value, max_value]
 */
static mc_f32_t mc_control_foc_clamp(mc_f32_t value, mc_f32_t min_value, mc_f32_t max_value)
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
 * @brief Initialise FOC controller instance
 * @param control Pointer to FOC control structure
 * @param cfg Pointer to PMSM FOC configuration
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if inputs are NULL
 */
mc_status_t mc_control_foc_init(mc_control_foc_t *control, const mc_pmsm_foc_cfg_t *cfg)
{
    if ((control == NULL) || (cfg == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *control = (mc_control_foc_t){0};
    control->iq_limit = 0.0F;

    return mc_pmsm_foc_init(&control->pmsm_foc, cfg);
}

/**
 * @brief Configure speed PI controller and q-axis current limit
 * @param control Pointer to FOC control structure
 * @param cfg Pointer to PI controller configuration
 * @param iq_limit Maximum allowable q-axis current (must be >= 0)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if inputs invalid
 */
mc_status_t mc_control_foc_set_speed_pi(mc_control_foc_t *control, const mc_pi_cfg_t *cfg, mc_f32_t iq_limit)
{
    if ((control == NULL) || (cfg == NULL) || (iq_limit < 0.0F))
    {
        return MC_STATUS_INVALID_ARG;
    }

    mc_pi_init(&control->speed_pi, cfg);
    control->iq_limit = iq_limit;

    return MC_STATUS_OK;
}

/**
 * @brief Execute one speed control step (outer PI loop)
 * @param control Pointer to FOC control structure
 * @param speed_ref_rpm Target speed in RPM
 * @param speed_feedback_rpm Measured speed in RPM
 * @param dt_s Time step in seconds
 * @param iq_ref Output pointer for computed q-axis current reference
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if inputs invalid
 */
mc_status_t mc_control_foc_speed_step(mc_control_foc_t *control,
                                      mc_f32_t speed_ref_rpm,
                                      mc_f32_t speed_feedback_rpm,
                                      mc_f32_t dt_s,
                                      mc_f32_t *iq_ref)
{
    mc_f32_t speed_error;
    mc_f32_t iq_cmd;

    if ((control == NULL) || (iq_ref == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    speed_error = speed_ref_rpm - speed_feedback_rpm;
    iq_cmd = mc_pi_run(&control->speed_pi, speed_error, dt_s);

    if (control->iq_limit > 0.0F)
    {
        iq_cmd = mc_control_foc_clamp(iq_cmd, -control->iq_limit, control->iq_limit);
    }

    *iq_ref = iq_cmd;

    return MC_STATUS_OK;
}

/**
 * @brief Execute one FOC control cycle (inner current loop)
 * @param control Pointer to FOC control structure
 * @param in Pointer to FOC input (currents, angle, etc.)
 * @param out Pointer to FOC output (voltage commands)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if inputs are NULL
 */
mc_status_t mc_control_foc_run(mc_control_foc_t *control, const mc_pmsm_foc_input_t *in, mc_pmsm_foc_output_t *out)
{
    if ((control == NULL) || (in == NULL) || (out == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    return mc_pmsm_foc_run(&control->pmsm_foc, in, out);
}
