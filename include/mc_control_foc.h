#ifndef MC_CONTROL_FOC_H
#define MC_CONTROL_FOC_H
/** @file mc_control_foc.h @brief Field-Oriented Control (FOC) types and API */

#include "mc_drive_pmsm.h"

/** @brief FOC speed-control instance combining PMSM FOC with speed PI loop */
typedef struct
{
    mc_pmsm_foc_t pmsm_foc; /**< Underlying PMSM FOC controller */
    mc_pi_t speed_pi;       /**< Speed-loop PI controller */
    mc_f32_t iq_limit;      /**< Q-axis current limit [A] */
} mc_control_foc_t;

/**
 * @brief Initialise the FOC controller
 * @param[out] control Pointer to FOC control instance.
 *   Range: non-NULL pointer to writable `mc_control_foc_t` storage.
 * @param[in] cfg PMSM FOC configuration.
 *   Range: non-NULL pointer to readable `mc_pmsm_foc_cfg_t` storage.
 * @retval MC_STATUS_OK Initialization completed successfully.
 * @retval MC_STATUS_INVALID_ARG `control == NULL` or `cfg == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `control` object.
 */
mc_status_t mc_control_foc_init(mc_control_foc_t *control, const mc_pmsm_foc_cfg_t *cfg);

/**
 * @brief Configure the speed-loop PI gains and q-axis current limit
 * @param[in,out] control Pointer to FOC control instance.
 *   Range: non-NULL pointer to writable `mc_control_foc_t` storage.
 * @param[in] cfg PI controller configuration.
 *   Range: non-NULL pointer to readable `mc_pi_cfg_t` storage.
 * @param[in] iq_limit Q-axis current limit [A].
 *   Range: `[0.0F, +inf)`.
 * @retval MC_STATUS_OK Configuration was applied successfully.
 * @retval MC_STATUS_INVALID_ARG `control == NULL`, `cfg == NULL`, or `iq_limit < 0.0F`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `control` object. Not reentrant for concurrent writes to the same `control`.
 */
mc_status_t mc_control_foc_set_speed_pi(mc_control_foc_t *control, const mc_pi_cfg_t *cfg, mc_f32_t iq_limit);

/**
 * @brief Execute one speed-control step (outer loop)
 * @param[in,out] control Pointer to FOC control instance.
 *   Range: non-NULL pointer to writable `mc_control_foc_t` storage.
 * @param[in] speed_ref_rpm Target speed [RPM].
 *   Range: application-defined `mc_f32_t` mechanical speed reference.
 * @param[in] speed_feedback_rpm Measured speed [RPM].
 *   Range: application-defined `mc_f32_t` mechanical speed feedback.
 * @param[in] dt_s Sample interval [s].
 *   Range: application-defined `mc_f32_t`; `dt_s >= 0.0F` is recommended.
 * @param[out] iq_ref Output q-axis current reference [A].
 *   Range: non-NULL pointer to writable `mc_f32_t` storage.
 * @retval MC_STATUS_OK Step completed successfully.
 * @retval MC_STATUS_INVALID_ARG `control == NULL` or `iq_ref == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `control` object and a different writable `iq_ref` output.
 */
mc_status_t mc_control_foc_speed_step(mc_control_foc_t *control,
                                      mc_f32_t speed_ref_rpm,
                                      mc_f32_t speed_feedback_rpm,
                                      mc_f32_t dt_s,
                                      mc_f32_t *iq_ref);

/**
 * @brief Run the full FOC current-loop step (inner loop)
 * @param[in,out] control Pointer to FOC control instance.
 *   Range: non-NULL pointer to writable `mc_control_foc_t` storage.
 * @param[in] in FOC input (phase currents, electrical angle).
 *   Range: non-NULL pointer to readable `mc_pmsm_foc_input_t` storage.
 * @param[out] out FOC output (D/Q voltages).
 *   Range: non-NULL pointer to writable `mc_pmsm_foc_output_t` storage.
 * @retval MC_STATUS_OK Step completed successfully.
 * @retval MC_STATUS_INVALID_ARG `control == NULL`, `in == NULL`, or `out == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `control` object and a different writable `out` structure.
 */
mc_status_t mc_control_foc_run(mc_control_foc_t *control, const mc_pmsm_foc_input_t *in, mc_pmsm_foc_output_t *out);

#endif /* MC_CONTROL_FOC_H */
