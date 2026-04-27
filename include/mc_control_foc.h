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
 * @param control  Pointer to FOC control instance
 * @param cfg      PMSM FOC configuration
 * @return Operation status
 */
mc_status_t mc_control_foc_init(mc_control_foc_t *control, const mc_pmsm_foc_cfg_t *cfg);

/**
 * @brief Configure the speed-loop PI gains and q-axis current limit
 * @param control  Pointer to FOC control instance
 * @param cfg      PI controller configuration
 * @param iq_limit Q-axis current limit [A]
 * @return Operation status
 */
mc_status_t mc_control_foc_set_speed_pi(mc_control_foc_t *control, const mc_pi_cfg_t *cfg, mc_f32_t iq_limit);

/**
 * @brief Execute one speed-control step (outer loop)
 * @param control           Pointer to FOC control instance
 * @param speed_ref_rpm     Target speed [RPM]
 * @param speed_feedback_rpm Measured speed [RPM]
 * @param dt_s              Sample interval [s]
 * @param iq_ref            Output q-axis current reference [A]
 * @return Operation status
 */
mc_status_t mc_control_foc_speed_step(mc_control_foc_t *control,
                                      mc_f32_t speed_ref_rpm,
                                      mc_f32_t speed_feedback_rpm,
                                      mc_f32_t dt_s,
                                      mc_f32_t *iq_ref);

/**
 * @brief Run the full FOC current-loop step (inner loop)
 * @param control  Pointer to FOC control instance
 * @param in       FOC input (phase currents, electrical angle)
 * @param out      FOC output (D/Q voltages)
 * @return Operation status
 */
mc_status_t mc_control_foc_run(mc_control_foc_t *control, const mc_pmsm_foc_input_t *in, mc_pmsm_foc_output_t *out);

#endif /* MC_CONTROL_FOC_H */
