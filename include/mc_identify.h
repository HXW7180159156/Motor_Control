#ifndef MC_IDENTIFY_H
#define MC_IDENTIFY_H

/** @file mc_identify.h @brief Motor parameter identification (auto-tuning) API. */

#include "mc_control_svpwm.h"
#include "mc_port_adc.h"
#include "mc_status.h"
#include "mc_transform.h"
#include "mc_types.h"

/** @brief State machine states for the identification process. */
typedef enum
{
    MC_IDENTIFY_STATE_IDLE = 0, /**< Not running. */
    MC_IDENTIFY_STATE_ALIGN, /**< Aligning rotor to a known position. */
    MC_IDENTIFY_STATE_RS_SETTLE, /**< Settling time before stator resistance measurement. */
    MC_IDENTIFY_STATE_RS_MEASURE, /**< Measuring stator resistance. */
    MC_IDENTIFY_STATE_LD_INJECT, /**< Injecting D-axis voltage for Ld measurement. */
    MC_IDENTIFY_STATE_LD_MEASURE, /**< Finalizing D-axis inductance measurement. */
    MC_IDENTIFY_STATE_LQ_ALIGN, /**< Aligning rotor for Q-axis measurement. */
    MC_IDENTIFY_STATE_LQ_INJECT, /**< Injecting Q-axis voltage for Lq measurement. */
    MC_IDENTIFY_STATE_LQ_MEASURE, /**< Measuring Q-axis inductance and finalizing flux. */
    MC_IDENTIFY_STATE_COMPLETE, /**< Identification completed successfully. */
    MC_IDENTIFY_STATE_ERROR /**< Identification failed with an error. */
} mc_identify_state_t;

/** @brief Configuration parameters for the identification process. */
typedef struct
{
    mc_f32_t align_voltage; /**< Alignment voltage command ratio scaled by `voltage_limit`. */
    mc_f32_t rs_voltage; /**< Rs measurement voltage command ratio scaled by `voltage_limit`. */
    mc_f32_t pulse_voltage; /**< Inductance pulse voltage command ratio scaled by `voltage_limit`. */
    mc_f32_t align_time_s; /**< Alignment duration [s]. */
    mc_f32_t rs_settle_time_s; /**< Settling time before Rs measurement [s]. */
    mc_f32_t rs_sample_time_s; /**< Rs sampling duration [s]. */
    mc_f32_t ld_pulse_time_s; /**< Ld pulse duration [s]. */
    mc_f32_t lq_align_time_s; /**< Lq alignment duration [s]. */
    mc_f32_t lq_pulse_time_s; /**< Lq pulse duration [s]. */
    mc_f32_t max_current_a; /**< Maximum allowed current during identification [A]. */
    mc_svpwm_cfg_t svpwm_cfg; /**< SVPWM configuration for voltage application. */
    mc_f32_t voltage_limit; /**< Voltage limit [V]. */
} mc_identify_cfg_t;

/*
 * Current identify implementation assumptions:
 * - `current_ab` is expected to be reconstructed and scaled in amperes before entering `mc_identify_run()`.
 * - `align_voltage`, `rs_voltage`, and `pulse_voltage` are normalized command ratios multiplied by `voltage_limit`.
 * - `Ld`, `Lq`, and final `flux_wb` windows use time-weighted averaging over the actual discrete injected duration observed by the state machine.
 * - `flux_wb` uses an open-loop q-axis voltage balance estimate and may keep its seeded value when no valid flux samples are collected.
 * - Raw measured `Rs`, `Ld`, `Lq`, and flux candidates are tracked internally before trust gates promote them to the public result fields.
 * - Internal candidate fields are not part of the public identify result contract and are never returned by `mc_identify_get_result()`.
 * - Non-physical `Rs`, `Ld`, and `Lq` estimates are rejected and reported as `0.0F`.
 */

/** @brief Input data to the identification state machine each cycle. */
typedef struct
{
    mc_alphabeta_t current_ab; /**< Alpha-beta current measurement. */
    mc_f32_t dt_s; /**< Elapsed time since last call [s]. */
} mc_identify_input_t;

/** @brief Output produced by the identification state machine each cycle. */
typedef struct
{
    mc_pwm_cmd_t pwm_cmd; /**< PWM command to apply. */
    mc_adc_trigger_plan_t adc_trigger_plan; /**< ADC trigger plan for sampling. */
} mc_identify_output_t;

/** @brief Persistent state for the motor parameter identifier. */
typedef struct
{
    mc_identify_state_t state; /**< Current state machine state. */
    mc_identify_cfg_t cfg; /**< Configuration parameters. */
    mc_f32_t v_alpha; /**< Alpha voltage component. */
    mc_f32_t v_beta; /**< Beta voltage component. */
    mc_f32_t i_alpha_prev; /**< Previous alpha current. */
    mc_f32_t i_beta_prev; /**< Previous beta current. */
    mc_f32_t vd_applied; /**< Applied D-axis voltage. */
    mc_f32_t i_alpha_start; /**< Alpha current at start of measurement window. */
    mc_f32_t i_beta_start; /**< Beta current at start of measurement window. */
    mc_f32_t id_acc; /**< Accumulated D-axis current integral. */
    mc_f32_t i_alpha_window_acc; /**< Time-integrated alpha current over the active pulse window [A*s]. */
    mc_f32_t i_beta_window_acc; /**< Time-integrated beta current over the active pulse window [A*s]. */
    mc_f32_t pulse_time_acc_s; /**< Accumulated active pulse duration in the current pulse window [s]. */
    uint32_t sample_count; /**< Number of samples accumulated. */
    uint32_t step_counter; /**< Step counter within current state. */
    uint32_t total_steps; /**< Total steps expected for current state. */
    mc_f32_t rs_ohm; /**< Identified stator resistance [Ohm]. */
    mc_f32_t rs_candidate_ohm; /**< Internal raw stator resistance estimate before trust gating [Ohm]. */
    mc_f32_t ld_h; /**< Identified D-axis inductance [H]. */
    mc_f32_t ld_candidate_h; /**< Internal raw D-axis inductance estimate before trust gating [H]. */
    mc_f32_t lq_h; /**< Identified Q-axis inductance [H]. */
    mc_f32_t lq_candidate_h; /**< Internal raw Q-axis inductance estimate before trust gating [H]. */
    mc_f32_t flux_wb; /**< Identified motor flux [Wb]. */
    mc_f32_t flux_candidate_wb; /**< Internal raw averaged flux estimate before trust gating [Wb]. */
    mc_f32_t flux_acc; /**< Time-integrated valid flux estimate accumulation [Wb*s]. */
    mc_f32_t flux_time_acc_s; /**< Accumulated duration of valid flux estimate samples [s]. */
    uint32_t flux_sample_count; /**< Number of valid flux estimate samples accumulated. */
} mc_identify_t;

/**
 * @brief Initialize the motor parameter identifier.
 * @param id Pointer to the identifier state structure.
 * @param cfg Pointer to the identification configuration.
 */
void mc_identify_init(mc_identify_t *id, const mc_identify_cfg_t *cfg);

/**
 * @brief Start the identification process.
 * @param id Pointer to the identifier state structure.
 */
void mc_identify_start(mc_identify_t *id);

/**
 * @brief Check if the identification process has completed.
 * @param id Pointer to the identifier state structure.
 * @return True if identification is complete, false otherwise.
 */
mc_bool_t mc_identify_is_done(const mc_identify_t *id);

/**
 * @brief Check if the identification process is currently active.
 * @param id Pointer to the identifier state structure.
 * @return True if identification is running, false otherwise.
 */
mc_bool_t mc_identify_is_active(const mc_identify_t *id);

/**
 * @brief Run one cycle of the identification state machine.
 * @param id Pointer to the identifier state structure.
 * @param in Input measurement data for this cycle.
 * @param out Output command data for this cycle.
 * @return MC_STATUS_OK on success, or an error code.
 */
mc_status_t mc_identify_run(mc_identify_t *id, const mc_identify_input_t *in, mc_identify_output_t *out);

/**
 * @brief Get the identification results.
 * @param id Pointer to the identifier state structure.
 * @param rs_ohm Output pointer for stator resistance [Ohm].
 * @param ld_h Output pointer for D-axis inductance [H].
 * @param lq_h Output pointer for Q-axis inductance [H].
 * @param flux_wb Output pointer for motor flux [Wb].
 * @note `rs_ohm`, `ld_h`, and `lq_h` return `0.0F` when the corresponding estimate is invalid or non-physical.
 * @note `ld_h`, `lq_h`, and `flux_wb` reflect time-weighted windows over the actual discrete injected duration.
 * @note Raw measured candidates may remain internal-only when trust gates are not met; `rs_ohm`, `ld_h`, `lq_h`, and `flux_wb` still return only trusted values or seeded fallbacks.
 * @note `flux_wb` may remain at its seeded value when the flux estimation window produces no valid samples.
 */
void mc_identify_get_result(const mc_identify_t *id, mc_f32_t *rs_ohm, mc_f32_t *ld_h, mc_f32_t *lq_h, mc_f32_t *flux_wb);

#endif /* MC_IDENTIFY_H */
