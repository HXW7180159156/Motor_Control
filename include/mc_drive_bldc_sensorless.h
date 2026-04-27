#ifndef MC_DRIVE_BLDC_SENSORLESS_H
#define MC_DRIVE_BLDC_SENSORLESS_H
/** @file mc_drive_bldc_sensorless.h @brief BLDC sensorless drive using BEMF zero-crossing detection */

#include "mc_control_pi.h"
#include "mc_port_pwm.h"
#include "mc_status.h"
#include "mc_types.h"

/**
 * @brief Startup phase for BLDC sensorless drive
 */
typedef enum
{
    MC_BLDC_SENSORLESS_IDLE = 0,    /**< Idle, not running */
    MC_BLDC_SENSORLESS_ALIGN,       /**< Rotor alignment stage */
    MC_BLDC_SENSORLESS_RAMP_OPEN,   /**< Open-loop frequency ramp */
    MC_BLDC_SENSORLESS_RUN          /**< Closed-loop BEMF detection */
} mc_bldc_sensorless_phase_t;

/**
 * @brief BLDC sensorless drive configuration
 */
typedef struct
{
    mc_f32_t align_duty;                /**< Duty cycle during rotor alignment [0,1] */
    mc_f32_t align_time_s;              /**< Alignment duration in seconds */
    mc_f32_t ramp_start_freq_hz;        /**< Starting electrical frequency for open-loop ramp */
    mc_f32_t ramp_end_freq_hz;          /**< Electrical frequency to transition at */
    mc_f32_t ramp_time_s;               /**< Duration of the open-loop ramp in seconds */
    mc_f32_t ramp_start_duty;           /**< Duty cycle at ramp start */
    mc_f32_t ramp_end_duty;             /**< Duty cycle at ramp end */
    mc_f32_t bemf_threshold_v;          /**< Minimum BEMF peak voltage for valid detection */
    mc_f32_t advance_angle_deg;         /**< Commutation advance angle in electrical degrees */
    uint8_t zc_debounce_threshold;      /**< Consecutive ZC samples required (0=disabled) */
    mc_pi_cfg_t speed_pi_cfg;           /**< Speed PI controller config for duty adjustment */
} mc_bldc_sensorless_cfg_t;

/**
 * @brief BLDC sensorless drive runtime state
 */
typedef struct
{
    mc_bldc_sensorless_cfg_t cfg;           /**< Configuration parameters */
    mc_bldc_sensorless_phase_t phase;       /**< Current startup phase */
    uint8_t commutation_step;               /**< Current 6-step sector (0-5) */
    mc_f32_t elec_freq_hz;                  /**< Current estimated electrical frequency */
    mc_f32_t mech_speed_rpm;                /**< Estimated mechanical speed */
    mc_f32_t duty_cmd;                      /**< Current duty cycle command */

    mc_f32_t align_timer_s;                 /**< Alignment phase timer */
    mc_f32_t step_timer_s;                  /**< Time since last commutation */
    mc_f32_t ramp_timer_s;                  /**< Open-loop ramp timer */

    mc_bool_t zc_detected;                  /**< BEMF zero-cross detected in current step */
    mc_f32_t zc_timer_s;                    /**< Time since ZC detection */
    mc_f32_t last_zc_period_s;              /**< Electrical period from last ZC cycle */
    mc_f32_t commutation_delay_s;           /**< 30-degree delay after ZC */

    uint8_t zc_debounce_cnt;                /**< Consecutive ZC detection counter */
    mc_f32_t last_zc_voltage_v;             /**< Floating phase voltage at last ZC */

    mc_pi_t speed_pi;                       /**< Speed PI controller instance */
    mc_pwm_cmd_t last_pwm_cmd;              /**< Most recent PWM output */
} mc_bldc_sensorless_t;

/**
 * @brief Default configuration for BLDC sensorless drive
 * @return Default BLDC sensorless configuration.
 *   Range: a fully initialized `mc_bldc_sensorless_cfg_t` with baseline startup and speed-loop defaults.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
mc_bldc_sensorless_cfg_t mc_bldc_sensorless_cfg_default(void);

/**
 * @brief Initialise BLDC sensorless drive
 * @param[out] ss Pointer to sensorless drive instance storage.
 *   Range: non-NULL pointer to writable `mc_bldc_sensorless_t` storage.
 * @param[in] cfg Configuration parameters copied into the instance.
 *   Range: non-NULL pointer to readable `mc_bldc_sensorless_cfg_t` storage with:
 *   `align_time_s > 0.0F`, `ramp_start_freq_hz > 0.0F`,
 *   `ramp_end_freq_hz >= ramp_start_freq_hz`, `ramp_time_s > 0.0F`,
 *   `ramp_start_duty` in `[0.0F, 1.0F]`, and `ramp_end_duty` in `[ramp_start_duty, 1.0F]`.
 * @retval MC_STATUS_OK Initialization completed successfully.
 * @retval MC_STATUS_INVALID_ARG `ss == NULL`, `cfg == NULL`, or any validated configuration constraint above is violated.
 * @note Other fields such as `align_duty`, `bemf_threshold_v`, `advance_angle_deg`, and `speed_pi_cfg` are copied as provided.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `ss` object.
 */
mc_status_t mc_bldc_sensorless_init(mc_bldc_sensorless_t *ss, const mc_bldc_sensorless_cfg_t *cfg);

/**
 * @brief Start BLDC sensorless drive from a clean alignment phase
 * @param[in,out] ss Pointer to sensorless drive instance.
 *   Range: non-NULL pointer to writable `mc_bldc_sensorless_t` storage.
 * @retval MC_STATUS_OK Startup state was reset and the phase was set to `MC_BLDC_SENSORLESS_ALIGN`.
 * @retval MC_STATUS_INVALID_ARG `ss == NULL`.
 * @retval MC_STATUS_INVALID_STATE `ss->cfg.ramp_start_freq_hz <= 0.0F`.
 * @par Sync/Async
 *   Synchronous start. The startup sequence advances across later `mc_bldc_sensorless_run()` calls.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `ss` object. Not reentrant for concurrent writes to the same `ss`.
 */
mc_status_t mc_bldc_sensorless_start(mc_bldc_sensorless_t *ss);

/**
 * @brief Reset BLDC sensorless drive to idle state
 * @param[in,out] ss Pointer to sensorless drive instance.
 *   Range: non-NULL pointer to writable `mc_bldc_sensorless_t` storage.
 * @retval MC_STATUS_OK Runtime state was reset successfully.
 * @retval MC_STATUS_INVALID_ARG `ss == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `ss` object. Not reentrant for concurrent writes to the same `ss`.
 */
mc_status_t mc_bldc_sensorless_reset(mc_bldc_sensorless_t *ss);

/**
 * @brief Run one control cycle of the BLDC sensorless drive
 * @param[in,out] ss Pointer to sensorless drive instance.
 *   Range: non-NULL pointer to writable `mc_bldc_sensorless_t` storage.
 * @param[in] dt_s Sample interval in seconds.
 *   Range: application-defined `mc_f32_t`; non-negative values are recommended.
 * @param[in] floating_phase_voltage_v Voltage sample of the currently floating phase.
 *   Range: application-defined `mc_f32_t`.
 * @param[in] bus_voltage_v DC bus voltage.
 *   Range: application-defined `mc_f32_t`; non-negative values are recommended.
 * @param[out] pwm_cmd Output PWM command for the three phases.
 *   Range: non-NULL pointer to writable `mc_pwm_cmd_t` storage.
 * @retval MC_STATUS_OK Cycle completed successfully.
 * @retval MC_STATUS_INVALID_ARG `ss == NULL`, `pwm_cmd == NULL`, or the stored commutation step becomes invalid for the internal six-step mapper.
 * @note When `ss->phase == MC_BLDC_SENSORLESS_IDLE`, the function zeroes `*pwm_cmd` and returns `MC_STATUS_OK`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `ss` object and different writable `pwm_cmd` storage. Not reentrant for concurrent writes to the same `ss`.
 */
mc_status_t mc_bldc_sensorless_run(mc_bldc_sensorless_t *ss,
    mc_f32_t dt_s, mc_f32_t floating_phase_voltage_v, mc_f32_t bus_voltage_v,
    mc_pwm_cmd_t *pwm_cmd);

/**
 * @brief Get the floating phase index for the current commutation step
 * @param[in] step Current commutation step.
 *   Range: intended `[0U, 5U]`.
 * @return Floating phase index.
 *   Range: `0U`, `1U`, or `2U` for valid six-step inputs; `0xFFU` when `step > 5U`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
uint8_t mc_bldc_sensorless_floating_phase(uint8_t step);

/**
 * @brief Get estimated mechanical speed in RPM
 * @param[in] ss Pointer to sensorless drive instance.
 *   Range: readable `mc_bldc_sensorless_t` storage or `NULL`.
 * @return Stored speed estimate in RPM.
 *   Range: `ss->mech_speed_rpm` when `ss != NULL`; otherwise `0.0F`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant for concurrent read-only calls.
 */
mc_f32_t mc_bldc_sensorless_get_speed_rpm(const mc_bldc_sensorless_t *ss);

/**
 * @brief Run speed PI control for BLDC sensorless drive
 * @param[in,out] ss Pointer to sensorless drive instance.
 *   Range: writable `mc_bldc_sensorless_t` storage or `NULL`.
 * @param[in] speed_ref_rpm Target speed reference in RPM.
 *   Range: application-defined `mc_f32_t`.
 * @param[in] dt_s Sample interval in seconds.
 *   Range: application-defined `mc_f32_t`; `dt_s > 0.0F` is required for the function to act.
 * @return None.
 *   Range: not applicable.
 * @note If `ss == NULL` or `dt_s <= 0.0F`, the call has no effect.
 * @note If the drive is not yet in `MC_BLDC_SENSORLESS_RUN`, the speed PI state is reset and no duty update is applied.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `ss` object. Not reentrant for concurrent writes to the same `ss`.
 */
void mc_bldc_sensorless_speed_step(mc_bldc_sensorless_t *ss, mc_f32_t speed_ref_rpm, mc_f32_t dt_s);

#endif /* MC_DRIVE_BLDC_SENSORLESS_H */
