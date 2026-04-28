#ifndef MC_CONTROL_PI_H
#define MC_CONTROL_PI_H
/** @file mc_control_pi.h @brief PI controller types and API */

#include "mc_types.h"
#include "mc_status.h"

typedef struct
{
    mc_q31_t kp;
    mc_q31_t ki;
    mc_q31_t integral_min;
    mc_q31_t integral_max;
    mc_q31_t output_min;
    mc_q31_t output_max;
} mc_pi_q31_cfg_t;

typedef struct
{
    mc_pi_q31_cfg_t cfg;
    mc_q31_t integral;
    mc_q31_t output;
} mc_pi_q31_t;

/** @brief PI controller configuration */
typedef struct
{
    mc_f32_t kp;            /**< Proportional gain */
    mc_f32_t ki;            /**< Integral gain */
    mc_f32_t integral_min;  /**< Minimum integrator clamp */
    mc_f32_t integral_max;  /**< Maximum integrator clamp */
    mc_f32_t output_min;    /**< Minimum output clamp */
    mc_f32_t output_max;    /**< Maximum output clamp */
} mc_pi_cfg_t;

/** @brief PI controller instance (runtime state) */
typedef struct
{
    mc_pi_cfg_t cfg;    /**< Configuration parameters */
    mc_f32_t integral;  /**< Accumulated integral term */
    mc_f32_t output;    /**< Most recent output value */
} mc_pi_t;

/**
 * @brief Initialise a PI controller from configuration
 * @param[out] pi Pointer to PI controller instance.
 *   Range: non-NULL pointer to writable `mc_pi_t` storage.
 * @param[in] cfg Pointer to PI configuration.
 *   Range: non-NULL pointer to readable `mc_pi_cfg_t` storage.
 * @return None.
 *   Range: not applicable.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `pi` object.
 */
void mc_pi_init(mc_pi_t *pi, const mc_pi_cfg_t *cfg);

/**
 * @brief Reset PI controller state (zeroes integral and output)
 * @param[in,out] pi Pointer to PI controller instance.
 *   Range: non-NULL pointer to writable `mc_pi_t` storage.
 * @return None.
 *   Range: not applicable.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `pi` object.
 */
void mc_pi_reset(mc_pi_t *pi);

/**
 * @brief Execute one step of the PI controller
 * @param[in,out] pi Pointer to PI controller instance.
 *   Range: non-NULL pointer to writable `mc_pi_t`; the function returns `0.0F` if `pi == NULL`.
 * @param[in] error Current error signal (setpoint - feedback).
 *   Range: application-defined `mc_f32_t` control error.
 * @param[in] dt_s Sample interval in seconds.
 *   Range: application-defined `mc_f32_t`; `dt_s >= 0.0F` is recommended for physically meaningful integration.
 * @return Controller output after clamping.
 *   Range: [`pi->cfg.output_min`, `pi->cfg.output_max`] when `pi != NULL`; `0.0F` when `pi == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `pi` object. Not reentrant for concurrent writes to the same `pi`.
 */
mc_f32_t mc_pi_run(mc_pi_t *pi, mc_f32_t error, mc_f32_t dt_s);

/**
 * @brief Initialise a Q31 PI controller from configuration
 * @param[out] pi Pointer to Q31 PI controller instance.
 *   Range: non-NULL pointer to writable `mc_pi_q31_t` storage.
 * @param[in] cfg Pointer to Q31 PI configuration.
 *   Range: non-NULL pointer to readable `mc_pi_q31_cfg_t` storage.
 * @return None.
 *   Range: not applicable.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `pi` object.
 */
void mc_pi_q31_init(mc_pi_q31_t *pi, const mc_pi_q31_cfg_t *cfg);

/**
 * @brief Reset Q31 PI controller state
 * @param[in,out] pi Pointer to Q31 PI controller instance.
 *   Range: non-NULL pointer to writable `mc_pi_q31_t` storage.
 * @return None.
 *   Range: not applicable.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `pi` object.
 */
void mc_pi_q31_reset(mc_pi_q31_t *pi);

/**
 * @brief Execute one step of the Q31 PI controller
 * @param[in,out] pi Pointer to Q31 PI controller instance.
 *   Range: non-NULL pointer to writable `mc_pi_q31_t`; the function returns `0` if `pi == NULL`.
 * @param[in] error Current error signal in Q31.
 *   Range: any `mc_q31_t` in [`INT32_MIN`, `INT32_MAX`].
 * @param[in] dt_q31 Sample interval in Q31.
 *   Range: application-defined `mc_q31_t`; non-negative values are recommended.
 * @return Q31 controller output after clamping.
 *   Range: [`pi->cfg.output_min`, `pi->cfg.output_max`] when `pi != NULL`; `0` when `pi == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `pi` object. Not reentrant for concurrent writes to the same `pi`.
 */
mc_q31_t mc_pi_q31_run(mc_pi_q31_t *pi, mc_q31_t error, mc_q31_t dt_q31);

/**
 * @brief Auto-tune current-loop PI gains from identified motor parameters
 *
 * Uses pole-zero cancellation with a configurable bandwidth divider:
 *   ω_c = 2π * pwm_freq_hz / bandwidth_divider
 *   Kp_d = ω_c * Ld,   Ki_d = ω_c * Rs
 *   Kp_q = ω_c * Lq,   Ki_q = ω_c * Rs
 *
 * Typical bandwidth_divider values:
 *   10 → aggressive (fast response, more noise)
 *   20 → balanced (recommended for most PMSM drives)
 *   40 → conservative (slower response, more noise margin)
 *
 * @param[in] rs_ohm  Stator resistance [Ω]. Range: > 0.
 * @param[in] ld_h    d-axis inductance [H]. Range: > 0.
 * @param[in] lq_h    q-axis inductance [H]. Range: > 0.
 * @param[in] pwm_freq_hz    PWM switching frequency [Hz]. Range: > 0.
 * @param[in] bandwidth_divider   ω_c = 2π·pwm_freq / divider. Range: ≥ 1.
 * @param[in] voltage_limit      Max phase voltage [V]. Range: > 0.
 * @param[in] current_limit      Max phase current [A]. Range: ≥ 0.
 * @param[out] id_pi_cfg  d-axis PI configuration output
 * @param[out] iq_pi_cfg  q-axis PI configuration output
 * @retval MC_STATUS_OK            PI gains computed successfully.
 * @retval MC_STATUS_INVALID_ARG   Any input pointer is NULL or parameter out of valid range.
 * @par Sync/Async Synchronous.
 * @par Reentrancy Reentrant when each call uses different output storage.
 */
mc_status_t mc_auto_tune_current_pi(mc_f32_t rs_ohm,
                                     mc_f32_t ld_h,
                                     mc_f32_t lq_h,
                                     uint32_t pwm_freq_hz,
                                     mc_f32_t bandwidth_divider,
                                     mc_f32_t voltage_limit,
                                     mc_f32_t current_limit,
                                     mc_pi_cfg_t *id_pi_cfg,
                                     mc_pi_cfg_t *iq_pi_cfg);

/**
 * @brief Auto-tune speed-loop PI gain from identified motor parameters
 *
 * Uses a heuristic based on the current-loop bandwidth:
 *   ω_s = ω_c / speed_ratio   (default speed_ratio = 10)
 *   Kp = damping * ω_s * J / Kt  (simplified: Kp ≈ ω_s * Ld * 0.5)
 *   Ki = ω_s * Kp / 4            (quarter-decay integration)
 *
 * Note: without inertia (J) and torque constant (Kt), the speed-loop
 * auto-tune is a heuristic approximation. Manual tuning of Kp/Ki based
 * on actual load dynamics is recommended for production.
 *
 * @param[in] ld_h       d-axis inductance [H] used as inertia proxy.
 * @param[in] pwm_freq_hz  PWM frequency [Hz] used to derive ω_c.
 * @param[in] bandwidth_divider Current-loop bandwidth divider.
 * @param[in] speed_ratio   ω_s = ω_c / speed_ratio. Default 10.
 * @param[in] iq_limit      q-axis current limit [A] used as output max.
 * @param[out] speed_pi_cfg Speed PI configuration output.
 * @retval MC_STATUS_OK            PI gains computed.
 * @retval MC_STATUS_INVALID_ARG   NULL pointer or invalid parameters.
 * @par Sync/Async Synchronous.
 * @par Reentrancy Reentrant.
 */
mc_status_t mc_auto_tune_speed_pi(mc_f32_t ld_h,
                                   uint32_t pwm_freq_hz,
                                   mc_f32_t bandwidth_divider,
                                   mc_f32_t speed_ratio,
                                   mc_f32_t iq_limit,
                                   mc_pi_cfg_t *speed_pi_cfg);

#endif /* MC_CONTROL_PI_H */
