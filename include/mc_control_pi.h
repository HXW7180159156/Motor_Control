#ifndef MC_CONTROL_PI_H
#define MC_CONTROL_PI_H
/** @file mc_control_pi.h @brief PI controller types and API */

#include "mc_types.h"

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

#endif /* MC_CONTROL_PI_H */
