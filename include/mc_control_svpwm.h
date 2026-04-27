#ifndef MC_CONTROL_SVPWM_H
#define MC_CONTROL_SVPWM_H
/** @file mc_control_svpwm.h @brief Space-vector PWM modulation types and API */

#include "mc_port_pwm.h"
#include "mc_transform.h"

/** @brief SVPWM modulator configuration */
typedef struct
{
    mc_f32_t duty_min;         /**< Minimum duty-cycle clamp */
    mc_f32_t duty_max;         /**< Maximum duty-cycle clamp */
    mc_f32_t modulation_limit; /**< Modulation index limit (typically sqrt(3)/2) */
} mc_svpwm_cfg_t;

/**
 * @brief Run SVPWM modulation to generate 3-phase PWM commands
 * @param[in] voltage_ab Alpha-beta voltage vector.
 *   Range: non-NULL pointer to readable `mc_alphabeta_t`; finite components are recommended.
 * @param[in] cfg SVPWM configuration.
 *   Range: non-NULL pointer to readable `mc_svpwm_cfg_t`; intended `duty_min <= duty_max` and `modulation_limit >= 0.0F`.
 * @param[in,out] pwm_cmd Output PWM command (3-phase duty cycles).
 *   Range: non-NULL pointer to writable `mc_pwm_cmd_t` storage.
 * @return None.
 *   Range: not applicable.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when concurrent calls do not share writable `pwm_cmd` storage.
 */
void mc_svpwm_run(const mc_alphabeta_t *voltage_ab, const mc_svpwm_cfg_t *cfg, mc_pwm_cmd_t *pwm_cmd);

/**
 * @brief Run SVPWM from a Q31 alpha-beta vector
 * @param[in] voltage_ab_q31 Alpha-beta voltage vector in Q31.
 *   Range: non-NULL pointer to readable `mc_alphabeta_q31_t` storage.
 * @param[in] cfg SVPWM configuration.
 *   Range: non-NULL pointer to readable `mc_svpwm_cfg_t`; intended `duty_min <= duty_max` and `modulation_limit >= 0.0F`.
 * @param[in,out] pwm_cmd Output PWM command.
 *   Range: non-NULL pointer to writable `mc_pwm_cmd_t` storage.
 * @return None.
 *   Range: not applicable.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when concurrent calls do not share writable `pwm_cmd` storage.
 */
void mc_svpwm_q31_run(const mc_alphabeta_q31_t *voltage_ab_q31, const mc_svpwm_cfg_t *cfg, mc_pwm_cmd_t *pwm_cmd);

#endif /* MC_CONTROL_SVPWM_H */
