#ifndef MC_DRIVE_BLDC_H
#define MC_DRIVE_BLDC_H
/** @file mc_drive_bldc.h @brief BLDC motor drive with Hall-sensor commutation */

#include "mc_port_pwm.h"
#include "mc_status.h"

/** @brief BLDC Hall-sensor drive configuration */
typedef struct
{
    mc_f32_t duty_min; /**< Minimum duty cycle */
    mc_f32_t duty_max; /**< Maximum duty cycle */
} mc_bldc_hall_cfg_t;

/** @brief BLDC Hall-sensor drive runtime state */
typedef struct
{
    mc_bldc_hall_cfg_t cfg;    /**< Configuration parameters */
    uint8_t last_hall_code;    /**< Previous Hall sensor code */
    mc_pwm_cmd_t last_pwm_cmd; /**< Previous PWM command used for sequencing */
} mc_bldc_hall_t;

/**
 * @brief Initialise a BLDC Hall-sensor drive
 * @param[out] drive Pointer to BLDC drive instance storage.
 *   Range: non-NULL pointer to writable `mc_bldc_hall_t` storage.
 * @param[in] cfg Drive configuration copied into the instance.
 *   Range: non-NULL pointer to readable `mc_bldc_hall_cfg_t` storage.
 * @retval MC_STATUS_OK Initialization completed successfully.
 * @retval MC_STATUS_INVALID_ARG `drive == NULL` or `cfg == NULL`.
 * @note This function copies the configuration and resets runtime state, but does not validate duty-range ordering.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `drive` object.
 */
mc_status_t mc_bldc_hall_init(mc_bldc_hall_t *drive, const mc_bldc_hall_cfg_t *cfg);

/**
 * @brief Run one commutation step based on Hall sensor input
 * @param[in,out] drive Pointer to BLDC drive instance.
 *   Range: non-NULL pointer to writable `mc_bldc_hall_t` storage.
 * @param[in] hall_code Current Hall sensor code.
 *   Range: valid six-step Hall codes accepted by the implementation are `1`, `5`, `4`, `6`, `2`, and `3`; other values are rejected.
 * @param[in] duty_cmd Desired duty-cycle command.
 *   Range: application-defined `mc_f32_t`; the implementation clamps it to [`drive->cfg.duty_min`, `drive->cfg.duty_max`].
 * @param[out] pwm_cmd Output PWM command for the three phases.
 *   Range: non-NULL pointer to writable `mc_pwm_cmd_t` storage.
 * @retval MC_STATUS_OK Commutation command was generated successfully.
 * @retval MC_STATUS_INVALID_ARG `drive == NULL`, `pwm_cmd == NULL`, or `hall_code` is not one of the supported six-step codes.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `drive` object and different writable `pwm_cmd` storage. Not reentrant for concurrent writes to the same `drive`.
 */
mc_status_t mc_bldc_hall_run(mc_bldc_hall_t *drive, uint8_t hall_code, mc_f32_t duty_cmd, mc_pwm_cmd_t *pwm_cmd);

#endif /* MC_DRIVE_BLDC_H */
