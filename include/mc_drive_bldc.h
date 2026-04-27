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
 * @param drive  Pointer to BLDC drive instance
 * @param cfg    Drive configuration
 * @return Operation status
 */
mc_status_t mc_bldc_hall_init(mc_bldc_hall_t *drive, const mc_bldc_hall_cfg_t *cfg);

/**
 * @brief Run one commutation step based on Hall sensor input
 * @param drive     Pointer to BLDC drive instance
 * @param hall_code Current Hall sensor code (bits 0-2)
 * @param duty_cmd  Desired duty cycle command
 * @param pwm_cmd   Output PWM command for the three phases
 * @return Operation status
 */
mc_status_t mc_bldc_hall_run(mc_bldc_hall_t *drive, uint8_t hall_code, mc_f32_t duty_cmd, mc_pwm_cmd_t *pwm_cmd);

#endif /* MC_DRIVE_BLDC_H */
