/** @file mc_drive_bldc.c @brief BLDC motor drive implementation using Hall sensor feedback */

#include "mc_drive_bldc.h"

/**
 * @brief Clamp a value within a specified range
 * @param value Input value to clamp
 * @param min_value Minimum allowable value
 * @param max_value Maximum allowable value
 * @return Clamped value within [min_value, max_value]
 */
static mc_f32_t mc_bldc_clamp(mc_f32_t value, mc_f32_t min_value, mc_f32_t max_value)
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
 * @brief Apply BLDC commutation based on Hall sensor code
 * @param hall_code Hall effect sensor code (1-6)
 * @param duty_cmd Duty cycle command value
 * @param pwm_cmd Output PWM command structure
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
static mc_status_t mc_bldc_apply_commutation(uint8_t hall_code, mc_f32_t duty_cmd, mc_pwm_cmd_t *pwm_cmd)
{
    if (pwm_cmd == NULL)
    {
        return MC_STATUS_INVALID_ARG;
    }

    *pwm_cmd = (mc_pwm_cmd_t){0};
    pwm_cmd->valid = 1U;

    switch (hall_code)
    {
        case 1U:
            pwm_cmd->duty_a = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_PWM;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_LOW;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_OFF;
            pwm_cmd->sector = 1U;
            break;

        case 5U:
            pwm_cmd->duty_a = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_PWM;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_OFF;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_LOW;
            pwm_cmd->sector = 2U;
            break;

        case 4U:
            pwm_cmd->duty_b = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_OFF;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_PWM;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_LOW;
            pwm_cmd->sector = 3U;
            break;

        case 6U:
            pwm_cmd->duty_b = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_LOW;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_PWM;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_OFF;
            pwm_cmd->sector = 4U;
            break;

        case 2U:
            pwm_cmd->duty_c = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_LOW;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_OFF;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_PWM;
            pwm_cmd->sector = 5U;
            break;

        case 3U:
            pwm_cmd->duty_c = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_OFF;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_LOW;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_PWM;
            pwm_cmd->sector = 6U;
            break;

        default:
            pwm_cmd->valid = 0U;
            return MC_STATUS_INVALID_ARG;
    }

    return MC_STATUS_OK;
}

/**
 * @brief Initialize a BLDC Hall-sensor drive instance
 * @param drive Pointer to the BLDC Hall drive structure
 * @param cfg Pointer to the Hall drive configuration
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if pointers are NULL
 */
mc_status_t mc_bldc_hall_init(mc_bldc_hall_t *drive, const mc_bldc_hall_cfg_t *cfg)
{
    if ((drive == NULL) || (cfg == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *drive = (mc_bldc_hall_t){0};
    drive->cfg = *cfg;

    return MC_STATUS_OK;
}

/**
 * @brief Execute one control cycle of the BLDC Hall-sensor drive
 * @param drive Pointer to the BLDC Hall drive structure
 * @param hall_code Hall effect sensor code for the current rotor position
 * @param duty_cmd Duty cycle command value
 * @param pwm_cmd Output PWM command structure
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input
 */
mc_status_t mc_bldc_hall_run(mc_bldc_hall_t *drive, uint8_t hall_code, mc_f32_t duty_cmd, mc_pwm_cmd_t *pwm_cmd)
{
    mc_status_t status;
    mc_f32_t clamped_duty;

    if ((drive == NULL) || (pwm_cmd == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    clamped_duty = mc_bldc_clamp(duty_cmd, drive->cfg.duty_min, drive->cfg.duty_max);
    status = mc_bldc_apply_commutation(hall_code, clamped_duty, pwm_cmd);
    if (status != MC_STATUS_OK)
    {
        return status;
    }

    drive->last_hall_code = hall_code;
    drive->last_pwm_cmd = *pwm_cmd;

    return MC_STATUS_OK;
}
