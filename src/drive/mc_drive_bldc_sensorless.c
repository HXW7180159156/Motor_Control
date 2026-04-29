/** @file mc_drive_bldc_sensorless.c @brief BLDC sensorless drive using BEMF zero-crossing detection */

#include "mc_constants.h"
#include "mc_drive_bldc_sensorless.h"
#include "mc_control_pi.h"
#include "mc_math.h"
#include <math.h>

/**
 * @brief Reset runtime-only state while preserving configuration
 * @param ss BLDC sensorless drive instance to reset in place
 */
static void mc_bldc_ss_reset_runtime(mc_bldc_sensorless_t *ss)
{
    if (ss == NULL)
    {
        return;
    }

    ss->phase = MC_BLDC_SENSORLESS_IDLE;
    ss->commutation_step = 0U;
    ss->elec_freq_hz = 0.0F;
    ss->mech_speed_rpm = 0.0F;
    ss->duty_cmd = 0.0F;
    ss->align_timer_s = 0.0F;
    ss->step_timer_s = 0.0F;
    ss->ramp_timer_s = 0.0F;
    ss->zc_detected = MC_FALSE;
    ss->zc_timer_s = 0.0F;
    ss->last_zc_period_s = (ss->cfg.ramp_start_freq_hz > 0.0F) ? (1.0F / ss->cfg.ramp_start_freq_hz) : 0.0F;
    ss->commutation_delay_s = 0.0F;
    ss->zc_debounce_cnt = 0U;
    ss->last_zc_voltage_v = 0.0F;
    ss->last_pwm_cmd = (mc_pwm_cmd_t){0};
    mc_pi_reset(&ss->speed_pi);
}

/**
 * @brief Apply 6-step commutation by step index
 * @param step     Commutation step (0-5)
 * @param duty_cmd Duty cycle command
 * @param pwm_cmd  Output PWM command
 * @return MC_STATUS_OK or MC_STATUS_INVALID_ARG
 */
static mc_status_t mc_bldc_ss_apply_step(uint8_t step, mc_f32_t duty_cmd, mc_pwm_cmd_t *pwm_cmd)
{
    if (pwm_cmd == NULL)
    {
        return MC_STATUS_INVALID_ARG;
    }
    if (step > 5U)
    {
        pwm_cmd->valid = 0U;
        return MC_STATUS_INVALID_ARG;
    }

    *pwm_cmd = (mc_pwm_cmd_t){0};
    pwm_cmd->valid = 1U;
    pwm_cmd->sector = step + 1U;

    switch (step)
    {
        case 0U:
            pwm_cmd->duty_a = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_PWM;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_LOW;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_OFF;
            break;

        case 1U:
            pwm_cmd->duty_a = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_PWM;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_OFF;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_LOW;
            break;

        case 2U:
            pwm_cmd->duty_b = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_OFF;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_PWM;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_LOW;
            break;

        case 3U:
            pwm_cmd->duty_b = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_LOW;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_PWM;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_OFF;
            break;

        case 4U:
            pwm_cmd->duty_c = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_LOW;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_OFF;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_PWM;
            break;

        case 5U:
            pwm_cmd->duty_c = duty_cmd;
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_OFF;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_LOW;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_PWM;
            break;

        default:
            pwm_cmd->valid = 0U;
            break;
    }

    return MC_STATUS_OK;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Return a baseline BLDC sensorless configuration
 * @return Default BLDC sensorless configuration values
 */
mc_bldc_sensorless_cfg_t mc_bldc_sensorless_cfg_default(void)
{
    mc_bldc_sensorless_cfg_t cfg;

    cfg.align_duty = MC_BLDC_SS_DEFAULT_ALIGN_DUTY;
    cfg.align_time_s = MC_BLDC_SS_DEFAULT_ALIGN_TIME_S;
    cfg.ramp_start_freq_hz = MC_BLDC_SS_DEFAULT_RAMP_START_FREQ_HZ;
    cfg.ramp_end_freq_hz = MC_BLDC_SS_DEFAULT_RAMP_END_FREQ_HZ;
    cfg.ramp_time_s = MC_BLDC_SS_DEFAULT_RAMP_TIME_S;
    cfg.ramp_start_duty = MC_BLDC_SS_DEFAULT_RAMP_START_DUTY;
    cfg.ramp_end_duty = MC_BLDC_SS_DEFAULT_RAMP_END_DUTY;
    cfg.bemf_threshold_v = MC_BLDC_SS_DEFAULT_BEMF_THRESHOLD_V;
    cfg.advance_angle_deg = MC_BLDC_SS_DEFAULT_ADVANCE_ANGLE_DEG;
    cfg.zc_debounce_threshold = 0U;
    cfg.speed_pi_cfg = (mc_pi_cfg_t){MC_BLDC_SS_DEFAULT_SPEED_KP, MC_BLDC_SS_DEFAULT_SPEED_KI,
        MC_BLDC_SS_DEFAULT_SPEED_INT_MIN, MC_BLDC_SS_DEFAULT_SPEED_INT_MAX,
        MC_BLDC_SS_DEFAULT_SPEED_OUT_MIN, MC_BLDC_SS_DEFAULT_SPEED_OUT_MAX};

    return cfg;
}

/**
 * @brief Initialise BLDC sensorless drive runtime and configuration
 * @param ss Pointer to BLDC sensorless drive instance
 * @param cfg Pointer to BLDC sensorless configuration
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input or configuration
 */
mc_status_t mc_bldc_sensorless_init(mc_bldc_sensorless_t *ss, const mc_bldc_sensorless_cfg_t *cfg)
{
    if ((ss == NULL) || (cfg == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    if ((cfg->align_time_s <= 0.0F) ||
        (cfg->ramp_start_freq_hz <= 0.0F) ||
        (cfg->ramp_end_freq_hz < cfg->ramp_start_freq_hz) ||
        (cfg->ramp_time_s <= 0.0F) ||
        (cfg->ramp_start_duty < 0.0F) || (cfg->ramp_start_duty > 1.0F) ||
        (cfg->ramp_end_duty < cfg->ramp_start_duty) || (cfg->ramp_end_duty > 1.0F))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *ss = (mc_bldc_sensorless_t){0};
    ss->cfg = *cfg;
    mc_pi_init(&ss->speed_pi, &cfg->speed_pi_cfg);
    mc_bldc_ss_reset_runtime(ss);

    return MC_STATUS_OK;
}

/**
 * @brief Start the BLDC sensorless startup sequence from ALIGN
 * @param ss Pointer to BLDC sensorless drive instance
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if `ss` is NULL, or MC_STATUS_INVALID_STATE if the stored startup configuration is invalid
 */
mc_status_t mc_bldc_sensorless_start(mc_bldc_sensorless_t *ss)
{
    if (ss == NULL)
    {
        return MC_STATUS_INVALID_ARG;
    }

    if (ss->cfg.ramp_start_freq_hz <= 0.0F)
    {
        return MC_STATUS_INVALID_STATE;
    }

    mc_bldc_ss_reset_runtime(ss);
    ss->phase = MC_BLDC_SENSORLESS_ALIGN;
    ss->duty_cmd = ss->cfg.align_duty;

    return MC_STATUS_OK;
}

/**
 * @brief Reset BLDC sensorless runtime state to IDLE while preserving configuration
 * @param ss Pointer to BLDC sensorless drive instance
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if `ss` is NULL
 */
mc_status_t mc_bldc_sensorless_reset(mc_bldc_sensorless_t *ss)
{
    if (ss == NULL)
    {
        return MC_STATUS_INVALID_ARG;
    }

    mc_bldc_ss_reset_runtime(ss);

    return MC_STATUS_OK;
}

/**
 * @brief Return the floating phase index for a six-step commutation state
 * @param step Commutation step in the range [0, 5]
 * @return Phase index (0=A, 1=B, 2=C), or `0xFF` when `step` is invalid
 */
uint8_t mc_bldc_sensorless_floating_phase(uint8_t step)
{
    static const uint8_t floating_phase_map[6] = {2U, 1U, 0U, 2U, 1U, 0U};
    if (step > 5U) { return MC_BLDC_SS_INVALID_STEP; }
    return floating_phase_map[step];
}

/**
 * @brief Advance to the next commutation step
 * @param ss BLDC sensorless drive instance to update in place
 */
static void mc_bldc_ss_advance_step(mc_bldc_sensorless_t *ss)
{
    ss->commutation_step = (ss->commutation_step + 1U) % MC_HALL_STEPS;
    ss->step_timer_s = 0.0F;
    ss->zc_detected = MC_FALSE;
    ss->zc_timer_s = 0.0F;
    ss->zc_debounce_cnt = 0U;
    ss->commutation_delay_s = ss->last_zc_period_s * (ss->cfg.advance_angle_deg / 60.0F);
}

/**
 * @brief Detect BEMF zero-crossing on the floating phase with debounce
 * @param ss Pointer to sensorless drive instance (updated with debounce count)
 * @param floating_phase_voltage_v Floating phase voltage sample in volts
 * @param bus_voltage_v DC bus voltage sample in volts
 * @return MC_TRUE if a valid (debounced) zero-crossing is detected
 */
static mc_bool_t mc_bldc_ss_detect_zc(mc_bldc_sensorless_t *ss,
    mc_f32_t floating_phase_voltage_v, mc_f32_t bus_voltage_v)
{
    mc_f32_t threshold = bus_voltage_v * MC_1SHUNT_HALF;
    mc_f32_t diff = floating_phase_voltage_v - threshold;
    mc_f32_t abs_diff = fabsf(diff);
    mc_bool_t crossing;

    if (abs_diff < ss->cfg.bemf_threshold_v)
    {
        ss->zc_debounce_cnt = 0U;
        return MC_FALSE;
    }

    if ((ss->commutation_step % 2U) == 0U)
    {
        crossing = (diff > 0.0F) ? MC_TRUE : MC_FALSE;
    }
    else
    {
        crossing = (diff < 0.0F) ? MC_TRUE : MC_FALSE;
    }

    if (ss->cfg.zc_debounce_threshold == 0U)
    {
        return crossing;
    }

    if (crossing)
    {
        if (ss->zc_debounce_cnt < ss->cfg.zc_debounce_threshold)
        {
            ss->zc_debounce_cnt++;
        }
    }
    else
    {
        ss->zc_debounce_cnt = 0U;
    }

    return (ss->zc_debounce_cnt >= ss->cfg.zc_debounce_threshold) ? MC_TRUE : MC_FALSE;
}

/**
 * @brief Execute one BLDC sensorless control cycle
 * @param ss Pointer to BLDC sensorless drive instance
 * @param dt_s Control-loop step time in seconds
 * @param floating_phase_voltage_v Floating phase voltage sample in volts
 * @param bus_voltage_v DC bus voltage sample in volts
 * @param pwm_cmd Output PWM command for the next inverter update
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on invalid input or internal invalid commutation state
 */
mc_status_t mc_bldc_sensorless_run(mc_bldc_sensorless_t *ss,
    mc_f32_t dt_s, mc_f32_t floating_phase_voltage_v, mc_f32_t bus_voltage_v,
    mc_pwm_cmd_t *pwm_cmd)
{
    mc_f32_t duty = 0.0F;
    mc_f32_t freq = 0.0F;

    if ((ss == NULL) || (pwm_cmd == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    switch (ss->phase)
    {
        case MC_BLDC_SENSORLESS_IDLE:
            *pwm_cmd = (mc_pwm_cmd_t){0};
            return MC_STATUS_OK;

        case MC_BLDC_SENSORLESS_ALIGN:
            ss->align_timer_s += dt_s;
            if (mc_bldc_ss_apply_step(ss->commutation_step, ss->cfg.align_duty, pwm_cmd) != MC_STATUS_OK)
            {
                return MC_STATUS_INVALID_ARG;
            }
            ss->duty_cmd = ss->cfg.align_duty;

            if (ss->align_timer_s >= ss->cfg.align_time_s)
            {
                ss->phase = MC_BLDC_SENSORLESS_RAMP_OPEN;
                ss->ramp_timer_s = 0.0F;
                ss->step_timer_s = 0.0F;
                ss->last_zc_period_s = 1.0F / ss->cfg.ramp_start_freq_hz;
            }
            break;

        case MC_BLDC_SENSORLESS_RAMP_OPEN:
            ss->ramp_timer_s += dt_s;

            {
                mc_f32_t ramp_progress = (ss->cfg.ramp_time_s > 0.0F) ?
                    (ss->ramp_timer_s / ss->cfg.ramp_time_s) : 1.0F;
                freq = ss->cfg.ramp_start_freq_hz +
                    (ss->cfg.ramp_end_freq_hz - ss->cfg.ramp_start_freq_hz) * ramp_progress;
                duty = ss->cfg.ramp_start_duty +
                    (ss->cfg.ramp_end_duty - ss->cfg.ramp_start_duty) * ramp_progress;
            }

            ss->elec_freq_hz = freq;
            ss->duty_cmd = duty;

            if (mc_bldc_ss_apply_step(ss->commutation_step, duty, pwm_cmd) != MC_STATUS_OK)
            {
                return MC_STATUS_INVALID_ARG;
            }

            ss->step_timer_s += dt_s;

            if (ss->step_timer_s >= (1.0F / (freq * (mc_f32_t)MC_HALL_STEPS)))
            {
                mc_bldc_ss_advance_step(ss);
            }

            if (freq >= ss->cfg.ramp_end_freq_hz)
            {
                ss->phase = MC_BLDC_SENSORLESS_RUN;
                ss->commutation_delay_s = ss->last_zc_period_s * (ss->cfg.advance_angle_deg / MC_DEG_PER_COMM_STEP);
            }
            break;

        case MC_BLDC_SENSORLESS_RUN:
            ss->step_timer_s += dt_s;

            if (!ss->zc_detected)
            {
                if (mc_bldc_ss_detect_zc(ss, floating_phase_voltage_v, bus_voltage_v))
                {
                    ss->zc_detected = MC_TRUE;
                    ss->zc_timer_s = 0.0F;
                    ss->last_zc_period_s = ss->step_timer_s;
                    ss->last_zc_voltage_v = floating_phase_voltage_v;
                    ss->commutation_delay_s = ss->last_zc_period_s * (ss->cfg.advance_angle_deg / MC_DEG_PER_COMM_STEP);
                    ss->elec_freq_hz = 1.0F / ss->last_zc_period_s;
                    ss->mech_speed_rpm = ss->elec_freq_hz * MC_SEC_PER_MIN;
            }
            freq = mc_math_clamp_f32(freq, ss->cfg.ramp_start_freq_hz, ss->cfg.ramp_end_freq_hz);
            duty = mc_math_clamp_f32(duty, ss->cfg.ramp_start_duty, ss->cfg.ramp_end_duty);
            }
            else
            {
                ss->zc_timer_s += dt_s;
            }

            if ((ss->zc_detected && (ss->zc_timer_s >= ss->commutation_delay_s)) ||
                (!ss->zc_detected && (ss->step_timer_s >= ss->last_zc_period_s * MC_BLDC_SS_FALLBACK_ZC_TIMEOUT_RATIO)))
            {
                mc_bldc_ss_advance_step(ss);
            }

            if (mc_bldc_ss_apply_step(ss->commutation_step, ss->duty_cmd, pwm_cmd) != MC_STATUS_OK)
            {
                return MC_STATUS_INVALID_ARG;
            }
            break;

        default:
            break;
    }

    ss->last_pwm_cmd = *pwm_cmd;
    return MC_STATUS_OK;
}

/**
 * @brief Return the latest mechanical speed estimate in RPM
 * @param ss Pointer to BLDC sensorless drive instance
 * @return Stored speed estimate, or 0.0F when `ss` is NULL
 */
mc_f32_t mc_bldc_sensorless_get_speed_rpm(const mc_bldc_sensorless_t *ss)
{
    if (ss == NULL) { return 0.0F; }
    return ss->mech_speed_rpm;
}

/**
 * @brief Execute the BLDC sensorless speed PI update
 * @param ss Pointer to BLDC sensorless drive instance
 * @param speed_ref_rpm Target mechanical speed in RPM
 * @param dt_s Speed-loop step time in seconds
 */
void mc_bldc_sensorless_speed_step(mc_bldc_sensorless_t *ss, mc_f32_t speed_ref_rpm, mc_f32_t dt_s)
{
    mc_f32_t error;
    mc_f32_t duty_pi;

    if ((ss == NULL) || (dt_s <= 0.0F))
    {
        return;
    }

    if (ss->phase != MC_BLDC_SENSORLESS_RUN)
    {
        mc_pi_reset(&ss->speed_pi);
        return;
    }

    error = speed_ref_rpm - ss->mech_speed_rpm;
    duty_pi = mc_pi_run(&ss->speed_pi, error, dt_s);
    ss->duty_cmd = mc_math_clamp_f32(duty_pi, ss->cfg.ramp_start_duty, MC_BLDC_SS_DUTY_MAX_CLAMP);
}
