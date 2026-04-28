/** @file mc_reconstruct_1shunt.c @brief 1-shunt phase current reconstruction */

#include "mc_constants.h"
#include "mc_math.h"
#include "mc_reconstruct_1shunt.h"

/**
 * @brief Finalize single shunt sampling window parameters
 * @param phase Sample phase (leading, trailing, or none)
 * @param pwm_period_s PWM period in seconds
 * @param sample_position Normalized sample position within PWM cycle
 * @param half_cycle [out] Half cycle type
 * @param half_position [out] Normalized position within half cycle
 * @param sample_time [out] Absolute sample time in seconds
 * @param half_time [out] Absolute half cycle time in seconds
 */
static void mc_reconstruct_1shunt_finalize_sample(mc_1shunt_sample_phase_t phase,
                                                   mc_f32_t pwm_period_s,
                                                   mc_f32_t sample_position,
                                                   mc_1shunt_half_cycle_t *half_cycle,
                                                   mc_f32_t *half_position,
                                                   mc_f32_t *sample_time,
                                                   mc_f32_t *half_time)
{
    mc_f32_t half_period_s;

    if ((half_cycle == NULL) || (half_position == NULL) || (sample_time == NULL) || (half_time == NULL))
    {
        return;
    }

    half_period_s = MC_1SHUNT_HALF * pwm_period_s;
    *sample_time = sample_position * pwm_period_s;

    if (phase == MC_1SHUNT_SAMPLE_PHASE_TRAILING)
    {
        *half_cycle = MC_1SHUNT_HALF_CYCLE_DOWN;
        *half_position = mc_math_clamp_f32((sample_position - MC_1SHUNT_HALF) * 2.0F, 0.0F, 1.0F);
    }
    else if (phase == MC_1SHUNT_SAMPLE_PHASE_LEADING)
    {
        *half_cycle = MC_1SHUNT_HALF_CYCLE_UP;
        *half_position = mc_math_clamp_f32(sample_position * 2.0F, 0.0F, 1.0F);
    }
    else
    {
        *half_cycle = MC_1SHUNT_HALF_CYCLE_NONE;
        *half_position = 0.0F;
    }

    *half_time = (*half_position) * half_period_s;
}

/**
 * @brief Plan 1-shunt sampling windows for the next PWM cycle
 * @param pwm_cmd PWM command with duty cycles and sector
 * @param cfg 1-shunt configuration parameters
 * @param meta [out] Metadata describing sampling windows
 */
void mc_reconstruct_1shunt_plan(const mc_pwm_cmd_t *pwm_cmd,
                                const mc_1shunt_cfg_t *cfg,
                                mc_1shunt_meta_t *meta)
{
    mc_f32_t duty_x;
    mc_f32_t duty_y;
    mc_f32_t active_duty_min;
    mc_f32_t threshold;
    mc_f32_t margin_half;
    mc_f32_t duty_low;
    mc_f32_t duty_high;
    mc_f32_t timing_guard;
    mc_f32_t pwm_period_s;

    if ((pwm_cmd == NULL) || (cfg == NULL) || (meta == NULL))
    {
        return;
    }

    *meta = (mc_1shunt_meta_t){0};
    meta->sector = pwm_cmd->sector;
    pwm_period_s = cfg->pwm_period_s;
    if (pwm_period_s <= 0.0F)
    {
        pwm_period_s = 1.0F;
    }

    switch (pwm_cmd->sector)
    {
        case 1U:
            duty_x = pwm_cmd->duty_a;
            duty_y = pwm_cmd->duty_b;
            break;

        case 2U:
            duty_x = pwm_cmd->duty_a;
            duty_y = pwm_cmd->duty_c;
            break;

        case 3U:
            duty_x = pwm_cmd->duty_b;
            duty_y = pwm_cmd->duty_c;
            break;

        case 4U:
            duty_x = pwm_cmd->duty_b;
            duty_y = pwm_cmd->duty_a;
            break;

        case 5U:
            duty_x = pwm_cmd->duty_c;
            duty_y = pwm_cmd->duty_a;
            break;

        case 6U:
            duty_x = pwm_cmd->duty_c;
            duty_y = pwm_cmd->duty_b;
            break;

        default:
            duty_x = 0.0F;
            duty_y = 0.0F;
            break;
    }

    meta->sample_window_a = duty_x;
    meta->sample_window_b = duty_y;
    threshold = cfg->min_active_duty + cfg->sample_window_margin;
    active_duty_min = (duty_x < duty_y) ? duty_x : duty_y;
    meta->sample_valid = (active_duty_min > threshold) ? MC_TRUE : MC_FALSE;
    meta->sample_count = 0U;
    if (duty_x > threshold)
    {
        meta->sample_count += 1U;
    }
    if (duty_y > threshold)
    {
        meta->sample_count += 1U;
    }
    meta->compensation_required = (meta->sample_count < MC_1SHUNT_MIN_SAMPLE_COUNT) ? MC_TRUE : MC_FALSE;
    meta->zero_vector_bias_high = (pwm_cmd->duty_a + pwm_cmd->duty_b + pwm_cmd->duty_c) < MC_1SHUNT_ZERO_VECTOR_DUTY_SUM ? MC_TRUE : MC_FALSE;
    meta->preferred_window = (duty_x >= duty_y) ? MC_1SHUNT_PREFERRED_WINDOW_HIGH : MC_1SHUNT_PREFERRED_WINDOW_LOW;
    meta->reorder_required = (meta->sample_count < MC_1SHUNT_MIN_SAMPLE_COUNT) ? MC_TRUE : MC_FALSE;
    timing_guard = cfg->sample_window_margin + ((cfg->deadtime_s + (MC_1SHUNT_HALF * cfg->adc_aperture_s)) / pwm_period_s);
    margin_half = MC_1SHUNT_HALF * timing_guard;
    duty_low = cfg->min_active_duty + margin_half;
    duty_high = 1.0F - duty_low;
    meta->sample_phase_a = MC_1SHUNT_SAMPLE_PHASE_NONE;
    meta->sample_phase_b = MC_1SHUNT_SAMPLE_PHASE_NONE;
    meta->sample_half_a = MC_1SHUNT_HALF_CYCLE_NONE;
    meta->sample_half_b = MC_1SHUNT_HALF_CYCLE_NONE;
    meta->sample_position_a = 0.0F;
    meta->sample_position_b = 0.0F;
    meta->sample_half_position_a = 0.0F;
    meta->sample_half_position_b = 0.0F;
    meta->sample_time_a = 0.0F;
    meta->sample_time_b = 0.0F;
    meta->sample_half_time_a = 0.0F;
    meta->sample_half_time_b = 0.0F;

    if (duty_x > timing_guard)
    {
        meta->sample_phase_a = (duty_x >= MC_1SHUNT_MIDPOINT) ? MC_1SHUNT_SAMPLE_PHASE_TRAILING : MC_1SHUNT_SAMPLE_PHASE_LEADING;
        meta->sample_position_a = (meta->sample_phase_a == MC_1SHUNT_SAMPLE_PHASE_LEADING) ?
                                  duty_low :
                                  (duty_x - margin_half);
    }

    if (duty_y > timing_guard)
    {
        meta->sample_phase_b = (duty_y >= MC_1SHUNT_MIDPOINT) ? MC_1SHUNT_SAMPLE_PHASE_TRAILING : MC_1SHUNT_SAMPLE_PHASE_LEADING;
        meta->sample_position_b = (meta->sample_phase_b == MC_1SHUNT_SAMPLE_PHASE_LEADING) ?
                                  duty_low :
                                  (duty_y - margin_half);
    }

    if (meta->reorder_required != MC_FALSE)
    {
        meta->zero_vector_bias_high = (meta->preferred_window == MC_1SHUNT_PREFERRED_WINDOW_HIGH) ? MC_TRUE : MC_FALSE;
        if (meta->preferred_window == MC_1SHUNT_PREFERRED_WINDOW_HIGH)
        {
            meta->sample_phase_a = MC_1SHUNT_SAMPLE_PHASE_TRAILING;
            meta->sample_phase_b = MC_1SHUNT_SAMPLE_PHASE_TRAILING;
            meta->sample_position_a = mc_math_clamp_f32(duty_x - margin_half, duty_low, duty_high);
            meta->sample_position_b = mc_math_clamp_f32(meta->sample_position_a + timing_guard, duty_low, duty_high);
        }
        else
        {
            meta->sample_phase_a = MC_1SHUNT_SAMPLE_PHASE_LEADING;
            meta->sample_phase_b = MC_1SHUNT_SAMPLE_PHASE_LEADING;
            meta->sample_position_b = duty_low;
            meta->sample_position_a = mc_math_clamp_f32(meta->sample_position_b + timing_guard, duty_low, duty_high);
        }
    }

    meta->sample_position_a = mc_math_clamp_f32(meta->sample_position_a, 0.0F, 1.0F);
    meta->sample_position_b = mc_math_clamp_f32(meta->sample_position_b, 0.0F, 1.0F);
    mc_reconstruct_1shunt_finalize_sample(meta->sample_phase_a,
                                          pwm_period_s,
                                          meta->sample_position_a,
                                          &meta->sample_half_a,
                                          &meta->sample_half_position_a,
                                          &meta->sample_time_a,
                                          &meta->sample_half_time_a);
    mc_reconstruct_1shunt_finalize_sample(meta->sample_phase_b,
                                          pwm_period_s,
                                          meta->sample_position_b,
                                          &meta->sample_half_b,
                                          &meta->sample_half_position_b,
                                          &meta->sample_time_b,
                                          &meta->sample_half_time_b);
}

/**
 * @brief Reconstruct phase currents from single shunt ADC sample
 * @param raw Raw ADC sample data
 * @param cfg 1-shunt configuration parameters
 * @param meta Metadata describing sampling windows
 * @param phase_current_abc [out] Reconstructed three-phase currents
 */
void mc_reconstruct_1shunt_run(const mc_adc_raw_t *raw,
                               const mc_1shunt_cfg_t *cfg,
                               const mc_1shunt_meta_t *meta,
                               mc_abc_t *phase_current_abc)
{
    mc_f32_t i_bus;

    if ((raw == NULL) || (cfg == NULL) || (meta == NULL) || (phase_current_abc == NULL))
    {
        return;
    }

    i_bus = (((mc_f32_t)raw->phase_a_raw) - cfg->offset) * cfg->scale;
    *phase_current_abc = (mc_abc_t){0.0F, 0.0F, 0.0F};

    if (meta->sample_valid == MC_FALSE)
    {
        return;
    }

    switch (meta->sector)
    {
        case 1U:
            phase_current_abc->a = i_bus;
            phase_current_abc->b = -i_bus;
            break;

        case 2U:
            phase_current_abc->a = i_bus;
            phase_current_abc->c = -i_bus;
            break;

        case 3U:
            phase_current_abc->b = i_bus;
            phase_current_abc->c = -i_bus;
            break;

        case 4U:
            phase_current_abc->b = i_bus;
            phase_current_abc->a = -i_bus;
            break;

        case 5U:
            phase_current_abc->c = i_bus;
            phase_current_abc->a = -i_bus;
            break;

        case 6U:
            phase_current_abc->c = i_bus;
            phase_current_abc->b = -i_bus;
            break;

        default:
            break;
    }
}
