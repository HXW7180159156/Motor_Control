/** @file mc_drive_pmsm.c @brief PMSM field-oriented control (FOC) drive implementation */

#include "mc_drive_pmsm.h"

#include <math.h>

/**
 * @brief Limit the voltage vector magnitude to the configured voltage limit
 * @param foc Pointer to the PMSM FOC drive structure
 * @param v_dq Voltage vector in dq-frame to limit (modified in place)
 */
static void mc_pmsm_limit_voltage(mc_pmsm_foc_t *foc, mc_dq_t *v_dq)
{
    mc_f32_t voltage_limit;
    mc_f32_t magnitude;

    if ((foc == NULL) || (v_dq == NULL))
    {
        return;
    }

    voltage_limit = foc->cfg.voltage_limit;
    if (voltage_limit <= 0.0F)
    {
        return;
    }

    magnitude = sqrtf((v_dq->d * v_dq->d) + (v_dq->q * v_dq->q));
    if (magnitude > voltage_limit)
    {
        mc_f32_t scale = voltage_limit / magnitude;
        v_dq->d *= scale;
        v_dq->q *= scale;
    }
}

/**
 * @brief Normalize the voltage vector by the effective DC bus voltage
 * @param foc Pointer to the PMSM FOC drive structure
 * @param in Pointer to the FOC input structure containing bus voltage
 * @param v_ab Voltage vector in alpha-beta frame to normalize (modified in place)
 */
static void mc_pmsm_normalize_voltage(const mc_pmsm_foc_t *foc,
                                      const mc_pmsm_foc_input_t *in,
                                      mc_alphabeta_t *v_ab)
{
    mc_f32_t effective_bus_voltage;

    if ((foc == NULL) || (in == NULL) || (v_ab == NULL))
    {
        return;
    }

    effective_bus_voltage = in->bus_voltage_v;
    if (effective_bus_voltage < foc->cfg.bus_voltage_min)
    {
        effective_bus_voltage = foc->cfg.bus_voltage_min;
    }

    if (effective_bus_voltage > 0.0F)
    {
        v_ab->alpha /= effective_bus_voltage;
        v_ab->beta /= effective_bus_voltage;
    }
}

/**
 * @brief Predict single-shunt phase currents using motor model for compensation
 * @param foc Pointer to the PMSM FOC drive structure
 * @param in Pointer to the FOC input structure
 * @param i_abc Output predicted three-phase currents
 * @param comp_status Output compensation status information
 */
static void mc_pmsm_predict_1shunt_current(const mc_pmsm_foc_t *foc,
                                           const mc_pmsm_foc_input_t *in,
                                           mc_abc_t *i_abc,
                                           mc_1shunt_comp_status_t *comp_status)
{
    mc_dq_t predicted_i_dq;
    mc_alphabeta_t predicted_i_ab;
    mc_f32_t omega_elec;
    mc_f32_t ld_h;
    mc_f32_t lq_h;
    mc_f32_t inv_ld;
    mc_f32_t inv_lq;
    mc_f32_t modulation_index;
    mc_f32_t prediction_weight;
    mc_f32_t delta_id;
    mc_f32_t delta_iq;
    mc_f32_t delta_limit;

    if ((foc == NULL) || (in == NULL) || (i_abc == NULL) || (comp_status == NULL))
    {
        return;
    }

    *comp_status = (mc_1shunt_comp_status_t){MC_TRUE, MC_1SHUNT_COMP_PREDICT_BASIC};

    ld_h = (foc->cfg.ld_h > 0.0F) ? foc->cfg.ld_h : 1.0F;
    lq_h = (foc->cfg.lq_h > 0.0F) ? foc->cfg.lq_h : 1.0F;
    inv_ld = 1.0F / ld_h;
    inv_lq = 1.0F / lq_h;
    omega_elec = in->elec_speed_rad_s;
    modulation_index = sqrtf((foc->v_dq.d * foc->v_dq.d) + (foc->v_dq.q * foc->v_dq.q));
    prediction_weight = 1.0F;
    delta_limit = 0.5F;

    if (modulation_index > (0.8F * foc->cfg.voltage_limit))
    {
        prediction_weight = 0.6F;
        delta_limit = 0.25F;
        comp_status->mode = MC_1SHUNT_COMP_PREDICT_HIGH_MODULATION;
    }

    if (in->id_ref < -0.1F)
    {
        prediction_weight *= 0.7F;
        delta_limit = 0.15F;
        comp_status->mode = MC_1SHUNT_COMP_PREDICT_FIELD_WEAKENING;
    }

    delta_id = ((foc->v_dq.d - (foc->cfg.rs_ohm * foc->i_dq.d) + (omega_elec * lq_h * foc->i_dq.q)) * inv_ld * in->dt_s);
    delta_iq = ((foc->v_dq.q - (foc->cfg.rs_ohm * foc->i_dq.q) -
                 (omega_elec * ((ld_h * foc->i_dq.d) + foc->cfg.flux_wb))) * inv_lq * in->dt_s);

    delta_id *= prediction_weight;
    delta_iq *= prediction_weight;
    if (delta_id > delta_limit)
    {
        delta_id = delta_limit;
    }
    else if (delta_id < -delta_limit)
    {
        delta_id = -delta_limit;
    }

    if (delta_iq > delta_limit)
    {
        delta_iq = delta_limit;
    }
    else if (delta_iq < -delta_limit)
    {
        delta_iq = -delta_limit;
    }

    predicted_i_dq.d = foc->i_dq.d + delta_id;
    predicted_i_dq.q = foc->i_dq.q + delta_iq;
    mc_ipark_run(&predicted_i_dq, in->sin_theta, in->cos_theta, &predicted_i_ab);

    i_abc->a = predicted_i_ab.alpha;
    i_abc->b = (-0.5F * predicted_i_ab.alpha) + (0.8660254038F * predicted_i_ab.beta);
    i_abc->c = (-0.5F * predicted_i_ab.alpha) - (0.8660254038F * predicted_i_ab.beta);
}

/**
 * @brief Reconstruct three-phase currents from shunt current measurements
 * @param foc Pointer to the PMSM FOC drive structure
 * @param in Pointer to the FOC input structure containing raw current measurements
 * @param i_abc Output reconstructed three-phase currents
 * @param comp_status Output compensation status for single-shunt prediction
 */
static void mc_pmsm_reconstruct_current(const mc_pmsm_foc_t *foc,
                                        const mc_pmsm_foc_input_t *in,
                                        mc_abc_t *i_abc,
                                        mc_1shunt_comp_status_t *comp_status)
{
    if ((foc == NULL) || (in == NULL) || (i_abc == NULL) || (comp_status == NULL))
    {
        return;
    }

    *comp_status = (mc_1shunt_comp_status_t){MC_FALSE, MC_1SHUNT_COMP_NONE};

    if (foc->cfg.current_cfg.type == MC_CURRENT_SENSE_2SHUNT)
    {
        mc_reconstruct_2shunt_run(&in->current_raw, &foc->cfg.current_cfg.cfg.shunt2, i_abc);
    }
    else if (foc->cfg.current_cfg.type == MC_CURRENT_SENSE_1SHUNT)
    {
        mc_reconstruct_1shunt_run(&in->current_raw, &foc->cfg.current_cfg.cfg.shunt1, &foc->shunt1_meta, i_abc);

        if (foc->shunt1_meta.compensation_required != MC_FALSE)
        {
            mc_pmsm_predict_1shunt_current(foc, in, i_abc, comp_status);
        }
    }
    else
    {
        mc_reconstruct_3shunt_run(&in->current_raw, &foc->cfg.current_cfg.cfg.shunt3, i_abc);
    }
}

/**
 * @brief Optimize PWM pattern for single-shunt current sensing
 * @param foc Pointer to the PMSM FOC drive structure
 * @param pwm_cmd PWM command to optimize (modified in place)
 */
static void mc_pmsm_optimize_1shunt_pwm(mc_pmsm_foc_t *foc, mc_pwm_cmd_t *pwm_cmd)
{
    mc_f32_t shift;

    if ((foc == NULL) || (pwm_cmd == NULL))
    {
        return;
    }

    if (foc->cfg.current_cfg.type != MC_CURRENT_SENSE_1SHUNT)
    {
        pwm_cmd->common_mode_shift = 0.0F;
        return;
    }

    shift = (foc->shunt1_meta.reorder_required != MC_FALSE) ? 0.04F : 0.02F;
    pwm_cmd->common_mode_shift = (foc->shunt1_meta.zero_vector_bias_high != MC_FALSE) ? shift : -shift;

    if (foc->shunt1_meta.reorder_required != MC_FALSE)
    {
        if (foc->shunt1_meta.zero_vector_bias_high != MC_FALSE)
        {
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_PWM;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_HIGH;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_HIGH;
        }
        else
        {
            pwm_cmd->phase_mode_a = MC_PWM_PHASE_PWM;
            pwm_cmd->phase_mode_b = MC_PWM_PHASE_LOW;
            pwm_cmd->phase_mode_c = MC_PWM_PHASE_LOW;
        }
    }
}

/**
 * @brief Plan ADC trigger timing for single-shunt current measurement
 * @param foc Pointer to the PMSM FOC drive structure
 * @param meta Pointer to the single-shunt metadata
 * @param trigger_plan Output ADC trigger plan structure
 */
static void mc_pmsm_plan_1shunt_adc_trigger(const mc_pmsm_foc_t *foc,
                                            const mc_1shunt_meta_t *meta,
                                            mc_adc_trigger_plan_t *trigger_plan)
{
    if ((foc == NULL) || (meta == NULL) || (trigger_plan == NULL))
    {
        return;
    }

    *trigger_plan = (mc_adc_trigger_plan_t){0};

    if (foc->cfg.current_cfg.type == MC_CURRENT_SENSE_3SHUNT)
    {
        trigger_plan->count = 1U;
        trigger_plan->trigger_a.valid = MC_TRUE;
        trigger_plan->trigger_a.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
        trigger_plan->trigger_a.event = MC_ADC_TRIGGER_EVENT_PWM_DOWN;
        trigger_plan->trigger_a.position = 0.5F;
        return;
    }

    if (foc->cfg.current_cfg.type == MC_CURRENT_SENSE_2SHUNT)
    {
        trigger_plan->count = 1U;
        trigger_plan->trigger_a.valid = MC_TRUE;
        trigger_plan->trigger_a.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
        trigger_plan->trigger_a.event = MC_ADC_TRIGGER_EVENT_PWM_DOWN;
        trigger_plan->trigger_a.position = 0.5F;
        return;
    }

    if (foc->cfg.current_cfg.type != MC_CURRENT_SENSE_1SHUNT)
    {
        return;
    }

    trigger_plan->count = meta->sample_count;

    if (meta->sample_phase_a != MC_1SHUNT_SAMPLE_PHASE_NONE)
    {
        trigger_plan->trigger_a.valid = MC_TRUE;
        trigger_plan->trigger_a.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
        trigger_plan->trigger_a.event = (meta->sample_half_a == MC_1SHUNT_HALF_CYCLE_UP) ? MC_ADC_TRIGGER_EVENT_PWM_UP : MC_ADC_TRIGGER_EVENT_PWM_DOWN;
        trigger_plan->trigger_a.sequence_index = 0U;
        trigger_plan->trigger_a.total_count = meta->sample_count;
        trigger_plan->trigger_a.soc_index = 0U;
        trigger_plan->trigger_a.sector = meta->sector;
        trigger_plan->trigger_a.preferred_window = meta->preferred_window;
        trigger_plan->trigger_a.reorder_applied = meta->reorder_required;
        trigger_plan->trigger_a.time_s = meta->sample_time_a;
        trigger_plan->trigger_a.half_time_s = meta->sample_half_time_a;
        trigger_plan->trigger_a.position = meta->sample_position_a;
        trigger_plan->trigger_a.half_position = meta->sample_half_position_a;
    }

    if (meta->sample_phase_b != MC_1SHUNT_SAMPLE_PHASE_NONE)
    {
        trigger_plan->trigger_b.valid = MC_TRUE;
        trigger_plan->trigger_b.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
        trigger_plan->trigger_b.event = (meta->sample_half_b == MC_1SHUNT_HALF_CYCLE_UP) ? MC_ADC_TRIGGER_EVENT_PWM_UP : MC_ADC_TRIGGER_EVENT_PWM_DOWN;
        trigger_plan->trigger_b.sequence_index = 1U;
        trigger_plan->trigger_b.total_count = meta->sample_count;
        trigger_plan->trigger_b.soc_index = 1U;
        trigger_plan->trigger_b.sector = meta->sector;
        trigger_plan->trigger_b.preferred_window = meta->preferred_window;
        trigger_plan->trigger_b.reorder_applied = meta->reorder_required;
        trigger_plan->trigger_b.time_s = meta->sample_time_b;
        trigger_plan->trigger_b.half_time_s = meta->sample_half_time_b;
        trigger_plan->trigger_b.position = meta->sample_position_b;
        trigger_plan->trigger_b.half_position = meta->sample_half_position_b;
    }
}

/**
 * @brief Plan single-shunt sampling from the voltage vector preview
 * @param foc Pointer to the PMSM FOC drive structure
 * @param voltage_ab Voltage vector in alpha-beta frame for preview
 * @param pwm_cmd PWM command to preview and configure (modified in place)
 */
static void mc_pmsm_plan_1shunt_from_voltage(mc_pmsm_foc_t *foc,
                                             const mc_alphabeta_t *voltage_ab,
                                             mc_pwm_cmd_t *pwm_cmd)
{
    mc_pwm_cmd_t preview_cmd;

    if ((foc == NULL) || (voltage_ab == NULL) || (pwm_cmd == NULL))
    {
        return;
    }

    if (foc->cfg.current_cfg.type != MC_CURRENT_SENSE_1SHUNT)
    {
        return;
    }

    preview_cmd = *pwm_cmd;
    preview_cmd.common_mode_shift = 0.0F;
    mc_svpwm_run(voltage_ab, &foc->cfg.svpwm_cfg, &preview_cmd);
    mc_reconstruct_1shunt_plan(&preview_cmd, &foc->cfg.current_cfg.cfg.shunt1, &foc->shunt1_meta);
}

/**
 * @brief Compute the optimal d-axis current for maximum torque per ampere (MTPA)
 * @param iq_ref Q-axis current reference
 * @param flux_wb Permanent magnet flux linkage in Webers
 * @param ld_h D-axis inductance in Henrys
 * @param lq_h Q-axis inductance in Henrys
 * @return Optimal d-axis current reference value
 */
mc_f32_t mc_pmsm_compute_mtpa_id(mc_f32_t iq_ref, mc_f32_t flux_wb, mc_f32_t ld_h, mc_f32_t lq_h)
{
    mc_f32_t delta_l = lq_h - ld_h;

    if ((delta_l < 1e-10F) && (delta_l > -1e-10F))
    {
        return 0.0F;
    }

    mc_f32_t discriminant = (flux_wb * flux_wb) + (4.0F * delta_l * delta_l * iq_ref * iq_ref);
    if (discriminant < 0.0F)
    {
        return 0.0F;
    }

    return (flux_wb - sqrtf(discriminant)) / (2.0F * delta_l);
}

/**
 * @brief Initialise a PMSM FOC drive instance
 * @param foc Pointer to the PMSM FOC drive structure
 * @param cfg Pointer to the FOC configuration structure
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if pointers are NULL
 */
mc_status_t mc_pmsm_foc_init(mc_pmsm_foc_t *foc, const mc_pmsm_foc_cfg_t *cfg)
{
    if ((foc == NULL) || (cfg == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *foc = (mc_pmsm_foc_t){0};
    foc->cfg = *cfg;
    mc_pi_init(&foc->id_pi, &cfg->id_pi_cfg);
    mc_pi_init(&foc->iq_pi, &cfg->iq_pi_cfg);

    return MC_STATUS_OK;
}

/**
 * @brief Execute one control cycle of the PMSM field-oriented control loop
 * @param foc Pointer to the PMSM FOC drive structure
 * @param in Pointer to the FOC input structure (currents, angles, references)
 * @param out Pointer to the FOC output structure (voltages, PWM commands)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if any pointer is NULL
 */
mc_status_t mc_pmsm_foc_run(mc_pmsm_foc_t *foc, const mc_pmsm_foc_input_t *in, mc_pmsm_foc_output_t *out)
{
    mc_f32_t id_error;
    mc_f32_t iq_error;
    mc_f32_t effective_id_ref;

    if ((foc == NULL) || (in == NULL) || (out == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    mc_pmsm_reconstruct_current(foc, in, &out->i_abc, &out->current_comp_status);
    mc_clarke_run(&out->i_abc, &out->i_ab);
    mc_park_run(&out->i_ab, in->sin_theta, in->cos_theta, &out->i_dq);

    effective_id_ref = in->id_ref;
    if (foc->cfg.fw_enable != MC_FALSE)
    {
        mc_f32_t v_mag = sqrtf((foc->v_dq.d * foc->v_dq.d) + (foc->v_dq.q * foc->v_dq.q));
        mc_f32_t fw_threshold = foc->cfg.voltage_limit * foc->cfg.fw_activation_ratio;
        mc_f32_t fw_error = fw_threshold - v_mag;

        foc->id_fw_adjustment += foc->cfg.fw_ki * fw_error * in->dt_s;
        if (foc->id_fw_adjustment > 0.0F)
        {
            foc->id_fw_adjustment = 0.0F;
        }
        else if (foc->id_fw_adjustment < foc->cfg.fw_min_id)
        {
            foc->id_fw_adjustment = foc->cfg.fw_min_id;
        }

        mc_f32_t fw_output = (foc->cfg.fw_kp * fw_error) + foc->id_fw_adjustment;
        if (fw_output > 0.0F)
        {
            fw_output = 0.0F;
        }
        else if (fw_output < foc->cfg.fw_min_id)
        {
            fw_output = foc->cfg.fw_min_id;
        }
        effective_id_ref += fw_output;
        if (effective_id_ref < foc->cfg.fw_min_id)
        {
            effective_id_ref = foc->cfg.fw_min_id;
        }
    }

    id_error = effective_id_ref - out->i_dq.d;
    iq_error = in->iq_ref - out->i_dq.q;

    out->v_dq.d = mc_pi_run(&foc->id_pi, id_error, in->dt_s);
    out->v_dq.q = mc_pi_run(&foc->iq_pi, iq_error, in->dt_s);

    mc_pmsm_limit_voltage(foc, &out->v_dq);

    mc_ipark_run(&out->v_dq, in->sin_theta, in->cos_theta, &out->v_ab);
    mc_pmsm_normalize_voltage(foc, in, &out->v_ab);
    mc_pmsm_plan_1shunt_from_voltage(foc, &out->v_ab, &out->pwm_cmd);
    mc_pmsm_optimize_1shunt_pwm(foc, &out->pwm_cmd);
    mc_svpwm_run(&out->v_ab, &foc->cfg.svpwm_cfg, &out->pwm_cmd);
    mc_pmsm_optimize_1shunt_pwm(foc, &out->pwm_cmd);
    mc_pmsm_plan_1shunt_adc_trigger(foc, &foc->shunt1_meta, &out->adc_trigger_plan);

    foc->i_dq = out->i_dq;
    foc->v_dq = out->v_dq;
    foc->i_abc_last = out->i_abc;
    foc->last_sector = out->pwm_cmd.sector;

    return MC_STATUS_OK;
}
