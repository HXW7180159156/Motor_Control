/**
 * @file mc_api.c
 * @brief Motor control API implementation
 */
#include "mc_api.h"
#include "mc_constants.h"
#include "mc_math.h"
#include "mc_version.h"

#include <math.h>

/**
 * @brief Apply the identify-side 1-shunt PWM adjustments required by the sampling plan
 * @param meta Planned 1-shunt sampling metadata
 * @param pwm_cmd PWM command to adjust in place
 */
static void mc_api_identify_optimize_1shunt_pwm(const mc_1shunt_meta_t *meta,
                                                mc_pwm_cmd_t *pwm_cmd)
{
    mc_f32_t shift;

    if ((meta == NULL) || (pwm_cmd == NULL))
    {
        return;
    }

    shift = (meta->reorder_required != MC_FALSE) ? MC_1SHUNT_SHIFT_REORDER : MC_1SHUNT_SHIFT_NORMAL;
    pwm_cmd->common_mode_shift = (meta->zero_vector_bias_high != MC_FALSE) ? shift : -shift;

    if (meta->reorder_required == MC_FALSE)
    {
        return;
    }

    if (meta->zero_vector_bias_high != MC_FALSE)
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

/**
 * @brief Build the identify-side ADC trigger plan from the 1-shunt sampling metadata
 * @param meta Planned 1-shunt sampling metadata
 * @param trigger_plan ADC trigger plan output
 */
static void mc_api_identify_plan_1shunt_adc_trigger(const mc_1shunt_meta_t *meta,
                                                    mc_adc_trigger_plan_t *trigger_plan)
{
    if ((meta == NULL) || (trigger_plan == NULL))
    {
        return;
    }

    *trigger_plan = (mc_adc_trigger_plan_t){0};
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
 * @brief Reconstruct alpha-beta current using the configured current-sense path
 * @param inst Motor control instance
 * @param adc_raw Raw ADC current samples
 * @param current_ab Reconstructed alpha-beta current output
 */
static void mc_api_reconstruct_current_ab(const mc_instance_t *inst,
                                          const mc_adc_raw_t *adc_raw,
                                          mc_alphabeta_t *current_ab)
{
    mc_abc_t current_abc;

    if ((inst == NULL) || (adc_raw == NULL) || (current_ab == NULL))
    {
        return;
    }

    if (inst->cfg.foc.current_cfg.type == MC_CURRENT_SENSE_2SHUNT)
    {
        mc_reconstruct_2shunt_run(adc_raw, &inst->cfg.foc.current_cfg.cfg.shunt2, &current_abc);
        mc_clarke_run(&current_abc, current_ab);
        return;
    }

    if (inst->cfg.foc.current_cfg.type == MC_CURRENT_SENSE_3SHUNT)
    {
        mc_reconstruct_3shunt_run(adc_raw, &inst->cfg.foc.current_cfg.cfg.shunt3, &current_abc);
        mc_clarke_run(&current_abc, current_ab);
        return;
    }

    mc_reconstruct_1shunt_run(adc_raw,
                              &inst->cfg.foc.current_cfg.cfg.shunt1,
                              &inst->foc.pmsm_foc.shunt1_meta,
                              &current_abc);
    mc_clarke_run(&current_abc, current_ab);
}

/**
 * @brief Initialise estimator subsystems (hall, encoder, resolver, sensorless)
 * @param inst Motor control instance
 * @return MC_STATUS_OK on success, or error status
 */
static mc_status_t mc_api_init_estimators(mc_instance_t *inst)
{
    mc_status_t status = MC_STATUS_OK;

    if ((inst->cfg.sensor.primary_mode == MC_MODE_PMSM_FOC_HALL) ||
        (inst->cfg.sensor.primary_mode == MC_MODE_BLDC_HALL))
    {
        status = mc_hall_init(&inst->hall, &inst->cfg.sensor.hall_cfg);
    }
    else if (inst->cfg.sensor.primary_mode == MC_MODE_PMSM_FOC_ENCODER)
    {
        status = mc_encoder_init(&inst->encoder, &inst->cfg.sensor.encoder_cfg);
    }
    else if (inst->cfg.sensor.primary_mode == MC_MODE_PMSM_FOC_RESOLVER)
    {
        status = mc_resolver_init(&inst->resolver, &inst->cfg.sensor.resolver_cfg);
        if ((status == MC_STATUS_OK) && (inst->cfg.resolver_port.excitation_set != NULL))
        {
            int16_t amplitude;
            uint16_t frequency_hz;

            status = mc_resolver_get_excitation_command(&inst->resolver, &amplitude, &frequency_hz);
            if (status == MC_STATUS_OK)
            {
                inst->cfg.resolver_port.excitation_set(amplitude, frequency_hz);
            }
        }
    }
    else if (inst->cfg.sensor.primary_mode == MC_MODE_PMSM_FOC_SENSORLESS)
    {
        mc_sensorless_cfg_t sensorless_cfg = inst->cfg.sensor.sensorless_cfg;

        sensorless_cfg.rs_ohm = inst->cfg.motor.rs_ohm;
        sensorless_cfg.ls_h = MC_1SHUNT_HALF * (inst->cfg.motor.ld_h + inst->cfg.motor.lq_h);
        sensorless_cfg.pole_pairs = (mc_f32_t)inst->cfg.motor.pole_pairs;
        if (sensorless_cfg.bemf_filter_alpha <= 0.0F)
        {
            sensorless_cfg.bemf_filter_alpha = MC_SENSORLESS_DEFAULT_BEMF_ALPHA;
        }
        if (sensorless_cfg.min_bemf <= 0.0F)
        {
            sensorless_cfg.min_bemf = MC_SENSORLESS_DEFAULT_MIN_BEMF;
        }
        if (sensorless_cfg.pll_kp <= 0.0F)
        {
            sensorless_cfg.pll_kp = MC_SENSORLESS_DEFAULT_PLL_KP;
        }
        if (sensorless_cfg.pll_ki <= 0.0F)
        {
            sensorless_cfg.pll_ki = MC_SENSORLESS_DEFAULT_PLL_KI;
        }
        if (sensorless_cfg.lock_bemf <= 0.0F)
        {
            sensorless_cfg.lock_bemf = sensorless_cfg.min_bemf * MC_SENSORLESS_DEFAULT_LOCK_BEMF_RATIO;
        }
        if (sensorless_cfg.startup_speed_rad_s <= 0.0F)
        {
            sensorless_cfg.startup_speed_rad_s = MC_SENSORLESS_DEFAULT_STARTUP_SPEED_RAD_S;
        }
        if (sensorless_cfg.startup_accel_rad_s2 <= 0.0F)
        {
            sensorless_cfg.startup_accel_rad_s2 = MC_SENSORLESS_DEFAULT_STARTUP_ACCEL_RAD_S2;
        }
        if (sensorless_cfg.open_loop_voltage_max <= 0.0F)
        {
            sensorless_cfg.open_loop_voltage_max = MC_SENSORLESS_DEFAULT_OL_VOLTAGE_RATIO * inst->cfg.foc.voltage_limit;
        }
        if (sensorless_cfg.open_loop_voltage_start < 0.0F)
        {
            sensorless_cfg.open_loop_voltage_start = 0.0F;
        }
        status = mc_sensorless_init(&inst->sensorless, &sensorless_cfg);
    }
    else if (inst->cfg.sensor.primary_mode == MC_MODE_PMSM_FOC_SMO)
    {
        mc_smo_cfg_t smo_cfg;

        smo_cfg.rs_ohm = inst->cfg.motor.rs_ohm;
        smo_cfg.ls_h = MC_1SHUNT_HALF * (inst->cfg.motor.ld_h + inst->cfg.motor.lq_h);
        smo_cfg.pole_pairs = (mc_f32_t)inst->cfg.motor.pole_pairs;
        smo_cfg.k_slide = inst->cfg.sensor.sensorless_cfg.lock_bemf * 3.0F;
        if (smo_cfg.k_slide <= 0.0F)
        {
            smo_cfg.k_slide = 1.0F;
        }
        smo_cfg.lpf_alpha = inst->cfg.sensor.sensorless_cfg.bemf_filter_alpha;
        if (smo_cfg.lpf_alpha <= 0.0F)
        {
            smo_cfg.lpf_alpha = 0.5F;
        }
        smo_cfg.min_bemf = inst->cfg.sensor.sensorless_cfg.min_bemf;
        if (smo_cfg.min_bemf <= 0.0F)
        {
            smo_cfg.min_bemf = MC_SENSORLESS_DEFAULT_MIN_BEMF;
        }
        smo_cfg.pll_kp = inst->cfg.sensor.sensorless_cfg.pll_kp;
        if (smo_cfg.pll_kp <= 0.0F)
        {
            smo_cfg.pll_kp = MC_SENSORLESS_DEFAULT_PLL_KP;
        }
        smo_cfg.pll_ki = inst->cfg.sensor.sensorless_cfg.pll_ki;
        if (smo_cfg.pll_ki <= 0.0F)
        {
            smo_cfg.pll_ki = MC_SENSORLESS_DEFAULT_PLL_KI;
        }
        smo_cfg.lock_bemf = inst->cfg.sensor.sensorless_cfg.lock_bemf;
        if (smo_cfg.lock_bemf <= 0.0F)
        {
            smo_cfg.lock_bemf = smo_cfg.min_bemf * MC_SENSORLESS_DEFAULT_LOCK_BEMF_RATIO;
        }
        smo_cfg.startup_speed_rad_s = inst->cfg.sensor.sensorless_cfg.startup_speed_rad_s;
        if (smo_cfg.startup_speed_rad_s <= 0.0F)
        {
            smo_cfg.startup_speed_rad_s = MC_SENSORLESS_DEFAULT_STARTUP_SPEED_RAD_S;
        }
        smo_cfg.startup_accel_rad_s2 = inst->cfg.sensor.sensorless_cfg.startup_accel_rad_s2;
        if (smo_cfg.startup_accel_rad_s2 <= 0.0F)
        {
            smo_cfg.startup_accel_rad_s2 = MC_SENSORLESS_DEFAULT_STARTUP_ACCEL_RAD_S2;
        }
        smo_cfg.open_loop_voltage_max = inst->cfg.sensor.sensorless_cfg.open_loop_voltage_max;
        if (smo_cfg.open_loop_voltage_max <= 0.0F)
        {
            smo_cfg.open_loop_voltage_max = MC_SENSORLESS_DEFAULT_OL_VOLTAGE_RATIO * inst->cfg.foc.voltage_limit;
        }
        smo_cfg.open_loop_voltage_start = inst->cfg.sensor.sensorless_cfg.open_loop_voltage_start;
        if (smo_cfg.open_loop_voltage_start < 0.0F)
        {
            smo_cfg.open_loop_voltage_start = 0.0F;
        }
        status = mc_smo_init(&inst->smo, &smo_cfg);
    }

    return status;
}

/**
 * @brief Initialise FOC controller from instance configuration
 * @param inst Motor control instance
 * @return MC_STATUS_OK on success, or error status
 */
static mc_status_t mc_api_init_foc(mc_instance_t *inst)
{
    mc_pmsm_foc_cfg_t foc_cfg;
    mc_status_t status;

    foc_cfg.id_pi_cfg = inst->cfg.foc.id_pi_cfg;
    foc_cfg.iq_pi_cfg = inst->cfg.foc.iq_pi_cfg;
    foc_cfg.svpwm_cfg = inst->cfg.foc.svpwm_cfg;
    foc_cfg.current_cfg = inst->cfg.foc.current_cfg;
    foc_cfg.voltage_limit = inst->cfg.foc.voltage_limit;
    foc_cfg.bus_voltage_min = inst->cfg.foc.bus_voltage_min;
    foc_cfg.rs_ohm = inst->cfg.motor.rs_ohm;
    foc_cfg.ld_h = inst->cfg.motor.ld_h;
    foc_cfg.lq_h = inst->cfg.motor.lq_h;
    foc_cfg.flux_wb = inst->cfg.motor.flux_wb;
    foc_cfg.pole_pairs = (mc_f32_t)inst->cfg.motor.pole_pairs;
    foc_cfg.mtpa_enable = inst->cfg.foc.mtpa_enable;
    foc_cfg.fw_enable = inst->cfg.foc.fw_enable;
    foc_cfg.fw_kp = inst->cfg.foc.fw_kp;
    foc_cfg.fw_ki = inst->cfg.foc.fw_ki;
    foc_cfg.fw_min_id = inst->cfg.foc.fw_min_id;
    foc_cfg.fw_activation_ratio = inst->cfg.foc.fw_activation_ratio;

    status = mc_control_foc_init(&inst->foc, &foc_cfg);
    if (status != MC_STATUS_OK)
    {
        return status;
    }

    return mc_control_foc_set_speed_pi(&inst->foc, &inst->cfg.foc.speed_pi_cfg, inst->cfg.foc.iq_limit);
}

/**
 * @brief Initialise the limited Q31 speed-loop state used by the top-level API
 * @param inst Motor control instance
 * @return MC_STATUS_OK on success, or error status
 */
static mc_status_t mc_api_init_q31_state(mc_instance_t *inst)
{
    mc_pi_q31_cfg_t speed_pi_cfg_q31;

    if (inst == NULL)
    {
        return MC_STATUS_INVALID_ARG;
    }

    speed_pi_cfg_q31.kp = mc_q31_from_f32(inst->cfg.foc.speed_pi_cfg.kp);
    speed_pi_cfg_q31.ki = mc_q31_from_f32(inst->cfg.foc.speed_pi_cfg.ki);
    speed_pi_cfg_q31.integral_min = mc_q31_from_f32(inst->cfg.foc.speed_pi_cfg.integral_min);
    speed_pi_cfg_q31.integral_max = mc_q31_from_f32(inst->cfg.foc.speed_pi_cfg.integral_max);
    speed_pi_cfg_q31.output_min = mc_q31_from_f32(inst->cfg.foc.speed_pi_cfg.output_min);
    speed_pi_cfg_q31.output_max = mc_q31_from_f32(inst->cfg.foc.speed_pi_cfg.output_max);
    mc_pi_q31_init(&inst->foc_speed_pi_q31, &speed_pi_cfg_q31);
    inst->iq_ref_q31 = 0;

    return MC_STATUS_OK;
}

/**
 * @brief Build the identification configuration from the instance configuration
 * @param inst Motor control instance
 * @param id_cfg Output identification configuration
 */
static void mc_api_build_identify_cfg(const mc_instance_t *inst, mc_identify_cfg_t *id_cfg)
{
    if ((inst == NULL) || (id_cfg == NULL))
    {
        return;
    }

    *id_cfg = (mc_identify_cfg_t){0};
    id_cfg->svpwm_cfg = inst->cfg.foc.svpwm_cfg;
    id_cfg->voltage_limit = inst->cfg.foc.voltage_limit;
    id_cfg->max_current_a = inst->cfg.foc.iq_limit;
    if (id_cfg->max_current_a <= 0.0F)
    {
        id_cfg->max_current_a = 1.0F;
    }
}

/**
 * @brief Apply identified motor parameters to the live instance and runtime models
 * @param inst Motor control instance
 * @param rs_ohm Identified stator resistance
 * @param ld_h Identified d-axis inductance
 * @param lq_h Identified q-axis inductance
 * @param flux_wb Identified flux linkage
 */
static void mc_api_apply_identified_params(mc_instance_t *inst,
                                           mc_f32_t rs_ohm,
                                           mc_f32_t ld_h,
                                           mc_f32_t lq_h,
                                           mc_f32_t flux_wb)
{
    if (inst == NULL)
    {
        return;
    }

    if (rs_ohm > 0.0F)
    {
        inst->cfg.motor.rs_ohm = rs_ohm;
    }
    if (ld_h > 0.0F)
    {
        inst->cfg.motor.ld_h = ld_h;
    }
    if (lq_h > 0.0F)
    {
        inst->cfg.motor.lq_h = lq_h;
    }
    if (flux_wb > 0.0F)
    {
        inst->cfg.motor.flux_wb = flux_wb;
    }

    inst->foc.pmsm_foc.cfg.rs_ohm = inst->cfg.motor.rs_ohm;
    inst->foc.pmsm_foc.cfg.ld_h = inst->cfg.motor.ld_h;
    inst->foc.pmsm_foc.cfg.lq_h = inst->cfg.motor.lq_h;
    inst->foc.pmsm_foc.cfg.flux_wb = inst->cfg.motor.flux_wb;

    if (inst->cfg.sensor.primary_mode == MC_MODE_PMSM_FOC_SENSORLESS)
    {
        inst->sensorless.cfg.rs_ohm = inst->cfg.motor.rs_ohm;
        inst->sensorless.cfg.ls_h = MC_1SHUNT_HALF * (inst->cfg.motor.ld_h + inst->cfg.motor.lq_h);
    }
    else if (inst->cfg.sensor.primary_mode == MC_MODE_PMSM_FOC_SMO)
    {
        inst->smo.cfg.rs_ohm = inst->cfg.motor.rs_ohm;
        inst->smo.cfg.ls_h = MC_1SHUNT_HALF * (inst->cfg.motor.ld_h + inst->cfg.motor.lq_h);
    }
}

/**
 * @brief Apply trusted identify results from the identify state into runtime models
 * @param inst Motor control instance
 */
static void mc_api_apply_trusted_identify_results(mc_instance_t *inst)
{
    mc_f32_t rs_ohm;
    mc_f32_t ld_h;
    mc_f32_t lq_h;
    mc_f32_t flux_wb;

    if (inst == NULL)
    {
        return;
    }

    mc_identify_get_result(&inst->identify, &rs_ohm, &ld_h, &lq_h, &flux_wb);
    mc_api_apply_identified_params(inst, rs_ohm, ld_h, lq_h, flux_wb);
    inst->foc.pmsm_foc.shunt1_meta = (mc_1shunt_meta_t){0};
}

/**
 * @brief Initialise BLDC Hall controller from instance configuration
 * @param inst Motor control instance
 * @return MC_STATUS_OK on success, or error status
 */
static mc_status_t mc_api_init_bldc(mc_instance_t *inst)
{
    mc_bldc_hall_cfg_t bldc_cfg;

    bldc_cfg.duty_min = 0.0F;
    bldc_cfg.duty_max = 1.0F;

    return mc_bldc_hall_init(&inst->bldc_hall, &bldc_cfg);
}

/**
 * @brief Update rotor electrical angle and compute sin/cos from sensor data
 * @param inst Motor control instance
 * @param in Fast input data (ADC raw, hall code, resolver, timestamp)
 * @param sin_theta Output sine of electrical angle
 * @param cos_theta Output cosine of electrical angle
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG or MC_STATUS_ERROR on failure
 */
static mc_status_t mc_api_update_position(mc_instance_t *inst, const mc_fast_input_t *in, mc_f32_t *sin_theta, mc_f32_t *cos_theta)
{
    mc_f32_t angle_rad = 0.0F;
    mc_status_t status = MC_STATUS_OK;

    if ((inst == NULL) || (in == NULL) || (sin_theta == NULL) || (cos_theta == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    if (inst->mode == MC_MODE_PMSM_FOC_HALL)
    {
        status = mc_hall_update(&inst->hall, in->hall_code, in->timestamp_us, inst->cfg.sensor.pole_pairs);
        angle_rad = inst->hall.elec_angle_rad;
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_ENCODER)
    {
        angle_rad = inst->encoder.elec_angle_rad;
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_RESOLVER)
    {
        if (mc_resolver_update(&inst->resolver, &in->resolver_raw, in->timestamp_us) != MC_STATUS_OK)
        {
            return MC_STATUS_ERROR;
        }

        if (inst->resolver.signal_valid == MC_FALSE)
        {
            return MC_STATUS_ERROR;
        }

        angle_rad = inst->resolver.elec_angle_rad;
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_SENSORLESS)
    {
        angle_rad = inst->sensorless.elec_angle_rad;
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_SMO)
    {
        angle_rad = inst->smo.elec_angle_rad;
    }

    angle_rad = mc_math_wrap_angle_rad(angle_rad);

    if (angle_rad == 0.0F)
    {
        *sin_theta = 0.0F;
        *cos_theta = 1.0F;
    }
    else
    {
        *sin_theta = sinf(angle_rad);
        *cos_theta = cosf(angle_rad);
    }

    return status;
}

/**
 * @brief Get electrical speed in rad/s from the active sensor
 * @param inst Motor control instance
 * @return Electrical speed in rad/s, or 0.0F on error
 */
static mc_f32_t mc_api_get_elec_speed_rad_s(const mc_instance_t *inst)
{
    mc_f32_t mech_speed_rpm = 0.0F;
    mc_f32_t pole_pairs;

    if (inst == NULL)
    {
        return 0.0F;
    }

    if (inst->mode == MC_MODE_PMSM_FOC_ENCODER)
    {
        mech_speed_rpm = inst->encoder.mech_speed_rpm;
        pole_pairs = inst->encoder.cfg.pole_pairs;
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_HALL)
    {
        mech_speed_rpm = inst->hall.mech_speed_rpm;
        pole_pairs = inst->cfg.sensor.pole_pairs;
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_RESOLVER)
    {
        mech_speed_rpm = inst->resolver.mech_speed_rpm;
        pole_pairs = inst->resolver.cfg.pole_pairs;
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_SENSORLESS)
    {
        mech_speed_rpm = inst->sensorless.mech_speed_rpm;
        pole_pairs = inst->sensorless.cfg.pole_pairs;
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_SMO)
    {
        mech_speed_rpm = inst->smo.mech_speed_rpm;
        pole_pairs = inst->smo.cfg.pole_pairs;
    }
    else
    {
        return 0.0F;
    }

    return mech_speed_rpm * MC_RPM_TO_RAD_S * pole_pairs;
}

/**
 * @brief Validate that the motor control instance pointer is non-NULL
 * @param inst Motor control instance
 * @return MC_STATUS_OK if valid, MC_STATUS_INVALID_ARG if NULL
 */
static mc_status_t mc_check_instance(const mc_instance_t *inst)
{
    if (inst == NULL)
    {
        return MC_STATUS_INVALID_ARG;
    }

    return MC_STATUS_OK;
}

/**
 * @brief Check whether the requested numeric-mode/mode combination is supported
 * @param cfg System configuration to validate
 * @return MC_STATUS_OK on success, MC_STATUS_UNSUPPORTED when unsupported
 */
static mc_status_t mc_api_validate_numeric_mode(const mc_system_cfg_t *cfg)
{
    if (cfg == NULL)
    {
        return MC_STATUS_INVALID_ARG;
    }

    if (cfg->control.numeric_mode == MC_NUMERIC_FLOAT32)
    {
        return MC_STATUS_OK;
    }

    if (cfg->control.numeric_mode != MC_NUMERIC_Q31)
    {
        return MC_STATUS_UNSUPPORTED;
    }

    if ((cfg->sensor.primary_mode != MC_MODE_PMSM_FOC_HALL) &&
        (cfg->sensor.primary_mode != MC_MODE_PMSM_FOC_ENCODER))
    {
        return MC_STATUS_UNSUPPORTED;
    }

    if ((cfg->foc.current_cfg.type != MC_CURRENT_SENSE_2SHUNT) &&
        (cfg->foc.current_cfg.type != MC_CURRENT_SENSE_3SHUNT))
    {
        return MC_STATUS_UNSUPPORTED;
    }

    if ((cfg->foc.mtpa_enable != MC_FALSE) || (cfg->foc.fw_enable != MC_FALSE))
    {
        return MC_STATUS_UNSUPPORTED;
    }

    return MC_STATUS_OK;
}

/**
 * @brief Reset runtime state while preserving configuration and init state
 * @param inst Motor control instance
 * @return MC_STATUS_OK on success or error status
 */
static mc_status_t mc_api_reset_runtime(mc_instance_t *inst)
{
    mc_status_t status = mc_check_instance(inst);

    if (status != MC_STATUS_OK)
    {
        return status;
    }

    inst->enabled = MC_FALSE;
    inst->speed_ctrl_enabled = MC_FALSE;
    inst->mode = inst->cfg.sensor.primary_mode;
    inst->speed_ref_rpm = 0.0F;
    inst->torque_ref = 0.0F;
    inst->id_ref = 0.0F;
    inst->iq_ref = 0.0F;
    inst->diag = (mc_diag_status_t){0};
    inst->foc_last_output = (mc_pmsm_foc_output_t){0};
    mc_pi_reset(&inst->foc.speed_pi);
    mc_pi_q31_reset(&inst->foc_speed_pi_q31);
    inst->iq_ref_q31 = 0;
    mc_pi_reset(&inst->foc.pmsm_foc.id_pi);
    mc_pi_reset(&inst->foc.pmsm_foc.iq_pi);
    inst->foc.pmsm_foc.i_dq = (mc_dq_t){0};
    inst->foc.pmsm_foc.v_dq = (mc_dq_t){0};
    inst->foc.pmsm_foc.i_abc_last = (mc_abc_t){0};
    inst->foc.pmsm_foc.shunt1_meta = (mc_1shunt_meta_t){0};
    inst->foc.pmsm_foc.id_fw_adjustment = 0.0F;
    inst->foc.pmsm_foc.last_sector = 0U;
    inst->hall.last_hall_code = 0U;
    inst->hall.last_transition_us = 0U;
    inst->hall.elec_angle_rad = 0.0F;
    inst->hall.mech_speed_rpm = 0.0F;
    inst->encoder.last_count = 0U;
    inst->encoder.last_timestamp_us = 0U;
    inst->encoder.elec_angle_rad = 0.0F;
    inst->encoder.mech_speed_rpm = 0.0F;
    inst->resolver.elec_angle_rad = 0.0F;
    inst->resolver.mech_speed_rpm = 0.0F;
    inst->resolver.signal_valid = MC_FALSE;
    inst->resolver.last_timestamp_us = 0U;
    inst->sensorless = (mc_sensorless_state_t){0};
    inst->smo = (mc_smo_state_t){0};
    if ((inst->cfg.sensor.primary_mode == MC_MODE_PMSM_FOC_SENSORLESS) ||
        (inst->cfg.sensor.primary_mode == MC_MODE_PMSM_FOC_SMO))
    {
        mc_status_t init_status = mc_api_init_estimators(inst);
        if (init_status != MC_STATUS_OK)
        {
            return init_status;
        }
    }
    inst->bldc_hall.last_hall_code = 0U;
    inst->bldc_hall.last_pwm_cmd = (mc_pwm_cmd_t){0};

    return mc_bldc_sensorless_reset(&inst->bldc_sensorless);
}

/**
 * @brief Initialise a motor control instance with system configuration
 * @param inst Motor control instance to initialize
 * @param cfg System configuration (sensor, motor, FOC, hooks)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if pointers are NULL
 */
mc_status_t mc_init(mc_instance_t *inst, const mc_system_cfg_t *cfg)
{
    mc_instance_t next_inst;
    mc_status_t status;

    if ((inst == NULL) || (cfg == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    status = mc_api_validate_numeric_mode(cfg);
    if (status != MC_STATUS_OK)
    {
        *inst = (mc_instance_t){0};
        return status;
    }

    next_inst = (mc_instance_t){0};
    next_inst.cfg = *cfg;
    next_inst.mode = cfg->sensor.primary_mode;

    status = mc_api_init_estimators(&next_inst);
    if (status != MC_STATUS_OK)
    {
        *inst = (mc_instance_t){0};
        return status;
    }

    if ((next_inst.mode == MC_MODE_PMSM_FOC_HALL) ||
        (next_inst.mode == MC_MODE_PMSM_FOC_ENCODER) ||
        (next_inst.mode == MC_MODE_PMSM_FOC_RESOLVER) ||
        (next_inst.mode == MC_MODE_PMSM_FOC_SENSORLESS) ||
        (next_inst.mode == MC_MODE_PMSM_FOC_SMO))
    {
        status = mc_api_init_foc(&next_inst);
        if (status != MC_STATUS_OK)
        {
            *inst = (mc_instance_t){0};
            return status;
        }
    }
    else if (next_inst.mode == MC_MODE_BLDC_HALL)
    {
        status = mc_api_init_bldc(&next_inst);
        if (status != MC_STATUS_OK)
        {
            *inst = (mc_instance_t){0};
            return status;
        }
    }
    else if (next_inst.mode == MC_MODE_BLDC_SENSORLESS)
    {
        status = mc_bldc_sensorless_init(&next_inst.bldc_sensorless, &next_inst.cfg.sensor.bldc_sensorless_cfg);
        if (status != MC_STATUS_OK)
        {
            *inst = (mc_instance_t){0};
            return status;
        }
    }

    {
        mc_identify_cfg_t id_cfg;

        mc_api_build_identify_cfg(&next_inst, &id_cfg);
        mc_identify_init(&next_inst.identify, &id_cfg);
        next_inst.identify.flux_wb = next_inst.cfg.motor.flux_wb;
    }

    next_inst.initialized = MC_TRUE;
    if (next_inst.cfg.control.numeric_mode == MC_NUMERIC_Q31)
    {
        status = mc_api_init_q31_state(&next_inst);
        if (status != MC_STATUS_OK)
        {
            *inst = (mc_instance_t){0};
            return status;
        }
    }
    *inst = next_inst;

    return MC_STATUS_OK;
}

/**
 * @brief Start motor parameter identification sequence
 * @param inst Motor control instance
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG or MC_STATUS_INVALID_STATE on failure
 */
mc_status_t mc_start_identification(mc_instance_t *inst)
{
    mc_status_t status = mc_check_instance(inst);

    if (status != MC_STATUS_OK)
    {
        return status;
    }

    if (inst->initialized == MC_FALSE)
    {
        return MC_STATUS_INVALID_STATE;
    }

    {
        mc_identify_cfg_t id_cfg;

        mc_api_build_identify_cfg(inst, &id_cfg);
        mc_identify_init(&inst->identify, &id_cfg);
        inst->identify.flux_wb = inst->cfg.motor.flux_wb;
        inst->foc.pmsm_foc.shunt1_meta = (mc_1shunt_meta_t){0};
    }

    inst->enabled = MC_TRUE;
    mc_identify_start(&inst->identify);

    return MC_STATUS_OK;
}

/**
 * @brief Check if parameter identification has completed
 * @param inst Motor control instance
 * @return MC_TRUE if identification is done, MC_FALSE otherwise or if inst is NULL
 */
mc_bool_t mc_is_identification_done(const mc_instance_t *inst)
{
    if (inst == NULL)
    {
        return MC_FALSE;
    }
    return mc_identify_is_done(&inst->identify);
}

/**
 * @brief Get identified motor parameters after completion
 * @param inst Motor control instance
 * @param params Output motor parameters (Rs, Ld, Lq, flux, pole pairs)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if pointers are NULL
 */
mc_status_t mc_get_identified_params(const mc_instance_t *inst, mc_motor_params_t *params)
{
    if ((inst == NULL) || (params == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    mc_identify_get_result(&inst->identify,
                           &params->rs_ohm,
                           &params->ld_h,
                           &params->lq_h,
                           &params->flux_wb);
    params->pole_pairs = inst->cfg.motor.pole_pairs;

    return MC_STATUS_OK;
}

/**
 * @brief Reset motor control instance to disabled state
 * @param inst Motor control instance
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if inst is NULL
 */
mc_status_t mc_reset(mc_instance_t *inst)
{
    return mc_api_reset_runtime(inst);
}

/**
 * @brief Enable the motor control instance (alias for mc_set_enable with MC_TRUE)
 * @param inst Motor control instance
 * @return MC_STATUS_OK on success, or error status
 */
mc_status_t mc_start(mc_instance_t *inst)
{
    mc_status_t status = mc_set_enable(inst, MC_TRUE);

    if (status != MC_STATUS_OK)
    {
        return status;
    }

    if (inst->mode == MC_MODE_BLDC_SENSORLESS)
    {
        return mc_bldc_sensorless_start(&inst->bldc_sensorless);
    }

    return MC_STATUS_OK;
}

/**
 * @brief Disable the motor control instance (alias for mc_set_enable with MC_FALSE)
 * @param inst Motor control instance
 * @return MC_STATUS_OK on success, or error status
 */
mc_status_t mc_stop(mc_instance_t *inst)
{
    mc_status_t status = mc_set_enable(inst, MC_FALSE);

    if (status != MC_STATUS_OK)
    {
        return status;
    }

    if (inst->mode == MC_MODE_BLDC_SENSORLESS)
    {
        status = mc_bldc_sensorless_reset(&inst->bldc_sensorless);
    }

    return status;
}

/**
 * @brief Set the operating mode of the motor control instance
 * @param inst Motor control instance
 * @param mode Operating mode to set
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if inst is NULL
 */
mc_status_t mc_set_mode(mc_instance_t *inst, mc_mode_t mode)
{
    mc_status_t status = mc_check_instance(inst);

    if (status != MC_STATUS_OK)
    {
        return status;
    }

    if ((inst->initialized != MC_FALSE) && (mode != inst->cfg.sensor.primary_mode))
    {
        return MC_STATUS_INVALID_STATE;
    }

    inst->mode = mode;

    return MC_STATUS_OK;
}

/**
 * @brief Enable or disable the motor control loop
 * @param inst Motor control instance
 * @param enable MC_TRUE to enable, MC_FALSE to disable
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG or MC_STATUS_INVALID_STATE on failure
 */
mc_status_t mc_set_enable(mc_instance_t *inst, mc_bool_t enable)
{
    mc_status_t status = mc_check_instance(inst);

    if (status != MC_STATUS_OK)
    {
        return status;
    }

    if (inst->initialized == MC_FALSE)
    {
        return MC_STATUS_INVALID_STATE;
    }

    inst->enabled = enable;

    return MC_STATUS_OK;
}

/**
 * @brief Set the speed reference and enable speed control
 * @param inst Motor control instance
 * @param speed_ref Speed reference in RPM
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if inst is NULL
 */
mc_status_t mc_set_speed_ref(mc_instance_t *inst, mc_f32_t speed_ref)
{
    mc_status_t status = mc_check_instance(inst);

    if (status != MC_STATUS_OK)
    {
        return status;
    }

    inst->speed_ref_rpm = speed_ref;
    inst->speed_ctrl_enabled = MC_TRUE;
    return MC_STATUS_OK;
}

/**
 * @brief Set the torque reference and disable speed control
 * @param inst Motor control instance
 * @param torque_ref Torque reference (maps to iq_ref, with optional MTPA id component)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if inst is NULL
 */
mc_status_t mc_set_torque_ref(mc_instance_t *inst, mc_f32_t torque_ref)
{
    mc_status_t status = mc_check_instance(inst);

    if (status != MC_STATUS_OK)
    {
        return status;
    }

    inst->torque_ref = torque_ref;
    inst->iq_ref = torque_ref;
    if (inst->cfg.foc.mtpa_enable != MC_FALSE)
    {
        inst->id_ref = mc_pmsm_compute_mtpa_id(inst->iq_ref,
                                                inst->cfg.motor.flux_wb,
                                                inst->cfg.motor.ld_h,
                                                inst->cfg.motor.lq_h);
    }
    inst->speed_ctrl_enabled = MC_FALSE;
    return MC_STATUS_OK;
}

/**
 * @brief Set direct current references (id, iq) and disable speed control
 * @param inst Motor control instance
 * @param id_ref Direct-axis current reference
 * @param iq_ref Quadrature-axis current reference
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if inst is NULL
 */
mc_status_t mc_set_current_ref_dq(mc_instance_t *inst, mc_f32_t id_ref, mc_f32_t iq_ref)
{
    mc_status_t status = mc_check_instance(inst);

    if (status != MC_STATUS_OK)
    {
        return status;
    }

    inst->id_ref = id_ref;
    inst->iq_ref = iq_ref;
    inst->speed_ctrl_enabled = MC_FALSE;
    return MC_STATUS_OK;
}

/**
 * @brief Run the fast control step (current loop, PWM generation)
 * @param inst Motor control instance
 * @param in Fast input (ADC samples, hall code, resolver, timestamp)
 * @param out Fast output (PWM command, ADC trigger plan, current compensation)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG or MC_STATUS_ERROR on failure
 */
mc_status_t mc_fast_step(mc_instance_t *inst, const mc_fast_input_t *in, mc_fast_output_t *out)
{
    mc_f32_t sin_theta;
    mc_f32_t cos_theta;

    if ((mc_check_instance(inst) != MC_STATUS_OK) || (in == NULL) || (out == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    if (inst->enabled == MC_FALSE)
    {
        inst->diag.current_comp_status = (mc_1shunt_comp_status_t){0};
        inst->diag.sensorless_observer_valid = MC_FALSE;
        inst->diag.sensorless_pll_locked = MC_FALSE;
        inst->diag.sensorless_open_loop_active = MC_FALSE;
        out->pwm_cmd = (mc_pwm_cmd_t){0};
        out->adc_trigger_plan = (mc_adc_trigger_plan_t){0};
        out->current_comp_status = (mc_1shunt_comp_status_t){0};
        return MC_STATUS_OK;
    }

    if (mc_identify_is_done(&inst->identify) != MC_FALSE)
    {
        mc_api_apply_trusted_identify_results(inst);
    }

    if (mc_identify_is_active(&inst->identify) != MC_FALSE)
    {
        mc_identify_input_t id_in;
        mc_identify_output_t id_out;
        mc_alphabeta_t current_ab;

        mc_api_reconstruct_current_ab(inst, &in->adc_raw, &current_ab);
        id_in.current_ab = current_ab;
        id_in.dt_s = inst->cfg.control.current_loop_dt_s;

        mc_identify_run(&inst->identify, &id_in, &id_out);

        if (inst->cfg.foc.current_cfg.type == MC_CURRENT_SENSE_1SHUNT)
        {
            mc_reconstruct_1shunt_plan(&id_out.pwm_cmd,
                                       &inst->cfg.foc.current_cfg.cfg.shunt1,
                                       &inst->foc.pmsm_foc.shunt1_meta);
            mc_api_identify_optimize_1shunt_pwm(&inst->foc.pmsm_foc.shunt1_meta, &id_out.pwm_cmd);
            mc_api_identify_plan_1shunt_adc_trigger(&inst->foc.pmsm_foc.shunt1_meta, &id_out.adc_trigger_plan);
        }

        out->pwm_cmd = id_out.pwm_cmd;
        out->adc_trigger_plan = id_out.adc_trigger_plan;
        out->current_comp_status = (mc_1shunt_comp_status_t){0};
        inst->diag.current_comp_status = out->current_comp_status;
        inst->diag.sensorless_observer_valid = MC_FALSE;
        inst->diag.sensorless_pll_locked = MC_FALSE;
        inst->diag.sensorless_open_loop_active = MC_FALSE;
        inst->diag.active_warning = MC_WARNING_NONE;

        if (mc_identify_is_done(&inst->identify) != MC_FALSE)
        {
            mc_api_apply_trusted_identify_results(inst);
        }

        if (inst->cfg.hooks.pwm_apply != NULL)
        {
            inst->cfg.hooks.pwm_apply(&out->pwm_cmd);
        }
        if (inst->cfg.hooks.adc_trigger_apply != NULL)
        {
            inst->cfg.hooks.adc_trigger_apply(&out->adc_trigger_plan);
        }

        return MC_STATUS_OK;
    }

    if ((inst->mode == MC_MODE_PMSM_FOC_HALL) ||
        (inst->mode == MC_MODE_PMSM_FOC_ENCODER) ||
        (inst->mode == MC_MODE_PMSM_FOC_RESOLVER) ||
        (inst->mode == MC_MODE_PMSM_FOC_SENSORLESS) ||
        (inst->mode == MC_MODE_PMSM_FOC_SMO))
    {
        mc_pmsm_foc_input_t foc_in;

        if (mc_api_update_position(inst, in, &sin_theta, &cos_theta) != MC_STATUS_OK)
        {
            return MC_STATUS_ERROR;
        }

        foc_in.current_raw = in->adc_raw;
        foc_in.bus_voltage_v = (((mc_f32_t)in->adc_raw.bus_voltage_raw) * inst->cfg.foc.bus_voltage_scale) +
                               inst->cfg.foc.bus_voltage_offset;
        foc_in.sin_theta = sin_theta;
        foc_in.cos_theta = cos_theta;
        foc_in.id_ref = inst->id_ref;
        foc_in.iq_ref = (inst->cfg.control.numeric_mode == MC_NUMERIC_Q31) ? mc_q31_to_f32(inst->iq_ref_q31) : inst->iq_ref;
        foc_in.dt_s = inst->cfg.control.current_loop_dt_s;
        foc_in.elec_speed_rad_s = mc_api_get_elec_speed_rad_s(inst);

        if (inst->mode == MC_MODE_PMSM_FOC_SENSORLESS)
        {
            mc_alphabeta_t current_ab;
            mc_alphabeta_t voltage_ab = inst->foc_last_output.v_ab;

            mc_api_reconstruct_current_ab(inst, &in->adc_raw, &current_ab);

            if (mc_sensorless_update(&inst->sensorless,
                                     &voltage_ab,
                                     &current_ab,
                                     inst->cfg.control.current_loop_dt_s,
                                     in->timestamp_us) != MC_STATUS_OK)
            {
                return MC_STATUS_ERROR;
            }

            if ((inst->sensorless.observer_valid != MC_FALSE) && (inst->sensorless.open_loop_active == MC_FALSE))
            {
                foc_in.sin_theta = sinf(inst->sensorless.elec_angle_rad);
                foc_in.cos_theta = cosf(inst->sensorless.elec_angle_rad);
                foc_in.elec_speed_rad_s = mc_api_get_elec_speed_rad_s(inst);
            }
        }

        if (inst->mode == MC_MODE_PMSM_FOC_SMO)
        {
            mc_alphabeta_t current_ab;
            mc_alphabeta_t voltage_ab = inst->foc_last_output.v_ab;

            mc_api_reconstruct_current_ab(inst, &in->adc_raw, &current_ab);

            if (mc_smo_update(&inst->smo,
                              &voltage_ab,
                              &current_ab,
                              inst->cfg.control.current_loop_dt_s,
                              in->timestamp_us) != MC_STATUS_OK)
            {
                return MC_STATUS_ERROR;
            }

            if ((inst->smo.observer_valid != MC_FALSE) && (inst->smo.open_loop_active == MC_FALSE))
            {
                foc_in.sin_theta = sinf(inst->smo.elec_angle_rad);
                foc_in.cos_theta = cosf(inst->smo.elec_angle_rad);
                foc_in.elec_speed_rad_s = mc_api_get_elec_speed_rad_s(inst);
            }
        }

        if (mc_control_foc_run(&inst->foc, &foc_in, &inst->foc_last_output) != MC_STATUS_OK)
        {
            return MC_STATUS_ERROR;
        }

        if ((inst->mode == MC_MODE_PMSM_FOC_SENSORLESS) &&
            (inst->sensorless.open_loop_active != MC_FALSE) &&
            (inst->sensorless.open_loop_voltage > 0.0F))
        {
            mc_alphabeta_t v_ab = inst->foc_last_output.v_ab;
            mc_f32_t v_mag = sqrtf(v_ab.alpha * v_ab.alpha + v_ab.beta * v_ab.beta);
            if (v_mag > inst->sensorless.open_loop_voltage)
            {
                mc_f32_t v_scale = inst->sensorless.open_loop_voltage / v_mag;
                v_ab.alpha *= v_scale;
                v_ab.beta *= v_scale;
                mc_svpwm_run(&v_ab, &inst->cfg.foc.svpwm_cfg, &inst->foc_last_output.pwm_cmd);
                inst->foc_last_output.v_ab = v_ab;
                inst->foc_last_output.v_dq.d *= v_scale;
                inst->foc_last_output.v_dq.q *= v_scale;
            }
        }

        if ((inst->mode == MC_MODE_PMSM_FOC_SMO) &&
            (inst->smo.open_loop_active != MC_FALSE) &&
            (inst->smo.open_loop_voltage > 0.0F))
        {
            mc_alphabeta_t v_ab = inst->foc_last_output.v_ab;
            mc_f32_t v_mag = sqrtf(v_ab.alpha * v_ab.alpha + v_ab.beta * v_ab.beta);
            if (v_mag > inst->smo.open_loop_voltage)
            {
                mc_f32_t v_scale = inst->smo.open_loop_voltage / v_mag;
                v_ab.alpha *= v_scale;
                v_ab.beta *= v_scale;
                mc_svpwm_run(&v_ab, &inst->cfg.foc.svpwm_cfg, &inst->foc_last_output.pwm_cmd);
                inst->foc_last_output.v_ab = v_ab;
                inst->foc_last_output.v_dq.d *= v_scale;
                inst->foc_last_output.v_dq.q *= v_scale;
            }
        }

        out->pwm_cmd = inst->foc_last_output.pwm_cmd;
        out->adc_trigger_plan = inst->foc_last_output.adc_trigger_plan;
        out->current_comp_status = inst->foc_last_output.current_comp_status;
        inst->diag.current_comp_status = out->current_comp_status;
        if (inst->mode == MC_MODE_PMSM_FOC_SENSORLESS)
        {
            inst->diag.sensorless_observer_valid = inst->sensorless.observer_valid;
            inst->diag.sensorless_pll_locked = inst->sensorless.pll_locked;
            inst->diag.sensorless_open_loop_active = inst->sensorless.open_loop_active;
        }
        else if (inst->mode == MC_MODE_PMSM_FOC_SMO)
        {
            inst->diag.sensorless_observer_valid = inst->smo.observer_valid;
            inst->diag.sensorless_pll_locked = inst->smo.pll_locked;
            inst->diag.sensorless_open_loop_active = inst->smo.open_loop_active;
        }
        if (((inst->mode == MC_MODE_PMSM_FOC_SENSORLESS) && (inst->sensorless.pll_locked == MC_FALSE)) ||
            ((inst->mode == MC_MODE_PMSM_FOC_SMO) && (inst->smo.pll_locked == MC_FALSE)))
        {
            inst->diag.active_warning = MC_WARNING_OBSERVER_UNLOCKED;
        }
        else
        {
            inst->diag.active_warning = MC_WARNING_NONE;
        }
        if (inst->cfg.hooks.pwm_apply != NULL)
        {
            inst->cfg.hooks.pwm_apply(&out->pwm_cmd);
        }
        if (inst->cfg.hooks.adc_trigger_apply != NULL)
        {
            inst->cfg.hooks.adc_trigger_apply(&out->adc_trigger_plan);
        }
    }
    else if (inst->mode == MC_MODE_BLDC_HALL)
    {
        inst->diag.current_comp_status = (mc_1shunt_comp_status_t){0};
        inst->diag.sensorless_observer_valid = MC_FALSE;
        inst->diag.sensorless_pll_locked = MC_FALSE;
        inst->diag.sensorless_open_loop_active = MC_FALSE;
        if (mc_hall_update(&inst->hall, in->hall_code, in->timestamp_us, inst->cfg.sensor.pole_pairs) != MC_STATUS_OK)
        {
            return MC_STATUS_ERROR;
        }

        if (mc_bldc_hall_run(&inst->bldc_hall, in->hall_code, inst->torque_ref, &out->pwm_cmd) != MC_STATUS_OK)
        {
            return MC_STATUS_ERROR;
        }

        out->adc_trigger_plan = (mc_adc_trigger_plan_t){0};
        out->current_comp_status = (mc_1shunt_comp_status_t){0};
        if (inst->cfg.hooks.pwm_apply != NULL)
        {
            inst->cfg.hooks.pwm_apply(&out->pwm_cmd);
        }
        if (inst->cfg.hooks.adc_trigger_apply != NULL)
        {
            inst->cfg.hooks.adc_trigger_apply(&out->adc_trigger_plan);
        }
    }
    else if (inst->mode == MC_MODE_BLDC_SENSORLESS)
    {
        mc_f32_t dt_s;
        mc_f32_t bus_voltage_v;
        mc_f32_t floating_phase_v;
        uint16_t floating_raw;
        uint8_t fp;

        inst->diag.current_comp_status = (mc_1shunt_comp_status_t){0};
        inst->diag.sensorless_observer_valid = (inst->bldc_sensorless.phase == MC_BLDC_SENSORLESS_RUN) ? MC_TRUE : MC_FALSE;
        inst->diag.sensorless_pll_locked = MC_FALSE;
        inst->diag.sensorless_open_loop_active = (inst->bldc_sensorless.phase != MC_BLDC_SENSORLESS_RUN) ? MC_TRUE : MC_FALSE;

        dt_s = inst->cfg.control.current_loop_dt_s;
        bus_voltage_v = 0.0F;
        floating_phase_v = 0.0F;

        if (in != NULL)
        {
            bus_voltage_v = (((mc_f32_t)in->adc_raw.bus_voltage_raw) * inst->cfg.foc.bus_voltage_scale) + inst->cfg.foc.bus_voltage_offset;
            fp = mc_bldc_sensorless_floating_phase(inst->bldc_sensorless.commutation_step);
            if (fp == 0U) { floating_raw = in->adc_raw.phase_a_raw; }
            else if (fp == 1U) { floating_raw = in->adc_raw.phase_b_raw; }
            else { floating_raw = in->adc_raw.phase_c_raw; }
            floating_phase_v = ((mc_f32_t)floating_raw / 4095.0F) * bus_voltage_v;
        }

        if (mc_bldc_sensorless_run(&inst->bldc_sensorless, dt_s,
            floating_phase_v, bus_voltage_v, &out->pwm_cmd) != MC_STATUS_OK)
        {
            return MC_STATUS_ERROR;
        }

        out->adc_trigger_plan = (mc_adc_trigger_plan_t){0};
        out->current_comp_status = (mc_1shunt_comp_status_t){0};
        if (inst->cfg.hooks.pwm_apply != NULL)
        {
            inst->cfg.hooks.pwm_apply(&out->pwm_cmd);
        }
        if (inst->cfg.hooks.adc_trigger_apply != NULL)
        {
            inst->cfg.hooks.adc_trigger_apply(&out->adc_trigger_plan);
        }
    }
    else
    {
        inst->diag.current_comp_status = (mc_1shunt_comp_status_t){0};
        inst->diag.sensorless_observer_valid = MC_FALSE;
        inst->diag.sensorless_pll_locked = MC_FALSE;
        inst->diag.sensorless_open_loop_active = MC_FALSE;
        out->pwm_cmd = (mc_pwm_cmd_t){0};
        out->adc_trigger_plan = (mc_adc_trigger_plan_t){0};
        out->current_comp_status = (mc_1shunt_comp_status_t){0};
    }

    return MC_STATUS_OK;
}

/**
 * @brief Run the medium-speed control step (speed loop, encoder update)
 * @param inst Motor control instance
 * @param in Medium input (encoder count, speed feedback, timestamp)
 * @param out Medium output (electrical angle, mechanical speed, current references)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG or MC_STATUS_ERROR on failure
 */
mc_status_t mc_medium_step(mc_instance_t *inst, const mc_medium_input_t *in, mc_medium_output_t *out)
{
    mc_bool_t control_active;

    if ((mc_check_instance(inst) != MC_STATUS_OK) || (in == NULL) || (out == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    control_active = (inst->enabled != MC_FALSE) ? MC_TRUE : MC_FALSE;

    if (inst->mode == MC_MODE_PMSM_FOC_ENCODER)
    {
        if (mc_encoder_update(&inst->encoder, in->encoder_count, in->timestamp_us) != MC_STATUS_OK)
        {
            return MC_STATUS_ERROR;
        }

        out->elec_angle_rad = inst->encoder.elec_angle_rad;
        out->mech_speed_rpm = inst->encoder.mech_speed_rpm;

        if ((control_active != MC_FALSE) && (inst->speed_ctrl_enabled != MC_FALSE) && (inst->cfg.foc.speed_loop_dt_s > 0.0F))
        {
            if (inst->cfg.control.numeric_mode == MC_NUMERIC_Q31)
            {
                inst->iq_ref_q31 = mc_pi_q31_run(&inst->foc_speed_pi_q31,
                                                 mc_q31_from_f32((inst->speed_ref_rpm - inst->encoder.mech_speed_rpm) / 10000.0F),
                                                 mc_q31_from_f32(inst->cfg.foc.speed_loop_dt_s));
                inst->iq_ref = mc_q31_to_f32(inst->iq_ref_q31) * 10000.0F;
                if (inst->cfg.foc.iq_limit > 0.0F)
                {
                    inst->iq_ref = mc_math_clamp_f32(inst->iq_ref, -inst->cfg.foc.iq_limit, inst->cfg.foc.iq_limit);
                    inst->iq_ref_q31 = mc_q31_from_f32(inst->iq_ref / 10000.0F);
                }
            }
            else if (mc_control_foc_speed_step(&inst->foc,
                                               inst->speed_ref_rpm,
                                               inst->encoder.mech_speed_rpm,
                                               inst->cfg.foc.speed_loop_dt_s,
                                               &inst->iq_ref) != MC_STATUS_OK)
            {
                return MC_STATUS_ERROR;
            }
        }
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_HALL)
    {
        out->elec_angle_rad = inst->hall.elec_angle_rad;
        out->mech_speed_rpm = inst->hall.mech_speed_rpm;

        if ((control_active != MC_FALSE) && (inst->speed_ctrl_enabled != MC_FALSE) && (inst->cfg.foc.speed_loop_dt_s > 0.0F))
        {
            if (inst->cfg.control.numeric_mode == MC_NUMERIC_Q31)
            {
                inst->iq_ref_q31 = mc_pi_q31_run(&inst->foc_speed_pi_q31,
                                                 mc_q31_from_f32((inst->speed_ref_rpm - inst->hall.mech_speed_rpm) / 10000.0F),
                                                 mc_q31_from_f32(inst->cfg.foc.speed_loop_dt_s));
                inst->iq_ref = mc_q31_to_f32(inst->iq_ref_q31) * 10000.0F;
                if (inst->cfg.foc.iq_limit > 0.0F)
                {
                    inst->iq_ref = mc_math_clamp_f32(inst->iq_ref, -inst->cfg.foc.iq_limit, inst->cfg.foc.iq_limit);
                    inst->iq_ref_q31 = mc_q31_from_f32(inst->iq_ref / 10000.0F);
                }
            }
            else if (mc_control_foc_speed_step(&inst->foc,
                                               inst->speed_ref_rpm,
                                               inst->hall.mech_speed_rpm,
                                               inst->cfg.foc.speed_loop_dt_s,
                                               &inst->iq_ref) != MC_STATUS_OK)
            {
                return MC_STATUS_ERROR;
            }
        }
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_RESOLVER)
    {
        out->elec_angle_rad = inst->resolver.elec_angle_rad;
        out->mech_speed_rpm = inst->resolver.mech_speed_rpm;

        if ((control_active != MC_FALSE) && (inst->speed_ctrl_enabled != MC_FALSE) && (inst->cfg.foc.speed_loop_dt_s > 0.0F))
        {
            if (mc_control_foc_speed_step(&inst->foc,
                                          inst->speed_ref_rpm,
                                          inst->resolver.mech_speed_rpm,
                                          inst->cfg.foc.speed_loop_dt_s,
                                          &inst->iq_ref) != MC_STATUS_OK)
            {
                return MC_STATUS_ERROR;
            }
        }
    }
    else if (inst->mode == MC_MODE_PMSM_FOC_SENSORLESS)
    {
        out->elec_angle_rad = inst->sensorless.elec_angle_rad;
        out->mech_speed_rpm = inst->sensorless.mech_speed_rpm;

        if ((control_active != MC_FALSE) && (inst->speed_ctrl_enabled != MC_FALSE) && (inst->cfg.foc.speed_loop_dt_s > 0.0F))
        {
            if (mc_control_foc_speed_step(&inst->foc,
                                          inst->speed_ref_rpm,
                                          inst->sensorless.mech_speed_rpm,
                                          inst->cfg.foc.speed_loop_dt_s,
                                          &inst->iq_ref) != MC_STATUS_OK)
            {
                return MC_STATUS_ERROR;
            }
        }
    }
    else if (inst->mode == MC_MODE_BLDC_HALL)
    {
        out->elec_angle_rad = inst->hall.elec_angle_rad;
        out->mech_speed_rpm = inst->hall.mech_speed_rpm;
    }
    else if (inst->mode == MC_MODE_BLDC_SENSORLESS)
    {
        out->elec_angle_rad = 0.0F;
        out->mech_speed_rpm = mc_bldc_sensorless_get_speed_rpm(&inst->bldc_sensorless);

        if ((control_active != MC_FALSE) && (inst->speed_ctrl_enabled != MC_FALSE) && (inst->cfg.foc.speed_loop_dt_s > 0.0F))
        {
            mc_bldc_sensorless_speed_step(
                &inst->bldc_sensorless,
                inst->speed_ref_rpm,
                inst->cfg.foc.speed_loop_dt_s);
        }
    }
    else
    {
        out->elec_angle_rad = 0.0F;
        out->mech_speed_rpm = in->speed_feedback_rpm;
    }

    if ((control_active != MC_FALSE) && (inst->cfg.foc.mtpa_enable != MC_FALSE) && (inst->speed_ctrl_enabled != MC_FALSE))
    {
        inst->id_ref = mc_pmsm_compute_mtpa_id(inst->iq_ref,
                                                 inst->cfg.motor.flux_wb,
                                                 inst->cfg.motor.ld_h,
                                                inst->cfg.motor.lq_h);
    }

    out->id_ref = inst->id_ref;
    out->iq_ref = inst->iq_ref;

    return MC_STATUS_OK;
}

/**
 * @brief Run the slow control step (fault handling, diagnostics update)
 * @param inst Motor control instance
 * @param in Slow input (clear fault request)
 * @param out Slow output (mode, diagnostics)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if pointers are NULL
 */
mc_status_t mc_slow_step(mc_instance_t *inst, const mc_slow_input_t *in, mc_slow_output_t *out)
{
    if ((mc_check_instance(inst) != MC_STATUS_OK) || (in == NULL) || (out == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    if (in->clear_fault_request != MC_FALSE)
    {
        inst->diag = (mc_diag_status_t){0};
    }

    out->mode = inst->mode;
    out->diag = inst->diag;

    return MC_STATUS_OK;
}

/**
 * @brief Get current operating status (mode and diagnostics)
 * @param inst Motor control instance
 * @param status Output status structure
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if pointers are NULL
 */
mc_status_t mc_get_status(const mc_instance_t *inst, mc_slow_output_t *status)
{
    if ((mc_check_instance(inst) != MC_STATUS_OK) || (status == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    status->mode = inst->mode;
    status->diag = inst->diag;

    return MC_STATUS_OK;
}

/**
 * @brief Get current diagnostic status from the motor control instance
 * @param inst Motor control instance
 * @param diag Output diagnostic status
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if pointers are NULL
 */
mc_status_t mc_get_diag(const mc_instance_t *inst, mc_diag_status_t *diag)
{
    if ((mc_check_instance(inst) != MC_STATUS_OK) || (diag == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    *diag = inst->diag;

    return MC_STATUS_OK;
}

/**
 * @brief Get the library version as a 32-bit integer
 * @param version Output version number (MC_VERSION_U32)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if version is NULL
 */
mc_status_t mc_get_version(uint32_t *version)
{
    if (version == NULL)
    {
        return MC_STATUS_INVALID_ARG;
    }

    *version = MC_VERSION_U32;
    return MC_STATUS_OK;
}

mc_status_t mc_auto_tune_pi(mc_instance_t *inst,
                             mc_f32_t bandwidth_divider,
                             mc_f32_t speed_ratio)
{
    mc_pi_cfg_t id_cfg;
    mc_pi_cfg_t iq_cfg;
    mc_pi_cfg_t speed_cfg;
    mc_status_t status;

    if (inst == NULL)
    {
        return MC_STATUS_INVALID_ARG;
    }

    if (bandwidth_divider < 1.0F)
    {
        bandwidth_divider = 20.0F;
    }
    if (speed_ratio < 1.0F)
    {
        speed_ratio = 10.0F;
    }

    status = mc_auto_tune_current_pi(inst->cfg.motor.rs_ohm,
                                      inst->cfg.motor.ld_h,
                                      inst->cfg.motor.lq_h,
                                      inst->cfg.control.pwm_frequency_hz,
                                      bandwidth_divider,
                                      inst->cfg.foc.voltage_limit,
                                      inst->cfg.foc.iq_limit,
                                      &id_cfg,
                                      &iq_cfg);
    if (status != MC_STATUS_OK)
    {
        return status;
    }

    status = mc_auto_tune_speed_pi(inst->cfg.motor.ld_h,
                                    inst->cfg.control.pwm_frequency_hz,
                                    bandwidth_divider,
                                    speed_ratio,
                                    inst->cfg.foc.iq_limit,
                                    &speed_cfg);
    if (status != MC_STATUS_OK)
    {
        return status;
    }

    inst->cfg.foc.id_pi_cfg = id_cfg;
    inst->cfg.foc.iq_pi_cfg = iq_cfg;
    inst->cfg.foc.speed_pi_cfg = speed_cfg;

    if ((inst->mode == MC_MODE_PMSM_FOC_HALL) ||
        (inst->mode == MC_MODE_PMSM_FOC_ENCODER) ||
        (inst->mode == MC_MODE_PMSM_FOC_RESOLVER) ||
        (inst->mode == MC_MODE_PMSM_FOC_SENSORLESS) ||
        (inst->mode == MC_MODE_PMSM_FOC_SMO))
    {
        mc_control_foc_set_speed_pi(&inst->foc, &speed_cfg, inst->cfg.foc.iq_limit);
    }

    return MC_STATUS_OK;
}
