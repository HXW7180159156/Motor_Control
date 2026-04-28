/**
 * @file mc_identify.c
 * @brief Motor parameter identification implementation
 */
#include "mc_constants.h"
#include "mc_identify.h"
#include "mc_math.h"
#include "mc_transform.h"

#include <math.h>

/**
 * @brief Set the identification state machine state and reset accumulators
 * @param id Identification instance
 * @param state New state to enter
 */
static void mc_identify_set_state(mc_identify_t *id, mc_identify_state_t state)
{
    id->state = state;
    id->step_counter = 0U;
    id->sample_count = 0U;
    id->id_acc = 0.0F;
    id->i_alpha_window_acc = 0.0F;
    id->i_beta_window_acc = 0.0F;
    id->pulse_time_acc_s = 0.0F;
}

/**
 * @brief Reset the flux estimate accumulation window
 * @param id Identification instance
 */
static void mc_identify_reset_flux_window(mc_identify_t *id)
{
    if (id == NULL)
    {
        return;
    }

    id->flux_acc = 0.0F;
    id->flux_time_acc_s = 0.0F;
    id->flux_sample_count = 0U;
}

/**
 * @brief Finalize the averaged flux estimate if valid samples were collected
 * @param id Identification instance
 */
static void mc_identify_finalize_flux_estimate(mc_identify_t *id)
{
    mc_f32_t flux_avg;
    mc_f32_t min_valid_flux_time_s;

    if ((id == NULL) || (id->flux_sample_count == 0U) || (id->flux_time_acc_s <= 0.0F))
    {
        return;
    }

    flux_avg = id->flux_acc / id->flux_time_acc_s;
    if (!(flux_avg > 0.0F))
    {
        return;
    }

    id->flux_candidate_wb = flux_avg;
    min_valid_flux_time_s = MC_IDENTIFY_MIN_VALID_FLUX_WINDOW_RATIO * id->cfg.lq_pulse_time_s;
    if (id->flux_time_acc_s < min_valid_flux_time_s)
    {
        return;
    }

    id->flux_wb = flux_avg;
}

/**
 * @brief Return a parameter estimate only when it is finite and strictly positive
 * @param estimate Raw identified parameter estimate
 * @return Positive estimate, or 0.0F when invalid/non-physical
 */
static mc_f32_t mc_identify_positive_estimate_or_zero(mc_f32_t estimate)
{
    if (isfinite(estimate) && (estimate > 0.0F))
    {
        return estimate;
    }

    return 0.0F;
}

/**
 * @brief Estimate inductance from a pulse response after subtracting the window-averaged resistive voltage drop
 * @param id Identification instance
 * @param current_start Current at the beginning of the pulse window
 * @param current_end Current at the end of the pulse window
 * @param current_window_avg Average current over the active pulse window
 * @param pulse_time_s Applied pulse duration in seconds
 * @return Positive inductance estimate, or 0.0F when invalid/non-physical
 */
static mc_f32_t mc_identify_compute_inductance_estimate(const mc_identify_t *id,
                                                        mc_f32_t current_start,
                                                        mc_f32_t current_end,
                                                        mc_f32_t current_window_avg,
                                                        mc_f32_t pulse_time_s)
{
    mc_f32_t delta_i;
    mc_f32_t current_avg_abs;
    mc_f32_t applied_v;
    mc_f32_t inductive_v;

    if ((id == NULL) || (pulse_time_s <= 0.0F))
    {
        return 0.0F;
    }

    delta_i = current_end - current_start;
    if (fabsf(delta_i) <= MC_EPSILON_F)
    {
        return 0.0F;
    }

    applied_v = fabsf(id->cfg.pulse_voltage * id->cfg.voltage_limit);
    current_avg_abs = fabsf(current_window_avg);
    inductive_v = applied_v - (id->rs_ohm * current_avg_abs);
    if (!isfinite(inductive_v) || (inductive_v <= 0.0F))
    {
        return 0.0F;
    }

    return mc_identify_positive_estimate_or_zero((inductive_v * pulse_time_s) / delta_i);
}

/**
 * @brief Estimate flux linkage from the q-axis voltage balance during open-loop injection
 * @param id Identification instance
 * @param i_beta Current q-axis sample proxy
 * @param dt_s Sampling interval in seconds
 */
static void mc_identify_update_flux_estimate(mc_identify_t *id, mc_f32_t i_beta, mc_f32_t dt_s)
{
    mc_f32_t di_beta_dt;
    mc_f32_t omega_elec;
    mc_f32_t effective_vq;
    mc_f32_t flux_estimate;

    if ((id == NULL) || (dt_s <= 0.0F) || (id->lq_h <= 0.0F) || (id->cfg.lq_align_time_s <= 0.0F))
    {
        return;
    }

    if (!isfinite(id->v_beta) || !isfinite(i_beta))
    {
        return;
    }

    /* Ignore samples that oppose the applied q-axis excitation direction. */
    if ((id->v_beta * i_beta) <= 0.0F)
    {
        return;
    }

    di_beta_dt = (i_beta - id->i_beta_prev) / dt_s;
    omega_elec = MC_PI_OVER_2 / id->cfg.lq_align_time_s;
    if (omega_elec <= 0.0F)
    {
        return;
    }

    effective_vq = fabsf(id->v_beta);
    flux_estimate = (effective_vq - (id->rs_ohm * i_beta) - (id->lq_h * di_beta_dt)) / omega_elec;
    if (isfinite(flux_estimate) && (flux_estimate > MC_IDENTIFY_MIN_FLUX_SAMPLE_WB))
    {
        id->flux_acc += flux_estimate * dt_s;
        id->flux_time_acc_s += dt_s;
        id->flux_sample_count++;
    }
}

/**
 * @brief Compute number of PWM steps for a given time duration
 * @param time_s Time duration in seconds
 * @param dt_s Control-loop step time in seconds
 * @return Number of steps (PWM cycles)
 */
static uint32_t mc_identify_compute_steps(mc_f32_t time_s, mc_f32_t dt_s)
{
    uint32_t steps;

    if ((time_s <= 0.0F) || (dt_s <= 0.0F))
    {
        return 0U;
    }

    steps = (uint32_t)ceilf(time_s / dt_s);
    return (steps > 0U) ? steps : 1U;
}

/**
 * @brief Normalize reference voltage (currently passthrough)
 * @param id Identification instance (unused)
 * @param v_ref Reference voltage
 * @return Normalized voltage value
 */
static mc_f32_t mc_identify_normalized_voltage(mc_identify_t *id, mc_f32_t v_ref)
{
    (void)id;
    return v_ref;
}

/**
 * @brief Initialise identification instance with configuration defaults
 * @param id Identification instance
 * @param cfg Identification configuration (defaults applied if values are zero)
 */
void mc_identify_init(mc_identify_t *id, const mc_identify_cfg_t *cfg)
{
    if ((id == NULL) || (cfg == NULL))
    {
        return;
    }

    *id = (mc_identify_t){0};
    id->cfg = *cfg;

    if (id->cfg.align_time_s <= 0.0F) id->cfg.align_time_s = MC_IDENTIFY_DEFAULT_ALIGN_TIME_S;
    if (id->cfg.rs_settle_time_s <= 0.0F) id->cfg.rs_settle_time_s = MC_IDENTIFY_DEFAULT_RS_SETTLE_TIME_S;
    if (id->cfg.rs_sample_time_s <= 0.0F) id->cfg.rs_sample_time_s = MC_IDENTIFY_DEFAULT_RS_SAMPLE_TIME_S;
    if (id->cfg.ld_pulse_time_s <= 0.0F) id->cfg.ld_pulse_time_s = MC_IDENTIFY_DEFAULT_PULSE_TIME_S;
    if (id->cfg.lq_align_time_s <= 0.0F) id->cfg.lq_align_time_s = MC_IDENTIFY_DEFAULT_LQ_ALIGN_TIME_S;
    if (id->cfg.lq_pulse_time_s <= 0.0F) id->cfg.lq_pulse_time_s = MC_IDENTIFY_DEFAULT_PULSE_TIME_S;
    if (id->cfg.align_voltage <= 0.0F) id->cfg.align_voltage = MC_IDENTIFY_DEFAULT_ALIGN_VOLTAGE;
    if (id->cfg.rs_voltage <= 0.0F) id->cfg.rs_voltage = MC_IDENTIFY_DEFAULT_RS_VOLTAGE;
    if (id->cfg.pulse_voltage <= 0.0F) id->cfg.pulse_voltage = MC_IDENTIFY_DEFAULT_PULSE_VOLTAGE;
    if (id->cfg.max_current_a <= 0.0F) id->cfg.max_current_a = MC_IDENTIFY_DEFAULT_MAX_CURRENT_A;
    if (id->cfg.voltage_limit <= 0.0F) id->cfg.voltage_limit = MC_IDENTIFY_DEFAULT_VOLTAGE_LIMIT;
}

/**
 * @brief Start the identification sequence (enters ALIGN state)
 * @param id Identification instance
 */
void mc_identify_start(mc_identify_t *id)
{
    if (id == NULL)
    {
        return;
    }

    mc_identify_reset_flux_window(id);
    mc_identify_set_state(id, MC_IDENTIFY_STATE_ALIGN);
}

/**
 * @brief Check if the identification sequence has completed
 * @param id Identification instance
 * @return MC_TRUE if in COMPLETE state, MC_FALSE otherwise or if id is NULL
 */
mc_bool_t mc_identify_is_done(const mc_identify_t *id)
{
    if (id == NULL)
    {
        return MC_FALSE;
    }
    return (id->state == MC_IDENTIFY_STATE_COMPLETE) ? MC_TRUE : MC_FALSE;
}

/**
 * @brief Check if the identification sequence is currently active
 * @param id Identification instance
 * @return MC_TRUE if in a running state (not IDLE, COMPLETE, or ERROR), MC_FALSE otherwise
 */
mc_bool_t mc_identify_is_active(const mc_identify_t *id)
{
    if (id == NULL)
    {
        return MC_FALSE;
    }
    return ((id->state != MC_IDENTIFY_STATE_IDLE) &&
            (id->state != MC_IDENTIFY_STATE_COMPLETE) &&
            (id->state != MC_IDENTIFY_STATE_ERROR)) ? MC_TRUE : MC_FALSE;
}

/**
 * @brief Get identified motor parameters (Rs, Ld, Lq, flux)
 * @param id Identification instance
 * @param rs_ohm Output stator resistance in ohms (can be NULL)
 * @param ld_h Output d-axis inductance in henries (can be NULL)
 * @param lq_h Output q-axis inductance in henries (can be NULL)
 * @param flux_wb Output flux linkage in webers (can be NULL)
 */
void mc_identify_get_result(const mc_identify_t *id, mc_f32_t *rs_ohm, mc_f32_t *ld_h, mc_f32_t *lq_h, mc_f32_t *flux_wb)
{
    if (id == NULL)
    {
        return;
    }
    if (rs_ohm != NULL) *rs_ohm = id->rs_ohm;
    if (ld_h != NULL)   *ld_h = id->ld_h;
    if (lq_h != NULL)   *lq_h = id->lq_h;
    if (flux_wb != NULL) *flux_wb = id->flux_wb;
}

/**
 * @brief Run one step of the identification state machine
 * @param id Identification instance
 * @param in Identification input (alpha-beta current, dt)
 * @param out Identification output (PWM command, ADC trigger plan)
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG on bad input
 */
mc_status_t mc_identify_run(mc_identify_t *id, const mc_identify_input_t *in, mc_identify_output_t *out)
{
    mc_alphabeta_t voltage_ab;
    mc_f32_t v_mag;
    mc_f32_t i_alpha;

    if ((id == NULL) || (in == NULL) || (out == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }

    if (in->dt_s <= 0.0F)
    {
        return MC_STATUS_INVALID_ARG;
    }

    *out = (mc_identify_output_t){0};

    if ((id->state == MC_IDENTIFY_STATE_IDLE) ||
        (id->state == MC_IDENTIFY_STATE_COMPLETE) ||
        (id->state == MC_IDENTIFY_STATE_ERROR))
    {
        return MC_STATUS_OK;
    }

    out->adc_trigger_plan.count = 1U;
    out->adc_trigger_plan.trigger_a.valid = MC_TRUE;
    out->adc_trigger_plan.trigger_a.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
    out->adc_trigger_plan.trigger_a.event = MC_ADC_TRIGGER_EVENT_PWM_DOWN;
    out->adc_trigger_plan.trigger_a.position = 0.5F;

    v_mag = mc_identify_normalized_voltage(id, id->cfg.voltage_limit);
    i_alpha = in->current_ab.alpha;

    if ((fabsf(i_alpha) > id->cfg.max_current_a) ||
        (fabsf(in->current_ab.beta) > id->cfg.max_current_a))
    {
        mc_identify_set_state(id, MC_IDENTIFY_STATE_ERROR);
        out->pwm_cmd = (mc_pwm_cmd_t){0};
        out->adc_trigger_plan = (mc_adc_trigger_plan_t){0};
        return MC_STATUS_OK;
    }

    switch (id->state)
    {
    case MC_IDENTIFY_STATE_ALIGN:
    {
        id->total_steps = mc_identify_compute_steps(id->cfg.align_time_s, in->dt_s);
        id->v_alpha = id->cfg.align_voltage * v_mag;
        id->v_beta = 0.0F;
        id->step_counter++;
        if (id->step_counter >= id->total_steps)
        {
            id->i_alpha_prev = i_alpha;
            mc_identify_set_state(id, MC_IDENTIFY_STATE_RS_SETTLE);
        }
        break;
    }

    case MC_IDENTIFY_STATE_RS_SETTLE:
    {
        id->total_steps = mc_identify_compute_steps(id->cfg.rs_settle_time_s, in->dt_s);
        id->v_alpha = id->cfg.rs_voltage * v_mag;
        id->v_beta = 0.0F;
        id->step_counter++;
        if (id->step_counter >= id->total_steps)
        {
            mc_identify_set_state(id, MC_IDENTIFY_STATE_RS_MEASURE);
        }
        break;
    }

    case MC_IDENTIFY_STATE_RS_MEASURE:
    {
        id->total_steps = mc_identify_compute_steps(id->cfg.rs_sample_time_s, in->dt_s);
        id->v_alpha = id->cfg.rs_voltage * v_mag;
        id->v_beta = 0.0F;
        id->id_acc += i_alpha;
        id->sample_count++;
        id->step_counter++;
        if (id->step_counter >= id->total_steps)
        {
            if (id->sample_count > 0U)
            {
                mc_f32_t id_avg = id->id_acc / (mc_f32_t)id->sample_count;
                mc_f32_t rs_estimate = (fabsf(id_avg) > MC_EPSILON_F) ? (id->cfg.rs_voltage * id->cfg.voltage_limit) / id_avg : 0.0F;

                id->rs_candidate_ohm = rs_estimate;
                id->rs_ohm = mc_identify_positive_estimate_or_zero(rs_estimate);
            }
            mc_identify_set_state(id, MC_IDENTIFY_STATE_LD_INJECT);
        }
        break;
    }

    case MC_IDENTIFY_STATE_LD_INJECT:
    {
        mc_f32_t i_alpha_window_avg;
        mc_f32_t pulse_time_s;

        id->total_steps = mc_identify_compute_steps(id->cfg.ld_pulse_time_s, in->dt_s);
        if (id->step_counter == 0U)
        {
            id->i_alpha_start = id->i_alpha_prev;
        }

        id->v_alpha = id->cfg.pulse_voltage * v_mag;
        id->v_beta = 0.0F;
        id->i_alpha_window_acc += i_alpha * in->dt_s;
        id->pulse_time_acc_s += in->dt_s;
        id->sample_count++;
        id->step_counter++;

        if (id->pulse_time_acc_s >= id->cfg.ld_pulse_time_s)
        {
            pulse_time_s = id->pulse_time_acc_s;
            i_alpha_window_avg = (pulse_time_s > 0.0F) ?
                (id->i_alpha_window_acc / pulse_time_s) :
                i_alpha;
            mc_f32_t ld_estimate = mc_identify_compute_inductance_estimate(id,
                                                                           id->i_alpha_start,
                                                                           i_alpha,
                                                                           i_alpha_window_avg,
                                                                           pulse_time_s);

            id->ld_candidate_h = ld_estimate;
            id->ld_h = mc_identify_positive_estimate_or_zero(ld_estimate);
            id->i_alpha_prev = i_alpha;
            mc_identify_set_state(id, MC_IDENTIFY_STATE_LD_MEASURE);
        }
        break;
    }

    case MC_IDENTIFY_STATE_LD_MEASURE:
    {
        id->total_steps = mc_identify_compute_steps(id->cfg.ld_pulse_time_s, in->dt_s);
        id->v_alpha = 0.0F;
        id->v_beta = 0.0F;
        id->step_counter++;
        if (id->step_counter >= id->total_steps)
        {
            mc_identify_set_state(id, MC_IDENTIFY_STATE_LQ_ALIGN);
        }
        break;
    }

    case MC_IDENTIFY_STATE_LQ_ALIGN:
    {
        id->total_steps = mc_identify_compute_steps(id->cfg.lq_align_time_s, in->dt_s);
        id->v_alpha = 0.0F;
        id->v_beta = id->cfg.pulse_voltage * v_mag;
        id->step_counter++;
        if (id->step_counter >= id->total_steps)
        {
            mc_identify_set_state(id, MC_IDENTIFY_STATE_LQ_INJECT);
        }
        break;
    }

    case MC_IDENTIFY_STATE_LQ_INJECT:
    {
        mc_f32_t i_beta_window_avg;
        mc_f32_t pulse_time_s;

        id->total_steps = mc_identify_compute_steps(id->cfg.lq_pulse_time_s, in->dt_s);
        if (id->step_counter == 0U)
        {
            id->i_beta_start = id->i_beta_prev;
            mc_identify_reset_flux_window(id);
        }

        id->v_alpha = 0.0F;
        id->v_beta = id->cfg.pulse_voltage * v_mag;
        id->i_beta_window_acc += in->current_ab.beta * in->dt_s;
        id->pulse_time_acc_s += in->dt_s;
        id->sample_count++;
        id->step_counter++;
        if (id->lq_h > 0.0F)
        {
            mc_identify_update_flux_estimate(id, in->current_ab.beta, in->dt_s);
        }

        if (id->pulse_time_acc_s >= id->cfg.lq_pulse_time_s)
        {
            pulse_time_s = id->pulse_time_acc_s;
            i_beta_window_avg = (pulse_time_s > 0.0F) ?
                (id->i_beta_window_acc / pulse_time_s) :
                in->current_ab.beta;
            mc_f32_t lq_estimate = mc_identify_compute_inductance_estimate(id,
                                                                           id->i_beta_start,
                                                                           in->current_ab.beta,
                                                                           i_beta_window_avg,
                                                                           pulse_time_s);

            id->lq_candidate_h = lq_estimate;
            id->lq_h = mc_identify_positive_estimate_or_zero(lq_estimate);
            mc_identify_set_state(id, MC_IDENTIFY_STATE_LQ_MEASURE);
        }
        break;
    }

    case MC_IDENTIFY_STATE_LQ_MEASURE:
    {
        id->total_steps = mc_identify_compute_steps(id->cfg.lq_pulse_time_s, in->dt_s);
        id->v_alpha = 0.0F;
        id->v_beta = id->cfg.pulse_voltage * v_mag;
        id->pulse_time_acc_s += in->dt_s;
        id->step_counter++;
        mc_identify_update_flux_estimate(id, in->current_ab.beta, in->dt_s);
        if (id->pulse_time_acc_s >= id->cfg.lq_pulse_time_s)
        {
            mc_identify_finalize_flux_estimate(id);
            id->v_alpha = 0.0F;
            id->v_beta = 0.0F;
            if (id->flux_wb < 0.0F)
            {
                id->flux_wb = 0.0F;
            }
            mc_identify_set_state(id, MC_IDENTIFY_STATE_COMPLETE);
        }
        break;
    }

    default:
    {
        id->v_alpha = 0.0F;
        id->v_beta = 0.0F;
        break;
    }
    }

    id->i_alpha_prev = i_alpha;
    id->i_beta_prev = in->current_ab.beta;

    voltage_ab.alpha = id->v_alpha;
    voltage_ab.beta = id->v_beta;
    mc_svpwm_run(&voltage_ab, &id->cfg.svpwm_cfg, &out->pwm_cmd);

    return MC_STATUS_OK;
}
