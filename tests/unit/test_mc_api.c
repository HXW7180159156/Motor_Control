#include "unity.h"
#include "mc_api.h"

#include <math.h>
#include <string.h>

static mc_pwm_cmd_t g_last_pwm_cmd;
static mc_adc_trigger_plan_t g_last_adc_trigger_plan;
static uint8_t g_pwm_apply_called;
static uint8_t g_adc_trigger_apply_called;

static void test_mc_api_pwm_apply_stub(const void *cmd)
{
    g_last_pwm_cmd = *(const mc_pwm_cmd_t *)cmd;
    g_pwm_apply_called += 1U;
}

static void test_mc_api_adc_trigger_apply_stub(const mc_adc_trigger_plan_t *plan)
{
    g_last_adc_trigger_plan = *plan;
    g_adc_trigger_apply_called += 1U;
}

void test_mc_math_clamp_and_wrap_work(void);
void test_mc_q31_helpers_convert_and_saturate(void);
void test_mc_clarke_computes_alpha_beta(void);
void test_mc_park_and_ipark_round_trip(void);
void test_mc_q31_transform_round_trip(void);
void test_mc_control_foc_delegates_to_pmsm_foc(void);
void test_mc_control_foc_speed_step_generates_iq_ref(void);
void test_mc_pi_generates_bounded_output(void);
void test_mc_pi_clamps_integral_and_output(void);
void test_mc_svpwm_generates_valid_centered_duties(void);
void test_mc_svpwm_limits_modulation_vector(void);
void test_mc_adc_map_trigger_plan_maps_valid_triggers(void);
void test_mc_adc_map_trigger_plan_keeps_invalid_trigger_empty(void);
void test_mc_adc_ref_build_apply_cfg_maps_hw_triggers(void);
void test_mc_adc_ref_build_apply_cfg_leaves_empty_slots_disabled(void);
void test_mc_adc_ref_apply_invokes_reference_port_callbacks(void);
void test_mc_adc_ref_apply_plan_runs_full_reference_chain(void);
void test_mc_reconstruct_1shunt_maps_sector_currents(void);
void test_mc_reconstruct_1shunt_invalid_window_returns_zero(void);
void test_mc_reconstruct_1shunt_high_modulation_prefers_trailing_samples(void);
void test_mc_reconstruct_2shunt_derives_third_phase(void);
void test_mc_reconstruct_3shunt_applies_offset_and_scale(void);
void test_mc_encoder_update_computes_angle_and_speed(void);
void test_mc_hall_update_maps_angle_and_speed(void);
void test_mc_resolver_reports_excitation_command(void);
void test_mc_resolver_update_decodes_angle_and_speed(void);
void test_mc_resolver_update_flags_low_signal_invalid(void);
void test_mc_sensorless_update_estimates_angle_and_speed(void);
void test_mc_sensorless_update_flags_low_bemf_invalid(void);
void test_mc_sensorless_init_rejects_invalid_pll_gains(void);
void test_mc_sensorless_init_rejects_lock_bemf_not_above_min_bemf(void);
void test_mc_sensorless_init_rejects_open_loop_voltage_max_below_start(void);
void test_mc_sensorless_uses_inductance_compensation(void);
void test_mc_sensorless_requires_consecutive_lock_samples(void);
void test_mc_sensorless_requires_consecutive_unlock_samples(void);
void test_mc_sensorless_open_loop_handoff_requires_lock_bemf_threshold(void);
void test_mc_sensorless_low_bemf_resets_lock_and_unlock_counters(void);
void test_mc_smo_init_accepts_valid_config(void);
void test_mc_smo_init_rejects_null_state(void);
void test_mc_smo_init_rejects_null_cfg(void);
void test_mc_smo_init_rejects_zero_pole_pairs(void);
void test_mc_smo_init_rejects_zero_k_slide(void);
void test_mc_smo_init_rejects_lock_bemf_below_min(void);
void test_mc_smo_init_rejects_open_loop_voltage_inverted(void);
void test_mc_smo_update_null_state_rejected(void);
void test_mc_smo_update_null_voltage_rejected(void);
void test_mc_smo_update_zero_current_does_not_diverge(void);
void test_mc_smo_update_valid_bemf_sets_observer_valid(void);
void test_mc_smo_open_loop_startup_advances_angle(void);
void test_mc_bldc_hall_commutates_phase_modes(void);
void test_default_cfg_has_reasonable_values(void);
void test_init_sets_idle_phase(void);
void test_init_null_returns_error(void);
void test_init_null_cfg_returns_error(void);
void test_init_rejects_zero_align_time(void);
void test_init_rejects_zero_ramp_start_freq(void);
void test_init_rejects_ramp_end_less_than_start(void);
void test_init_rejects_zero_ramp_time(void);
void test_init_rejects_start_duty_negative(void);
void test_init_rejects_start_duty_over_one(void);
void test_init_rejects_end_duty_less_than_start(void);
void test_init_rejects_end_duty_over_one(void);
void test_init_accepts_end_duty_equal_one(void);
void test_init_accepts_end_freq_equal_start(void);
void test_start_transitions_from_idle_to_align(void);
void test_start_null_returns_error(void);
void test_reset_returns_to_idle(void);
void test_reset_preserves_configuration(void);
void test_reset_null_returns_error(void);
void test_idle_produces_zero_pwm(void);
void test_align_commutates_and_transitions_to_ramp(void);
void test_ramp_open_step_advances_on_timer(void);
void test_run_detects_zc_and_commutates(void);
void test_run_fallback_on_missing_zc(void);
void test_ramp_advances_frequency_over_time(void);
void test_floating_phase_map(void);
void test_get_speed_rpm_returns_stored_value(void);
void test_get_speed_rpm_null_returns_zero(void);
void test_run_advances_to_run_after_ramp(void);
void test_run_null_pwm_returns_error(void);
void test_run_null_ss_returns_error(void);
void test_zero_dt_does_not_advance_timer(void);
void test_large_dt_skips_align_directly_to_ramp(void);
void test_zero_bus_voltage_no_zc_detected(void);
void test_bemf_threshold_blocks_small_crossing(void);
void test_bemf_threshold_allows_large_crossing(void);
void test_align_with_already_expired_timer_immediately_transitions(void);
void test_ramp_saturates_at_end_values(void);
void test_invalid_step_apply_returns_invalid_pwm(void);
void test_debounce_rejects_single_detection_below_threshold(void);
void test_debounce_resets_on_missing_crossing(void);
void test_speed_pi_is_initialized_after_init(void);
void test_speed_step_not_in_run_resets_pi(void);
void test_speed_step_adjusts_duty_in_run(void);
void test_speed_step_null_ss_does_nothing(void);
void test_speed_step_zero_dt_does_nothing(void);
void test_speed_step_negative_dt_does_nothing(void);
void test_advance_angle_default_is_30_degrees(void);
void test_advance_angle_affects_commutation_delay_in_advance_step(void);
void test_advance_angle_zero_commutates_immediately_after_zc(void);
void test_advance_angle_60_waits_full_step_period(void);
void test_advance_angle_used_at_ramp_to_run_transition(void);
void test_mc_api_fast_step_runs_bldc_sensorless_pipeline(void);
void test_mc_api_fast_step_runs_bldc_sensorless_run_and_medium_step_speed_pi(void);
void test_mc_pmsm_foc_runs_pipeline_and_updates_pwm(void);
void test_mc_pmsm_foc_limits_voltage_vector(void);
void test_mc_pmsm_foc_normalizes_with_bus_voltage(void);
void test_mc_pmsm_foc_runs_with_2shunt_reconstruction(void);
void test_mc_pmsm_foc_runs_with_1shunt_reconstruction(void);
void test_mc_pmsm_foc_1shunt_invalid_window_uses_predicted_current(void);
void test_mc_pmsm_foc_1shunt_prediction_depends_on_motor_model(void);
void test_mc_pmsm_foc_1shunt_prediction_depends_on_elec_speed(void);
void test_mc_pmsm_foc_1shunt_prediction_is_limited_in_high_modulation(void);
void test_mc_pmsm_foc_1shunt_prediction_is_limited_in_field_weakening(void);
void test_mc_pmsm_foc_1shunt_high_modulation_uses_trailing_sampling(void);
void test_mc_pmsm_foc_1shunt_field_weakening_reorders_to_low_zero_vector(void);

void test_mc_pi_zero_error_produces_zero_output(void);
void test_mc_pi_negative_error_produces_negative_output(void);
void test_mc_pi_large_error_clamps_output(void);
void test_mc_pi_integral_windup_limited_by_clamp(void);
void test_mc_pi_kp_only_behaves_as_proportional_gain(void);
void test_mc_pi_ki_only_integrates_error(void);
void test_mc_pi_q31_generates_bounded_output(void);
void test_mc_svpwm_q31_generates_valid_centered_duties(void);
void test_mc_reconstruct_2shunt_q31_derives_third_phase(void);
void test_mc_reconstruct_3shunt_q31_applies_offset_and_scale(void);

void test_mc_cfg_has_bldc_enabled(void);
void test_mc_cfg_has_pmsm_enabled(void);
void test_mc_cfg_has_float32_enabled(void);
void test_mc_cfg_has_q31_enabled(void);
void test_mc_cfg_has_1shunt_enabled(void);
void test_mc_cfg_has_2shunt_enabled(void);
void test_mc_cfg_has_3shunt_enabled(void);
void test_mc_cfg_has_hall_enabled(void);
void test_mc_cfg_has_encoder_enabled(void);
void test_mc_cfg_has_resolver_enabled(void);
void test_mc_cfg_has_sensorless_enabled(void);

void test_mc_init_failed_init_leaves_instance_uninitialized(void);
void test_mc_set_mode_rejects_switching_to_uninitialized_mode(void);
void test_mc_init_rejects_unsupported_q31_mode_matrix(void);
void test_mc_init_accepts_supported_q31_mode_matrix(void);
void test_mc_init_rejects_invalid_sensorless_cfg_constraints(void);
void test_mc_medium_step_disabled_does_not_advance_speed_control_state(void);
void test_mc_stop_resets_bldc_sensorless_to_idle(void);
void test_mc_identify_completion_applies_results_to_foc_model(void);
void test_mc_identify_completion_applies_results_to_sensorless_model(void);
void test_mc_identify_completion_uses_trusted_flux_not_raw_candidate(void);
void test_mc_start_identification_preserves_flux_seed(void);
void test_mc_identify_fast_step_uses_3shunt_reconstruction(void);
void test_mc_identify_fast_step_uses_1shunt_sampling_plan(void);
void test_mc_api_sensorless_observer_uses_3shunt_reconstruction(void);
void test_mc_api_sensorless_observer_uses_1shunt_reconstruction(void);
void test_mc_identify_flux_estimate_is_generated_after_lq_stage(void);
void test_mc_identify_ld_measure_holds_zero_voltage_before_lq_align(void);
void test_mc_identify_ld_estimate_uses_window_average_current_for_rs_drop(void);
void test_mc_identify_lq_estimate_uses_window_average_current_for_rs_drop(void);
void test_mc_identify_ld_estimate_uses_actual_discrete_pulse_time_when_dt_overshoots_target(void);
void test_mc_identify_lq_inject_finishes_on_accumulated_discrete_pulse_time_with_variable_dt(void);
void test_mc_identify_flux_estimate_averages_multiple_valid_samples(void);
void test_mc_identify_flux_estimate_time_weights_variable_dt_samples(void);
void test_mc_identify_flux_estimate_rejects_current_opposite_to_q_axis_excitation(void);
void test_mc_identify_lq_inject_defers_first_flux_sample_until_lq_measure_when_lq_is_unknown(void);
void test_mc_identify_flux_estimate_keeps_seed_when_valid_flux_window_is_too_short(void);
void test_mc_identify_flux_estimate_updates_once_valid_flux_window_is_long_enough(void);
void test_mc_identify_lq_measure_requires_full_window_before_complete(void);
void test_mc_identify_rs_negative_estimate_is_rejected(void);
void test_mc_identify_ld_negative_estimate_is_rejected(void);
void test_mc_identify_lq_negative_estimate_is_rejected(void);
void test_mc_identify_flux_estimate_keeps_seed_when_no_valid_samples_exist(void);
void test_mc_identify_get_result_returns_flux_estimate(void);
void test_mc_identify_flux_candidate_tracks_raw_measurement_before_trust_gate(void);

void test_mc_auto_tune_current_pi_produces_positive_gains(void);
void test_mc_auto_tune_current_pi_rejects_null_output(void);
void test_mc_auto_tune_current_pi_rejects_zero_rs(void);
void test_mc_auto_tune_current_pi_rejects_zero_pwm_freq(void);
void test_mc_auto_tune_current_pi_salient_motor_different_kp(void);
void test_mc_auto_tune_current_pi_aggressive_divider_produces_higher_gains(void);
void test_mc_auto_tune_speed_pi_produces_positive_gains(void);
void test_mc_auto_tune_speed_pi_rejects_null(void);
void test_mc_auto_tune_speed_pi_rejects_zero_ld(void);
void test_mc_dtc_disabled_does_not_affect_voltage(void);
void test_mc_dtc_enabled_produces_nonzero_compensation(void);
void test_mc_dtc_higher_deadtime_produces_larger_compensation(void);
void test_mc_dtc_config_propagated_to_foc_cfg(void);
void test_mc_debug_map_collect_one_var(void);
void test_mc_debug_map_collect_two_vars(void);
void test_mc_debug_map_inactive_mask_skips(void);
void test_mc_debug_map_slot_out_of_range_collects_zero(void);
void test_mc_debug_fm_placeholder(void);
void test_mc_debug_transp_placeholder(void);

void test_mc_diag_1shunt_comp_mode_name_none(void);
void test_mc_diag_1shunt_comp_mode_name_predict_basic(void);
void test_mc_diag_1shunt_comp_mode_name_predict_high_modulation(void);
void test_mc_diag_1shunt_comp_mode_name_predict_field_weakening(void);
void test_mc_diag_1shunt_comp_mode_name_unknown_returns_unknown(void);
void test_mc_diag_status_defaults_to_zero(void);
void test_mc_diag_status_reports_fault(void);
void test_mc_diag_status_reports_warning(void);

void test_mc_adc_calibrate_linear_computes_scale_and_offset(void);
void test_mc_adc_calibrate_linear_rejects_null_scale(void);
void test_mc_adc_calibrate_linear_rejects_null_offset(void);
void test_mc_adc_calibrate_linear_rejects_equal_raw_points(void);
void test_mc_adc_calibrate_linear_handles_negative_physical(void);
void test_mc_adc_calibrate_linear_handles_zero_span(void);

void setUp(void)
{
    g_last_pwm_cmd = (mc_pwm_cmd_t){0};
    g_last_adc_trigger_plan = (mc_adc_trigger_plan_t){0};
    g_pwm_apply_called = 0U;
    g_adc_trigger_apply_called = 0U;
}

void tearDown(void)
{
}

void test_mc_init_marks_instance_initialized(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_status_t status;

    cfg.sensor.primary_mode = MC_MODE_DISABLED;

    status = mc_init(&inst, &cfg);

    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.initialized == MC_TRUE);
}

void test_mc_init_failed_init_leaves_instance_uninitialized(void)
{
    mc_instance_t inst = {0};
    mc_system_cfg_t cfg = {0};
    mc_status_t status;

    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 0U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;

    status = mc_init(&inst, &cfg);

    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
    TEST_ASSERT_TRUE(inst.initialized == MC_FALSE);
    TEST_ASSERT_TRUE(inst.enabled == MC_FALSE);
}

void test_mc_set_mode_rejects_switching_to_uninitialized_mode(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_status_t status;

    cfg.sensor.primary_mode = MC_MODE_DISABLED;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_STATE, mc_set_mode(&inst, MC_MODE_PMSM_FOC_ENCODER));
}

void test_mc_init_rejects_unsupported_q31_mode_matrix(void)
{
    mc_instance_t inst = {0};
    mc_system_cfg_t cfg = {0};

    cfg.control.numeric_mode = MC_NUMERIC_Q31;
    cfg.sensor.primary_mode = MC_MODE_BLDC_SENSORLESS;
    cfg.sensor.bldc_sensorless_cfg = mc_bldc_sensorless_cfg_default();

    TEST_ASSERT_EQUAL_INT(MC_STATUS_UNSUPPORTED, mc_init(&inst, &cfg));
    TEST_ASSERT_TRUE(inst.initialized == MC_FALSE);
}

void test_mc_init_accepts_supported_q31_mode_matrix(void)
{
    mc_instance_t inst = {0};
    mc_system_cfg_t cfg = {0};
    mc_status_t status;

    cfg.control.numeric_mode = MC_NUMERIC_Q31;
    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -0.5F, 0.5F, -0.5F, 0.5F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);

    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.initialized == MC_TRUE);
}

void test_mc_api_fast_step_runs_encoder_foc_pipeline(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_medium_input_t medium_in = {0.0F, 0U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_fast_input_t fast_in = {{1010U, 990U, 1000U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 2000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_current_ref_dq(&inst, 0.0F, 0.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 1U);
    TEST_ASSERT_TRUE(inst.foc.pmsm_foc.last_sector >= 1U);
}

void test_mc_api_fast_step_runs_hall_bldc_pipeline(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_fast_input_t fast_in = {{0U, 0U, 0U, 0U, 0U}, {0U}, {0, 0, 0U}, 5U, 1000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.sensor.primary_mode = MC_MODE_BLDC_HALL;
    cfg.sensor.pole_pairs = 2.0F;
    cfg.sensor.hall_cfg = (mc_hall_cfg_t){{1U, 5U, 4U, 6U, 2U, 3U}, {0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F}};

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_torque_ref(&inst, 0.6F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 1U);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.sector == 2U);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.phase_mode_a == MC_PWM_PHASE_PWM);
}

void test_mc_api_fast_step_runs_hall_foc_pipeline(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_fast_input_t fast_in = {{1010U, 990U, 1000U, 2400U, 0U}, {0U}, {0, 0, 0U}, 5U, 1000U};
    mc_fast_output_t fast_out = {0};
    mc_medium_input_t medium_in = {0.0F, 0U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_HALL;
    cfg.sensor.pole_pairs = 2.0F;
    cfg.sensor.hall_cfg = (mc_hall_cfg_t){{1U, 5U, 4U, 6U, 2U, 3U}, {0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F}};
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_current_ref_dq(&inst, 0.0F, 0.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 1U);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.count == 1U);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.trigger_a.valid == MC_TRUE);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.trigger_a.event == MC_ADC_TRIGGER_EVENT_PWM_DOWN);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.trigger_a.position == 0.5F);
}

void test_mc_api_medium_step_speed_loop_updates_iq_ref(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_medium_input_t medium_in = {0.0F, 0U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_speed_ref(&inst, 1000.0F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.iq_ref > 1.0F);
    TEST_ASSERT_TRUE(inst.iq_ref <= 1.5F);
}

void test_mc_medium_step_disabled_does_not_advance_speed_control_state(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_medium_input_t medium_in = {0.0F, 0U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_set_speed_ref(&inst, 1000.0F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.iq_ref == 0.0F);
    TEST_ASSERT_TRUE(inst.foc.speed_pi.integral == 0.0F);
}

void test_mc_api_fast_step_runs_resolver_foc_pipeline(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_fast_input_t fast_in = {{1010U, 990U, 1000U, 2400U, 0U}, {0U}, {1000, 0, 0U}, 0U, 1000U};
    mc_medium_input_t medium_in = {0.0F, 0U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_RESOLVER;
    cfg.sensor.resolver_cfg = (mc_resolver_cfg_t){1200, 10000U, 0.001F, 0.1F, 2.0F};
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_current_ref_dq(&inst, 0.0F, 0.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 1U);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.count == 1U);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.trigger_a.valid == MC_TRUE);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.trigger_a.event == MC_ADC_TRIGGER_EVENT_PWM_DOWN);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.trigger_a.position == 0.5F);
}

void test_mc_api_fast_step_applies_pwm_and_adc_trigger_hooks(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_medium_input_t medium_in = {0.0F, 0U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_fast_input_t fast_in = {{1100U, 0U, 0U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 2000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.10F, 0.02F, 0.0001F, 0.000002F, 0.000004F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.hooks.pwm_apply = test_mc_api_pwm_apply_stub;
    cfg.hooks.adc_trigger_apply = test_mc_api_adc_trigger_apply_stub;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_current_ref_dq(&inst, 0.0F, 0.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(g_pwm_apply_called == 1U);
    TEST_ASSERT_TRUE(g_adc_trigger_apply_called == 1U);
    TEST_ASSERT_TRUE(g_last_pwm_cmd.valid == fast_out.pwm_cmd.valid);
    TEST_ASSERT_TRUE(g_last_adc_trigger_plan.count == fast_out.adc_trigger_plan.count);
    TEST_ASSERT_TRUE(g_last_adc_trigger_plan.trigger_a.event == fast_out.adc_trigger_plan.trigger_a.event);
}

void test_mc_api_fast_step_runs_encoder_foc_pipeline_with_2shunt(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_medium_input_t medium_in = {0.0F, 256U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_fast_input_t fast_in = {{1010U, 990U, 0U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 2000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_2SHUNT, {{1000.0F, 1000.0F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_current_ref_dq(&inst, 0.0F, 0.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 1U);
    TEST_ASSERT_TRUE(fast_out.current_comp_status.active == MC_FALSE);
    TEST_ASSERT_TRUE(fast_out.current_comp_status.mode == MC_1SHUNT_COMP_NONE);
}

void test_mc_api_fast_step_runs_encoder_foc_pipeline_with_1shunt(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_medium_input_t medium_in = {0.0F, 256U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_fast_input_t fast_in = {{1100U, 0U, 0U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 2000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.10F, 0.02F, 0.0001F, 0.000002F, 0.000004F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    inst.foc.pmsm_foc.last_sector = 2U;

    status = mc_set_current_ref_dq(&inst, 0.0F, 0.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 1U);
}

void test_mc_api_get_diag_reports_last_1shunt_compensation_mode(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_medium_input_t medium_in = {0.0F, 256U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_fast_input_t fast_in = {{1100U, 0U, 0U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 2000U};
    mc_fast_output_t fast_out = {0};
    mc_diag_status_t diag = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.10F, 0.02F, 0.0001F, 0.000002F, 0.000004F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;
    cfg.motor.pole_pairs = 2U;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_current_ref_dq(&inst, 0.0F, 0.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    inst.foc.pmsm_foc.i_dq = (mc_dq_t){0.05F, 0.10F};
    inst.foc.pmsm_foc.v_dq = (mc_dq_t){0.40F, 0.20F};
    inst.foc.pmsm_foc.shunt1_meta = (mc_1shunt_meta_t){0};
    inst.foc.pmsm_foc.shunt1_meta.compensation_required = MC_TRUE;

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fast_out.current_comp_status.active == MC_TRUE);
    TEST_ASSERT_TRUE(fast_out.current_comp_status.mode == MC_1SHUNT_COMP_PREDICT_BASIC);

    status = mc_get_diag(&inst, &diag);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(diag.current_comp_status.active == MC_TRUE);
    TEST_ASSERT_TRUE(diag.current_comp_status.mode == MC_1SHUNT_COMP_PREDICT_BASIC);
}

void test_mc_identify_completion_applies_results_to_foc_model(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_fast_input_t fast_in = {{1010U, 990U, 1000U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 1000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    inst.enabled = MC_TRUE;
    inst.identify.state = MC_IDENTIFY_STATE_COMPLETE;
    inst.identify.rs_ohm = 0.8F;
    inst.identify.ld_h = 0.004F;
    inst.identify.lq_h = 0.006F;
    inst.identify.flux_wb = 0.03F;

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.cfg.motor.rs_ohm == 0.8F);
    TEST_ASSERT_TRUE(inst.cfg.motor.ld_h == 0.004F);
    TEST_ASSERT_TRUE(inst.cfg.motor.lq_h == 0.006F);
    TEST_ASSERT_TRUE(inst.cfg.motor.flux_wb == 0.03F);
    TEST_ASSERT_TRUE(inst.foc.pmsm_foc.cfg.rs_ohm == 0.8F);
    TEST_ASSERT_TRUE(inst.foc.pmsm_foc.cfg.ld_h == 0.004F);
    TEST_ASSERT_TRUE(inst.foc.pmsm_foc.cfg.lq_h == 0.006F);
    TEST_ASSERT_TRUE(inst.foc.pmsm_foc.cfg.flux_wb == 0.03F);
}

void test_mc_identify_completion_applies_results_to_sensorless_model(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_fast_input_t fast_in = {{1010U, 990U, 1000U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 1000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_SENSORLESS;
    cfg.sensor.sensorless_cfg = (mc_sensorless_cfg_t){0.5F, 0.0025F, 2.0F, 0.2F, 0.01F, 400.0F, 20000.0F, 0.02F, 10.0F, 30000.0F, 0.0F, 1.0F};
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    inst.enabled = MC_TRUE;
    inst.identify.state = MC_IDENTIFY_STATE_COMPLETE;
    inst.identify.rs_ohm = 0.9F;
    inst.identify.ld_h = 0.005F;
    inst.identify.lq_h = 0.007F;
    inst.identify.flux_wb = 0.04F;

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.sensorless.cfg.rs_ohm == 0.9F);
    TEST_ASSERT_TRUE(inst.sensorless.cfg.ls_h == 0.006F);
    TEST_ASSERT_TRUE(inst.cfg.motor.flux_wb == 0.04F);
}

void test_mc_identify_completion_uses_trusted_flux_not_raw_candidate(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_fast_input_t fast_in = {{1010U, 990U, 1000U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 1000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    inst.enabled = MC_TRUE;
    inst.identify.state = MC_IDENTIFY_STATE_COMPLETE;
    inst.identify.rs_ohm = 0.8F;
    inst.identify.ld_h = 0.004F;
    inst.identify.lq_h = 0.006F;
    inst.identify.flux_wb = 0.02F;
    inst.identify.flux_candidate_wb = 0.05F;

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.cfg.motor.flux_wb == 0.02F);
    TEST_ASSERT_TRUE(inst.foc.pmsm_foc.cfg.flux_wb == 0.02F);
}

void test_mc_start_identification_preserves_flux_seed(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_status_t status;

    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.motor.flux_wb = 0.025F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_start_identification(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.identify.flux_wb == 0.025F);
}

void test_mc_identify_fast_step_uses_3shunt_reconstruction(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_fast_input_t fast_in = {{1001U, 1000U, 999U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 1000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 10.0F, 10.0F, 10.0F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start_identification(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    /* These raw values only exceed iq_limit after 3-shunt offset/scale reconstruction. */
    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_ERROR, inst.identify.state);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 0U);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.count == 0U);
}

void test_mc_identify_fast_step_uses_1shunt_sampling_plan(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_fast_input_t fast_in_a = {{1000U, 0U, 0U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 1000U};
    mc_fast_input_t fast_in_b = {{1001U, 0U, 0U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 2000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 10.0F, 0.10F, 0.02F, 0.0001F, 0.000002F, 0.000004F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start_identification(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_fast_step(&inst, &fast_in_a, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.identify.state != MC_IDENTIFY_STATE_ERROR);
    TEST_ASSERT_TRUE(inst.foc.pmsm_foc.shunt1_meta.sample_count > 0U);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.count == inst.foc.pmsm_foc.shunt1_meta.sample_count);
    TEST_ASSERT_TRUE(fast_out.adc_trigger_plan.trigger_a.valid == MC_TRUE);

    /* This step only trips overcurrent if identify uses the prior-cycle 1-shunt sampling metadata. */
    status = mc_fast_step(&inst, &fast_in_b, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_ERROR, inst.identify.state);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 0U);
}

void test_mc_api_sensorless_observer_uses_3shunt_reconstruction(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_fast_input_t fast_in = {{1001U, 1000U, 999U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 1000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_SENSORLESS;
    cfg.sensor.sensorless_cfg = (mc_sensorless_cfg_t){0.5F, 0.0025F, 2.0F, 0.2F, 0.01F, 400.0F, 20000.0F, 0.02F, 10.0F, 30000.0F, 0.0F, 1.0F};
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 10.0F, 10.0F, 10.0F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_current_ref_dq(&inst, 0.0F, 0.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    inst.foc_last_output.v_ab = (mc_alphabeta_t){1.0F, 0.0F};
    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.sensorless.last_current_ab.alpha > 9.9F);
    TEST_ASSERT_TRUE(inst.sensorless.last_current_ab.beta > 5.7F);
    TEST_ASSERT_TRUE(inst.sensorless.last_current_ab.beta < 5.9F);
}

void test_mc_api_sensorless_observer_uses_1shunt_reconstruction(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_fast_input_t fast_in_a = {{1000U, 0U, 0U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 1000U};
    mc_fast_input_t fast_in_b = {{1002U, 0U, 0U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 2000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_SENSORLESS;
    cfg.sensor.sensorless_cfg = (mc_sensorless_cfg_t){0.5F, 0.0025F, 2.0F, 0.2F, 0.01F, 400.0F, 20000.0F, 0.02F, 10.0F, 30000.0F, 0.0F, 1.0F};
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 10.0F, 0.10F, 0.02F, 0.0001F, 0.000002F, 0.000004F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_current_ref_dq(&inst, 0.0F, 0.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_fast_step(&inst, &fast_in_a, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.foc.pmsm_foc.shunt1_meta.sample_count > 0U);

    status = mc_fast_step(&inst, &fast_in_b, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.sensorless.last_current_ab.alpha > 19.9F);
}

void test_mc_api_fast_step_runs_sensorless_foc_pipeline(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_medium_input_t medium_in = {0.0F, 0U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_diag_status_t diag = {0};
    mc_fast_input_t fast_in_a = {{100U, 0U, 0U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 1000U};
    mc_fast_input_t fast_in_b = {{0U, 100U, 0U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 2000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_SENSORLESS;
    cfg.sensor.sensorless_cfg = (mc_sensorless_cfg_t){0.0F, 0.0F, 0.0F, 1.0F, 0.01F, 400.0F, 20000.0F, 0.02F, 10.0F, 30000.0F};
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_current_ref_dq(&inst, 0.0F, 0.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    inst.sensorless.elec_angle_rad = -1.5707963268F;
    inst.foc_last_output.v_ab = (mc_alphabeta_t){1.0F, 0.0F};

    status = mc_fast_step(&inst, &fast_in_a, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    inst.foc_last_output.v_ab = (mc_alphabeta_t){0.0F, 1.0F};
    status = mc_fast_step(&inst, &fast_in_b, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_get_diag(&inst, &diag);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 1U);
    TEST_ASSERT_TRUE(inst.sensorless.observer_valid == MC_TRUE);
    TEST_ASSERT_TRUE(inst.sensorless.mech_speed_rpm > 0.0F);
    TEST_ASSERT_TRUE(inst.sensorless.mech_speed_rpm < 20000.0F);
    TEST_ASSERT_TRUE(medium_out.mech_speed_rpm == inst.sensorless.mech_speed_rpm);
    TEST_ASSERT_TRUE(diag.sensorless_observer_valid == MC_TRUE);
    TEST_ASSERT_TRUE(diag.sensorless_pll_locked == inst.sensorless.pll_locked);
    TEST_ASSERT_TRUE(diag.sensorless_open_loop_active == inst.sensorless.open_loop_active);
    TEST_ASSERT_TRUE(diag.active_warning == ((inst.sensorless.pll_locked != MC_FALSE) ? MC_WARNING_NONE : MC_WARNING_OBSERVER_UNLOCKED));
}

void test_mc_init_rejects_invalid_sensorless_cfg_constraints(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_SENSORLESS;
    cfg.sensor.sensorless_cfg = (mc_sensorless_cfg_t){0.0F, 0.0F, 0.0F, 1.0F, 0.02F, 400.0F, 20000.0F, 0.02F, 10.0F, 30000.0F, 0.0F, 1.0F};
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.002F;
    cfg.motor.lq_h = 0.003F;
    cfg.motor.flux_wb = 0.02F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
}

void test_mc_diag_1shunt_comp_mode_name_returns_readable_strings(void)
{
    TEST_ASSERT_TRUE(strcmp(mc_diag_1shunt_comp_mode_name(MC_1SHUNT_COMP_NONE), "none") == 0);
    TEST_ASSERT_TRUE(strcmp(mc_diag_1shunt_comp_mode_name(MC_1SHUNT_COMP_PREDICT_BASIC), "predict_basic") == 0);
    TEST_ASSERT_TRUE(strcmp(mc_diag_1shunt_comp_mode_name(MC_1SHUNT_COMP_PREDICT_HIGH_MODULATION), "predict_high_modulation") == 0);
    TEST_ASSERT_TRUE(strcmp(mc_diag_1shunt_comp_mode_name(MC_1SHUNT_COMP_PREDICT_FIELD_WEAKENING), "predict_field_weakening") == 0);
    TEST_ASSERT_TRUE(strcmp(mc_diag_1shunt_comp_mode_name((mc_1shunt_comp_mode_t)99), "unknown") == 0);
}

void test_mc_set_torque_ref_with_mtpa_updates_id_ref(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.0002F;
    cfg.motor.lq_h = 0.0006F;
    cfg.motor.flux_wb = 0.02F;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.foc.mtpa_enable = MC_TRUE;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_torque_ref(&inst, 5.0F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(inst.id_ref < 0.0F);
}

void test_mc_set_torque_ref_without_mtpa_leaves_id_at_zero(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.0002F;
    cfg.motor.lq_h = 0.0006F;
    cfg.motor.flux_wb = 0.02F;
    cfg.sensor.encoder_cfg.counts_per_rev = 1024U;
    cfg.sensor.encoder_cfg.pole_pairs = 2.0F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    cfg.foc.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.foc.current_cfg = (mc_current_sense_cfg_t){MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}};
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.5F;
    cfg.foc.voltage_limit = 1.0F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.foc.mtpa_enable = MC_FALSE;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_torque_ref(&inst, 5.0F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fabsf(inst.id_ref) < 1e-12F);
}

void test_mc_api_fast_step_runs_bldc_sensorless_pipeline(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_medium_input_t medium_in = {0.0F, 0U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_fast_input_t fast_in = {{100U, 200U, 300U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 1000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_BLDC_SENSORLESS;
    cfg.sensor.bldc_sensorless_cfg = mc_bldc_sensorless_cfg_default();
    cfg.sensor.bldc_sensorless_cfg.align_time_s = 0.5F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.speed_loop_dt_s = 0.01F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_torque_ref(&inst, 0.6F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_ALIGN, inst.bldc_sensorless.phase);

    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 1U);

    /* Medium step should return speed estimate */
    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(medium_out.elec_angle_rad == 0.0F);
}

void test_mc_api_fast_step_runs_bldc_sensorless_run_and_medium_step_speed_pi(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_medium_input_t medium_in = {0.0F, 0U, 1000U};
    mc_medium_output_t medium_out = {0};
    mc_fast_input_t fast_in = {{1300U, 1400U, 1500U, 2400U, 0U}, {0U}, {0, 0, 0U}, 0U, 1000U};
    mc_fast_output_t fast_out = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_BLDC_SENSORLESS;
    cfg.sensor.bldc_sensorless_cfg = mc_bldc_sensorless_cfg_default();
    cfg.sensor.bldc_sensorless_cfg.align_time_s = 0.5F;
    cfg.sensor.bldc_sensorless_cfg.speed_pi_cfg.kp = 0.001F;
    cfg.sensor.bldc_sensorless_cfg.speed_pi_cfg.ki = 0.1F;
    cfg.foc.bus_voltage_scale = 0.01F;
    cfg.foc.bus_voltage_offset = 0.0F;
    cfg.foc.speed_loop_dt_s = 0.01F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_set_speed_ref(&inst, 2000.0F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    /* Move to RUN after a real public start path */
    inst.bldc_sensorless.phase = MC_BLDC_SENSORLESS_RUN;
    inst.bldc_sensorless.mech_speed_rpm = 1000.0F;
    inst.bldc_sensorless.duty_cmd = 0.3F;

    /* Fast step produces PWM */
    status = mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fast_out.pwm_cmd.valid == 1U);

    /* Medium step runs speed PI (error=1000 -> should increase duty) */
    status = mc_medium_step(&inst, &medium_in, &medium_out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(medium_out.mech_speed_rpm == 1000.0F);
    TEST_ASSERT_TRUE(inst.bldc_sensorless.duty_cmd > 0.3F);
}

void test_mc_stop_resets_bldc_sensorless_to_idle(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};
    mc_status_t status;

    cfg.control.current_loop_dt_s = 0.001F;
    cfg.sensor.primary_mode = MC_MODE_BLDC_SENSORLESS;
    cfg.sensor.bldc_sensorless_cfg = mc_bldc_sensorless_cfg_default();
    cfg.foc.bus_voltage_scale = 0.01F;

    status = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_ALIGN, inst.bldc_sensorless.phase);

    status = mc_stop(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_IDLE, inst.bldc_sensorless.phase);
    TEST_ASSERT_TRUE(inst.enabled == MC_FALSE);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_mc_init_marks_instance_initialized);
    RUN_TEST(test_mc_init_failed_init_leaves_instance_uninitialized);
    RUN_TEST(test_mc_set_mode_rejects_switching_to_uninitialized_mode);
    RUN_TEST(test_mc_init_rejects_unsupported_q31_mode_matrix);
    RUN_TEST(test_mc_init_accepts_supported_q31_mode_matrix);
    RUN_TEST(test_mc_init_rejects_invalid_sensorless_cfg_constraints);
    RUN_TEST(test_mc_api_fast_step_runs_encoder_foc_pipeline);
    RUN_TEST(test_mc_api_fast_step_runs_hall_bldc_pipeline);
    RUN_TEST(test_mc_api_fast_step_runs_hall_foc_pipeline);
    RUN_TEST(test_mc_api_fast_step_runs_sensorless_foc_pipeline);
    RUN_TEST(test_mc_api_fast_step_runs_bldc_sensorless_pipeline);
    RUN_TEST(test_mc_api_fast_step_runs_bldc_sensorless_run_and_medium_step_speed_pi);
    RUN_TEST(test_mc_api_fast_step_runs_encoder_foc_pipeline_with_1shunt);
    RUN_TEST(test_mc_api_get_diag_reports_last_1shunt_compensation_mode);
    RUN_TEST(test_mc_identify_completion_applies_results_to_foc_model);
    RUN_TEST(test_mc_identify_completion_applies_results_to_sensorless_model);
    RUN_TEST(test_mc_identify_completion_uses_trusted_flux_not_raw_candidate);
    RUN_TEST(test_mc_start_identification_preserves_flux_seed);
    RUN_TEST(test_mc_identify_fast_step_uses_3shunt_reconstruction);
    RUN_TEST(test_mc_identify_fast_step_uses_1shunt_sampling_plan);
    RUN_TEST(test_mc_api_sensorless_observer_uses_3shunt_reconstruction);
    RUN_TEST(test_mc_api_sensorless_observer_uses_1shunt_reconstruction);
    RUN_TEST(test_mc_identify_flux_estimate_is_generated_after_lq_stage);
    RUN_TEST(test_mc_identify_ld_measure_holds_zero_voltage_before_lq_align);
    RUN_TEST(test_mc_identify_ld_estimate_uses_window_average_current_for_rs_drop);
    RUN_TEST(test_mc_identify_lq_estimate_uses_window_average_current_for_rs_drop);
    RUN_TEST(test_mc_identify_ld_estimate_uses_actual_discrete_pulse_time_when_dt_overshoots_target);
    RUN_TEST(test_mc_identify_lq_inject_finishes_on_accumulated_discrete_pulse_time_with_variable_dt);
    RUN_TEST(test_mc_identify_flux_estimate_averages_multiple_valid_samples);
    RUN_TEST(test_mc_identify_flux_estimate_time_weights_variable_dt_samples);
    RUN_TEST(test_mc_identify_flux_estimate_rejects_current_opposite_to_q_axis_excitation);
    RUN_TEST(test_mc_identify_lq_inject_defers_first_flux_sample_until_lq_measure_when_lq_is_unknown);
    RUN_TEST(test_mc_identify_flux_estimate_keeps_seed_when_valid_flux_window_is_too_short);
    RUN_TEST(test_mc_identify_flux_estimate_updates_once_valid_flux_window_is_long_enough);
    RUN_TEST(test_mc_identify_lq_measure_requires_full_window_before_complete);
    RUN_TEST(test_mc_identify_rs_negative_estimate_is_rejected);
    RUN_TEST(test_mc_identify_ld_negative_estimate_is_rejected);
    RUN_TEST(test_mc_identify_lq_negative_estimate_is_rejected);
    RUN_TEST(test_mc_identify_flux_estimate_keeps_seed_when_no_valid_samples_exist);
    RUN_TEST(test_mc_identify_get_result_returns_flux_estimate);
    RUN_TEST(test_mc_identify_flux_candidate_tracks_raw_measurement_before_trust_gate);
    RUN_TEST(test_mc_auto_tune_current_pi_produces_positive_gains);
    RUN_TEST(test_mc_auto_tune_current_pi_rejects_null_output);
    RUN_TEST(test_mc_auto_tune_current_pi_rejects_zero_rs);
    RUN_TEST(test_mc_auto_tune_current_pi_rejects_zero_pwm_freq);
    RUN_TEST(test_mc_auto_tune_current_pi_salient_motor_different_kp);
    RUN_TEST(test_mc_auto_tune_current_pi_aggressive_divider_produces_higher_gains);
    RUN_TEST(test_mc_auto_tune_speed_pi_produces_positive_gains);
    RUN_TEST(test_mc_auto_tune_speed_pi_rejects_null);
    RUN_TEST(test_mc_auto_tune_speed_pi_rejects_zero_ld);
    RUN_TEST(test_mc_dtc_disabled_does_not_affect_voltage);
    RUN_TEST(test_mc_dtc_enabled_produces_nonzero_compensation);
    RUN_TEST(test_mc_dtc_higher_deadtime_produces_larger_compensation);
    RUN_TEST(test_mc_dtc_config_propagated_to_foc_cfg);
    RUN_TEST(test_mc_debug_map_collect_one_var);
    RUN_TEST(test_mc_debug_map_collect_two_vars);
    RUN_TEST(test_mc_debug_map_inactive_mask_skips);
    RUN_TEST(test_mc_debug_map_slot_out_of_range_collects_zero);
    RUN_TEST(test_mc_debug_fm_placeholder);
    RUN_TEST(test_mc_debug_transp_placeholder);
    RUN_TEST(test_mc_diag_1shunt_comp_mode_name_returns_readable_strings);
    RUN_TEST(test_mc_diag_1shunt_comp_mode_name_none);
    RUN_TEST(test_mc_diag_1shunt_comp_mode_name_predict_basic);
    RUN_TEST(test_mc_diag_1shunt_comp_mode_name_predict_high_modulation);
    RUN_TEST(test_mc_diag_1shunt_comp_mode_name_predict_field_weakening);
    RUN_TEST(test_mc_diag_1shunt_comp_mode_name_unknown_returns_unknown);
    RUN_TEST(test_mc_diag_status_defaults_to_zero);
    RUN_TEST(test_mc_diag_status_reports_fault);
    RUN_TEST(test_mc_diag_status_reports_warning);
    RUN_TEST(test_mc_api_fast_step_applies_pwm_and_adc_trigger_hooks);
    RUN_TEST(test_mc_api_fast_step_runs_encoder_foc_pipeline_with_2shunt);
    RUN_TEST(test_mc_api_fast_step_runs_resolver_foc_pipeline);
    RUN_TEST(test_mc_api_medium_step_speed_loop_updates_iq_ref);
    RUN_TEST(test_mc_medium_step_disabled_does_not_advance_speed_control_state);
    RUN_TEST(test_mc_set_torque_ref_with_mtpa_updates_id_ref);
    RUN_TEST(test_mc_set_torque_ref_without_mtpa_leaves_id_at_zero);
    RUN_TEST(test_mc_math_clamp_and_wrap_work);
    RUN_TEST(test_mc_q31_helpers_convert_and_saturate);
    RUN_TEST(test_mc_clarke_computes_alpha_beta);
    RUN_TEST(test_mc_park_and_ipark_round_trip);
    RUN_TEST(test_mc_q31_transform_round_trip);
    RUN_TEST(test_mc_control_foc_delegates_to_pmsm_foc);
    RUN_TEST(test_mc_control_foc_speed_step_generates_iq_ref);
    RUN_TEST(test_mc_pi_generates_bounded_output);
    RUN_TEST(test_mc_pi_clamps_integral_and_output);
    RUN_TEST(test_mc_pi_zero_error_produces_zero_output);
    RUN_TEST(test_mc_pi_negative_error_produces_negative_output);
    RUN_TEST(test_mc_pi_large_error_clamps_output);
    RUN_TEST(test_mc_pi_integral_windup_limited_by_clamp);
    RUN_TEST(test_mc_pi_kp_only_behaves_as_proportional_gain);
    RUN_TEST(test_mc_pi_ki_only_integrates_error);
    RUN_TEST(test_mc_pi_q31_generates_bounded_output);
    RUN_TEST(test_mc_svpwm_generates_valid_centered_duties);
    RUN_TEST(test_mc_svpwm_limits_modulation_vector);
    RUN_TEST(test_mc_svpwm_q31_generates_valid_centered_duties);
    RUN_TEST(test_mc_cfg_has_bldc_enabled);
    RUN_TEST(test_mc_cfg_has_pmsm_enabled);
    RUN_TEST(test_mc_cfg_has_float32_enabled);
    RUN_TEST(test_mc_cfg_has_q31_enabled);
    RUN_TEST(test_mc_cfg_has_1shunt_enabled);
    RUN_TEST(test_mc_cfg_has_2shunt_enabled);
    RUN_TEST(test_mc_cfg_has_3shunt_enabled);
    RUN_TEST(test_mc_cfg_has_hall_enabled);
    RUN_TEST(test_mc_cfg_has_encoder_enabled);
    RUN_TEST(test_mc_cfg_has_resolver_enabled);
    RUN_TEST(test_mc_cfg_has_sensorless_enabled);
    RUN_TEST(test_mc_adc_map_trigger_plan_maps_valid_triggers);
    RUN_TEST(test_mc_adc_map_trigger_plan_keeps_invalid_trigger_empty);
    RUN_TEST(test_mc_adc_ref_build_apply_cfg_maps_hw_triggers);
    RUN_TEST(test_mc_adc_ref_build_apply_cfg_leaves_empty_slots_disabled);
    RUN_TEST(test_mc_adc_ref_apply_invokes_reference_port_callbacks);
    RUN_TEST(test_mc_adc_ref_apply_plan_runs_full_reference_chain);
    RUN_TEST(test_mc_adc_calibrate_linear_computes_scale_and_offset);
    RUN_TEST(test_mc_adc_calibrate_linear_rejects_null_scale);
    RUN_TEST(test_mc_adc_calibrate_linear_rejects_null_offset);
    RUN_TEST(test_mc_adc_calibrate_linear_rejects_equal_raw_points);
    RUN_TEST(test_mc_adc_calibrate_linear_handles_negative_physical);
    RUN_TEST(test_mc_adc_calibrate_linear_handles_zero_span);
    RUN_TEST(test_mc_reconstruct_1shunt_maps_sector_currents);
    RUN_TEST(test_mc_reconstruct_1shunt_invalid_window_returns_zero);
    RUN_TEST(test_mc_reconstruct_1shunt_high_modulation_prefers_trailing_samples);
    RUN_TEST(test_mc_reconstruct_2shunt_derives_third_phase);
    RUN_TEST(test_mc_reconstruct_2shunt_q31_derives_third_phase);
    RUN_TEST(test_mc_reconstruct_3shunt_applies_offset_and_scale);
    RUN_TEST(test_mc_reconstruct_3shunt_q31_applies_offset_and_scale);
    RUN_TEST(test_mc_encoder_update_computes_angle_and_speed);
    RUN_TEST(test_mc_hall_update_maps_angle_and_speed);
    RUN_TEST(test_mc_resolver_reports_excitation_command);
    RUN_TEST(test_mc_resolver_update_decodes_angle_and_speed);
    RUN_TEST(test_mc_resolver_update_flags_low_signal_invalid);
    RUN_TEST(test_mc_sensorless_update_estimates_angle_and_speed);
    RUN_TEST(test_mc_sensorless_update_flags_low_bemf_invalid);
    RUN_TEST(test_mc_sensorless_init_rejects_invalid_pll_gains);
    RUN_TEST(test_mc_sensorless_init_rejects_lock_bemf_not_above_min_bemf);
    RUN_TEST(test_mc_sensorless_init_rejects_open_loop_voltage_max_below_start);
    RUN_TEST(test_mc_sensorless_uses_inductance_compensation);
    RUN_TEST(test_mc_sensorless_requires_consecutive_lock_samples);
    RUN_TEST(test_mc_sensorless_requires_consecutive_unlock_samples);
    RUN_TEST(test_mc_sensorless_open_loop_handoff_requires_lock_bemf_threshold);
    RUN_TEST(test_mc_sensorless_low_bemf_resets_lock_and_unlock_counters);
    RUN_TEST(test_mc_smo_init_accepts_valid_config);
    RUN_TEST(test_mc_smo_init_rejects_null_state);
    RUN_TEST(test_mc_smo_init_rejects_null_cfg);
    RUN_TEST(test_mc_smo_init_rejects_zero_pole_pairs);
    RUN_TEST(test_mc_smo_init_rejects_zero_k_slide);
    RUN_TEST(test_mc_smo_init_rejects_lock_bemf_below_min);
    RUN_TEST(test_mc_smo_init_rejects_open_loop_voltage_inverted);
    RUN_TEST(test_mc_smo_update_null_state_rejected);
    RUN_TEST(test_mc_smo_update_null_voltage_rejected);
    RUN_TEST(test_mc_smo_update_zero_current_does_not_diverge);
    RUN_TEST(test_mc_smo_update_valid_bemf_sets_observer_valid);
    RUN_TEST(test_mc_smo_open_loop_startup_advances_angle);
    RUN_TEST(test_mc_bldc_hall_commutates_phase_modes);
    RUN_TEST(test_default_cfg_has_reasonable_values);
    RUN_TEST(test_init_sets_idle_phase);
    RUN_TEST(test_init_null_returns_error);
    RUN_TEST(test_init_null_cfg_returns_error);
    RUN_TEST(test_init_rejects_zero_align_time);
    RUN_TEST(test_init_rejects_zero_ramp_start_freq);
    RUN_TEST(test_init_rejects_ramp_end_less_than_start);
    RUN_TEST(test_init_rejects_zero_ramp_time);
    RUN_TEST(test_init_rejects_start_duty_negative);
    RUN_TEST(test_init_rejects_start_duty_over_one);
    RUN_TEST(test_init_rejects_end_duty_less_than_start);
    RUN_TEST(test_init_rejects_end_duty_over_one);
    RUN_TEST(test_init_accepts_end_duty_equal_one);
    RUN_TEST(test_init_accepts_end_freq_equal_start);
    RUN_TEST(test_start_transitions_from_idle_to_align);
    RUN_TEST(test_start_null_returns_error);
    RUN_TEST(test_reset_returns_to_idle);
    RUN_TEST(test_reset_preserves_configuration);
    RUN_TEST(test_reset_null_returns_error);
    RUN_TEST(test_idle_produces_zero_pwm);
    RUN_TEST(test_align_commutates_and_transitions_to_ramp);
    RUN_TEST(test_ramp_open_step_advances_on_timer);
    RUN_TEST(test_run_detects_zc_and_commutates);
    RUN_TEST(test_run_fallback_on_missing_zc);
    RUN_TEST(test_ramp_advances_frequency_over_time);
    RUN_TEST(test_floating_phase_map);
    RUN_TEST(test_get_speed_rpm_returns_stored_value);
    RUN_TEST(test_get_speed_rpm_null_returns_zero);
    RUN_TEST(test_run_advances_to_run_after_ramp);
    RUN_TEST(test_run_null_pwm_returns_error);
    RUN_TEST(test_run_null_ss_returns_error);
    RUN_TEST(test_bemf_threshold_blocks_small_crossing);
    RUN_TEST(test_bemf_threshold_allows_large_crossing);
    RUN_TEST(test_advance_angle_default_is_30_degrees);
    RUN_TEST(test_advance_angle_affects_commutation_delay_in_advance_step);
    RUN_TEST(test_advance_angle_zero_commutates_immediately_after_zc);
    RUN_TEST(test_advance_angle_60_waits_full_step_period);
    RUN_TEST(test_advance_angle_used_at_ramp_to_run_transition);
    RUN_TEST(test_mc_stop_resets_bldc_sensorless_to_idle);
    RUN_TEST(test_mc_pmsm_foc_runs_pipeline_and_updates_pwm);
    RUN_TEST(test_mc_pmsm_foc_limits_voltage_vector);
    RUN_TEST(test_mc_pmsm_foc_normalizes_with_bus_voltage);
    RUN_TEST(test_mc_pmsm_foc_runs_with_1shunt_reconstruction);
    RUN_TEST(test_mc_pmsm_foc_1shunt_invalid_window_uses_predicted_current);
    RUN_TEST(test_mc_pmsm_foc_1shunt_prediction_depends_on_motor_model);
    RUN_TEST(test_mc_pmsm_foc_1shunt_prediction_depends_on_elec_speed);
    RUN_TEST(test_mc_pmsm_foc_1shunt_prediction_is_limited_in_high_modulation);
    RUN_TEST(test_mc_pmsm_foc_1shunt_prediction_is_limited_in_field_weakening);
    RUN_TEST(test_mc_pmsm_foc_1shunt_high_modulation_uses_trailing_sampling);
    RUN_TEST(test_mc_pmsm_foc_1shunt_field_weakening_reorders_to_low_zero_vector);
    RUN_TEST(test_mc_pmsm_foc_runs_with_2shunt_reconstruction);
    return UNITY_END();
}
