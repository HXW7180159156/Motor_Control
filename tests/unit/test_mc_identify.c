#include "unity.h"
#include "mc_identify.h"

#include <math.h>

void test_mc_identify_flux_estimate_is_generated_after_lq_stage(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in = {{0.2F, 0.1F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.ld_pulse_time_s = 0.001F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.001F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_INJECT;
    id.rs_ohm = 0.2F;
    id.lq_h = 0.05F;
    id.i_beta_prev = 0.08F;
    id.step_counter = 0U;
    id.total_steps = 1U;

    status = mc_identify_run(&id, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);

    status = mc_identify_run(&id, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_COMPLETE, id.state);
    TEST_ASSERT_TRUE(id.flux_wb > 0.0F);
    TEST_ASSERT_TRUE(id.flux_sample_count > 0U);
}

void test_mc_identify_ld_measure_holds_zero_voltage_before_lq_align(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in = {{0.1F, 0.0F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.ld_pulse_time_s = 0.002F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LD_MEASURE;

    status = mc_identify_run(&id, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LD_MEASURE, id.state);
    TEST_ASSERT_TRUE(id.v_alpha == 0.0F);
    TEST_ASSERT_TRUE(id.v_beta == 0.0F);

    status = mc_identify_run(&id, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_ALIGN, id.state);
}

void test_mc_identify_ld_estimate_uses_window_average_current_for_rs_drop(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_a = {{0.2F, 0.0F}, 0.001F};
    mc_identify_input_t in_b = {{0.4F, 0.0F}, 0.001F};
    mc_identify_input_t in_c = {{0.7F, 0.0F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;
    mc_f32_t expected_ld;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.ld_pulse_time_s = 0.003F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LD_INJECT;
    id.rs_ohm = 0.5F;
    id.i_alpha_prev = 0.1F;

    expected_ld = (1.0F - (0.5F * ((0.2F + 0.4F + 0.7F) / 3.0F))) * 0.003F / (0.7F - 0.1F);

    status = mc_identify_run(&id, &in_a, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LD_INJECT, id.state);

    status = mc_identify_run(&id, &in_b, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LD_INJECT, id.state);

    status = mc_identify_run(&id, &in_c, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LD_MEASURE, id.state);
    TEST_ASSERT_TRUE(fabsf(id.ld_h - expected_ld) < 1e-6F);
}

void test_mc_identify_lq_estimate_uses_window_average_current_for_rs_drop(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_a = {{0.0F, 0.25F}, 0.001F};
    mc_identify_input_t in_b = {{0.0F, 0.45F}, 0.001F};
    mc_identify_input_t in_c = {{0.0F, 0.8F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;
    mc_f32_t expected_lq;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.003F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_INJECT;
    id.rs_ohm = 0.4F;
    id.i_beta_prev = 0.1F;

    expected_lq = (1.0F - (0.4F * ((0.25F + 0.45F + 0.8F) / 3.0F))) * 0.003F / (0.8F - 0.1F);

    status = mc_identify_run(&id, &in_a, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_INJECT, id.state);

    status = mc_identify_run(&id, &in_b, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_INJECT, id.state);

    status = mc_identify_run(&id, &in_c, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);
    TEST_ASSERT_TRUE(fabsf(id.lq_h - expected_lq) < 1e-6F);
}

void test_mc_identify_ld_estimate_uses_actual_discrete_pulse_time_when_dt_overshoots_target(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_a = {{0.2F, 0.0F}, 0.001F};
    mc_identify_input_t in_b = {{0.4F, 0.0F}, 0.001F};
    mc_identify_input_t in_c = {{0.8F, 0.0F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;
    mc_f32_t expected_ld;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.ld_pulse_time_s = 0.0025F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LD_INJECT;
    id.rs_ohm = 0.5F;
    id.i_alpha_prev = 0.1F;

    expected_ld = (1.0F - (0.5F * ((0.2F + 0.4F + 0.8F) / 3.0F))) * 0.003F / (0.8F - 0.1F);

    status = mc_identify_run(&id, &in_a, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LD_INJECT, id.state);

    status = mc_identify_run(&id, &in_b, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LD_INJECT, id.state);

    status = mc_identify_run(&id, &in_c, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LD_MEASURE, id.state);
    TEST_ASSERT_TRUE(fabsf(id.ld_h - expected_ld) < 1e-6F);
}

void test_mc_identify_lq_inject_finishes_on_accumulated_discrete_pulse_time_with_variable_dt(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_a = {{0.0F, 0.25F}, 0.001F};
    mc_identify_input_t in_b = {{0.0F, 0.45F}, 0.0004F};
    mc_identify_input_t in_c = {{0.0F, 0.65F}, 0.0004F};
    mc_identify_input_t in_d = {{0.0F, 0.9F}, 0.0004F};
    mc_identify_output_t out = {0};
    mc_status_t status;
    mc_f32_t expected_lq;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.002F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_INJECT;
    id.rs_ohm = 0.4F;
    id.i_beta_prev = 0.1F;

    expected_lq = (1.0F - (0.4F * ((0.25F * 0.001F) + (0.45F * 0.0004F) + (0.65F * 0.0004F) + (0.9F * 0.0004F)) / 0.0022F)) *
        0.0022F / (0.9F - 0.1F);

    status = mc_identify_run(&id, &in_a, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_INJECT, id.state);

    status = mc_identify_run(&id, &in_b, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_INJECT, id.state);

    status = mc_identify_run(&id, &in_c, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_INJECT, id.state);

    status = mc_identify_run(&id, &in_d, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);
    TEST_ASSERT_TRUE(fabsf(id.lq_h - expected_lq) < 1e-6F);
}

void test_mc_identify_flux_estimate_averages_multiple_valid_samples(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_a = {{0.0F, 0.105F}, 0.001F};
    mc_identify_input_t in_b = {{0.0F, 0.11F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;
    mc_f32_t expected_flux_a;
    mc_f32_t expected_flux_b;
    mc_f32_t expected_avg;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.002F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_MEASURE;
    id.rs_ohm = 0.2F;
    id.lq_h = 0.05F;
    id.v_beta = cfg.pulse_voltage * cfg.voltage_limit;
    id.i_beta_prev = 0.10F;

    expected_flux_a = (1.0F - (0.2F * 0.105F) - (0.05F * ((0.105F - 0.10F) / 0.001F))) /
        (1.5707963268F / cfg.lq_align_time_s);
    expected_flux_b = (1.0F - (0.2F * 0.11F) - (0.05F * ((0.11F - 0.105F) / 0.001F))) /
        (1.5707963268F / cfg.lq_align_time_s);
    expected_avg = 0.5F * (expected_flux_a + expected_flux_b);

    status = mc_identify_run(&id, &in_a, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);
    TEST_ASSERT_EQUAL_INT(1, (int)id.flux_sample_count);

    status = mc_identify_run(&id, &in_b, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_COMPLETE, id.state);
    TEST_ASSERT_EQUAL_INT(2, (int)id.flux_sample_count);
    TEST_ASSERT_TRUE(fabsf(id.flux_wb - expected_avg) < 1e-6F);
}

void test_mc_identify_flux_estimate_time_weights_variable_dt_samples(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_a = {{0.0F, 0.104F}, 0.001F};
    mc_identify_input_t in_b = {{0.0F, 0.105F}, 0.0004F};
    mc_identify_input_t in_c = {{0.0F, 0.106F}, 0.0004F};
    mc_identify_input_t in_d = {{0.0F, 0.107F}, 0.0004F};
    mc_identify_output_t out = {0};
    mc_status_t status;
    mc_f32_t omega_elec;
    mc_f32_t flux_a;
    mc_f32_t flux_b;
    mc_f32_t flux_c;
    mc_f32_t flux_d;
    mc_f32_t expected_flux;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.002F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_MEASURE;
    id.rs_ohm = 0.2F;
    id.lq_h = 0.05F;
    id.v_beta = cfg.pulse_voltage * cfg.voltage_limit;
    id.i_beta_prev = 0.10F;

    omega_elec = 1.5707963268F / cfg.lq_align_time_s;
    flux_a = (1.0F - (0.2F * 0.104F) - (0.05F * ((0.104F - 0.10F) / 0.001F))) / omega_elec;
    flux_b = (1.0F - (0.2F * 0.105F) - (0.05F * ((0.105F - 0.104F) / 0.0004F))) / omega_elec;
    flux_c = (1.0F - (0.2F * 0.106F) - (0.05F * ((0.106F - 0.105F) / 0.0004F))) / omega_elec;
    flux_d = (1.0F - (0.2F * 0.107F) - (0.05F * ((0.107F - 0.106F) / 0.0004F))) / omega_elec;
    expected_flux = ((flux_a * 0.001F) + (flux_b * 0.0004F) + (flux_c * 0.0004F) + (flux_d * 0.0004F)) / 0.0022F;

    status = mc_identify_run(&id, &in_a, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);

    status = mc_identify_run(&id, &in_b, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);

    status = mc_identify_run(&id, &in_c, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);

    status = mc_identify_run(&id, &in_d, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_COMPLETE, id.state);
    TEST_ASSERT_EQUAL_INT(4, (int)id.flux_sample_count);
    TEST_ASSERT_TRUE(fabsf(id.flux_wb - expected_flux) < 1e-6F);
}

void test_mc_identify_flux_estimate_rejects_current_opposite_to_q_axis_excitation(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in = {{0.0F, -0.05F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.001F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_MEASURE;
    id.rs_ohm = 0.2F;
    id.lq_h = 0.05F;
    id.v_beta = cfg.pulse_voltage * cfg.voltage_limit;
    id.i_beta_prev = -0.05F;
    id.flux_wb = 0.025F;

    status = mc_identify_run(&id, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_COMPLETE, id.state);
    TEST_ASSERT_EQUAL_INT(0, (int)id.flux_sample_count);
    TEST_ASSERT_TRUE(fabsf(id.flux_time_acc_s) < 1e-12F);
    TEST_ASSERT_TRUE(fabsf(id.flux_candidate_wb) < 1e-12F);
    TEST_ASSERT_TRUE(fabsf(id.flux_wb - 0.025F) < 1e-6F);
}

void test_mc_identify_lq_inject_defers_first_flux_sample_until_lq_measure_when_lq_is_unknown(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_inject = {{0.0F, 0.11F}, 0.001F};
    mc_identify_input_t in_measure = {{0.0F, 0.12F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.001F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_INJECT;
    id.rs_ohm = 0.2F;
    id.i_beta_prev = 0.08F;

    status = mc_identify_run(&id, &in_inject, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);
    TEST_ASSERT_TRUE(id.lq_h > 0.0F);
    TEST_ASSERT_EQUAL_INT(0, (int)id.flux_sample_count);

    status = mc_identify_run(&id, &in_measure, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_COMPLETE, id.state);
    TEST_ASSERT_EQUAL_INT(1, (int)id.flux_sample_count);
    TEST_ASSERT_TRUE(id.flux_wb > 0.0F);
}

void test_mc_identify_flux_estimate_keeps_seed_when_valid_flux_window_is_too_short(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_a = {{0.0F, 0.2F}, 0.001F};
    mc_identify_input_t in_b = {{0.0F, 0.2F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.002F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_MEASURE;
    id.rs_ohm = 0.2F;
    id.lq_h = 0.2F;
    id.v_beta = cfg.pulse_voltage * cfg.voltage_limit;
    id.i_beta_prev = 0.0F;
    id.flux_wb = 0.025F;

    status = mc_identify_run(&id, &in_a, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);
    TEST_ASSERT_EQUAL_INT(0, (int)id.flux_sample_count);

    status = mc_identify_run(&id, &in_b, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_COMPLETE, id.state);
    TEST_ASSERT_EQUAL_INT(1, (int)id.flux_sample_count);
    TEST_ASSERT_TRUE(fabsf(id.flux_time_acc_s - 0.001F) < 1e-9F);
    TEST_ASSERT_TRUE(fabsf(id.flux_wb - 0.025F) < 1e-6F);
}

void test_mc_identify_flux_estimate_updates_once_valid_flux_window_is_long_enough(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_a = {{0.0F, 0.1F}, 0.0004F};
    mc_identify_input_t in_b = {{0.0F, 0.1F}, 0.0008F};
    mc_identify_input_t in_c = {{0.0F, 0.1F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;
    mc_f32_t omega_elec;
    mc_f32_t flux_b;
    mc_f32_t flux_c;
    mc_f32_t expected_flux;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.002F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_MEASURE;
    id.rs_ohm = 0.2F;
    id.lq_h = 0.05F;
    id.v_beta = cfg.pulse_voltage * cfg.voltage_limit;
    id.i_beta_prev = 0.0F;
    id.flux_wb = 0.025F;

    omega_elec = 1.5707963268F / cfg.lq_align_time_s;
    flux_b = (1.0F - (0.2F * 0.1F) - (0.05F * ((0.1F - 0.1F) / 0.0008F))) / omega_elec;
    flux_c = (1.0F - (0.2F * 0.1F) - (0.05F * ((0.1F - 0.1F) / 0.001F))) / omega_elec;
    expected_flux = ((flux_b * 0.0008F) + (flux_c * 0.001F)) / 0.0018F;

    status = mc_identify_run(&id, &in_a, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);
    TEST_ASSERT_EQUAL_INT(0, (int)id.flux_sample_count);

    status = mc_identify_run(&id, &in_b, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);
    TEST_ASSERT_EQUAL_INT(1, (int)id.flux_sample_count);

    status = mc_identify_run(&id, &in_c, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_COMPLETE, id.state);
    TEST_ASSERT_EQUAL_INT(2, (int)id.flux_sample_count);
    TEST_ASSERT_TRUE(fabsf(id.flux_time_acc_s - 0.0018F) < 1e-9F);
    TEST_ASSERT_TRUE(fabsf(id.flux_wb - expected_flux) < 1e-6F);
}

void test_mc_identify_lq_measure_requires_full_window_before_complete(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_a = {{0.0F, 0.105F}, 0.001F};
    mc_identify_input_t in_b = {{0.0F, 0.11F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.002F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_MEASURE;
    id.rs_ohm = 0.2F;
    id.lq_h = 0.05F;
    id.v_beta = cfg.pulse_voltage * cfg.voltage_limit;
    id.i_beta_prev = 0.10F;

    status = mc_identify_run(&id, &in_a, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_LQ_MEASURE, id.state);
    TEST_ASSERT_TRUE(id.v_beta > 0.0F);

    status = mc_identify_run(&id, &in_b, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_COMPLETE, id.state);
}

void test_mc_identify_rs_negative_estimate_is_rejected(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in = {{-0.1F, 0.0F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.rs_voltage = 0.5F;
    cfg.rs_sample_time_s = 0.001F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_RS_MEASURE;

    status = mc_identify_run(&id, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(id.rs_candidate_ohm < 0.0F);
    TEST_ASSERT_TRUE(fabsf(id.rs_ohm) < 1e-12F);
}

void test_mc_identify_ld_negative_estimate_is_rejected(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in = {{0.1F, 0.0F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.ld_pulse_time_s = 0.001F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LD_INJECT;
    id.i_alpha_prev = 0.2F;

    status = mc_identify_run(&id, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(id.ld_candidate_h <= 0.0F);
    TEST_ASSERT_TRUE(fabsf(id.ld_h) < 1e-12F);
}

void test_mc_identify_lq_negative_estimate_is_rejected(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in = {{0.0F, 0.1F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_pulse_time_s = 0.001F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_INJECT;
    id.i_beta_prev = 0.2F;

    status = mc_identify_run(&id, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(id.lq_candidate_h <= 0.0F);
    TEST_ASSERT_TRUE(fabsf(id.lq_h) < 1e-12F);
}

void test_mc_identify_flux_estimate_keeps_seed_when_no_valid_samples_exist(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in = {{0.0F, 1.0F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.001F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_MEASURE;
    id.rs_ohm = 0.2F;
    id.lq_h = 0.05F;
    id.v_beta = cfg.pulse_voltage * cfg.voltage_limit;
    id.i_beta_prev = 0.0F;
    id.flux_wb = 0.025F;

    status = mc_identify_run(&id, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_COMPLETE, id.state);
    TEST_ASSERT_EQUAL_INT(0, (int)id.flux_sample_count);
    TEST_ASSERT_TRUE(fabsf(id.flux_wb - 0.025F) < 1e-6F);
}

void test_mc_identify_get_result_returns_flux_estimate(void)
{
    mc_identify_t id = {0};
    mc_f32_t flux = 0.0F;

    id.flux_wb = 0.123F;
    mc_identify_get_result(&id, NULL, NULL, NULL, &flux);
    TEST_ASSERT_TRUE(flux > 0.122F);
    TEST_ASSERT_TRUE(flux < 0.124F);
}

void test_mc_identify_flux_candidate_tracks_raw_measurement_before_trust_gate(void)
{
    mc_identify_t id;
    mc_identify_cfg_t cfg = {0};
    mc_identify_input_t in_a = {{0.0F, 0.2F}, 0.001F};
    mc_identify_input_t in_b = {{0.0F, 0.2F}, 0.001F};
    mc_identify_output_t out = {0};
    mc_status_t status;
    mc_f32_t omega_elec;
    mc_f32_t expected_candidate;

    cfg.svpwm_cfg = (mc_svpwm_cfg_t){0.0F, 1.0F, 0.95F};
    cfg.voltage_limit = 2.0F;
    cfg.max_current_a = 10.0F;
    cfg.pulse_voltage = 0.5F;
    cfg.lq_align_time_s = 0.1F;
    cfg.lq_pulse_time_s = 0.002F;

    mc_identify_init(&id, &cfg);
    id.state = MC_IDENTIFY_STATE_LQ_MEASURE;
    id.rs_ohm = 0.2F;
    id.lq_h = 0.2F;
    id.v_beta = cfg.pulse_voltage * cfg.voltage_limit;
    id.i_beta_prev = 0.0F;
    id.flux_wb = 0.025F;

    omega_elec = 1.5707963268F / cfg.lq_align_time_s;
    expected_candidate = (1.0F - (0.2F * 0.2F) - (0.2F * ((0.2F - 0.2F) / 0.001F))) / omega_elec;

    status = mc_identify_run(&id, &in_a, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_identify_run(&id, &in_b, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_COMPLETE, id.state);
    TEST_ASSERT_EQUAL_INT(1, (int)id.flux_sample_count);
    TEST_ASSERT_TRUE(fabsf(id.flux_candidate_wb - expected_candidate) < 1e-6F);
    TEST_ASSERT_TRUE(fabsf(id.flux_wb - 0.025F) < 1e-6F);
}
