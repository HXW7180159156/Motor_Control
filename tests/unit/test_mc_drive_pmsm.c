#include "unity.h"
#include "mc_drive_pmsm.h"

#include <math.h>

void test_mc_pmsm_foc_runs_pipeline_and_updates_pwm(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}},
        1.0F,
        5.0F
    };
    mc_pmsm_foc_input_t in = {
        {1010U, 990U, 1000U, 0U, 0U},
        24.0F,
        0.0F,
        1.0F,
        0.0F,
        0.5F,
        0.001F,
        0.0F
    };
    mc_pmsm_foc_output_t out = {0};
    mc_status_t status;

    status = mc_pmsm_foc_init(&foc, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(out.pwm_cmd.valid == 1U);
    TEST_ASSERT_TRUE(out.i_dq.q < 0.0F);
    TEST_ASSERT_TRUE(out.v_dq.q > 0.5F);
    TEST_ASSERT_TRUE(out.v_dq.q < 1.0F);
}

void test_mc_pmsm_foc_limits_voltage_vector(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {
        {5.0F, 0.0F, -10.0F, 10.0F, -10.0F, 10.0F},
        {5.0F, 0.0F, -10.0F, 10.0F, -10.0F, 10.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}},
        0.5F,
        5.0F
    };
    mc_pmsm_foc_input_t in = {
        {900U, 900U, 1000U, 0U, 0U},
        24.0F,
        0.0F,
        1.0F,
        1.0F,
        1.0F,
        0.001F,
        0.0F
    };
    mc_pmsm_foc_output_t out = {0};
    mc_status_t status;
    mc_f32_t magnitude_sq;

    status = mc_pmsm_foc_init(&foc, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    magnitude_sq = (out.v_dq.d * out.v_dq.d) + (out.v_dq.q * out.v_dq.q);
    TEST_ASSERT_TRUE(magnitude_sq < 0.251F);
}

void test_mc_pmsm_foc_normalizes_with_bus_voltage(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {
        {0.0F, 0.0F, -10.0F, 10.0F, -10.0F, 10.0F},
        {2.0F, 0.0F, -10.0F, 10.0F, -10.0F, 10.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}},
        10.0F,
        5.0F
    };
    mc_pmsm_foc_input_t in = {
        {1000U, 1000U, 1000U, 0U, 0U},
        10.0F,
        0.0F,
        1.0F,
        0.0F,
        1.0F,
        0.001F,
        0.0F
    };
    mc_pmsm_foc_output_t out = {0};
    mc_status_t status;

    status = mc_pmsm_foc_init(&foc, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(out.v_ab.alpha > -0.01F);
    TEST_ASSERT_TRUE(out.v_ab.alpha < 0.01F);
    TEST_ASSERT_TRUE(out.v_ab.beta > 0.19F);
    TEST_ASSERT_TRUE(out.v_ab.beta < 0.21F);
    TEST_ASSERT_TRUE(out.pwm_cmd.duty_a > 0.49F);
    TEST_ASSERT_TRUE(out.pwm_cmd.duty_a < 0.51F);
    TEST_ASSERT_TRUE(out.pwm_cmd.duty_b > out.pwm_cmd.duty_c);
}

void test_mc_pmsm_foc_runs_with_2shunt_reconstruction(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_2SHUNT, {{1000.0F, 1000.0F, 0.01F, 0.01F}}},
        1.0F,
        5.0F
    };
    mc_pmsm_foc_input_t in = {
        {1010U, 990U, 0U, 2400U, 0U},
        24.0F,
        0.0F,
        1.0F,
        0.0F,
        0.5F,
        0.001F,
        0.0F
    };
    mc_pmsm_foc_output_t out = {0};
    mc_status_t status;

    status = mc_pmsm_foc_init(&foc, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(out.pwm_cmd.valid == 1U);
    TEST_ASSERT_TRUE(out.i_abc.a > 0.09F);
    TEST_ASSERT_TRUE(out.i_abc.a < 0.11F);
    TEST_ASSERT_TRUE(out.i_abc.b > -0.11F);
    TEST_ASSERT_TRUE(out.i_abc.b < -0.09F);
    TEST_ASSERT_TRUE(out.i_abc.c > -0.01F);
    TEST_ASSERT_TRUE(out.i_abc.c < 0.01F);
}

void test_mc_pmsm_foc_runs_with_1shunt_reconstruction(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.10F, 0.02F, 0.0001F, 0.000002F, 0.000004F}}},
        1.0F,
        5.0F,
        0.5F,
        0.002F,
        0.003F,
        0.02F,
        2.0F
    };
    mc_pmsm_foc_input_t in = {
        {1100U, 0U, 0U, 2400U, 0U},
        24.0F,
        0.0F,
        1.0F,
        0.0F,
        0.5F,
        0.001F,
        0.0F
    };
    mc_pmsm_foc_output_t out = {0};
    mc_status_t status;

    status = mc_pmsm_foc_init(&foc, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    foc.last_sector = 2U;

    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(out.pwm_cmd.valid == 1U);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_valid == MC_TRUE);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_count == 2U);
    TEST_ASSERT_TRUE(foc.shunt1_meta.reorder_required == MC_FALSE);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_a != MC_1SHUNT_HALF_CYCLE_NONE);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_b != MC_1SHUNT_HALF_CYCLE_NONE);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_position_a > 0.0F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_position_b > 0.0F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_position_a >= 0.0F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_position_b >= 0.0F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_time_a > 0.0F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_time_b > 0.0F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_time_a >= 0.0F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_time_b >= 0.0F);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.count == 2U);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.valid == MC_TRUE);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_b.valid == MC_TRUE);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.signal == MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_b.signal == MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.event == MC_ADC_TRIGGER_EVENT_PWM_DOWN);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_b.event == ((foc.shunt1_meta.sample_half_b == MC_1SHUNT_HALF_CYCLE_UP) ? MC_ADC_TRIGGER_EVENT_PWM_UP : MC_ADC_TRIGGER_EVENT_PWM_DOWN));
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.soc_index == 0U);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_b.soc_index == 1U);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.reorder_applied == MC_FALSE);
    TEST_ASSERT_TRUE(foc.last_sector >= 1U);
    TEST_ASSERT_TRUE(foc.last_sector <= 6U);

    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(out.pwm_cmd.common_mode_shift > 0.0F);
}

void test_mc_pmsm_foc_1shunt_invalid_window_uses_predicted_current(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.40F, 0.10F, 0.0001F, 0.000002F, 0.000004F}}},
        1.0F,
        5.0F,
        0.5F,
        0.002F,
        0.003F,
        0.02F,
        2.0F
    };
    mc_pmsm_foc_input_t in = {
        {1100U, 0U, 0U, 2400U, 0U},
        24.0F,
        0.0F,
        1.0F,
        0.0F,
        0.5F,
        0.001F,
        20.0F
    };
    mc_pmsm_foc_output_t out = {0};
    mc_status_t status;

    status = mc_pmsm_foc_init(&foc, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    foc.i_abc_last = (mc_abc_t){0.25F, -0.15F, -0.10F};
    foc.i_dq = (mc_dq_t){0.05F, 0.10F};
    foc.v_dq = (mc_dq_t){0.40F, 0.20F};
    foc.shunt1_meta = (mc_1shunt_meta_t){0};
    foc.shunt1_meta.sector = 2U;
    foc.shunt1_meta.sample_count = 1U;
    foc.shunt1_meta.compensation_required = MC_TRUE;
    foc.shunt1_meta.zero_vector_bias_high = MC_TRUE;
    foc.shunt1_meta.reorder_required = MC_TRUE;
    foc.shunt1_meta.preferred_window = 2U;
    foc.shunt1_meta.sample_window_a = 0.20F;
    foc.shunt1_meta.sample_window_b = 0.15F;
    foc.shunt1_meta.sample_phase_a = MC_1SHUNT_SAMPLE_PHASE_TRAILING;
    foc.shunt1_meta.sample_phase_b = MC_1SHUNT_SAMPLE_PHASE_TRAILING;
    foc.shunt1_meta.sample_half_a = MC_1SHUNT_HALF_CYCLE_DOWN;
    foc.shunt1_meta.sample_half_b = MC_1SHUNT_HALF_CYCLE_DOWN;
    foc.shunt1_meta.sample_position_a = 0.19F;
    foc.shunt1_meta.sample_position_b = 0.21F;
    foc.shunt1_meta.sample_half_position_a = 0.38F;
    foc.shunt1_meta.sample_half_position_b = 0.42F;
    foc.shunt1_meta.sample_time_a = 0.000019F;
    foc.shunt1_meta.sample_time_b = 0.000021F;
    foc.shunt1_meta.sample_half_time_a = 0.000019F;
    foc.shunt1_meta.sample_half_time_b = 0.000021F;

    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(out.i_abc.a > 0.0F);
    TEST_ASSERT_TRUE(out.i_abc.c < -0.05F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.reorder_required == MC_TRUE);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_a == MC_1SHUNT_HALF_CYCLE_DOWN);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_b == MC_1SHUNT_HALF_CYCLE_DOWN);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_position_a >= 0.44F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_position_b >= 0.44F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_time_a >= 0.000044F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_time_b >= 0.000044F);
    TEST_ASSERT_TRUE(out.pwm_cmd.common_mode_shift > 0.03F);
    TEST_ASSERT_TRUE(out.pwm_cmd.phase_mode_b == MC_PWM_PHASE_HIGH);
    TEST_ASSERT_TRUE(out.pwm_cmd.phase_mode_c == MC_PWM_PHASE_HIGH);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.count == foc.shunt1_meta.sample_count);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.valid == MC_TRUE);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_b.valid == MC_TRUE);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.total_count == 1U);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_b.total_count == 1U);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.reorder_applied == MC_TRUE);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_b.reorder_applied == MC_TRUE);
}

void test_mc_pmsm_foc_1shunt_prediction_depends_on_motor_model(void)
{
    mc_pmsm_foc_t foc_fast;
    mc_pmsm_foc_t foc_slow;
    mc_pmsm_foc_cfg_t cfg_fast = {
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.40F, 0.10F, 0.0001F, 0.000002F, 0.000004F}}},
        1.0F,
        5.0F,
        0.2F,
        0.001F,
        0.0015F,
        0.01F,
        2.0F
    };
    mc_pmsm_foc_cfg_t cfg_slow = cfg_fast;
    mc_pmsm_foc_input_t in = {
        {1100U, 0U, 0U, 2400U, 0U},
        24.0F,
        0.0F,
        1.0F,
        0.0F,
        0.5F,
        0.001F,
        20.0F
    };
    mc_pmsm_foc_output_t out_fast = {0};
    mc_pmsm_foc_output_t out_slow = {0};
    mc_status_t status;

    cfg_slow.ld_h = 0.01F;
    cfg_slow.lq_h = 0.015F;
    cfg_slow.rs_ohm = 1.0F;

    status = mc_pmsm_foc_init(&foc_fast, &cfg_fast);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_pmsm_foc_init(&foc_slow, &cfg_slow);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    foc_fast.i_abc_last = (mc_abc_t){0.25F, -0.15F, -0.10F};
    foc_fast.i_dq = (mc_dq_t){0.05F, 0.10F};
    foc_fast.v_dq = (mc_dq_t){0.40F, 0.20F};
    foc_fast.shunt1_meta = (mc_1shunt_meta_t){0};
    foc_fast.shunt1_meta.compensation_required = MC_TRUE;
    foc_fast.shunt1_meta.sample_count = 1U;
    foc_fast.shunt1_meta.reorder_required = MC_TRUE;

    foc_slow.i_abc_last = foc_fast.i_abc_last;
    foc_slow.i_dq = foc_fast.i_dq;
    foc_slow.v_dq = foc_fast.v_dq;
    foc_slow.shunt1_meta = foc_fast.shunt1_meta;

    status = mc_pmsm_foc_run(&foc_fast, &in, &out_fast);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_pmsm_foc_run(&foc_slow, &in, &out_slow);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    TEST_ASSERT_TRUE((out_fast.i_abc.a > (out_slow.i_abc.a + 0.001F)) || (out_fast.i_abc.a < (out_slow.i_abc.a - 0.001F)));
}

void test_mc_pmsm_foc_1shunt_prediction_depends_on_elec_speed(void)
{
    mc_pmsm_foc_t foc_low_speed;
    mc_pmsm_foc_t foc_high_speed;
    mc_pmsm_foc_cfg_t cfg = {
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.40F, 0.10F, 0.0001F, 0.000002F, 0.000004F}}},
        1.0F,
        5.0F,
        0.5F,
        0.002F,
        0.003F,
        0.02F,
        2.0F
    };
    mc_pmsm_foc_input_t in_low_speed = {
        {1100U, 0U, 0U, 2400U, 0U},
        24.0F,
        0.0F,
        1.0F,
        0.0F,
        0.5F,
        0.001F,
        10.0F
    };
    mc_pmsm_foc_input_t in_high_speed = in_low_speed;
    mc_pmsm_foc_output_t out_low_speed = {0};
    mc_pmsm_foc_output_t out_high_speed = {0};
    mc_status_t status;

    in_high_speed.elec_speed_rad_s = 200.0F;

    status = mc_pmsm_foc_init(&foc_low_speed, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_pmsm_foc_init(&foc_high_speed, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    foc_low_speed.i_abc_last = (mc_abc_t){0.25F, -0.15F, -0.10F};
    foc_low_speed.i_dq = (mc_dq_t){0.05F, 0.10F};
    foc_low_speed.v_dq = (mc_dq_t){0.40F, 0.20F};
    foc_low_speed.shunt1_meta = (mc_1shunt_meta_t){0};
    foc_low_speed.shunt1_meta.compensation_required = MC_TRUE;
    foc_low_speed.shunt1_meta.sample_count = 1U;
    foc_low_speed.shunt1_meta.reorder_required = MC_TRUE;

    foc_high_speed.i_abc_last = foc_low_speed.i_abc_last;
    foc_high_speed.i_dq = foc_low_speed.i_dq;
    foc_high_speed.v_dq = foc_low_speed.v_dq;
    foc_high_speed.shunt1_meta = foc_low_speed.shunt1_meta;

    status = mc_pmsm_foc_run(&foc_low_speed, &in_low_speed, &out_low_speed);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_pmsm_foc_run(&foc_high_speed, &in_high_speed, &out_high_speed);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    TEST_ASSERT_TRUE((out_low_speed.i_abc.a > (out_high_speed.i_abc.a + 0.001F)) ||
                     (out_low_speed.i_abc.a < (out_high_speed.i_abc.a - 0.001F)));
}

void test_mc_pmsm_foc_1shunt_prediction_is_limited_in_high_modulation(void)
{
    mc_pmsm_foc_t foc_nominal;
    mc_pmsm_foc_t foc_high_mod;
    mc_pmsm_foc_cfg_t cfg = {
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.40F, 0.10F, 0.0001F, 0.000002F, 0.000004F}}},
        1.0F,
        5.0F,
        0.5F,
        0.002F,
        0.003F,
        0.02F,
        2.0F
    };
    mc_pmsm_foc_input_t in = {
        {1100U, 0U, 0U, 2400U, 0U},
        24.0F,
        0.0F,
        1.0F,
        0.0F,
        0.5F,
        0.001F,
        20.0F
    };
    mc_pmsm_foc_output_t out_nominal = {0};
    mc_pmsm_foc_output_t out_high_mod = {0};
    mc_status_t status;

    status = mc_pmsm_foc_init(&foc_nominal, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_pmsm_foc_init(&foc_high_mod, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    foc_nominal.i_dq = (mc_dq_t){0.05F, 0.10F};
    foc_nominal.v_dq = (mc_dq_t){0.20F, 0.10F};
    foc_nominal.shunt1_meta.compensation_required = MC_TRUE;

    foc_high_mod.i_dq = foc_nominal.i_dq;
    foc_high_mod.v_dq = (mc_dq_t){0.90F, 0.40F};
    foc_high_mod.shunt1_meta.compensation_required = MC_TRUE;

    status = mc_pmsm_foc_run(&foc_nominal, &in, &out_nominal);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_pmsm_foc_run(&foc_high_mod, &in, &out_high_mod);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    TEST_ASSERT_TRUE(out_nominal.current_comp_status.active == MC_TRUE);
    TEST_ASSERT_TRUE(out_nominal.current_comp_status.mode == MC_1SHUNT_COMP_PREDICT_BASIC);
    TEST_ASSERT_TRUE(out_high_mod.current_comp_status.active == MC_TRUE);
    TEST_ASSERT_TRUE(out_high_mod.current_comp_status.mode == MC_1SHUNT_COMP_PREDICT_HIGH_MODULATION);
    TEST_ASSERT_TRUE((out_high_mod.i_dq.q > (out_nominal.i_dq.q + 0.001F)) ||
                     (out_high_mod.i_dq.q < (out_nominal.i_dq.q - 0.001F)));
}

void test_mc_pmsm_foc_1shunt_prediction_is_limited_in_field_weakening(void)
{
    mc_pmsm_foc_t foc_nominal;
    mc_pmsm_foc_t foc_fw;
    mc_pmsm_foc_cfg_t cfg = {
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.40F, 0.10F, 0.0001F, 0.000002F, 0.000004F}}},
        1.0F,
        5.0F,
        0.5F,
        0.002F,
        0.003F,
        0.02F,
        2.0F
    };
    mc_pmsm_foc_input_t in_nominal = {
        {1100U, 0U, 0U, 2400U, 0U},
        24.0F,
        0.0F,
        1.0F,
        0.0F,
        0.5F,
        0.001F,
        50.0F
    };
    mc_pmsm_foc_input_t in_fw = in_nominal;
    mc_pmsm_foc_output_t out_nominal = {0};
    mc_pmsm_foc_output_t out_fw = {0};
    mc_status_t status;

    in_fw.id_ref = -0.5F;

    status = mc_pmsm_foc_init(&foc_nominal, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_pmsm_foc_init(&foc_fw, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    foc_nominal.i_dq = (mc_dq_t){0.05F, 0.10F};
    foc_nominal.v_dq = (mc_dq_t){0.30F, 0.15F};
    foc_nominal.shunt1_meta.compensation_required = MC_TRUE;

    foc_fw.i_dq = foc_nominal.i_dq;
    foc_fw.v_dq = foc_nominal.v_dq;
    foc_fw.shunt1_meta.compensation_required = MC_TRUE;

    status = mc_pmsm_foc_run(&foc_nominal, &in_nominal, &out_nominal);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_pmsm_foc_run(&foc_fw, &in_fw, &out_fw);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    TEST_ASSERT_TRUE(out_nominal.current_comp_status.active == MC_TRUE);
    TEST_ASSERT_TRUE(out_nominal.current_comp_status.mode == MC_1SHUNT_COMP_PREDICT_BASIC);
    TEST_ASSERT_TRUE(out_fw.current_comp_status.active == MC_TRUE);
    TEST_ASSERT_TRUE(out_fw.current_comp_status.mode == MC_1SHUNT_COMP_PREDICT_FIELD_WEAKENING);
    TEST_ASSERT_TRUE((out_fw.i_dq.q > (out_nominal.i_dq.q + 0.001F)) ||
                     (out_fw.i_dq.q < (out_nominal.i_dq.q - 0.001F)));
}

void test_mc_pmsm_foc_1shunt_high_modulation_uses_trailing_sampling(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {
        {3.0F, 0.0F, -3.0F, 3.0F, -3.0F, 3.0F},
        {3.0F, 0.0F, -3.0F, 3.0F, -3.0F, 3.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.10F, 0.02F, 0.0001F, 0.000002F, 0.000004F}}},
        2.0F,
        5.0F,
        0.5F,
        0.002F,
        0.003F,
        0.02F,
        2.0F
    };
    mc_pmsm_foc_input_t in = {
        {1100U, 0U, 0U, 2400U, 0U},
        5.0F,
        0.0F,
        1.0F,
        0.0F,
        3.0F,
        0.001F,
        0.0F
    };
    mc_pmsm_foc_output_t out = {0};
    mc_status_t status;

    status = mc_pmsm_foc_init(&foc, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_count == 2U);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_phase_a == MC_1SHUNT_SAMPLE_PHASE_TRAILING);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_phase_b == MC_1SHUNT_SAMPLE_PHASE_TRAILING);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_a == MC_1SHUNT_HALF_CYCLE_DOWN);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_b == MC_1SHUNT_HALF_CYCLE_DOWN);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_position_a > 0.46F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_position_a < 0.48F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_position_b > 0.80F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_position_b < 0.82F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_position_a >= 0.0F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_position_b > 0.60F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_time_a > 0.000045F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_time_a < 0.000047F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_time_b > 0.000080F);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_time_b < 0.000082F);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.count == 2U);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_b.half_position > 0.60F);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.event == MC_ADC_TRIGGER_EVENT_PWM_DOWN);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_b.event == MC_ADC_TRIGGER_EVENT_PWM_DOWN);
}

void test_mc_pmsm_foc_1shunt_field_weakening_reorders_to_low_zero_vector(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {
        {2.5F, 0.0F, -3.0F, 3.0F, -3.0F, 3.0F},
        {2.5F, 0.0F, -3.0F, 3.0F, -3.0F, 3.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_1SHUNT, {.shunt1 = {1000.0F, 0.01F, 0.30F, 0.08F, 0.0001F, 0.000002F, 0.000004F}}},
        1.5F,
        5.0F,
        0.5F,
        0.002F,
        0.003F,
        0.02F,
        2.0F
    };
    mc_pmsm_foc_input_t in = {
        {1100U, 0U, 0U, 2400U, 0U},
        5.0F,
        0.0F,
        1.0F,
        -0.5F,
        -2.5F,
        0.001F,
        40.0F
    };
    mc_pmsm_foc_output_t out = {0};
    mc_status_t status;

    status = mc_pmsm_foc_init(&foc, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(foc.shunt1_meta.reorder_required == MC_TRUE);
    TEST_ASSERT_TRUE(foc.shunt1_meta.zero_vector_bias_high == MC_FALSE);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_phase_a == MC_1SHUNT_SAMPLE_PHASE_LEADING);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_phase_b == MC_1SHUNT_SAMPLE_PHASE_LEADING);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_a == MC_1SHUNT_HALF_CYCLE_UP);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_half_b == MC_1SHUNT_HALF_CYCLE_UP);
    TEST_ASSERT_TRUE(foc.shunt1_meta.sample_position_a > foc.shunt1_meta.sample_position_b);
    TEST_ASSERT_TRUE(out.pwm_cmd.common_mode_shift < -0.03F);
    TEST_ASSERT_TRUE(out.pwm_cmd.phase_mode_b == MC_PWM_PHASE_LOW);
    TEST_ASSERT_TRUE(out.pwm_cmd.phase_mode_c == MC_PWM_PHASE_LOW);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.count == foc.shunt1_meta.sample_count);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.half_position > out.adc_trigger_plan.trigger_b.half_position);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_a.event == MC_ADC_TRIGGER_EVENT_PWM_UP);
    TEST_ASSERT_TRUE(out.adc_trigger_plan.trigger_b.event == MC_ADC_TRIGGER_EVENT_PWM_UP);
}

void test_mc_pmsm_compute_mtpa_id_returns_negative_for_ipm(void)
{
    mc_f32_t id = mc_pmsm_compute_mtpa_id(5.0F, 0.01F, 0.0001F, 0.0003F);
    TEST_ASSERT_TRUE(id < 0.0F);
}

void test_mc_pmsm_compute_mtpa_id_returns_zero_for_spm(void)
{
    mc_f32_t id = mc_pmsm_compute_mtpa_id(5.0F, 0.01F, 0.0002F, 0.0002F);
    TEST_ASSERT_TRUE(fabsf(id) < 1e-12F);
}

void test_mc_pmsm_compute_mtpa_id_returns_zero_at_zero_torque(void)
{
    mc_f32_t id = mc_pmsm_compute_mtpa_id(0.0F, 0.01F, 0.0001F, 0.0003F);
    TEST_ASSERT_TRUE(fabsf(id) < 1e-6F);
}

void test_mc_pmsm_fw_adjustment_increases_when_voltage_saturated(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {
        {1.0F, 0.0F, -10.0F, 10.0F, -10.0F, 10.0F},
        {1.0F, 0.0F, -10.0F, 10.0F, -10.0F, 10.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}},
        0.5F,
        5.0F,
        0.1F,
        0.0005F,
        0.0005F,
        0.02F,
        2.0F,
        MC_FALSE,
        MC_TRUE,
        0.5F,
        1.0F,
        -2.0F,
        0.90F
    };
    mc_pmsm_foc_input_t in = {
        {1010U, 990U, 1000U, 0U, 0U},
        24.0F,
        0.0F,
        1.0F,
        0.0F,
        5.0F,
        0.001F,
        0.0F
    };
    mc_pmsm_foc_output_t out = {0};
    mc_status_t status;

    status = mc_pmsm_foc_init(&foc, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fabsf(foc.id_fw_adjustment) < 1e-12F);

    /* First call: v_dq_prev=(0,0) -> fw_error positive -> stays at 0 */
    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fabsf(foc.id_fw_adjustment) < 1e-6F);

    /* Second call: uses prev cycle limited Vdq -> v_mag>threshold -> fw_error negative -> id_fw goes negative */
    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(foc.id_fw_adjustment < -1e-6F);
}

void test_mc_pmsm_fw_adjustment_unchanged_when_voltage_low(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {
        {1.0F, 0.0F, -10.0F, 10.0F, -10.0F, 10.0F},
        {1.0F, 0.0F, -10.0F, 10.0F, -10.0F, 10.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}},
        100.0F,
        5.0F,
        0.1F,
        0.0005F,
        0.0005F,
        0.02F,
        2.0F,
        MC_FALSE,
        MC_TRUE,
        0.5F,
        1.0F,
        -2.0F,
        0.90F
    };
    mc_pmsm_foc_input_t in = {
        {1010U, 990U, 1000U, 0U, 0U},
        24.0F,
        0.0F,
        1.0F,
        0.0F,
        5.0F,
        0.001F,
        0.0F
    };
    mc_pmsm_foc_output_t out = {0};
    mc_status_t status;

    status = mc_pmsm_foc_init(&foc, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    foc.id_fw_adjustment = -1.0F;

    /* fw_error positive (under threshold) -> integral drifts toward zero */
    status = mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(foc.id_fw_adjustment > -1.0F);
    TEST_ASSERT_TRUE(foc.id_fw_adjustment < 0.0F);
}
