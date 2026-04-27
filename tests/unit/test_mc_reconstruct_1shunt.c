#include "unity.h"
#include "mc_reconstruct_1shunt.h"

void test_mc_reconstruct_1shunt_maps_sector_currents(void)
{
    mc_adc_raw_t raw = {1100U, 0U, 0U, 0U, 0U};
    mc_1shunt_cfg_t cfg = {1000.0F, 0.01F, 0.10F, 0.02F, 0.0001F, 0.000002F, 0.000004F};
    mc_pwm_cmd_t pwm_cmd = {0.60F, 0.20F, 0.20F, 0.0F, MC_PWM_PHASE_PWM, MC_PWM_PHASE_OFF, MC_PWM_PHASE_LOW, 2U, 1U};
    mc_1shunt_meta_t meta = {0};
    mc_abc_t abc = {0.0F, 0.0F, 0.0F};

    mc_reconstruct_1shunt_plan(&pwm_cmd, &cfg, &meta);
    mc_reconstruct_1shunt_run(&raw, &cfg, &meta, &abc);

    TEST_ASSERT_TRUE(meta.sample_valid == MC_TRUE);
    TEST_ASSERT_TRUE(meta.sample_count == 2U);
    TEST_ASSERT_TRUE(meta.compensation_required == MC_FALSE);
    TEST_ASSERT_TRUE(meta.reorder_required == MC_FALSE);
    TEST_ASSERT_TRUE(meta.preferred_window == 1U);
    TEST_ASSERT_TRUE(meta.zero_vector_bias_high == MC_TRUE);
    TEST_ASSERT_TRUE(meta.sample_phase_a == MC_1SHUNT_SAMPLE_PHASE_TRAILING);
    TEST_ASSERT_TRUE(meta.sample_phase_b == MC_1SHUNT_SAMPLE_PHASE_LEADING);
    TEST_ASSERT_TRUE(meta.sample_half_a == MC_1SHUNT_HALF_CYCLE_DOWN);
    TEST_ASSERT_TRUE(meta.sample_half_b == MC_1SHUNT_HALF_CYCLE_UP);
    TEST_ASSERT_TRUE(meta.sample_position_a > 0.56F);
    TEST_ASSERT_TRUE(meta.sample_position_a < 0.58F);
    TEST_ASSERT_TRUE(meta.sample_position_b > 0.12F);
    TEST_ASSERT_TRUE(meta.sample_position_b < 0.14F);
    TEST_ASSERT_TRUE(meta.sample_half_position_a > 0.13F);
    TEST_ASSERT_TRUE(meta.sample_half_position_a < 0.15F);
    TEST_ASSERT_TRUE(meta.sample_half_position_b > 0.25F);
    TEST_ASSERT_TRUE(meta.sample_half_position_b < 0.27F);
    TEST_ASSERT_TRUE(meta.sample_time_a > 0.000056F);
    TEST_ASSERT_TRUE(meta.sample_time_a < 0.000058F);
    TEST_ASSERT_TRUE(meta.sample_time_b > 0.000012F);
    TEST_ASSERT_TRUE(meta.sample_time_b < 0.000014F);
    TEST_ASSERT_TRUE(meta.sample_half_time_a > 0.000006F);
    TEST_ASSERT_TRUE(meta.sample_half_time_a < 0.000008F);
    TEST_ASSERT_TRUE(meta.sample_half_time_b > 0.000012F);
    TEST_ASSERT_TRUE(meta.sample_half_time_b < 0.000014F);

    TEST_ASSERT_TRUE(abc.a > 0.99F);
    TEST_ASSERT_TRUE(abc.a < 1.01F);
    TEST_ASSERT_TRUE(abc.b > -0.01F);
    TEST_ASSERT_TRUE(abc.b < 0.01F);
    TEST_ASSERT_TRUE(abc.c > -1.01F);
    TEST_ASSERT_TRUE(abc.c < -0.99F);
}

void test_mc_reconstruct_1shunt_invalid_window_returns_zero(void)
{
    mc_adc_raw_t raw = {1100U, 0U, 0U, 0U, 0U};
    mc_1shunt_cfg_t cfg = {1000.0F, 0.01F, 0.30F, 0.05F, 0.0001F, 0.000002F, 0.000004F};
    mc_pwm_cmd_t pwm_cmd = {0.32F, 0.31F, 0.37F, 0.0F, MC_PWM_PHASE_PWM, MC_PWM_PHASE_OFF, MC_PWM_PHASE_LOW, 2U, 1U};
    mc_1shunt_meta_t meta = {0};
    mc_abc_t abc = {1.0F, 1.0F, 1.0F};

    mc_reconstruct_1shunt_plan(&pwm_cmd, &cfg, &meta);
    mc_reconstruct_1shunt_run(&raw, &cfg, &meta, &abc);

    TEST_ASSERT_TRUE(meta.sample_valid == MC_FALSE);
    TEST_ASSERT_TRUE(meta.sample_count == 1U);
    TEST_ASSERT_TRUE(meta.compensation_required == MC_TRUE);
    TEST_ASSERT_TRUE(meta.reorder_required == MC_TRUE);
    TEST_ASSERT_TRUE(meta.preferred_window == 2U);
    TEST_ASSERT_TRUE(meta.sample_phase_a == MC_1SHUNT_SAMPLE_PHASE_LEADING);
    TEST_ASSERT_TRUE(meta.sample_phase_b == MC_1SHUNT_SAMPLE_PHASE_LEADING);
    TEST_ASSERT_TRUE(meta.sample_half_a == MC_1SHUNT_HALF_CYCLE_UP);
    TEST_ASSERT_TRUE(meta.sample_half_b == MC_1SHUNT_HALF_CYCLE_UP);
    TEST_ASSERT_TRUE(meta.sample_position_b > 0.33F);
    TEST_ASSERT_TRUE(meta.sample_position_b < 0.35F);
    TEST_ASSERT_TRUE(meta.sample_half_position_b > 0.66F);
    TEST_ASSERT_TRUE(meta.sample_half_position_b < 0.70F);
    TEST_ASSERT_TRUE(meta.sample_position_a > meta.sample_position_b);
    TEST_ASSERT_TRUE(meta.sample_time_b > 0.000033F);
    TEST_ASSERT_TRUE(meta.sample_time_b < 0.000035F);
    TEST_ASSERT_TRUE(meta.sample_half_time_b > 0.000033F);
    TEST_ASSERT_TRUE(meta.sample_half_time_b < 0.000035F);
    TEST_ASSERT_TRUE(abc.a > -0.01F);
    TEST_ASSERT_TRUE(abc.a < 0.01F);
    TEST_ASSERT_TRUE(abc.b > -0.01F);
    TEST_ASSERT_TRUE(abc.b < 0.01F);
    TEST_ASSERT_TRUE(abc.c > -0.01F);
    TEST_ASSERT_TRUE(abc.c < 0.01F);
}

void test_mc_reconstruct_1shunt_high_modulation_prefers_trailing_samples(void)
{
    mc_1shunt_cfg_t cfg = {1000.0F, 0.01F, 0.10F, 0.02F, 0.0001F, 0.000002F, 0.000004F};
    mc_pwm_cmd_t pwm_cmd = {0.88F, 0.80F, 0.12F, 0.0F, MC_PWM_PHASE_PWM, MC_PWM_PHASE_PWM, MC_PWM_PHASE_PWM, 1U, 1U};
    mc_1shunt_meta_t meta = {0};

    mc_reconstruct_1shunt_plan(&pwm_cmd, &cfg, &meta);

    TEST_ASSERT_TRUE(meta.sample_valid == MC_TRUE);
    TEST_ASSERT_TRUE(meta.sample_count == 2U);
    TEST_ASSERT_TRUE(meta.reorder_required == MC_FALSE);
    TEST_ASSERT_TRUE(meta.sample_phase_a == MC_1SHUNT_SAMPLE_PHASE_TRAILING);
    TEST_ASSERT_TRUE(meta.sample_phase_b == MC_1SHUNT_SAMPLE_PHASE_TRAILING);
    TEST_ASSERT_TRUE(meta.sample_half_a == MC_1SHUNT_HALF_CYCLE_DOWN);
    TEST_ASSERT_TRUE(meta.sample_half_b == MC_1SHUNT_HALF_CYCLE_DOWN);
    TEST_ASSERT_TRUE(meta.sample_position_a > 0.83F);
    TEST_ASSERT_TRUE(meta.sample_position_b > 0.75F);
    TEST_ASSERT_TRUE(meta.sample_half_position_a > 0.66F);
    TEST_ASSERT_TRUE(meta.sample_half_position_b > 0.50F);
    TEST_ASSERT_TRUE(meta.sample_time_a > 0.000083F);
    TEST_ASSERT_TRUE(meta.sample_time_b > 0.000075F);
}
