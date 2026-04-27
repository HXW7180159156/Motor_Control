#include "unity.h"
#include <math.h>
#include "mc_control_pi.h"
#include "mc_math.h"

void test_mc_pi_generates_bounded_output(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {1.0F, 10.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    mc_f32_t output;

    mc_pi_init(&pi, &cfg);
    output = mc_pi_run(&pi, 0.2F, 0.01F);

    TEST_ASSERT_TRUE(output > 0.21F);
    TEST_ASSERT_TRUE(output < 0.23F);
}

void test_mc_pi_clamps_integral_and_output(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {0.0F, 100.0F, -0.2F, 0.2F, -0.1F, 0.1F};
    mc_f32_t output;

    mc_pi_init(&pi, &cfg);
    output = mc_pi_run(&pi, 1.0F, 0.1F);

    TEST_ASSERT_TRUE(pi.integral > 0.19F);
    TEST_ASSERT_TRUE(pi.integral < 0.21F);
    TEST_ASSERT_TRUE(output > 0.09F);
    TEST_ASSERT_TRUE(output < 0.11F);
}

void test_mc_pi_zero_error_produces_zero_output(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {1.0F, 10.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    mc_f32_t output;

    mc_pi_init(&pi, &cfg);
    output = mc_pi_run(&pi, 0.0F, 0.01F);

    TEST_ASSERT_TRUE(fabsf(output) < 1e-6F);
}

void test_mc_pi_negative_error_produces_negative_output(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {1.0F, 10.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    mc_f32_t output;

    mc_pi_init(&pi, &cfg);
    output = mc_pi_run(&pi, -0.2F, 0.01F);

    TEST_ASSERT_TRUE(output < 0.0F);
}

void test_mc_pi_large_error_clamps_output(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {5.0F, 0.0F, -1.0F, 1.0F, -2.0F, 2.0F};
    mc_f32_t output;

    mc_pi_init(&pi, &cfg);
    output = mc_pi_run(&pi, 1.0F, 0.01F);

    TEST_ASSERT_TRUE(output <= 2.0F);
    TEST_ASSERT_TRUE(output >= 1.0F);
}

void test_mc_pi_integral_windup_limited_by_clamp(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {0.0F, 100.0F, -0.3F, 0.3F, -0.5F, 0.5F};
    mc_f32_t output;

    mc_pi_init(&pi, &cfg);
    mc_pi_run(&pi, 1.0F, 0.1F);
    output = mc_pi_run(&pi, 1.0F, 0.1F);

    TEST_ASSERT_TRUE(pi.integral > 0.15F);
    TEST_ASSERT_TRUE(pi.integral < 0.31F);
    TEST_ASSERT_TRUE(output > 0.0F);
}

void test_mc_pi_kp_only_behaves_as_proportional_gain(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {2.0F, 0.0F, -1.0F, 1.0F, -5.0F, 5.0F};
    mc_f32_t output;

    mc_pi_init(&pi, &cfg);
    output = mc_pi_run(&pi, 0.5F, 0.01F);

    TEST_ASSERT_TRUE(fabsf(pi.integral) < 1e-6F);
    TEST_ASSERT_TRUE(fabsf(output - 1.0F) < 1e-6F);
}

void test_mc_pi_ki_only_integrates_error(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {0.0F, 5.0F, -2.0F, 2.0F, -10.0F, 10.0F};
    mc_f32_t output;

    mc_pi_init(&pi, &cfg);
    output = mc_pi_run(&pi, 1.0F, 0.1F);

    TEST_ASSERT_TRUE(pi.integral > 0.0F);
    TEST_ASSERT_TRUE(fabsf(output - pi.integral) < 1e-6F);
}

void test_mc_pi_q31_generates_bounded_output(void)
{
    mc_pi_q31_t pi;
    mc_pi_q31_cfg_t cfg = {
        mc_q31_from_f32(0.5F),
        mc_q31_from_f32(0.25F),
        mc_q31_from_f32(-0.5F),
        mc_q31_from_f32(0.5F),
        mc_q31_from_f32(-0.75F),
        mc_q31_from_f32(0.75F)
    };
    mc_q31_t output;

    mc_pi_q31_init(&pi, &cfg);
    output = mc_pi_q31_run(&pi, mc_q31_from_f32(0.5F), mc_q31_from_f32(0.5F));

    TEST_ASSERT_TRUE(mc_q31_to_f32(output) > 0.30F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(output) < 0.33F);
}
