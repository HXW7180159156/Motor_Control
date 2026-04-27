#include "unity.h"
#include "mc_control_svpwm.h"
#include "mc_math.h"

void test_mc_svpwm_generates_valid_centered_duties(void)
{
    mc_alphabeta_t v_ab = {0.1F, 0.0F};
    mc_svpwm_cfg_t cfg = {0.0F, 1.0F, 0.95F};
    mc_pwm_cmd_t pwm = {0};

    mc_svpwm_run(&v_ab, &cfg, &pwm);

    TEST_ASSERT_TRUE(pwm.valid == 1U);
    TEST_ASSERT_TRUE(pwm.duty_a >= 0.0F);
    TEST_ASSERT_TRUE(pwm.duty_a <= 1.0F);
    TEST_ASSERT_TRUE(pwm.duty_b >= 0.0F);
    TEST_ASSERT_TRUE(pwm.duty_b <= 1.0F);
    TEST_ASSERT_TRUE(pwm.duty_c >= 0.0F);
    TEST_ASSERT_TRUE(pwm.duty_c <= 1.0F);
}

void test_mc_svpwm_limits_modulation_vector(void)
{
    mc_alphabeta_t v_ab = {1.0F, 0.0F};
    mc_svpwm_cfg_t cfg = {0.0F, 1.0F, 0.5F};
    mc_pwm_cmd_t pwm = {0};

    mc_svpwm_run(&v_ab, &cfg, &pwm);

    TEST_ASSERT_TRUE(pwm.valid == 1U);
    TEST_ASSERT_TRUE(pwm.duty_a > 0.85F);
    TEST_ASSERT_TRUE(pwm.duty_a < 0.90F);
    TEST_ASSERT_TRUE(pwm.duty_b > 0.10F);
    TEST_ASSERT_TRUE(pwm.duty_b < 0.20F);
    TEST_ASSERT_TRUE(pwm.duty_c > 0.10F);
    TEST_ASSERT_TRUE(pwm.duty_c < 0.20F);
}

void test_mc_svpwm_q31_generates_valid_centered_duties(void)
{
    mc_alphabeta_q31_t v_ab = {mc_q31_from_f32(0.1F), mc_q31_from_f32(0.0F)};
    mc_svpwm_cfg_t cfg = {0.0F, 1.0F, 0.95F};
    mc_pwm_cmd_t pwm = {0};

    mc_svpwm_q31_run(&v_ab, &cfg, &pwm);

    TEST_ASSERT_TRUE(pwm.valid == 1U);
    TEST_ASSERT_TRUE(pwm.duty_a >= 0.0F);
    TEST_ASSERT_TRUE(pwm.duty_a <= 1.0F);
}
