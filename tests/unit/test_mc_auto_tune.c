#include "unity.h"
#include "mc_constants.h"
#include "mc_control_pi.h"
#include <math.h>

void test_mc_auto_tune_current_pi_produces_positive_gains(void)
{
    mc_pi_cfg_t id_cfg, iq_cfg;
    mc_status_t s = mc_auto_tune_current_pi(
        0.5F, 0.001F, 0.0012F, 20000U, 20.0F, 24.0F, 10.0F,
        &id_cfg, &iq_cfg);

    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, s);
    TEST_ASSERT_TRUE(id_cfg.kp > 0.0F);
    TEST_ASSERT_TRUE(id_cfg.ki > 0.0F);
    TEST_ASSERT_TRUE(iq_cfg.kp > 0.0F);
    TEST_ASSERT_TRUE(iq_cfg.ki > 0.0F);
    TEST_ASSERT_TRUE(id_cfg.output_max > id_cfg.output_min);
}

void test_mc_auto_tune_current_pi_rejects_null_output(void)
{
    mc_pi_cfg_t iq_cfg;
    mc_status_t s = mc_auto_tune_current_pi(
        0.5F, 0.001F, 0.001F, 20000U, 20.0F, 24.0F, 10.0F,
        NULL, &iq_cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_auto_tune_current_pi_rejects_zero_rs(void)
{
    mc_pi_cfg_t id_cfg, iq_cfg;
    mc_status_t s = mc_auto_tune_current_pi(
        0.0F, 0.001F, 0.001F, 20000U, 20.0F, 24.0F, 10.0F,
        &id_cfg, &iq_cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_auto_tune_current_pi_rejects_zero_pwm_freq(void)
{
    mc_pi_cfg_t id_cfg, iq_cfg;
    mc_status_t s = mc_auto_tune_current_pi(
        0.5F, 0.001F, 0.001F, 0U, 20.0F, 24.0F, 10.0F,
        &id_cfg, &iq_cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_auto_tune_current_pi_salient_motor_different_kp(void)
{
    mc_pi_cfg_t id_cfg, iq_cfg;
    mc_status_t s = mc_auto_tune_current_pi(
        0.3F, 0.0008F, 0.0015F, 16000U, 20.0F, 48.0F, 20.0F,
        &id_cfg, &iq_cfg);

    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, s);
    TEST_ASSERT_TRUE(iq_cfg.kp > id_cfg.kp);  /* Lq > Ld → Kp_q > Kp_d */
}

void test_mc_auto_tune_current_pi_aggressive_divider_produces_higher_gains(void)
{
    mc_pi_cfg_t id_conservative, iq_conservative;
    mc_pi_cfg_t id_aggressive, iq_aggressive;

    mc_auto_tune_current_pi(0.5F, 0.001F, 0.001F, 20000U, 40.0F, 24.0F, 10.0F,
                            &id_conservative, &iq_conservative);
    mc_auto_tune_current_pi(0.5F, 0.001F, 0.001F, 20000U, 10.0F, 24.0F, 10.0F,
                            &id_aggressive, &iq_aggressive);

    TEST_ASSERT_TRUE(id_aggressive.kp > id_conservative.kp);
    TEST_ASSERT_TRUE(id_aggressive.ki > id_conservative.ki);
}

void test_mc_auto_tune_speed_pi_produces_positive_gains(void)
{
    mc_pi_cfg_t speed_cfg;
    mc_status_t s = mc_auto_tune_speed_pi(
        0.001F, 20000U, 20.0F, 10.0F, 5.0F, &speed_cfg);

    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, s);
    TEST_ASSERT_TRUE(speed_cfg.kp > 0.0F);
    TEST_ASSERT_TRUE(speed_cfg.ki > 0.0F);
    TEST_ASSERT_TRUE(speed_cfg.output_max > 0.0F);
}

void test_mc_auto_tune_speed_pi_rejects_null(void)
{
    mc_status_t s = mc_auto_tune_speed_pi(0.001F, 20000U, 20.0F, 10.0F, 5.0F, NULL);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_auto_tune_speed_pi_rejects_zero_ld(void)
{
    mc_pi_cfg_t speed_cfg;
    mc_status_t s = mc_auto_tune_speed_pi(0.0F, 20000U, 20.0F, 10.0F, 5.0F, &speed_cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}
