#include "unity.h"
#include "mc_drive_bldc.h"

void test_mc_bldc_hall_commutates_phase_modes(void)
{
    mc_bldc_hall_t drive;
    mc_bldc_hall_cfg_t cfg = {0.0F, 1.0F};
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_status_t status;

    status = mc_bldc_hall_init(&drive, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_bldc_hall_run(&drive, 5U, 0.6F, &pwm_cmd);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(pwm_cmd.valid == 1U);
    TEST_ASSERT_TRUE(pwm_cmd.sector == 2U);
    TEST_ASSERT_TRUE(pwm_cmd.phase_mode_a == MC_PWM_PHASE_PWM);
    TEST_ASSERT_TRUE(pwm_cmd.phase_mode_b == MC_PWM_PHASE_OFF);
    TEST_ASSERT_TRUE(pwm_cmd.phase_mode_c == MC_PWM_PHASE_LOW);
    TEST_ASSERT_TRUE(pwm_cmd.duty_a > 0.59F);
    TEST_ASSERT_TRUE(pwm_cmd.duty_a < 0.61F);
}
