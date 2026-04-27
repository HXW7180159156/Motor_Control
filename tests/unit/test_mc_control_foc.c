#include "unity.h"
#include "mc_control_foc.h"

void test_mc_control_foc_delegates_to_pmsm_foc(void)
{
    mc_control_foc_t control;
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

    status = mc_control_foc_init(&control, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_control_foc_run(&control, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(out.pwm_cmd.valid == 1U);
}

void test_mc_control_foc_speed_step_generates_iq_ref(void)
{
    mc_control_foc_t control;
    mc_pmsm_foc_cfg_t cfg = {
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {1.0F, 0.0F, -1.0F, 1.0F, -1.0F, 1.0F},
        {0.0F, 1.0F, 0.95F},
        {MC_CURRENT_SENSE_3SHUNT, {{1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F}}},
        1.0F,
        5.0F
    };
    mc_pi_cfg_t speed_pi_cfg = {0.01F, 1.0F, -2.0F, 2.0F, -2.0F, 2.0F};
    mc_f32_t iq_ref = 0.0F;
    mc_status_t status;

    status = mc_control_foc_init(&control, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_control_foc_set_speed_pi(&control, &speed_pi_cfg, 1.5F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_control_foc_speed_step(&control, 1000.0F, 900.0F, 0.001F, &iq_ref);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(iq_ref > 1.0F);
    TEST_ASSERT_TRUE(iq_ref < 1.5F);
}
