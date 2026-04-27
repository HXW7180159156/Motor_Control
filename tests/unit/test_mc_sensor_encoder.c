#include "unity.h"
#include "mc_sensor_encoder.h"

void test_mc_encoder_update_computes_angle_and_speed(void)
{
    mc_encoder_cfg_t cfg = {1024U, 2.0F};
    mc_encoder_state_t state;
    mc_status_t status;

    status = mc_encoder_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_encoder_update(&state, 256U, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.elec_angle_rad > 3.13F);
    TEST_ASSERT_TRUE(state.elec_angle_rad < 3.15F);

    status = mc_encoder_update(&state, 512U, 2000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.mech_speed_rpm > 14000.0F);
    TEST_ASSERT_TRUE(state.mech_speed_rpm < 16000.0F);
}
