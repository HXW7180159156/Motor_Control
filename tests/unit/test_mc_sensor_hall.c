#include "unity.h"
#include "mc_sensor_hall.h"

void test_mc_hall_update_maps_angle_and_speed(void)
{
    mc_hall_cfg_t cfg = {
        {1U, 5U, 4U, 6U, 2U, 3U},
        {0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F}
    };
    mc_hall_state_t state;
    mc_status_t status;

    status = mc_hall_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_hall_update(&state, 1U, 1000U, 2.0F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.elec_angle_rad > -0.01F);
    TEST_ASSERT_TRUE(state.elec_angle_rad < 0.01F);

    status = mc_hall_update(&state, 5U, 2000U, 2.0F);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.elec_angle_rad > 0.99F);
    TEST_ASSERT_TRUE(state.elec_angle_rad < 1.01F);
    TEST_ASSERT_TRUE(state.mech_speed_rpm > 4000.0F);
    TEST_ASSERT_TRUE(state.mech_speed_rpm < 6000.0F);
}
