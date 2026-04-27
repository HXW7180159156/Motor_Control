#include "unity.h"
#include "mc_sensor_resolver.h"

void test_mc_resolver_reports_excitation_command(void)
{
    mc_resolver_cfg_t cfg = {1200, 10000U, 0.001F, 0.1F, 2.0F};
    mc_resolver_state_t state;
    int16_t amplitude = 0;
    uint16_t frequency_hz = 0U;
    mc_status_t status;

    status = mc_resolver_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_resolver_get_excitation_command(&state, &amplitude, &frequency_hz);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(amplitude == 1200);
    TEST_ASSERT_TRUE(frequency_hz == 10000U);
}

void test_mc_resolver_update_decodes_angle_and_speed(void)
{
    mc_resolver_cfg_t cfg = {1200, 10000U, 0.001F, 0.1F, 2.0F};
    mc_resolver_state_t state;
    mc_resolver_raw_t raw_a = {1000, 0, 0U};
    mc_resolver_raw_t raw_b = {0, 1000, 0U};
    mc_status_t status;

    status = mc_resolver_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_resolver_update(&state, &raw_a, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.signal_valid == MC_TRUE);
    TEST_ASSERT_TRUE(state.elec_angle_rad > 3.13F);
    TEST_ASSERT_TRUE(state.elec_angle_rad < 3.15F);

    status = mc_resolver_update(&state, &raw_b, 2000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.elec_angle_rad > -0.01F);
    TEST_ASSERT_TRUE(state.elec_angle_rad < 0.01F);
    TEST_ASSERT_TRUE(state.mech_speed_rpm < -14000.0F);
    TEST_ASSERT_TRUE(state.mech_speed_rpm > -16000.0F);
}

void test_mc_resolver_update_flags_low_signal_invalid(void)
{
    mc_resolver_cfg_t cfg = {1200, 10000U, 0.001F, 0.5F, 2.0F};
    mc_resolver_state_t state;
    mc_resolver_raw_t raw = {100, 100, 0U};
    mc_status_t status;

    status = mc_resolver_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_resolver_update(&state, &raw, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.signal_valid == MC_FALSE);
    TEST_ASSERT_TRUE(state.mech_speed_rpm == 0.0F);
}
