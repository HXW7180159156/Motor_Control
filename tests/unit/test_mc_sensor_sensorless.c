#include "unity.h"
#include "mc_sensor_sensorless.h"

void test_mc_sensorless_update_estimates_angle_and_speed(void)
{
    mc_sensorless_cfg_t cfg = {0.5F, 0.002F, 2.0F, 1.0F, 0.01F, 400.0F, 20000.0F, 0.02F, 20.0F, 20000.0F};
    mc_sensorless_state_t state;
    mc_alphabeta_t voltage = {0.0F, 1.0F};
    mc_alphabeta_t current = {0.0F, 0.0F};
    mc_status_t status;

    status = mc_sensorless_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.observer_valid == MC_TRUE);
    TEST_ASSERT_TRUE(state.open_loop_active == MC_TRUE);
    TEST_ASSERT_TRUE(state.pll_locked == MC_FALSE);

    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 2000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.open_loop_active == MC_FALSE);
    TEST_ASSERT_TRUE(state.pll_locked == MC_TRUE);
    TEST_ASSERT_TRUE(state.mech_speed_rpm > 0.0F);
}

void test_mc_sensorless_update_flags_low_bemf_invalid(void)
{
    mc_sensorless_cfg_t cfg = {0.5F, 0.002F, 2.0F, 1.0F, 0.5F, 400.0F, 20000.0F, 1.0F, 20.0F, 20000.0F};
    mc_sensorless_state_t state;
    mc_alphabeta_t voltage = {0.1F, 0.1F};
    mc_alphabeta_t current = {0.0F, 0.0F};
    mc_status_t status;

    status = mc_sensorless_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.observer_valid == MC_FALSE);
    TEST_ASSERT_TRUE(state.mech_speed_rpm == 0.0F);
    TEST_ASSERT_TRUE(state.open_loop_active == MC_TRUE);
    TEST_ASSERT_TRUE(state.elec_angle_rad > 0.0F);
}

void test_mc_sensorless_init_rejects_invalid_pll_gains(void)
{
    mc_sensorless_cfg_t cfg = {0.5F, 0.002F, 2.0F, 1.0F, 0.01F, -1.0F, 0.0F, 0.02F, 20.0F, 20000.0F};
    mc_sensorless_state_t state;

    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_sensorless_init(&state, &cfg));
}

void test_mc_sensorless_init_rejects_lock_bemf_not_above_min_bemf(void)
{
    mc_sensorless_cfg_t cfg = {0.5F, 0.002F, 2.0F, 1.0F, 0.02F, 400.0F, 20000.0F, 0.02F, 20.0F, 20000.0F};
    mc_sensorless_state_t state;

    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_sensorless_init(&state, &cfg));
}

void test_mc_sensorless_init_rejects_open_loop_voltage_max_below_start(void)
{
    mc_sensorless_cfg_t cfg = {0.5F, 0.002F, 2.0F, 1.0F, 0.01F, 400.0F, 20000.0F, 0.02F, 20.0F, 20000.0F, 0.2F, 0.1F};
    mc_sensorless_state_t state;

    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_sensorless_init(&state, &cfg));
}

void test_mc_sensorless_uses_inductance_compensation(void)
{
    mc_sensorless_cfg_t cfg = {0.5F, 0.01F, 2.0F, 1.0F, 0.01F, 200.0F, 10000.0F, 0.02F, 10.0F, 30000.0F};
    mc_sensorless_state_t state;
    mc_alphabeta_t voltage = {1.0F, 0.0F};
    mc_alphabeta_t current = {0.0F, 0.0F};
    mc_status_t status;

    status = mc_sensorless_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.bemf_ab.alpha > 0.0F);

    current.alpha = 0.2F;
    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 2000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.bemf_ab.alpha < 0.0F);
}

void test_mc_sensorless_requires_consecutive_lock_samples(void)
{
    mc_sensorless_cfg_t cfg = {0.5F, 0.002F, 2.0F, 1.0F, 0.01F, 400.0F, 20000.0F, 0.02F, 20.0F, 20000.0F};
    mc_sensorless_state_t state;
    mc_alphabeta_t voltage = {0.0F, 1.0F};
    mc_alphabeta_t current = {0.0F, 0.0F};
    mc_status_t status;

    status = mc_sensorless_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.open_loop_active == MC_TRUE);
    TEST_ASSERT_TRUE(state.open_loop_lock_counter == 1U);

    state.last_current_ab = (mc_alphabeta_t){10.0F, 10.0F};
    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 2000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.open_loop_active == MC_TRUE);
    TEST_ASSERT_TRUE(state.open_loop_lock_counter == 0U);
}

void test_mc_sensorless_requires_consecutive_unlock_samples(void)
{
    mc_sensorless_cfg_t cfg = {0.5F, 0.002F, 2.0F, 1.0F, 0.01F, 400.0F, 20000.0F, 0.02F, 20.0F, 20000.0F};
    mc_sensorless_state_t state;
    mc_alphabeta_t voltage = {1.0F, 0.0F};
    mc_alphabeta_t current = {0.0F, 0.0F};
    mc_status_t status;

    status = mc_sensorless_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    state.open_loop_active = MC_FALSE;
    state.pll_locked = MC_TRUE;
    state.pll_lock_counter = 2U;
    state.elec_angle_rad = 0.0F;

    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.pll_locked == MC_TRUE);
    TEST_ASSERT_TRUE(state.pll_unlock_counter == 1U);

    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 2000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.pll_locked == MC_FALSE);
}

void test_mc_sensorless_open_loop_handoff_requires_lock_bemf_threshold(void)
{
    mc_sensorless_cfg_t cfg = {0.5F, 0.002F, 2.0F, 1.0F, 0.01F, 400.0F, 20000.0F, 2.0F, 20.0F, 20000.0F};
    mc_sensorless_state_t state;
    mc_alphabeta_t voltage = {0.0F, 1.0F};
    mc_alphabeta_t current = {0.0F, 0.0F};
    mc_status_t status;

    status = mc_sensorless_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 2000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    TEST_ASSERT_TRUE(state.observer_valid == MC_TRUE);
    TEST_ASSERT_TRUE(state.bemf_magnitude < cfg.lock_bemf);
    TEST_ASSERT_TRUE(state.open_loop_active == MC_TRUE);
    TEST_ASSERT_TRUE(state.open_loop_lock_counter == 0U);
}

void test_mc_sensorless_low_bemf_resets_lock_and_unlock_counters(void)
{
    mc_sensorless_cfg_t cfg = {0.5F, 0.002F, 2.0F, 1.0F, 0.5F, 400.0F, 20000.0F, 1.0F, 20.0F, 20000.0F};
    mc_sensorless_state_t state;
    mc_alphabeta_t voltage = {0.1F, 0.1F};
    mc_alphabeta_t current = {0.0F, 0.0F};
    mc_status_t status;

    status = mc_sensorless_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);

    state.open_loop_lock_counter = 3U;
    state.pll_lock_counter = 4U;
    state.pll_unlock_counter = 5U;
    state.open_loop_active = MC_FALSE;
    state.pll_locked = MC_TRUE;
    state.pll_speed_rad_s = 10.0F;
    state.pll_integrator = 2.0F;

    status = mc_sensorless_update(&state, &voltage, &current, 0.001F, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(state.observer_valid == MC_FALSE);
    TEST_ASSERT_TRUE(state.pll_locked == MC_FALSE);
    TEST_ASSERT_TRUE(state.open_loop_lock_counter == 0U);
    TEST_ASSERT_TRUE(state.pll_lock_counter == 0U);
    TEST_ASSERT_TRUE(state.pll_unlock_counter == 0U);
    TEST_ASSERT_TRUE(state.pll_speed_rad_s == 0.0F);
    TEST_ASSERT_TRUE(state.pll_integrator == 0.0F);
}
