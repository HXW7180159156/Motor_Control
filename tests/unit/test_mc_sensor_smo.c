#include "unity.h"
#include "mc_constants.h"
#include "mc_math.h"
#include "mc_sensor_smo.h"
#include <math.h>

void test_mc_smo_init_accepts_valid_config(void)
{
    mc_smo_state_t state;
    mc_smo_cfg_t cfg = {
        0.5F, 0.001F, 4.0F,
        2.0F, 0.5F, 0.01F,
        400.0F, 20000.0F, 0.02F,
        80.0F, 4000.0F, 0.0F, 3.6F
    };

    mc_status_t s = mc_smo_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, s);
    TEST_ASSERT_TRUE(state.open_loop_active == MC_TRUE);
    TEST_ASSERT_TRUE(fabsf(state.cfg.rs_ohm - 0.5F) < 0.001F);
}

void test_mc_smo_init_rejects_null_state(void)
{
    mc_smo_cfg_t cfg = {0.5F, 0.001F, 4.0F, 2.0F, 0.5F, 0.01F, 400.0F, 20000.0F, 0.02F, 80.0F, 4000.0F, 0.0F, 3.6F};
    mc_status_t s = mc_smo_init(NULL, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_smo_init_rejects_null_cfg(void)
{
    mc_smo_state_t state;
    mc_status_t s = mc_smo_init(&state, NULL);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_smo_init_rejects_zero_pole_pairs(void)
{
    mc_smo_state_t state;
    mc_smo_cfg_t cfg = {0.5F, 0.001F, 0.0F, 2.0F, 0.5F, 0.01F, 400.0F, 20000.0F, 0.02F, 80.0F, 4000.0F, 0.0F, 3.6F};
    mc_status_t s = mc_smo_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_smo_init_rejects_zero_k_slide(void)
{
    mc_smo_state_t state;
    mc_smo_cfg_t cfg = {0.5F, 0.001F, 4.0F, 0.0F, 0.5F, 0.01F, 400.0F, 20000.0F, 0.02F, 80.0F, 4000.0F, 0.0F, 3.6F};
    mc_status_t s = mc_smo_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_smo_init_rejects_lock_bemf_below_min(void)
{
    mc_smo_state_t state;
    mc_smo_cfg_t cfg = {0.5F, 0.001F, 4.0F, 2.0F, 0.5F, 0.05F, 400.0F, 20000.0F, 0.02F, 80.0F, 4000.0F, 0.0F, 3.6F};
    mc_status_t s = mc_smo_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_smo_init_rejects_open_loop_voltage_inverted(void)
{
    mc_smo_state_t state;
    mc_smo_cfg_t cfg = {0.5F, 0.001F, 4.0F, 2.0F, 0.5F, 0.01F, 400.0F, 20000.0F, 0.02F, 80.0F, 4000.0F, 3.6F, 1.0F};
    mc_status_t s = mc_smo_init(&state, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_smo_update_null_state_rejected(void)
{
    mc_alphabeta_t v = {1.0F, 0.0F};
    mc_alphabeta_t i = {0.5F, 0.0F};
    mc_status_t s = mc_smo_update(NULL, &v, &i, 0.0001F, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_smo_update_null_voltage_rejected(void)
{
    mc_smo_state_t state;
    mc_smo_cfg_t cfg = {0.5F, 0.001F, 4.0F, 2.0F, 0.5F, 0.01F, 400.0F, 20000.0F, 0.02F, 80.0F, 4000.0F, 0.0F, 3.6F};
    mc_alphabeta_t i = {0.5F, 0.0F};
    mc_smo_init(&state, &cfg);
    mc_status_t s = mc_smo_update(&state, NULL, &i, 0.0001F, 1000U);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

void test_mc_smo_update_zero_current_does_not_diverge(void)
{
    mc_smo_state_t state;
    mc_smo_cfg_t cfg = {0.5F, 0.001F, 4.0F, 2.0F, 0.5F, 0.01F, 400.0F, 20000.0F, 0.02F, 80.0F, 4000.0F, 0.0F, 3.6F};
    mc_alphabeta_t v = {1.0F, 0.0F};
    mc_alphabeta_t i = {0.0F, 0.0F};
    mc_smo_init(&state, &cfg);

    int step;
    for (step = 0; step < 20; step++)
    {
        mc_status_t s = mc_smo_update(&state, &v, &i, 0.0001F, 1000U + (uint32_t)step * 50U);
        TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, s);
        TEST_ASSERT_TRUE(isfinite(state.i_hat_ab.alpha));
        TEST_ASSERT_TRUE(isfinite(state.i_hat_ab.beta));
    }
}

void test_mc_smo_update_valid_bemf_sets_observer_valid(void)
{
    mc_smo_state_t state;
    mc_smo_cfg_t cfg = {0.5F, 0.001F, 4.0F, 5.0F, 0.5F, 0.001F, 400.0F, 20000.0F, 0.002F, 80.0F, 4000.0F, 0.0F, 3.6F};
    mc_alphabeta_t v = {2.0F, 0.0F};
    mc_alphabeta_t i = {0.5F, 0.0F};
    mc_smo_init(&state, &cfg);

    int step;
    for (step = 0; step < 50; step++)
    {
        mc_smo_update(&state, &v, &i, 0.0001F, 1000U + (uint32_t)step * 50U);
    }

    TEST_ASSERT_TRUE(state.bemf_magnitude > 0.0F);
}

void test_mc_smo_open_loop_startup_advances_angle(void)
{
    mc_smo_state_t state;
    mc_smo_cfg_t cfg = {0.5F, 0.001F, 4.0F, 2.0F, 0.5F, 0.01F, 400.0F, 20000.0F, 0.02F, 200.0F, 1000.0F, 0.0F, 3.6F};
    mc_alphabeta_t v = {1.0F, 0.0F};
    mc_alphabeta_t i = {0.0F, 0.0F};
    mc_smo_init(&state, &cfg);

    mc_f32_t prev_angle = state.elec_angle_rad;

    int step;
    for (step = 0; step < 10; step++)
    {
        mc_smo_update(&state, &v, &i, 0.0001F, 1000U + (uint32_t)step * 50U);
        TEST_ASSERT_TRUE(state.open_loop_active == MC_TRUE);
    }

    TEST_ASSERT_TRUE(state.elec_angle_rad != prev_angle);
}
