#include "unity.h"
#include "mc_constants.h"
#include "mc_control_svpwm.h"
#include "mc_transform.h"
#include <stdint.h>

static uint32_t g_bench_cycles;

/* === Benchmark: Clarke transform throughput === */
void test_bench_clarke_throughput(void)
{
    mc_abc_t abc = {1.0F, -0.5F, -0.5F};
    mc_alphabeta_t ab;
    int iterations = 1000;
    int i;

    g_bench_cycles = 0U;
    for (i = 0; i < iterations; i++)
    {
        mc_clarke_run(&abc, &ab);
        g_bench_cycles++;
    }

    TEST_ASSERT_TRUE(fabsf(ab.alpha - 1.0F) < 1e-5F);
    TEST_ASSERT_TRUE(fabsf(ab.beta) < 1e-5F);
    TEST_ASSERT_TRUE(g_bench_cycles >= (uint32_t)iterations);
}

/* === Benchmark: Park transform throughput === */
void test_bench_park_throughput(void)
{
    mc_alphabeta_t ab = {1.0F, 0.0F};
    mc_dq_t dq;
    int iterations = 1000;
    int i;

    g_bench_cycles = 0U;
    for (i = 0; i < iterations; i++)
    {
        mc_park_run(&ab, 0.0F, 1.0F, &dq);
        g_bench_cycles++;
    }

    TEST_ASSERT_TRUE(fabsf(dq.d - 1.0F) < 1e-5F);
    TEST_ASSERT_TRUE(g_bench_cycles >= (uint32_t)iterations);
}

/* === Benchmark: SVPWM throughput === */
void test_bench_svpwm_throughput(void)
{
    mc_alphabeta_t v_ab = {0.5F, 0.0F};
    mc_svpwm_cfg_t cfg = {0.95F, 0.0F, 1.0F};
    mc_pwm_cmd_t pwm = {0};
    int iterations = 500;
    int i;

    g_bench_cycles = 0U;
    for (i = 0; i < iterations; i++)
    {
        mc_svpwm_run(&v_ab, &cfg, &pwm);
        g_bench_cycles++;
    }

    TEST_ASSERT_EQUAL_INT(1U, pwm.valid);
    TEST_ASSERT_TRUE(g_bench_cycles >= (uint32_t)iterations);
}
