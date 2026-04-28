#include "unity.h"
#include "mc_control_pi.h"
#include <stdint.h>

static uint32_t g_bench_cycles;

/* === Benchmark: PI controller single-step throughput === */
void test_bench_pi_single_step_throughput(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {1.0F, 10.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    int iterations = 2000;
    int i;

    mc_pi_init(&pi, &cfg);

    g_bench_cycles = 0U;
    for (i = 0; i < iterations; i++)
    {
        mc_pi_run(&pi, 0.2F, 0.01F);
        g_bench_cycles++;
    }

    TEST_ASSERT_TRUE(g_bench_cycles >= (uint32_t)iterations);
    TEST_ASSERT_TRUE(pi.output >= -1.0F && pi.output <= 1.0F);
}

/* === Benchmark: Q31 PI controller throughput === */
void test_bench_pi_q31_throughput(void)
{
    mc_pi_q31_t pi;
    mc_pi_q31_cfg_t cfg = {
        mc_q31_from_f32(0.5F),
        mc_q31_from_f32(5.0F),
        mc_q31_from_f32(-0.5F),
        mc_q31_from_f32(0.5F),
        mc_q31_from_f32(-1.0F),
        mc_q31_from_f32(1.0F)
    };
    mc_q31_t error = mc_q31_from_f32(0.1F);
    mc_q31_t dt = mc_q31_from_f32(0.01F);
    int iterations = 1000;
    int i;

    mc_pi_q31_init(&pi, &cfg);

    g_bench_cycles = 0U;
    for (i = 0; i < iterations; i++)
    {
        mc_pi_q31_run(&pi, error, dt);
        g_bench_cycles++;
    }

    TEST_ASSERT_TRUE(g_bench_cycles >= (uint32_t)iterations);
}

/* === Benchmark: float32 vs Q31 PI relative throughput === */
void test_bench_pi_f32_vs_q31_relative(void)
{
    mc_pi_t pi_f32;
    mc_pi_q31_t pi_q31;
    mc_pi_cfg_t cfg_f32 = {0.5F, 5.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    mc_pi_q31_cfg_t cfg_q31 = {
        mc_q31_from_f32(0.5F),
        mc_q31_from_f32(5.0F),
        mc_q31_from_f32(-0.5F),
        mc_q31_from_f32(0.5F),
        mc_q31_from_f32(-1.0F),
        mc_q31_from_f32(1.0F)
    };

    mc_pi_init(&pi_f32, &cfg_f32);
    mc_pi_q31_init(&pi_q31, &cfg_q31);

    uint32_t f32_cycles, q31_cycles;
    int iterations = 1000;
    int i;

    g_bench_cycles = 0U;
    for (i = 0; i < iterations; i++) { mc_pi_run(&pi_f32, 0.2F, 0.01F); g_bench_cycles++; }
    f32_cycles = g_bench_cycles;

    g_bench_cycles = 0U;
    mc_q31_t err = mc_q31_from_f32(0.2F);
    mc_q31_t dt = mc_q31_from_f32(0.01F);
    for (i = 0; i < iterations; i++) { mc_pi_q31_run(&pi_q31, err, dt); g_bench_cycles++; }
    q31_cycles = g_bench_cycles;

    TEST_ASSERT_TRUE(f32_cycles > 0U);
    TEST_ASSERT_TRUE(q31_cycles > 0U);

    /* On Cortex-M4F with FPU+DSP, f32 PI is typically ~15-20% slower than Q31.
     * On Cortex-M0+ (no FPU), Q31 is dramatically faster. Actual ratio depends
     * on hardware and compiler optimization level. */
}
