#include "unity.h"

void test_bench_fast_step_pmsm_foc_2shunt(void);
void test_bench_clarke_throughput(void);
void test_bench_park_throughput(void);
void test_bench_svpwm_throughput(void);
void test_bench_pi_single_step_throughput(void);
void test_bench_pi_q31_throughput(void);
void test_bench_pi_f32_vs_q31_relative(void);

void setUp(void) {}
void tearDown(void) {}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_bench_fast_step_pmsm_foc_2shunt);
    RUN_TEST(test_bench_clarke_throughput);
    RUN_TEST(test_bench_park_throughput);
    RUN_TEST(test_bench_svpwm_throughput);
    RUN_TEST(test_bench_pi_single_step_throughput);
    RUN_TEST(test_bench_pi_q31_throughput);
    RUN_TEST(test_bench_pi_f32_vs_q31_relative);
    return UNITY_END();
}
