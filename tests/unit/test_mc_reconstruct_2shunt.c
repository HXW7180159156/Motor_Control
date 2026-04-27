#include "unity.h"
#include "mc_reconstruct_2shunt.h"
#include "mc_math.h"

void test_mc_reconstruct_2shunt_derives_third_phase(void)
{
    mc_adc_raw_t raw = {1100U, 900U, 0U, 0U, 0U};
    mc_2shunt_cfg_t cfg = {1000.0F, 1000.0F, 0.01F, 0.01F};
    mc_abc_t abc = {0.0F, 0.0F, 0.0F};

    mc_reconstruct_2shunt_run(&raw, &cfg, &abc);

    TEST_ASSERT_TRUE(abc.a > 0.99F);
    TEST_ASSERT_TRUE(abc.a < 1.01F);
    TEST_ASSERT_TRUE(abc.b > -1.01F);
    TEST_ASSERT_TRUE(abc.b < -0.99F);
    TEST_ASSERT_TRUE(abc.c > -0.01F);
    TEST_ASSERT_TRUE(abc.c < 0.01F);
}

void test_mc_reconstruct_2shunt_q31_derives_third_phase(void)
{
    mc_adc_raw_t raw = {3071U, 1024U, 0U, 0U, 0U};
    mc_2shunt_q31_cfg_t cfg = {
        mc_q31_from_f32(0.0F),
        mc_q31_from_f32(0.0F),
        mc_q31_from_f32(1.0F),
        mc_q31_from_f32(1.0F)
    };
    mc_abc_q31_t abc = {0};

    mc_reconstruct_2shunt_q31_run(&raw, &cfg, &abc);

    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.a) > 0.74F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.a) < 0.76F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.b) > 0.24F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.b) < 0.26F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.c) > -1.01F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.c) < -0.99F);
}
