#include "unity.h"
#include "mc_reconstruct_3shunt.h"
#include "mc_math.h"

void test_mc_reconstruct_3shunt_applies_offset_and_scale(void)
{
    mc_adc_raw_t raw = {1100U, 900U, 1000U, 0U, 0U};
    mc_3shunt_cfg_t cfg = {1000.0F, 1000.0F, 1000.0F, 0.01F, 0.01F, 0.01F};
    mc_abc_t abc = {0.0F, 0.0F, 0.0F};

    mc_reconstruct_3shunt_run(&raw, &cfg, &abc);

    TEST_ASSERT_TRUE(abc.a > 0.99F);
    TEST_ASSERT_TRUE(abc.a < 1.01F);
    TEST_ASSERT_TRUE(abc.b > -1.01F);
    TEST_ASSERT_TRUE(abc.b < -0.99F);
    TEST_ASSERT_TRUE(abc.c > -0.01F);
    TEST_ASSERT_TRUE(abc.c < 0.01F);
}

void test_mc_reconstruct_3shunt_q31_applies_offset_and_scale(void)
{
    mc_adc_raw_t raw = {2048U, 1024U, 3071U, 0U, 0U};
    mc_3shunt_q31_cfg_t cfg = {
        mc_q31_from_f32(0.0F),
        mc_q31_from_f32(0.0F),
        mc_q31_from_f32(0.0F),
        mc_q31_from_f32(1.0F),
        mc_q31_from_f32(1.0F),
        mc_q31_from_f32(1.0F)
    };
    mc_abc_q31_t abc = {0};

    mc_reconstruct_3shunt_q31_run(&raw, &cfg, &abc);

    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.a) > 0.49F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.a) < 0.51F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.b) > 0.24F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.b) < 0.26F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.c) > 0.74F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(abc.c) < 0.76F);
}
