#include "unity.h"
#include "mc_math.h"

void test_mc_math_clamp_and_wrap_work(void)
{
    mc_f32_t clamped = mc_math_clamp_f32(2.0F, -1.0F, 1.0F);
    mc_f32_t wrapped = mc_math_wrap_angle_rad(3.5F);
    mc_f32_t filtered = mc_math_lpf_f32(0.0F, 10.0F, 0.2F);

    TEST_ASSERT_TRUE(clamped > 0.99F);
    TEST_ASSERT_TRUE(clamped < 1.01F);
    TEST_ASSERT_TRUE(wrapped < 3.15F);
    TEST_ASSERT_TRUE(wrapped > -3.15F);
    TEST_ASSERT_TRUE(filtered > 1.99F);
    TEST_ASSERT_TRUE(filtered < 2.01F);
}

void test_mc_q31_helpers_convert_and_saturate(void)
{
    mc_q31_t half = mc_q31_from_f32(0.5F);
    mc_q31_t near_one = mc_q31_from_f32(2.0F);
    mc_q31_t sum = mc_q31_add_sat(INT32_MAX, 1);
    mc_q31_t product = mc_q31_mul(half, half);

    TEST_ASSERT_TRUE(mc_q31_to_f32(half) > 0.49F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(half) < 0.51F);
    TEST_ASSERT_TRUE(near_one == INT32_MAX);
    TEST_ASSERT_TRUE(sum == INT32_MAX);
    TEST_ASSERT_TRUE(mc_q31_to_f32(product) > 0.24F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(product) < 0.26F);
}
