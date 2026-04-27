#include "unity.h"
#include "mc_transform.h"
#include "mc_math.h"

void test_mc_clarke_computes_alpha_beta(void)
{
    mc_abc_t abc = {1.0F, -0.5F, -0.5F};
    mc_alphabeta_t ab = {0.0F, 0.0F};

    mc_clarke_run(&abc, &ab);

    TEST_ASSERT_TRUE(ab.alpha > 0.99F);
    TEST_ASSERT_TRUE(ab.alpha < 1.01F);
    TEST_ASSERT_TRUE(ab.beta > -0.01F);
    TEST_ASSERT_TRUE(ab.beta < 0.01F);
}

void test_mc_park_and_ipark_round_trip(void)
{
    mc_alphabeta_t ab = {0.3F, 0.4F};
    mc_dq_t dq = {0.0F, 0.0F};
    mc_alphabeta_t back = {0.0F, 0.0F};

    mc_park_run(&ab, 0.0F, 1.0F, &dq);
    mc_ipark_run(&dq, 0.0F, 1.0F, &back);

    TEST_ASSERT_TRUE(back.alpha > 0.29F);
    TEST_ASSERT_TRUE(back.alpha < 0.31F);
    TEST_ASSERT_TRUE(back.beta > 0.39F);
    TEST_ASSERT_TRUE(back.beta < 0.41F);
}

void test_mc_q31_transform_round_trip(void)
{
    mc_alphabeta_q31_t ab = {mc_q31_from_f32(0.25F), mc_q31_from_f32(-0.125F)};
    mc_dq_q31_t dq = {0};
    mc_alphabeta_q31_t back = {0};

    mc_park_q31_run(&ab, mc_q31_from_f32(0.0F), mc_q31_from_f32(1.0F), &dq);
    mc_ipark_q31_run(&dq, mc_q31_from_f32(0.0F), mc_q31_from_f32(1.0F), &back);

    TEST_ASSERT_TRUE(mc_q31_to_f32(back.alpha) > 0.24F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(back.alpha) < 0.26F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(back.beta) < -0.11F);
    TEST_ASSERT_TRUE(mc_q31_to_f32(back.beta) > -0.14F);
}
