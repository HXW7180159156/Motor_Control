#include "unity.h"
#include "mc_cfg.h"

void test_mc_cfg_has_bldc_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_BLDC == 1U);
}

void test_mc_cfg_has_pmsm_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_PMSM == 1U);
}

void test_mc_cfg_has_float32_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_FLOAT32 == 1U);
}

void test_mc_cfg_has_q31_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_Q31 == 1U);
}

void test_mc_cfg_has_1shunt_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_1SHUNT == 1U);
}

void test_mc_cfg_has_2shunt_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_2SHUNT == 1U);
}

void test_mc_cfg_has_3shunt_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_3SHUNT == 1U);
}

void test_mc_cfg_has_hall_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_HALL == 1U);
}

void test_mc_cfg_has_encoder_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_ENCODER == 1U);
}

void test_mc_cfg_has_resolver_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_RESOLVER == 1U);
}

void test_mc_cfg_has_sensorless_enabled(void)
{
    TEST_ASSERT_TRUE(MC_CFG_ENABLE_SENSORLESS == 1U);
}
