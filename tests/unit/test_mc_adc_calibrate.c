#include "unity.h"
#include <math.h>
#include "mc_port_adc.h"

void test_mc_adc_calibrate_linear_computes_scale_and_offset(void)
{
    mc_f32_t scale;
    mc_f32_t offset;
    mc_status_t status;

    status = mc_adc_calibrate_linear(1000, 0.0F, 3000, 10.0F, &scale, &offset);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(scale > 0.0049F);
    TEST_ASSERT_TRUE(scale < 0.0051F);
    TEST_ASSERT_TRUE(offset > -5.01F);
    TEST_ASSERT_TRUE(offset < -4.99F);
}

void test_mc_adc_calibrate_linear_rejects_null_scale(void)
{
    mc_f32_t offset;
    mc_status_t status;

    status = mc_adc_calibrate_linear(1000, 0.0F, 3000, 10.0F, NULL, &offset);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
}

void test_mc_adc_calibrate_linear_rejects_null_offset(void)
{
    mc_f32_t scale;
    mc_status_t status;

    status = mc_adc_calibrate_linear(1000, 0.0F, 3000, 10.0F, &scale, NULL);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
}

void test_mc_adc_calibrate_linear_rejects_equal_raw_points(void)
{
    mc_f32_t scale;
    mc_f32_t offset;
    mc_status_t status;

    status = mc_adc_calibrate_linear(2000, 0.0F, 2000, 10.0F, &scale, &offset);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
}

void test_mc_adc_calibrate_linear_handles_negative_physical(void)
{
    mc_f32_t scale;
    mc_f32_t offset;
    mc_status_t status;

    status = mc_adc_calibrate_linear(0, -5.0F, 4095, 5.0F, &scale, &offset);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(scale > 0.0024F);
    TEST_ASSERT_TRUE(scale < 0.0025F);
    TEST_ASSERT_TRUE(offset > -5.01F);
    TEST_ASSERT_TRUE(offset < -4.99F);
}

void test_mc_adc_calibrate_linear_handles_zero_span(void)
{
    mc_f32_t scale;
    mc_f32_t offset;
    mc_status_t status;

    status = mc_adc_calibrate_linear(0, 0.0F, 4095, 0.0F, &scale, &offset);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(fabsf(scale) < 1e-6F);
    TEST_ASSERT_TRUE(fabsf(offset) < 1e-6F);
}
