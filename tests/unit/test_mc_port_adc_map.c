#include "unity.h"
#include "mc_port_adc_map.h"

void test_mc_adc_map_trigger_plan_maps_valid_triggers(void)
{
    mc_adc_trigger_plan_t plan = {0};
    mc_adc_hw_trigger_map_t hw_map = {0};

    plan.count = 2U;
    plan.trigger_a.valid = MC_TRUE;
    plan.trigger_a.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
    plan.trigger_a.event = MC_ADC_TRIGGER_EVENT_PWM_UP;
    plan.trigger_a.soc_index = 0U;
    plan.trigger_a.time_s = 0.000012F;
    plan.trigger_a.position = 0.12F;
    plan.trigger_b.valid = MC_TRUE;
    plan.trigger_b.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
    plan.trigger_b.event = MC_ADC_TRIGGER_EVENT_PWM_DOWN;
    plan.trigger_b.soc_index = 1U;
    plan.trigger_b.time_s = 0.000082F;
    plan.trigger_b.position = 0.82F;

    mc_adc_map_trigger_plan(&plan, &hw_map);

    TEST_ASSERT_TRUE(hw_map.count == 2U);
    TEST_ASSERT_TRUE(hw_map.trigger_a.compare_index == 0U);
    TEST_ASSERT_TRUE(hw_map.trigger_a.soc_index == 0U);
    TEST_ASSERT_TRUE(hw_map.trigger_a.event == MC_ADC_TRIGGER_EVENT_PWM_UP);
    TEST_ASSERT_TRUE(hw_map.trigger_a.signal == MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT);
    TEST_ASSERT_TRUE(hw_map.trigger_a.compare_time_s > 0.000011F);
    TEST_ASSERT_TRUE(hw_map.trigger_a.compare_position > 0.11F);
    TEST_ASSERT_TRUE(hw_map.trigger_b.compare_index == 1U);
    TEST_ASSERT_TRUE(hw_map.trigger_b.soc_index == 1U);
    TEST_ASSERT_TRUE(hw_map.trigger_b.event == MC_ADC_TRIGGER_EVENT_PWM_DOWN);
    TEST_ASSERT_TRUE(hw_map.trigger_b.signal == MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT);
    TEST_ASSERT_TRUE(hw_map.trigger_b.compare_time_s > 0.000081F);
    TEST_ASSERT_TRUE(hw_map.trigger_b.compare_position > 0.81F);
}

void test_mc_adc_map_trigger_plan_keeps_invalid_trigger_empty(void)
{
    mc_adc_trigger_plan_t plan = {0};
    mc_adc_hw_trigger_map_t hw_map = {0};

    plan.count = 1U;
    plan.trigger_a.valid = MC_TRUE;
    plan.trigger_a.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
    plan.trigger_a.event = MC_ADC_TRIGGER_EVENT_PWM_UP;
    plan.trigger_a.soc_index = 0U;
    plan.trigger_a.time_s = 0.000034F;
    plan.trigger_a.position = 0.34F;

    mc_adc_map_trigger_plan(&plan, &hw_map);

    TEST_ASSERT_TRUE(hw_map.count == 1U);
    TEST_ASSERT_TRUE(hw_map.trigger_a.signal == MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT);
    TEST_ASSERT_TRUE(hw_map.trigger_b.signal == MC_ADC_TRIGGER_NONE);
    TEST_ASSERT_TRUE(hw_map.trigger_b.event == MC_ADC_TRIGGER_EVENT_NONE);
    TEST_ASSERT_TRUE(hw_map.trigger_b.compare_time_s > -0.000001F);
    TEST_ASSERT_TRUE(hw_map.trigger_b.compare_time_s < 0.000001F);
}
