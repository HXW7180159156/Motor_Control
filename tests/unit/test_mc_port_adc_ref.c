#include "unity.h"
#include "mc_port_adc_ref.h"

static uint8_t g_last_compare_index;
static mc_f32_t g_last_compare_time_s;
static mc_adc_trigger_event_t g_last_compare_event;
static uint8_t g_last_soc_index;
static mc_adc_trigger_signal_t g_last_soc_signal;
static mc_bool_t g_last_soc_enable;
static uint8_t g_compare_call_count;
static uint8_t g_soc_call_count;

static void test_mc_adc_ref_set_compare(uint8_t compare_index,
                                        mc_f32_t compare_time_s,
                                        mc_adc_trigger_event_t event)
{
    g_last_compare_index = compare_index;
    g_last_compare_time_s = compare_time_s;
    g_last_compare_event = event;
    g_compare_call_count += 1U;
}

static void test_mc_adc_ref_set_soc(uint8_t soc_index,
                                    mc_adc_trigger_signal_t signal,
                                    mc_bool_t enable)
{
    g_last_soc_index = soc_index;
    g_last_soc_signal = signal;
    g_last_soc_enable = enable;
    g_soc_call_count += 1U;
}

void test_mc_adc_ref_build_apply_cfg_maps_hw_triggers(void)
{
    mc_adc_hw_trigger_map_t hw_map = {0};
    mc_adc_ref_apply_cfg_t apply_cfg = {0};

    hw_map.count = 2U;
    hw_map.trigger_a.compare_index = 0U;
    hw_map.trigger_a.soc_index = 0U;
    hw_map.trigger_a.event = MC_ADC_TRIGGER_EVENT_PWM_UP;
    hw_map.trigger_a.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
    hw_map.trigger_a.compare_time_s = 0.000012F;
    hw_map.trigger_b.compare_index = 1U;
    hw_map.trigger_b.soc_index = 1U;
    hw_map.trigger_b.event = MC_ADC_TRIGGER_EVENT_PWM_DOWN;
    hw_map.trigger_b.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
    hw_map.trigger_b.compare_time_s = 0.000082F;

    mc_adc_ref_build_apply_cfg(&hw_map, &apply_cfg);

    TEST_ASSERT_TRUE(apply_cfg.count == 2U);
    TEST_ASSERT_TRUE(apply_cfg.soc_a.enable == MC_TRUE);
    TEST_ASSERT_TRUE(apply_cfg.soc_a.compare_index == 0U);
    TEST_ASSERT_TRUE(apply_cfg.soc_a.soc_index == 0U);
    TEST_ASSERT_TRUE(apply_cfg.soc_a.event == MC_ADC_TRIGGER_EVENT_PWM_UP);
    TEST_ASSERT_TRUE(apply_cfg.soc_a.signal == MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT);
    TEST_ASSERT_TRUE(apply_cfg.soc_a.compare_time_s > 0.000011F);
    TEST_ASSERT_TRUE(apply_cfg.soc_b.enable == MC_TRUE);
    TEST_ASSERT_TRUE(apply_cfg.soc_b.compare_index == 1U);
    TEST_ASSERT_TRUE(apply_cfg.soc_b.soc_index == 1U);
    TEST_ASSERT_TRUE(apply_cfg.soc_b.event == MC_ADC_TRIGGER_EVENT_PWM_DOWN);
    TEST_ASSERT_TRUE(apply_cfg.soc_b.signal == MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT);
    TEST_ASSERT_TRUE(apply_cfg.soc_b.compare_time_s > 0.000081F);
}

void test_mc_adc_ref_build_apply_cfg_leaves_empty_slots_disabled(void)
{
    mc_adc_hw_trigger_map_t hw_map = {0};
    mc_adc_ref_apply_cfg_t apply_cfg = {0};

    hw_map.count = 1U;
    hw_map.trigger_a.compare_index = 0U;
    hw_map.trigger_a.soc_index = 0U;
    hw_map.trigger_a.event = MC_ADC_TRIGGER_EVENT_PWM_UP;
    hw_map.trigger_a.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
    hw_map.trigger_a.compare_time_s = 0.000034F;

    mc_adc_ref_build_apply_cfg(&hw_map, &apply_cfg);

    TEST_ASSERT_TRUE(apply_cfg.count == 1U);
    TEST_ASSERT_TRUE(apply_cfg.soc_a.enable == MC_TRUE);
    TEST_ASSERT_TRUE(apply_cfg.soc_b.enable == MC_FALSE);
    TEST_ASSERT_TRUE(apply_cfg.soc_b.signal == MC_ADC_TRIGGER_NONE);
    TEST_ASSERT_TRUE(apply_cfg.soc_b.event == MC_ADC_TRIGGER_EVENT_NONE);
}

void test_mc_adc_ref_apply_invokes_reference_port_callbacks(void)
{
    mc_adc_ref_apply_cfg_t apply_cfg = {0};
    mc_adc_ref_port_t port = {test_mc_adc_ref_set_compare, test_mc_adc_ref_set_soc};

    g_last_compare_index = 0U;
    g_last_compare_time_s = 0.0F;
    g_last_compare_event = MC_ADC_TRIGGER_EVENT_NONE;
    g_last_soc_index = 0U;
    g_last_soc_signal = MC_ADC_TRIGGER_NONE;
    g_last_soc_enable = MC_FALSE;
    g_compare_call_count = 0U;
    g_soc_call_count = 0U;

    apply_cfg.count = 1U;
    apply_cfg.soc_a.enable = MC_TRUE;
    apply_cfg.soc_a.compare_index = 0U;
    apply_cfg.soc_a.soc_index = 0U;
    apply_cfg.soc_a.event = MC_ADC_TRIGGER_EVENT_PWM_UP;
    apply_cfg.soc_a.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
    apply_cfg.soc_a.compare_time_s = 0.000021F;

    mc_adc_ref_apply(&apply_cfg, &port);

    TEST_ASSERT_TRUE(g_compare_call_count == 2U);
    TEST_ASSERT_TRUE(g_soc_call_count == 2U);
    TEST_ASSERT_TRUE(g_last_compare_index == 0U);
    TEST_ASSERT_TRUE(g_last_compare_time_s > -0.000001F);
    TEST_ASSERT_TRUE(g_last_compare_time_s < 0.000001F);
    TEST_ASSERT_TRUE(g_last_compare_event == MC_ADC_TRIGGER_EVENT_NONE);
    TEST_ASSERT_TRUE(g_last_soc_index == 0U);
    TEST_ASSERT_TRUE(g_last_soc_signal == MC_ADC_TRIGGER_NONE);
    TEST_ASSERT_TRUE(g_last_soc_enable == MC_FALSE);
}

void test_mc_adc_ref_apply_plan_runs_full_reference_chain(void)
{
    mc_adc_trigger_plan_t plan = {0};
    mc_adc_ref_port_t port = {test_mc_adc_ref_set_compare, test_mc_adc_ref_set_soc};

    g_last_compare_index = 0U;
    g_last_compare_time_s = 0.0F;
    g_last_compare_event = MC_ADC_TRIGGER_EVENT_NONE;
    g_last_soc_index = 0U;
    g_last_soc_signal = MC_ADC_TRIGGER_NONE;
    g_last_soc_enable = MC_FALSE;
    g_compare_call_count = 0U;
    g_soc_call_count = 0U;

    plan.count = 1U;
    plan.trigger_a.valid = MC_TRUE;
    plan.trigger_a.signal = MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT;
    plan.trigger_a.event = MC_ADC_TRIGGER_EVENT_PWM_UP;
    plan.trigger_a.soc_index = 0U;
    plan.trigger_a.time_s = 0.000015F;

    mc_adc_ref_apply_plan(&plan, &port);

    TEST_ASSERT_TRUE(g_compare_call_count == 2U);
    TEST_ASSERT_TRUE(g_soc_call_count == 2U);
    TEST_ASSERT_TRUE(g_last_compare_index == 0U);
    TEST_ASSERT_TRUE(g_last_compare_time_s > -0.000001F);
    TEST_ASSERT_TRUE(g_last_compare_time_s < 0.000001F);
    TEST_ASSERT_TRUE(g_last_soc_index == 0U);
    TEST_ASSERT_TRUE(g_last_soc_signal == MC_ADC_TRIGGER_NONE);
    TEST_ASSERT_TRUE(g_last_soc_enable == MC_FALSE);
}
