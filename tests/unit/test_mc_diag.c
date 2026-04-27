#include "unity.h"
#include <string.h>
#include "mc_diag.h"

void test_mc_diag_1shunt_comp_mode_name_none(void)
{
    const char *name = mc_diag_1shunt_comp_mode_name(MC_1SHUNT_COMP_NONE);
    TEST_ASSERT_TRUE(name != NULL);
    TEST_ASSERT_TRUE(strcmp("none", name) == 0);
}

void test_mc_diag_1shunt_comp_mode_name_predict_basic(void)
{
    const char *name = mc_diag_1shunt_comp_mode_name(MC_1SHUNT_COMP_PREDICT_BASIC);
    TEST_ASSERT_TRUE(name != NULL);
    TEST_ASSERT_TRUE(strcmp("predict_basic", name) == 0);
}

void test_mc_diag_1shunt_comp_mode_name_predict_high_modulation(void)
{
    const char *name = mc_diag_1shunt_comp_mode_name(MC_1SHUNT_COMP_PREDICT_HIGH_MODULATION);
    TEST_ASSERT_TRUE(name != NULL);
    TEST_ASSERT_TRUE(strcmp("predict_high_modulation", name) == 0);
}

void test_mc_diag_1shunt_comp_mode_name_predict_field_weakening(void)
{
    const char *name = mc_diag_1shunt_comp_mode_name(MC_1SHUNT_COMP_PREDICT_FIELD_WEAKENING);
    TEST_ASSERT_TRUE(name != NULL);
    TEST_ASSERT_TRUE(strcmp("predict_field_weakening", name) == 0);
}

void test_mc_diag_1shunt_comp_mode_name_unknown_returns_unknown(void)
{
    const char *name = mc_diag_1shunt_comp_mode_name((mc_1shunt_comp_mode_t)99);
    TEST_ASSERT_TRUE(name != NULL);
    TEST_ASSERT_TRUE(strcmp("unknown", name) == 0);
}

void test_mc_diag_status_defaults_to_zero(void)
{
    mc_diag_status_t d = {0};
    TEST_ASSERT_EQUAL_INT(0, (int)d.active_fault);
    TEST_ASSERT_EQUAL_INT(0, (int)d.active_warning);
    TEST_ASSERT_TRUE(d.fault_counter == 0U);
    TEST_ASSERT_TRUE(d.warning_counter == 0U);
    TEST_ASSERT_EQUAL_INT(0, (int)d.sensorless_observer_valid);
}

void test_mc_diag_status_reports_fault(void)
{
    mc_diag_status_t d = {0};
    d.active_fault = MC_FAULT_OVERCURRENT;
    TEST_ASSERT_EQUAL_INT((int)MC_FAULT_OVERCURRENT, (int)d.active_fault);
}

void test_mc_diag_status_reports_warning(void)
{
    mc_diag_status_t d = {0};
    d.active_warning = MC_WARNING_OBSERVER_UNLOCKED;
    TEST_ASSERT_EQUAL_INT((int)MC_WARNING_OBSERVER_UNLOCKED, (int)d.active_warning);
}
