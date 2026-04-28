#include "unity.h"

/* --- sensor faults --- */
void test_ft_hall_invalid_code_stops_angle_update(void);
void test_ft_resolver_low_signal_marks_invalid(void);
void test_ft_sensorless_bemf_drop_while_locked(void);

/* --- reconstruct faults --- */
void test_ft_1shunt_dead_short_duty_single_phase_only(void);
void test_ft_3shunt_adc_phase_a_drift(void);

/* --- lifecycle faults --- */
void test_ft_identify_overcurrent_during_rs_measure(void);
void test_ft_init_null_config_leaves_uninitialized(void);
void test_ft_fast_step_null_inst_rejected(void);
void test_ft_fast_step_null_out_rejected(void);
void test_ft_set_mode_rejects_runtime_switch(void);

void setUp(void) {}
void tearDown(void) {}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_ft_hall_invalid_code_stops_angle_update);
    RUN_TEST(test_ft_resolver_low_signal_marks_invalid);
    RUN_TEST(test_ft_sensorless_bemf_drop_while_locked);

    RUN_TEST(test_ft_1shunt_dead_short_duty_single_phase_only);
    RUN_TEST(test_ft_3shunt_adc_phase_a_drift);

    RUN_TEST(test_ft_identify_overcurrent_during_rs_measure);
    RUN_TEST(test_ft_init_null_config_leaves_uninitialized);
    RUN_TEST(test_ft_fast_step_null_inst_rejected);
    RUN_TEST(test_ft_fast_step_null_out_rejected);
    RUN_TEST(test_ft_set_mode_rejects_runtime_switch);

    return UNITY_END();
}
