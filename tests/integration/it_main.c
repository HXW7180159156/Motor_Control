#include "unity.h"

/* --- PMSM FOC pipeline --- */
void test_mc_pmsm_foc_fast_step_produces_valid_pwm(void);
void test_mc_pmsm_foc_2shunt_reconstructs_3phases(void);
void test_mc_pmsm_foc_field_weakening_adjusts_id(void);

/* --- PMSM sensorless --- */
void test_mc_sensorless_init_and_open_loop_startup(void);
void test_mc_sensorless_fast_step_fills_diagnostic_fields(void);
void test_mc_sensorless_1shunt_reconstruction_path(void);

/* --- BLDC Hall pipeline --- */
void test_mc_bldc_hall_init_and_first_step(void);
void test_mc_bldc_hall_six_step_covers_all_sectors(void);
void test_mc_bldc_hall_disabled_produces_zero_pwm(void);

/* --- BLDC sensorless --- */
void test_mc_bldc_sensorless_full_startup_and_run_cycle(void);
void test_mc_bldc_sensorless_start_after_stop(void);
void test_mc_bldc_sensorless_rejects_invalid_start_cfg(void);

/* --- Identify pipeline --- */
void test_mc_identify_completes_full_sequence(void);
void test_mc_identify_overcurrent_aborts(void);
void test_mc_identify_uses_configured_current_sense_path(void);

/* --- Fast/medium/slow scheduling --- */
void test_mc_fast_medium_slow_scheduling_order(void);
void test_mc_fast_step_disabled_returns_zero_pwm(void);
void test_mc_slow_step_clears_fault_on_request(void);

void setUp(void) {}
void tearDown(void) {}

int main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_mc_pmsm_foc_fast_step_produces_valid_pwm);
    RUN_TEST(test_mc_pmsm_foc_2shunt_reconstructs_3phases);
    RUN_TEST(test_mc_pmsm_foc_field_weakening_adjusts_id);

    RUN_TEST(test_mc_sensorless_init_and_open_loop_startup);
    RUN_TEST(test_mc_sensorless_fast_step_fills_diagnostic_fields);
    RUN_TEST(test_mc_sensorless_1shunt_reconstruction_path);

    RUN_TEST(test_mc_bldc_hall_init_and_first_step);
    RUN_TEST(test_mc_bldc_hall_six_step_covers_all_sectors);
    RUN_TEST(test_mc_bldc_hall_disabled_produces_zero_pwm);

    RUN_TEST(test_mc_bldc_sensorless_full_startup_and_run_cycle);
    RUN_TEST(test_mc_bldc_sensorless_start_after_stop);
    RUN_TEST(test_mc_bldc_sensorless_rejects_invalid_start_cfg);

    RUN_TEST(test_mc_identify_completes_full_sequence);

    RUN_TEST(test_mc_fast_medium_slow_scheduling_order);
    RUN_TEST(test_mc_fast_step_disabled_returns_zero_pwm);
    RUN_TEST(test_mc_slow_step_clears_fault_on_request);

    return UNITY_END();
}
