#include "unity.h"
#include "mc_api.h"
#include "mc_constants.h"
#include <string.h>

static mc_pwm_cmd_t g_pwm;
static uint8_t g_pwm_called;

static void stub_pwm(const void *cmd) { g_pwm = *(const mc_pwm_cmd_t *)cmd; g_pwm_called++; }
static void stub_adc(const mc_adc_trigger_plan_t *p) { (void)p; }

void test_mc_bldc_hall_init_and_first_step(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_fast_output_t out;
    mc_system_cfg_t cfg = {0};

    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;

    cfg.limits.bus_voltage_max_v = 24.0F;
    cfg.limits.phase_current_max_a = 10.0F;
    cfg.limits.speed_max_rpm = 3000.0F;

    cfg.control.pwm_frequency_hz = 20000U;
    cfg.control.numeric_mode = MC_NUMERIC_FLOAT32;
    cfg.control.current_loop_dt_s = 0.00005F;

    cfg.sensor.primary_mode = MC_MODE_BLDC_HALL;
    cfg.sensor.hall_cfg.hall_code_sequence[0] = 1U;
    cfg.sensor.hall_cfg.hall_code_sequence[1] = 5U;
    cfg.sensor.hall_cfg.hall_code_sequence[2] = 4U;
    cfg.sensor.hall_cfg.hall_code_sequence[3] = 6U;
    cfg.sensor.hall_cfg.hall_code_sequence[4] = 2U;
    cfg.sensor.hall_cfg.hall_code_sequence[5] = 3U;
    cfg.sensor.hall_cfg.elec_angle_table_rad[1] = 1.047F;
    cfg.sensor.hall_cfg.elec_angle_table_rad[5] = 2.094F;
    cfg.sensor.hall_cfg.elec_angle_table_rad[4] = 3.142F;
    cfg.sensor.hall_cfg.elec_angle_table_rad[6] = 4.189F;
    cfg.sensor.hall_cfg.elec_angle_table_rad[2] = 5.236F;
    cfg.sensor.hall_cfg.elec_angle_table_rad[3] = 0.0F;
    cfg.sensor.pole_pairs = 2.0F;

    cfg.hooks.pwm_apply = stub_pwm;
    cfg.hooks.adc_trigger_apply = stub_adc;

    mc_status_t s = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, s);
    TEST_ASSERT_EQUAL_INT(MC_MODE_BLDC_HALL, inst.mode);

    mc_start(&inst);
    mc_set_torque_ref(&inst, 0.5F);

    memset(&in, 0, sizeof(in));
    in.hall_code = 4U;
    in.timestamp_us = 1000U;

    mc_fast_step(&inst, &in, &out);

    TEST_ASSERT_EQUAL_INT(1U, out.pwm_cmd.valid);
    TEST_ASSERT_EQUAL_INT(3U, out.pwm_cmd.sector);
    TEST_ASSERT_TRUE(g_pwm_called > 0U);
}

void test_mc_bldc_hall_six_step_covers_all_sectors(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_fast_output_t out;
    mc_system_cfg_t cfg = {0};

    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;

    cfg.limits.bus_voltage_max_v = 24.0F;
    cfg.limits.phase_current_max_a = 10.0F;
    cfg.limits.speed_max_rpm = 3000.0F;

    cfg.control.pwm_frequency_hz = 20000U;
    cfg.control.numeric_mode = MC_NUMERIC_FLOAT32;
    cfg.control.current_loop_dt_s = 0.00005F;

    cfg.sensor.primary_mode = MC_MODE_BLDC_HALL;
    uint8_t i;
    for (i = 0U; i < MC_HALL_STEPS; i++)
    {
        cfg.sensor.hall_cfg.hall_code_sequence[i] = (uint8_t)(i + 1U);
        cfg.sensor.hall_cfg.elec_angle_table_rad[i] = (mc_f32_t)i * 1.047F;
    }
    cfg.sensor.pole_pairs = 2.0F;

    cfg.hooks.pwm_apply = stub_pwm;
    cfg.hooks.adc_trigger_apply = stub_adc;

    mc_init(&inst, &cfg);
    mc_start(&inst);
    mc_set_torque_ref(&inst, 0.5F);

    for (i = 1U; i <= MC_HALL_STEPS; i++)
    {
        memset(&in, 0, sizeof(in));
        in.hall_code = i;
        in.timestamp_us = (uint32_t)(1000U + i * 100U);
        mc_fast_step(&inst, &in, &out);
        TEST_ASSERT_EQUAL_INT(i, out.pwm_cmd.sector);
    }
}

void test_mc_bldc_hall_disabled_produces_zero_pwm(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_fast_output_t out;
    mc_system_cfg_t cfg = {0};

    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;

    cfg.limits.bus_voltage_max_v = 24.0F;
    cfg.limits.phase_current_max_a = 10.0F;
    cfg.limits.speed_max_rpm = 3000.0F;

    cfg.control.pwm_frequency_hz = 20000U;
    cfg.control.numeric_mode = MC_NUMERIC_FLOAT32;
    cfg.control.current_loop_dt_s = 0.00005F;

    cfg.sensor.primary_mode = MC_MODE_BLDC_HALL;
    cfg.sensor.hall_cfg.hall_code_sequence[0] = 1U;
    cfg.sensor.hall_cfg.elec_angle_table_rad[0] = 0.0F;
    cfg.sensor.pole_pairs = 2.0F;

    cfg.hooks.pwm_apply = stub_pwm;
    cfg.hooks.adc_trigger_apply = stub_adc;

    mc_init(&inst, &cfg);

    memset(&in, 0, sizeof(in));
    in.hall_code = 1U;
    in.timestamp_us = 1000U;

    mc_fast_step(&inst, &in, &out);
    TEST_ASSERT_EQUAL_INT(0U, out.pwm_cmd.valid);
}

