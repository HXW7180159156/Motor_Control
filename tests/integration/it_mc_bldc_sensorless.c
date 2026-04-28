#include "unity.h"
#include "mc_api.h"
#include "mc_constants.h"
#include <string.h>

static mc_pwm_cmd_t g_pwm;
static uint8_t g_pwm_called;

static void stub_pwm(const void *cmd) { g_pwm = *(const mc_pwm_cmd_t *)cmd; g_pwm_called++; }
static void stub_adc(const mc_adc_trigger_plan_t *p) { (void)p; }

void test_mc_bldc_sensorless_full_startup_and_run_cycle(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_fast_output_t out;
    mc_medium_input_t med_in;
    mc_medium_output_t med_out;
    mc_system_cfg_t cfg = {0};

    cfg.motor.pole_pairs = 2U;
    cfg.motor.rs_ohm = 0.5F;

    cfg.limits.bus_voltage_max_v = 24.0F;
    cfg.limits.phase_current_max_a = 10.0F;
    cfg.limits.speed_max_rpm = 3000.0F;

    cfg.control.pwm_frequency_hz = 20000U;
    cfg.control.numeric_mode = MC_NUMERIC_FLOAT32;
    cfg.control.current_loop_dt_s = 0.00005F;

    cfg.sensor.primary_mode = MC_MODE_BLDC_SENSORLESS;
    cfg.sensor.bldc_sensorless_cfg = mc_bldc_sensorless_cfg_default();

    cfg.hooks.pwm_apply = stub_pwm;
    cfg.hooks.adc_trigger_apply = stub_adc;

    mc_status_t s = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, s);
    TEST_ASSERT_EQUAL_INT(MC_MODE_BLDC_SENSORLESS, inst.mode);

    s = mc_start(&inst);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, s);

    memset(&in, 0, sizeof(in));
    in.timestamp_us = 100000U;

    uint32_t us = 100000U;
    int step;
    for (step = 0; step < 200; step++)
    {
        us += 50U;
        in.timestamp_us = us;
        mc_fast_step(&inst, &in, &out);
    }

    mc_stop(&inst);

    memset(&in, 0, sizeof(in));
    in.timestamp_us = us + 50U;
    mc_fast_step(&inst, &in, &out);

    TEST_ASSERT_EQUAL_INT(0U, out.pwm_cmd.valid);
}

void test_mc_bldc_sensorless_start_after_stop(void)
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

    cfg.sensor.primary_mode = MC_MODE_BLDC_SENSORLESS;
    cfg.sensor.bldc_sensorless_cfg = mc_bldc_sensorless_cfg_default();

    cfg.hooks.pwm_apply = stub_pwm;
    cfg.hooks.adc_trigger_apply = stub_adc;

    mc_init(&inst, &cfg);

    memset(&in, 0, sizeof(in));
    in.timestamp_us = 100000U;

    mc_start(&inst);
    mc_fast_step(&inst, &in, &out);
    mc_stop(&inst);

    mc_start(&inst);
    mc_fast_step(&inst, &in, &out);
    TEST_ASSERT_EQUAL_INT(1U, out.pwm_cmd.valid);
}

void test_mc_bldc_sensorless_rejects_invalid_start_cfg(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};

    cfg.motor.pole_pairs = 2U;
    cfg.control.pwm_frequency_hz = 20000U;
    cfg.control.numeric_mode = MC_NUMERIC_FLOAT32;
    cfg.control.current_loop_dt_s = 0.00005F;

    cfg.sensor.primary_mode = MC_MODE_BLDC_SENSORLESS;
    cfg.sensor.bldc_sensorless_cfg = mc_bldc_sensorless_cfg_default();
    cfg.sensor.bldc_sensorless_cfg.ramp_start_freq_hz = 0.0F;

    cfg.limits.bus_voltage_max_v = 24.0F;
    cfg.limits.phase_current_max_a = 10.0F;
    cfg.limits.speed_max_rpm = 3000.0F;

    mc_status_t s = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

