#include "unity.h"
#include "mc_api.h"
#include "mc_constants.h"
#include <string.h>
#include <math.h>

static mc_pwm_cmd_t g_pwm;
static uint8_t g_pwm_called;

static void ft_pwm(const void *cmd) { g_pwm = *(const mc_pwm_cmd_t *)cmd; g_pwm_called++; }
static void ft_adc(const mc_adc_trigger_plan_t *p) { (void)p; }

/* --- Identify: overcurrent during active sequence → ERROR state --- */
void test_ft_identify_overcurrent_during_rs_measure(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_fast_output_t out;
    mc_system_cfg_t cfg = {0};

    cfg.motor.pole_pairs = 4U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.001F;
    cfg.motor.lq_h = 0.001F;
    cfg.motor.flux_wb = 0.01F;

    cfg.limits.bus_voltage_max_v = 24.0F;
    cfg.limits.phase_current_max_a = 10.0F;
    cfg.limits.speed_max_rpm = 5000.0F;

    cfg.control.pwm_frequency_hz = 20000U;
    cfg.control.numeric_mode = MC_NUMERIC_FLOAT32;
    cfg.control.current_loop_dt_s = 0.00005F;

    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 100.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 100.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.1F, 10.0F, -0.5F, 0.5F, -5.0F, 5.0F};
    cfg.foc.svpwm_cfg.modulation_limit = 0.95F;
    cfg.foc.svpwm_cfg.duty_min = 0.0F;
    cfg.foc.svpwm_cfg.duty_max = 1.0F;
    cfg.foc.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.foc.current_cfg.cfg.shunt2.offset_a = 0.0F;
    cfg.foc.current_cfg.cfg.shunt2.scale_a = 1.0F;
    cfg.foc.current_cfg.cfg.shunt2.offset_b = 0.0F;
    cfg.foc.current_cfg.cfg.shunt2.scale_b = 1.0F;
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 1.0F;
    cfg.foc.voltage_limit = 24.0F;
    cfg.foc.bus_voltage_scale = 1.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.foc.mtpa_enable = MC_FALSE;
    cfg.foc.fw_enable = MC_FALSE;

    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 4096U;
    cfg.sensor.encoder_cfg.pole_pairs = 4.0F;

    cfg.hooks.pwm_apply = ft_pwm;
    cfg.hooks.adc_trigger_apply = ft_adc;

    mc_init(&inst, &cfg);
    mc_start_identification(&inst);

    memset(&in, 0, sizeof(in));
    in.adc_raw.phase_a_raw = 200;
    in.adc_raw.phase_b_raw = 200;
    in.timestamp_us = 1000U;

    int i;
    for (i = 0; i < 2000; i++)
    {
        in.timestamp_us += 50U;

        if (i > 1000)
        {
            in.adc_raw.phase_a_raw = 10000;
            in.adc_raw.phase_b_raw = 10000;
        }

        mc_fast_step(&inst, &in, &out);

        if (inst.identify.state == MC_IDENTIFY_STATE_ERROR)
        {
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_ERROR, inst.identify.state);
    TEST_ASSERT_TRUE(fabsf(out.pwm_cmd.duty_a) < 0.001F);
}

/* --- mc_init: null config → returns error, initialized stays false --- */
void test_ft_init_null_config_leaves_uninitialized(void)
{
    mc_instance_t inst;
    memset(&inst, 0xAA, sizeof(inst));

    mc_status_t s = mc_init(&inst, NULL);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
    TEST_ASSERT_TRUE(inst.initialized == MC_FALSE);
}

/* --- mc_fast_step: null inst → INVALID_ARG --- */
void test_ft_fast_step_null_inst_rejected(void)
{
    mc_fast_input_t in;
    mc_fast_output_t out;
    memset(&in, 0, sizeof(in));
    memset(&out, 0xFF, sizeof(out));

    mc_status_t s = mc_fast_step(NULL, &in, &out);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

/* --- mc_fast_step: null out → INVALID_ARG --- */
void test_ft_fast_step_null_out_rejected(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_system_cfg_t cfg = {0};

    cfg.motor.pole_pairs = 4U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.001F;
    cfg.motor.lq_h = 0.001F;
    cfg.motor.flux_wb = 0.01F;

    cfg.limits.bus_voltage_max_v = 24.0F;
    cfg.limits.phase_current_max_a = 10.0F;
    cfg.limits.speed_max_rpm = 5000.0F;

    cfg.control.pwm_frequency_hz = 20000U;
    cfg.control.numeric_mode = MC_NUMERIC_FLOAT32;
    cfg.control.current_loop_dt_s = 0.00005F;

    cfg.foc.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.foc.current_cfg.cfg.shunt2.offset_a = 0.0F;
    cfg.foc.current_cfg.cfg.shunt2.scale_a = 1.0F;
    cfg.foc.current_cfg.cfg.shunt2.offset_b = 0.0F;
    cfg.foc.current_cfg.cfg.shunt2.scale_b = 1.0F;
    cfg.foc.svpwm_cfg.modulation_limit = 0.95F;
    cfg.foc.svpwm_cfg.duty_min = 0.0F;
    cfg.foc.svpwm_cfg.duty_max = 1.0F;
    cfg.foc.iq_limit = 5.0F;
    cfg.foc.voltage_limit = 24.0F;
    cfg.foc.bus_voltage_scale = 1.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.foc.speed_loop_dt_s = 0.001F;

    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 4096U;
    cfg.sensor.encoder_cfg.pole_pairs = 4.0F;

    cfg.hooks.pwm_apply = ft_pwm;
    cfg.hooks.adc_trigger_apply = ft_adc;

    mc_init(&inst, &cfg);

    memset(&in, 0, sizeof(in));
    in.adc_raw.phase_a_raw = 500;
    in.adc_raw.phase_b_raw = 500;
    in.timestamp_us = 1000U;

    mc_status_t s = mc_fast_step(&inst, &in, NULL);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, s);
}

/* --- mc_set_mode rejects switching after initialization --- */
void test_ft_set_mode_rejects_runtime_switch(void)
{
    mc_instance_t inst;
    mc_system_cfg_t cfg = {0};

    cfg.motor.pole_pairs = 4U;
    cfg.motor.rs_ohm = 0.5F;
    cfg.motor.ld_h = 0.001F;
    cfg.motor.lq_h = 0.001F;
    cfg.motor.flux_wb = 0.01F;

    cfg.limits.bus_voltage_max_v = 24.0F;
    cfg.limits.phase_current_max_a = 10.0F;
    cfg.limits.speed_max_rpm = 5000.0F;

    cfg.control.pwm_frequency_hz = 20000U;
    cfg.control.numeric_mode = MC_NUMERIC_FLOAT32;
    cfg.control.current_loop_dt_s = 0.00005F;

    cfg.foc.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.foc.svpwm_cfg.modulation_limit = 0.95F;
    cfg.foc.svpwm_cfg.duty_min = 0.0F;
    cfg.foc.svpwm_cfg.duty_max = 1.0F;
    cfg.foc.iq_limit = 5.0F;
    cfg.foc.voltage_limit = 24.0F;
    cfg.foc.speed_loop_dt_s = 0.001F;

    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 4096U;
    cfg.sensor.encoder_cfg.pole_pairs = 4.0F;

    mc_init(&inst, &cfg);
    TEST_ASSERT_TRUE(inst.initialized == MC_TRUE);

    mc_status_t s = mc_set_mode(&inst, MC_MODE_BLDC_HALL);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_STATE, s);
}
