#include "unity.h"
#include "mc_api.h"
#include "mc_constants.h"
#include <string.h>

static mc_pwm_cmd_t g_pwm;
static mc_adc_trigger_plan_t g_adc;
static uint8_t g_pwm_called;

static void stub_pwm(const void *cmd) { g_pwm = *(const mc_pwm_cmd_t *)cmd; g_pwm_called++; }
static void stub_adc(const mc_adc_trigger_plan_t *p) { g_adc = *p; }

void test_mc_fast_medium_slow_scheduling_order(void)
{
    mc_instance_t inst;
    mc_fast_input_t fast_in;
    mc_fast_output_t fast_out;
    mc_medium_input_t med_in;
    mc_medium_output_t med_out;
    mc_slow_input_t slow_in;
    mc_slow_output_t slow_out;
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
    cfg.foc.iq_limit = 5.0F;
    cfg.foc.voltage_limit = 24.0F;
    cfg.foc.bus_voltage_scale = 1.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.foc.mtpa_enable = MC_FALSE;
    cfg.foc.fw_enable = MC_FALSE;

    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 4096U;
    cfg.sensor.encoder_cfg.pole_pairs = 4.0F;

    cfg.hooks.pwm_apply = stub_pwm;
    cfg.hooks.adc_trigger_apply = stub_adc;

    mc_init(&inst, &cfg);
    mc_start(&inst);
    mc_set_speed_ref(&inst, 1000.0F);

    memset(&fast_in, 0, sizeof(fast_in));
    fast_in.adc_raw.phase_a_raw = 500;
    fast_in.adc_raw.phase_b_raw = 500;
    fast_in.timestamp_us = 1000U;

    mc_fast_step(&inst, &fast_in, &fast_out);
    TEST_ASSERT_EQUAL_INT(1U, fast_out.pwm_cmd.valid);

    memset(&med_in, 0, sizeof(med_in));
    med_in.speed_feedback_rpm = 950.0F;
    med_in.encoder_count = 1000U;
    med_in.timestamp_us = 2000U;

    mc_medium_step(&inst, &med_in, &med_out);
    TEST_ASSERT_TRUE(med_out.mech_speed_rpm >= 0.0F);

    memset(&slow_in, 0, sizeof(slow_in));
    slow_in.temperature_deg_c = 25.0F;
    slow_in.clear_fault_request = MC_FALSE;
    slow_in.timestamp_us = 3000U;

    mc_slow_step(&inst, &slow_in, &slow_out);
    TEST_ASSERT_EQUAL_INT(MC_MODE_PMSM_FOC_ENCODER, slow_out.mode);
}

void test_mc_fast_step_disabled_returns_zero_pwm(void)
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

    cfg.hooks.pwm_apply = stub_pwm;
    cfg.hooks.adc_trigger_apply = stub_adc;

    mc_init(&inst, &cfg);

    memset(&in, 0, sizeof(in));
    in.adc_raw.phase_a_raw = 500;
    in.adc_raw.phase_b_raw = 500;
    in.timestamp_us = 1000U;

    mc_fast_step(&inst, &in, &out);
    TEST_ASSERT_EQUAL_INT(0U, out.pwm_cmd.valid);
    TEST_ASSERT_TRUE(out.pwm_cmd.duty_a < 0.001F);

    mc_slow_output_t slow_out;
    mc_slow_input_t slow_in = {0};
    mc_slow_step(&inst, &slow_in, &slow_out);
    TEST_ASSERT_EQUAL_INT(MC_MODE_PMSM_FOC_ENCODER, slow_out.mode);
}

void test_mc_slow_step_clears_fault_on_request(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_fast_output_t out;
    mc_slow_input_t slow_in;
    mc_slow_output_t slow_out;
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

    cfg.hooks.pwm_apply = stub_pwm;
    cfg.hooks.adc_trigger_apply = stub_adc;

    mc_init(&inst, &cfg);

    memset(&slow_in, 0, sizeof(slow_in));
    slow_in.clear_fault_request = MC_TRUE;

    mc_slow_step(&inst, &slow_in, &slow_out);
    TEST_ASSERT_EQUAL_INT(MC_FAULT_NONE, slow_out.diag.active_fault);
    TEST_ASSERT_EQUAL_INT(MC_WARNING_NONE, slow_out.diag.active_warning);
}

