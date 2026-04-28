#include "unity.h"
#include "mc_api.h"
#include "mc_constants.h"
#include <string.h>

static mc_pwm_cmd_t g_pwm;
static mc_adc_trigger_plan_t g_adc;
static uint8_t g_pwm_called;
static uint8_t g_adc_called;

static void stub_pwm(const void *cmd) { g_pwm = *(const mc_pwm_cmd_t *)cmd; g_pwm_called++; }
static void stub_adc(const mc_adc_trigger_plan_t *p) { g_adc = *p; g_adc_called++; }

static mc_system_cfg_t build_identify_cfg(void)
{
    mc_system_cfg_t cfg = {0};

    cfg.motor.pole_pairs = 4U;
    cfg.motor.rs_ohm = 0.1F;
    cfg.motor.ld_h = 0.0005F;
    cfg.motor.lq_h = 0.0007F;
    cfg.motor.flux_wb = 0.005F;

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
    cfg.foc.current_cfg.type = MC_CURRENT_SENSE_3SHUNT;
    cfg.foc.current_cfg.cfg.shunt3.offset_a = 0.0F;
    cfg.foc.current_cfg.cfg.shunt3.scale_a = 1.0F;
    cfg.foc.current_cfg.cfg.shunt3.offset_b = 0.0F;
    cfg.foc.current_cfg.cfg.shunt3.scale_b = 1.0F;
    cfg.foc.current_cfg.cfg.shunt3.offset_c = 0.0F;
    cfg.foc.current_cfg.cfg.shunt3.scale_c = 1.0F;
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

    return cfg;
}

void test_mc_identify_completes_full_sequence(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_fast_output_t out;
    mc_system_cfg_t cfg = build_identify_cfg();

    mc_init(&inst, &cfg);
    mc_start_identification(&inst);

    TEST_ASSERT_TRUE(mc_identify_is_active(&inst.identify));

    memset(&in, 0, sizeof(in));
    in.adc_raw.phase_a_raw = 500;
    in.adc_raw.phase_b_raw = 500;
    in.adc_raw.phase_c_raw = 500;
    in.timestamp_us = 1000U;

    int step;
    for (step = 0; step < 8000; step++)
    {
        in.timestamp_us += 50U;

        if (mc_identify_is_done(&inst.identify))
        {
            break;
        }

        mc_fast_step(&inst, &in, &out);
    }

    TEST_ASSERT_TRUE(mc_is_identification_done(&inst));

    mc_motor_params_t params;
    mc_status_t s = mc_get_identified_params(&inst, &params);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, s);
    TEST_ASSERT_EQUAL_INT(4U, params.pole_pairs);
}

void test_mc_identify_overcurrent_aborts(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_fast_output_t out;
    mc_system_cfg_t cfg = build_identify_cfg();

    cfg.foc.iq_limit = 1.0F;

    mc_init(&inst, &cfg);
    mc_start_identification(&inst);

    memset(&in, 0, sizeof(in));
    in.adc_raw.phase_a_raw = 500;
    in.adc_raw.phase_b_raw = 500;
    in.adc_raw.phase_c_raw = 500;
    in.timestamp_us = 1000U;

    int step;
    for (step = 0; step < 2000; step++)
    {
        in.timestamp_us += 50U;

        if (step > 1000)
        {
            in.adc_raw.phase_a_raw = 5000;
        }

        mc_fast_step(&inst, &in, &out);

        if (inst.identify.state == MC_IDENTIFY_STATE_ERROR)
        {
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT(MC_IDENTIFY_STATE_ERROR, inst.identify.state);
}

void test_mc_identify_uses_configured_current_sense_path(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_fast_output_t out;
    mc_system_cfg_t cfg = build_identify_cfg();

    cfg.foc.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.foc.current_cfg.cfg.shunt2.offset_a = 0.0F;
    cfg.foc.current_cfg.cfg.shunt2.scale_a = 1.0F;
    cfg.foc.current_cfg.cfg.shunt2.offset_b = 0.0F;
    cfg.foc.current_cfg.cfg.shunt2.scale_b = 1.0F;

    mc_init(&inst, &cfg);
    mc_start_identification(&inst);

    memset(&in, 0, sizeof(in));
    in.adc_raw.phase_a_raw = 100;
    in.adc_raw.phase_b_raw = 100;
    in.timestamp_us = 1000U;

    int step;
    for (step = 0; step < 500; step++)
    {
        in.timestamp_us += 50U;
        if (mc_identify_is_done(&inst.identify)) break;
        mc_fast_step(&inst, &in, &out);
    }

    (void)step;
}

