#include "unity.h"
#include "mc_api.h"
#include "mc_constants.h"
#include <string.h>
#include <math.h>

static mc_pwm_cmd_t g_pwm;
static uint8_t g_pwm_called;

static void ft_pwm(const void *cmd) { g_pwm = *(const mc_pwm_cmd_t *)cmd; g_pwm_called++; }
static void ft_adc(const mc_adc_trigger_plan_t *p) { (void)p; }

/* --- Hall sensor code out of legal range --- */
void test_ft_hall_invalid_code_stops_angle_update(void)
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

    cfg.hooks.pwm_apply = ft_pwm;
    cfg.hooks.adc_trigger_apply = ft_adc;

    mc_init(&inst, &cfg);
    mc_start(&inst);
    mc_set_torque_ref(&inst, 0.5F);

    memset(&in, 0, sizeof(in));
    in.hall_code = 0xFFU;
    in.timestamp_us = 1000U;

    mc_fast_step(&inst, &in, &out);
    TEST_ASSERT_EQUAL_INT(0U, out.pwm_cmd.valid);
}

/* --- Resolver low signal amplitude --- */
void test_ft_resolver_low_signal_marks_invalid(void)
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
    cfg.foc.iq_limit = 5.0F;
    cfg.foc.voltage_limit = 24.0F;
    cfg.foc.bus_voltage_scale = 1.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.foc.mtpa_enable = MC_FALSE;
    cfg.foc.fw_enable = MC_FALSE;

    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_RESOLVER;
    cfg.sensor.resolver_cfg.pole_pairs = 4.0F;
    cfg.sensor.resolver_cfg.signal_scale = 0.001F;
    cfg.sensor.resolver_cfg.min_signal_amplitude = 0.5F;
    cfg.sensor.resolver_cfg.excitation_amplitude = 1000;
    cfg.sensor.resolver_cfg.excitation_frequency_hz = 10000U;

    cfg.hooks.pwm_apply = ft_pwm;
    cfg.hooks.adc_trigger_apply = ft_adc;

    mc_status_t s = mc_init(&inst, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, s);

    mc_set_enable(&inst, MC_TRUE);

    memset(&in, 0, sizeof(in));
    in.resolver_raw.sin_raw = 10;
    in.resolver_raw.cos_raw = 10;
    in.timestamp_us = 1000U;

    mc_fast_step(&inst, &in, &out);
    TEST_ASSERT_TRUE(inst.resolver.signal_valid == MC_FALSE);
}

/* --- Sensorless low BEMF during PLL locked state --- */
void test_ft_sensorless_bemf_drop_while_locked(void)
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
    cfg.foc.iq_limit = 5.0F;
    cfg.foc.voltage_limit = 24.0F;
    cfg.foc.bus_voltage_scale = 1.0F;
    cfg.foc.bus_voltage_min = 5.0F;
    cfg.foc.mtpa_enable = MC_FALSE;
    cfg.foc.fw_enable = MC_FALSE;

    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_SENSORLESS;
    cfg.sensor.sensorless_cfg.bemf_filter_alpha = MC_SENSORLESS_DEFAULT_BEMF_ALPHA;
    cfg.sensor.sensorless_cfg.min_bemf = MC_SENSORLESS_DEFAULT_MIN_BEMF;
    cfg.sensor.sensorless_cfg.pll_kp = MC_SENSORLESS_DEFAULT_PLL_KP;
    cfg.sensor.sensorless_cfg.pll_ki = MC_SENSORLESS_DEFAULT_PLL_KI;
    cfg.sensor.sensorless_cfg.lock_bemf = MC_SENSORLESS_DEFAULT_MIN_BEMF * MC_SENSORLESS_DEFAULT_LOCK_BEMF_RATIO;
    cfg.sensor.sensorless_cfg.startup_speed_rad_s = MC_SENSORLESS_DEFAULT_STARTUP_SPEED_RAD_S;
    cfg.sensor.sensorless_cfg.startup_accel_rad_s2 = MC_SENSORLESS_DEFAULT_STARTUP_ACCEL_RAD_S2;
    cfg.sensor.sensorless_cfg.open_loop_voltage_max = MC_SENSORLESS_DEFAULT_OL_VOLTAGE_RATIO * cfg.foc.voltage_limit;

    cfg.hooks.pwm_apply = ft_pwm;
    cfg.hooks.adc_trigger_apply = ft_adc;

    mc_init(&inst, &cfg);
    mc_start(&inst);
    mc_set_current_ref_dq(&inst, 0.0F, 0.3F);

    memset(&in, 0, sizeof(in));
    in.adc_raw.phase_a_raw = 500;
    in.adc_raw.phase_b_raw = 500;
    in.timestamp_us = 1000U;

    int i;
    for (i = 0; i < 5; i++)
    {
        in.timestamp_us += 50U;
        mc_fast_step(&inst, &in, &out);
    }

    TEST_ASSERT_TRUE(inst.sensorless.observer_valid == MC_FALSE);
}
