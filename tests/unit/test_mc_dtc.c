#include "unity.h"
#include "mc_constants.h"
#include "mc_control_foc.h"
#include "mc_drive_pmsm.h"
#include "mc_math.h"
#include <math.h>
#include <string.h>

void test_mc_dtc_disabled_does_not_affect_voltage(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {0};
    mc_pmsm_foc_input_t in;
    mc_pmsm_foc_output_t out;

    cfg.id_pi_cfg = (mc_pi_cfg_t){0.5F, 50.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.iq_pi_cfg = (mc_pi_cfg_t){0.5F, 50.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.svpwm_cfg.modulation_limit = 0.95F;
    cfg.svpwm_cfg.duty_min = 0.0F;
    cfg.svpwm_cfg.duty_max = 1.0F;
    cfg.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.current_cfg.cfg.shunt2.offset_a = 0.0F;
    cfg.current_cfg.cfg.shunt2.scale_a = 1.0F;
    cfg.current_cfg.cfg.shunt2.offset_b = 0.0F;
    cfg.current_cfg.cfg.shunt2.scale_b = 1.0F;
    cfg.voltage_limit = 24.0F;
    cfg.bus_voltage_min = 5.0F;
    cfg.rs_ohm = 0.5F;
    cfg.ld_h = 0.001F;
    cfg.lq_h = 0.001F;
    cfg.flux_wb = 0.01F;
    cfg.pole_pairs = 4.0F;
    cfg.dtc_enable = MC_FALSE;
    cfg.dtc_deadtime_ns = 1000.0F;
    cfg.pwm_freq_hz = 20000.0F;

    mc_pmsm_foc_init(&foc, &cfg);

    memset(&in, 0, sizeof(in));
    in.current_raw.phase_a_raw = 100;
    in.current_raw.phase_b_raw = 100;
    in.bus_voltage_v = 24.0F;
    in.sin_theta = 0.0F;
    in.cos_theta = 1.0F;
    in.id_ref = 0.0F;
    in.iq_ref = 0.5F;
    in.dt_s = 0.00005F;
    in.elec_speed_rad_s = 0.0F;

    mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(1U, out.pwm_cmd.valid);
}

void test_mc_dtc_enabled_produces_nonzero_compensation(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {0};
    mc_pmsm_foc_input_t in;
    mc_pmsm_foc_output_t out;

    cfg.id_pi_cfg = (mc_pi_cfg_t){0.5F, 50.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.iq_pi_cfg = (mc_pi_cfg_t){0.5F, 50.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.svpwm_cfg.modulation_limit = 0.95F;
    cfg.svpwm_cfg.duty_min = 0.0F;
    cfg.svpwm_cfg.duty_max = 1.0F;
    cfg.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.current_cfg.cfg.shunt2.offset_a = 0.0F;
    cfg.current_cfg.cfg.shunt2.scale_a = 1.0F;
    cfg.current_cfg.cfg.shunt2.offset_b = 0.0F;
    cfg.current_cfg.cfg.shunt2.scale_b = 1.0F;
    cfg.voltage_limit = 24.0F;
    cfg.bus_voltage_min = 5.0F;
    cfg.rs_ohm = 0.5F;
    cfg.ld_h = 0.001F;
    cfg.lq_h = 0.001F;
    cfg.flux_wb = 0.01F;
    cfg.pole_pairs = 4.0F;
    cfg.dtc_enable = MC_TRUE;
    cfg.dtc_deadtime_ns = 1000.0F;
    cfg.pwm_freq_hz = 20000.0F;

    mc_pmsm_foc_init(&foc, &cfg);

    memset(&in, 0, sizeof(in));
    in.current_raw.phase_a_raw = 100;
    in.current_raw.phase_b_raw = 100;
    in.bus_voltage_v = 24.0F;
    in.sin_theta = 0.0F;
    in.cos_theta = 1.0F;
    in.id_ref = 0.0F;
    in.iq_ref = 1.0F;
    in.dt_s = 0.00005F;
    in.elec_speed_rad_s = 0.0F;

    mc_pmsm_foc_run(&foc, &in, &out);
    TEST_ASSERT_EQUAL_INT(1U, out.pwm_cmd.valid);
    TEST_ASSERT_TRUE(isfinite(out.v_dq.d));
    TEST_ASSERT_TRUE(isfinite(out.v_dq.q));
}

void test_mc_dtc_higher_deadtime_produces_larger_compensation(void)
{
    mc_pmsm_foc_t foc1, foc2;
    mc_pmsm_foc_cfg_t cfg = {0};
    mc_pmsm_foc_input_t in;
    mc_pmsm_foc_output_t out1, out2;

    cfg.id_pi_cfg = (mc_pi_cfg_t){0.5F, 50.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.iq_pi_cfg = (mc_pi_cfg_t){0.5F, 50.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.svpwm_cfg.modulation_limit = 0.95F;
    cfg.svpwm_cfg.duty_min = 0.0F;
    cfg.svpwm_cfg.duty_max = 1.0F;
    cfg.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.current_cfg.cfg.shunt2.offset_a = 0.0F;
    cfg.current_cfg.cfg.shunt2.scale_a = 1.0F;
    cfg.current_cfg.cfg.shunt2.offset_b = 0.0F;
    cfg.current_cfg.cfg.shunt2.scale_b = 1.0F;
    cfg.voltage_limit = 24.0F;
    cfg.bus_voltage_min = 5.0F;
    cfg.rs_ohm = 0.5F;
    cfg.ld_h = 0.001F;
    cfg.lq_h = 0.001F;
    cfg.flux_wb = 0.01F;
    cfg.pole_pairs = 4.0F;
    cfg.dtc_enable = MC_TRUE;
    cfg.pwm_freq_hz = 20000.0F;

    memset(&in, 0, sizeof(in));
    in.current_raw.phase_a_raw = 100;
    in.current_raw.phase_b_raw = 100;
    in.bus_voltage_v = 24.0F;
    in.sin_theta = 0.0F;
    in.cos_theta = 1.0F;
    in.id_ref = 0.0F;
    in.iq_ref = 1.0F;
    in.dt_s = 0.00005F;
    in.elec_speed_rad_s = 0.0F;

    cfg.dtc_deadtime_ns = 200.0F;
    mc_pmsm_foc_init(&foc1, &cfg);
    mc_pmsm_foc_run(&foc1, &in, &out1);

    cfg.dtc_deadtime_ns = 2000.0F;
    mc_pmsm_foc_init(&foc2, &cfg);
    mc_pmsm_foc_run(&foc2, &in, &out2);

    TEST_ASSERT_EQUAL_INT(1U, out1.pwm_cmd.valid);
    TEST_ASSERT_EQUAL_INT(1U, out2.pwm_cmd.valid);
}

void test_mc_dtc_config_propagated_to_foc_cfg(void)
{
    mc_pmsm_foc_t foc;
    mc_pmsm_foc_cfg_t cfg = {0};

    cfg.id_pi_cfg = (mc_pi_cfg_t){0.5F, 50.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.iq_pi_cfg = (mc_pi_cfg_t){0.5F, 50.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.svpwm_cfg.modulation_limit = 0.95F;
    cfg.svpwm_cfg.duty_min = 0.0F;
    cfg.svpwm_cfg.duty_max = 1.0F;
    cfg.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.current_cfg.cfg.shunt2.offset_a = 0.0F;
    cfg.current_cfg.cfg.shunt2.scale_a = 1.0F;
    cfg.current_cfg.cfg.shunt2.offset_b = 0.0F;
    cfg.current_cfg.cfg.shunt2.scale_b = 1.0F;
    cfg.voltage_limit = 24.0F;
    cfg.bus_voltage_min = 5.0F;
    cfg.rs_ohm = 0.5F;
    cfg.ld_h = 0.001F;
    cfg.lq_h = 0.001F;
    cfg.flux_wb = 0.01F;
    cfg.pole_pairs = 4.0F;
    cfg.dtc_enable = MC_TRUE;
    cfg.dtc_deadtime_ns = 500.0F;
    cfg.pwm_freq_hz = 16000.0F;

    mc_pmsm_foc_init(&foc, &cfg);

    TEST_ASSERT_EQUAL_INT(MC_TRUE, foc.cfg.dtc_enable);
    TEST_ASSERT_TRUE(fabsf(foc.cfg.dtc_deadtime_ns - 500.0F) < 0.1F);
    TEST_ASSERT_TRUE(fabsf(foc.cfg.pwm_freq_hz - 16000.0F) < 0.1F);
}
