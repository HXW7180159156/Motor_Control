#include "unity.h"
#include "mc_debug.h"
#include <math.h>
#include <string.h>

static mc_f32_t g_fm_val;
static const mc_debug_var_t g_fm_table[] = {
    MC_DEBUG_VAR_RW("calib", mc_f32_t, &g_fm_val),
};

extern void mc_debug_map_set_table(const mc_debug_var_t *table, uint8_t count);
extern void mc_debug_fm_init(mc_debug_t *dbg, void (*tx_flush)(const uint8_t *data, uint16_t len));
extern void mc_debug_fm_poll(mc_debug_t *dbg);
extern void mc_debug_fm_feed_rx(mc_debug_t *dbg, const uint8_t *frame, uint16_t len);

static uint8_t g_fm_tx[256];
static uint16_t g_fm_tx_len;
static void fm_stub_tx(const uint8_t *data, uint16_t len)
{
    memcpy(g_fm_tx, data, len); g_fm_tx_len = len;
}

void test_mc_debug_fm_get_info_responds(void)
{
    mc_debug_t dbg;
    mc_debug_map_set_table(g_fm_table, 1U);
    mc_debug_fm_init(&dbg, fm_stub_tx);

    uint8_t cmd[] = { 0xDA, 0x1A, 0x05, 0x00, 0x01 };
    g_fm_tx_len = 0U;
    mc_debug_fm_feed_rx(&dbg, cmd, sizeof(cmd));
    mc_debug_fm_poll(&dbg);

    TEST_ASSERT_TRUE(g_fm_tx_len > 0U);
    TEST_ASSERT_EQUAL_INT(MC_DEBUG_FM_CMD_RESPONSE, g_fm_tx[4]);
}

void test_mc_debug_fm_unknown_cmd_ignored(void)
{
    mc_debug_t dbg;
    mc_debug_map_set_table(g_fm_table, 1U);
    mc_debug_fm_init(&dbg, fm_stub_tx);

    uint8_t cmd[] = { 0xDA, 0x1A, 0x05, 0x00, 0xFF };
    g_fm_tx_len = 0U;
    mc_debug_fm_feed_rx(&dbg, cmd, sizeof(cmd));
    mc_debug_fm_poll(&dbg);

    TEST_ASSERT_EQUAL_INT(0U, g_fm_tx_len);
}

void test_mc_debug_fm_write_var_updates_value(void)
{
    mc_debug_t dbg;
    mc_debug_map_set_table(g_fm_table, 1U);
    mc_debug_fm_init(&dbg, fm_stub_tx);

    g_fm_val = 0.0F;
    mc_f32_t new_val = 2.5F;
    uint8_t cmd[14] = { 0xDA, 0x1A, 0x0E, 0x00, 0x04, 0x00, 0x00, 0x04 };
    memcpy(&cmd[8], &new_val, sizeof(mc_f32_t));
    cmd[12] = 0x00; cmd[13] = 0x00;

    g_fm_tx_len = 0U;
    mc_debug_fm_feed_rx(&dbg, cmd, sizeof(cmd));
    mc_debug_fm_poll(&dbg);

    TEST_ASSERT_TRUE(g_fm_tx_len > 0U);
    TEST_ASSERT_TRUE(fabsf(g_fm_val - 2.5F) < 0.01F);
}

void test_mc_debug_fm_scope_starts_streaming(void)
{
    mc_debug_t dbg;
    mc_debug_map_set_table(g_fm_table, 1U);
    mc_debug_fm_init(&dbg, fm_stub_tx);
    dbg.active_var_mask = 0x01U;
    dbg.scope_active = MC_TRUE;
    dbg.scope_period_us = 100U;

    g_fm_tx_len = 0U;
    mc_debug_fm_poll(&dbg);

    TEST_ASSERT_TRUE(g_fm_tx_len > 0U);
}

/* === End-to-end tests with real mc_instance_t === */

#include "mc_api.h"

extern const mc_debug_var_t *mc_debug_get_var_table(void);
extern uint8_t mc_debug_get_var_count(void);

void test_mc_debug_e2e_init_populates_var_table(void)
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
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 100.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 100.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.1F, 10.0F, -0.5F, 0.5F, -5.0F, 5.0F};
    cfg.foc.svpwm_cfg.modulation_limit = 0.95F;
    cfg.foc.svpwm_cfg.duty_min = 0.0F; cfg.foc.svpwm_cfg.duty_max = 1.0F;
    cfg.foc.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.foc.current_cfg.cfg.shunt2.offset_a = 0.0F; cfg.foc.current_cfg.cfg.shunt2.scale_a = 1.0F;
    cfg.foc.current_cfg.cfg.shunt2.offset_b = 0.0F; cfg.foc.current_cfg.cfg.shunt2.scale_b = 1.0F;
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 5.0F; cfg.foc.voltage_limit = 24.0F;
    cfg.foc.bus_voltage_scale = 1.0F; cfg.foc.bus_voltage_min = 5.0F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 4096U; cfg.sensor.encoder_cfg.pole_pairs = 4.0F;

    mc_init(&inst, &cfg);
    TEST_ASSERT_TRUE(mc_debug_get_var_count() > 10U);
}

void test_mc_debug_e2e_fast_step_poll_does_not_crash(void)
{
    mc_instance_t inst;
    mc_fast_input_t in;
    mc_fast_output_t out;
    mc_system_cfg_t cfg = {0};

    cfg.motor.pole_pairs = 4U; cfg.motor.rs_ohm = 0.5F; cfg.motor.ld_h = 0.001F;
    cfg.motor.lq_h = 0.001F; cfg.motor.flux_wb = 0.01F;
    cfg.limits.bus_voltage_max_v = 24.0F; cfg.limits.phase_current_max_a = 10.0F;
    cfg.limits.speed_max_rpm = 5000.0F;
    cfg.control.pwm_frequency_hz = 20000U; cfg.control.numeric_mode = MC_NUMERIC_FLOAT32;
    cfg.control.current_loop_dt_s = 0.00005F;
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){1.0F, 100.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){1.0F, 100.0F, -0.5F, 0.5F, -1.0F, 1.0F};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.1F, 10.0F, -0.5F, 0.5F, -5.0F, 5.0F};
    cfg.foc.svpwm_cfg.modulation_limit = 0.95F; cfg.foc.svpwm_cfg.duty_min = 0.0F; cfg.foc.svpwm_cfg.duty_max = 1.0F;
    cfg.foc.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.foc.current_cfg.cfg.shunt2.offset_a = 0.0F; cfg.foc.current_cfg.cfg.shunt2.scale_a = 1.0F;
    cfg.foc.current_cfg.cfg.shunt2.offset_b = 0.0F; cfg.foc.current_cfg.cfg.shunt2.scale_b = 1.0F;
    cfg.foc.speed_loop_dt_s = 0.001F;
    cfg.foc.iq_limit = 5.0F; cfg.foc.voltage_limit = 24.0F;
    cfg.foc.bus_voltage_scale = 1.0F; cfg.foc.bus_voltage_min = 5.0F;
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 4096U; cfg.sensor.encoder_cfg.pole_pairs = 4.0F;

    mc_init(&inst, &cfg);
    mc_start(&inst);
    mc_set_current_ref_dq(&inst, 0.0F, 0.5F);

    memset(&in, 0, sizeof(in));
    in.adc_raw.phase_a_raw = 500; in.adc_raw.phase_b_raw = 500;
    in.timestamp_us = 1000U;

    mc_fast_step(&inst, &in, &out);
    TEST_ASSERT_EQUAL_INT(1U, out.pwm_cmd.valid);
}
