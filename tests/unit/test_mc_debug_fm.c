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
