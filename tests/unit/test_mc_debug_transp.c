#include "unity.h"
#include "mc_debug.h"
#include "mc_port_debug.h"
#include <string.h>

extern void mc_debug_transp_reset_rx(mc_debug_transp_t *t);
extern mc_bool_t mc_debug_transp_push_byte(mc_debug_transp_t *t, uint8_t byte);
extern void mc_debug_transp_send(mc_debug_transp_t *t, const uint8_t *data, uint16_t len);

static uint8_t g_tx_buf[256];
static uint16_t g_tx_len;
static void stub_flush(const uint8_t *data, uint16_t len)
{
    memcpy(g_tx_buf, data, len); g_tx_len = len;
}

void test_mc_debug_transp_valid_frame_detected(void)
{
    mc_debug_transp_t t;
    memset(&t, 0, sizeof(t));
    t.tx_flush = stub_flush;

    uint8_t frame[] = { 0xDA, 0x1A, 0x08, 0x00, 'H','e','l','l','o','!','!','!' };
    mc_bool_t ready = MC_FALSE;
    int i;
    for (i = 0; i < (int)sizeof(frame); i++)
    {
        ready = mc_debug_transp_push_byte(&t, frame[i]);
    }
    TEST_ASSERT_TRUE(ready == MC_TRUE);
    TEST_ASSERT_EQUAL_INT(8U, t.rx_len);
}

void test_mc_debug_transp_bad_magic_rejected(void)
{
    mc_debug_transp_t t;
    memset(&t, 0, sizeof(t));

    mc_debug_transp_push_byte(&t, 0xAA);
    mc_debug_transp_push_byte(&t, 0xBB);
    TEST_ASSERT_EQUAL_INT(0U, t.rx_len);
}

void test_mc_debug_transp_good_magic_bad_len_resets(void)
{
    mc_debug_transp_t t;
    memset(&t, 0, sizeof(t));

    uint8_t frame[] = { 0xDA, 0x1A, 0xFF, 0xFF, 0x00 };
    int i;
    for (i = 0; i < (int)sizeof(frame); i++)
    {
        mc_debug_transp_push_byte(&t, frame[i]);
    }
    TEST_ASSERT_EQUAL_INT(0U, t.rx_len);
}

void test_mc_debug_transp_send_calls_flush(void)
{
    mc_debug_transp_t t;
    memset(&t, 0, sizeof(t));
    t.tx_flush = stub_flush;
    g_tx_len = 0U;

    uint8_t payload[] = { 0xDA, 0x1A, 0x05, 0x00, 0x01 };
    mc_debug_transp_send(&t, payload, 5U);

    TEST_ASSERT_EQUAL_INT(5U, g_tx_len);
    TEST_ASSERT_EQUAL_INT(0xDA, g_tx_buf[0]);
}
