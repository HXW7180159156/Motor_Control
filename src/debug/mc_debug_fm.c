/** @file mc_debug_fm.c @brief FreeMASTER protocol engine */

#include "mc_debug.h"
#include "mc_types.h"
#include <string.h>

extern const mc_debug_var_t *mc_debug_get_var_table(void);
extern uint8_t mc_debug_get_var_count(void);
extern void mc_debug_map_collect(uint32_t active_mask, uint32_t timestamp_us,
                                  uint8_t *buf, uint16_t *buf_len);
extern mc_bool_t mc_debug_transp_push_byte(mc_debug_transp_t *t, uint8_t byte);
extern void mc_debug_transp_reset_rx(mc_debug_transp_t *t);

static void fm_send_response(mc_debug_t *dbg, uint8_t cmd, const uint8_t *payload, uint16_t pay_len)
{
    uint8_t hdr[MC_DEBUG_HEADER_SIZE + 1U];
    uint16_t total = (uint16_t)(MC_DEBUG_HEADER_SIZE + 1U + pay_len);

    hdr[0] = MC_DEBUG_FRAME_MAGIC_0;
    hdr[1] = MC_DEBUG_FRAME_MAGIC_1;
    hdr[2] = (uint8_t)(total & 0xFFU);
    hdr[3] = (uint8_t)((total >> 8) & 0xFFU);
    hdr[4] = cmd;

    if (dbg->transp.tx_flush != NULL)
    {
        dbg->transp.tx_flush(hdr, MC_DEBUG_HEADER_SIZE + 1U);
        if (pay_len > 0U && payload != NULL)
        {
            dbg->transp.tx_flush(payload, pay_len);
        }
    }
}

void mc_debug_fm_init(mc_debug_t *dbg, void (*tx_flush)(const uint8_t *data, uint16_t len))
{
    if (dbg == NULL) { return; }
    memset(dbg, 0, sizeof(*dbg));
    dbg->transp.tx_flush = tx_flush;
    dbg->transp.initialized = MC_TRUE;
}

void mc_debug_fm_feed_rx(mc_debug_t *dbg, const uint8_t *frame, uint16_t len)
{
    uint16_t i;
    if ((dbg == NULL) || (frame == NULL)) { return; }
    for (i = 0U; i < len; i++)
    {
        mc_bool_t ready = mc_debug_transp_push_byte(&dbg->transp, frame[i]);
        if (ready != MC_FALSE) { break; }
    }
}

void mc_debug_fm_poll(mc_debug_t *dbg)
{
    uint8_t cmd;

    if (dbg == NULL) { return; }

    if (dbg->transp.rx_len >= MC_DEBUG_HEADER_SIZE + 1U)
    {
        cmd = dbg->transp.rx_buf[4];
        switch (cmd)
        {
        case MC_DEBUG_FM_CMD_GET_INFO:
            {
                uint8_t buf[128];
                uint8_t vc = mc_debug_get_var_count();
                uint16_t rlen = 3U;
                uint8_t j;
                buf[0] = 0x01U; buf[1] = 0x00U; buf[2] = vc;
                for (j = 0U; j < vc; j++)
                {
                    uint8_t nl = (uint8_t)strlen(mc_debug_get_var_table()[j].name);
                    if (rlen + nl + 2U > sizeof(buf)) { break; }
                    buf[rlen++] = (uint8_t)j;
                    buf[rlen++] = mc_debug_get_var_table()[j].flags;
                    buf[rlen++] = nl;
                    memcpy(&buf[rlen], mc_debug_get_var_table()[j].name, nl);
                    rlen += nl;
                }
                fm_send_response(dbg, MC_DEBUG_FM_CMD_RESPONSE, buf, rlen);
            }
            break;

        case MC_DEBUG_FM_CMD_READ_VARS:
            {
                uint32_t mask = 0U;
                if (dbg->transp.rx_len >= 9U)
                {
                    mask = (uint32_t)dbg->transp.rx_buf[5]
                         | ((uint32_t)dbg->transp.rx_buf[6] << 8)
                         | ((uint32_t)dbg->transp.rx_buf[7] << 16)
                         | ((uint32_t)dbg->transp.rx_buf[8] << 24);
                }
                dbg->active_var_mask = mask;
                uint8_t buf[256];
                uint16_t blen = 0U;
                mc_debug_map_collect(mask, 0U, buf, &blen);
                fm_send_response(dbg, MC_DEBUG_FM_CMD_RESPONSE, buf, blen);
            }
            break;

        case MC_DEBUG_FM_CMD_WRITE_VAR:
            {
                if (dbg->transp.rx_len >= 13U)
                {
                    uint8_t vi = dbg->transp.rx_buf[5];
                    if (vi < mc_debug_get_var_count() &&
                        (mc_debug_get_var_table()[vi].flags & MC_DEBUG_FLAG_WRITABLE))
                    {
                        memcpy(mc_debug_get_var_table()[vi].addr,
                               &dbg->transp.rx_buf[8],
                               mc_debug_get_var_table()[vi].size_bytes);
                    }
                }
                fm_send_response(dbg, MC_DEBUG_FM_CMD_RESPONSE, (const uint8_t *)"OK", 2U);
            }
            break;

        case MC_DEBUG_FM_CMD_SCOPE_START:
            dbg->scope_active = MC_TRUE;
            dbg->scope_period_us = 1000U;
            dbg->scope_last_us = 0U;
            fm_send_response(dbg, MC_DEBUG_FM_CMD_RESPONSE, (const uint8_t *)"OK", 2U);
            break;

        case MC_DEBUG_FM_CMD_SCOPE_STOP:
            dbg->scope_active = MC_FALSE;
            fm_send_response(dbg, MC_DEBUG_FM_CMD_RESPONSE, (const uint8_t *)"OK", 2U);
            break;

        case MC_DEBUG_FM_CMD_REC_START:
            dbg->recorder_active = MC_TRUE;
            dbg->recorder_total_frames = 100U;
            dbg->recorder_frame_count = 0U;
            fm_send_response(dbg, MC_DEBUG_FM_CMD_RESPONSE, (const uint8_t *)"OK", 2U);
            break;

        default:
            break;
        }
        mc_debug_transp_reset_rx(&dbg->transp);
    }

    if (dbg->scope_active != MC_FALSE && dbg->active_var_mask != 0U)
    {
        uint8_t sbuf[256];
        uint16_t slen = 0U;
        mc_debug_map_collect(dbg->active_var_mask, dbg->scope_last_us, sbuf, &slen);
        mc_debug_transp_send(&dbg->transp, sbuf, slen);
        dbg->scope_last_us += dbg->scope_period_us;
    }
}
