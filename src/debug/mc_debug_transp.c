/** @file mc_debug_transp.c @brief Transport abstraction for debug subsystem */

#include "mc_debug.h"
#include "mc_port_debug.h"
#include "mc_types.h"
#include <string.h>

void mc_debug_transp_init(mc_debug_transp_t *t, const mc_debug_port_t *port,
                           mc_debug_transp_type_t type)
{
    if (t == NULL) { return; }
    memset(t, 0, sizeof(*t));
    t->type = type;
    t->initialized = MC_TRUE;
    (void)port;
}

void mc_debug_transp_reset_rx(mc_debug_transp_t *t)
{
    if (t == NULL) { return; }
    t->rx_len = 0U;
}

mc_bool_t mc_debug_transp_push_byte(mc_debug_transp_t *t, uint8_t byte)
{
    uint16_t frame_len;

    if (t == NULL) { return MC_FALSE; }

    if (t->rx_len == 0U)
    {
        if (byte != MC_DEBUG_FRAME_MAGIC_0) { return MC_FALSE; }
        t->rx_buf[t->rx_len++] = byte;
        return MC_FALSE;
    }

    if (t->rx_len == 1U)
    {
        if (byte != MC_DEBUG_FRAME_MAGIC_1) { t->rx_len = 0U; return MC_FALSE; }
        t->rx_buf[t->rx_len++] = byte;
        return MC_FALSE;
    }

    if (t->rx_len < MC_DEBUG_HEADER_SIZE)
    {
        t->rx_buf[t->rx_len++] = byte;
        return MC_FALSE;
    }

    if (t->rx_len == MC_DEBUG_HEADER_SIZE)
    {
        frame_len = (uint16_t)t->rx_buf[2] | ((uint16_t)t->rx_buf[3] << 8);
        if (frame_len < MC_DEBUG_HEADER_SIZE || frame_len > MC_DEBUG_BUF_SIZE)
        {
            t->rx_len = 0U;
            return MC_FALSE;
        }
    }

    t->rx_buf[t->rx_len++] = byte;
    if (t->rx_len >= MC_DEBUG_HEADER_SIZE)
    {
        frame_len = (uint16_t)t->rx_buf[2] | ((uint16_t)t->rx_buf[3] << 8);
        if (t->rx_len >= frame_len)
        {
            return MC_TRUE;
        }
    }
    return MC_FALSE;
}

void mc_debug_transp_send(mc_debug_transp_t *t, const uint8_t *data, uint16_t len)
{
    if ((t == NULL) || (data == NULL) || (len == 0U)) { return; }
    if (len > MC_DEBUG_BUF_SIZE) { return; }
    memcpy(t->tx_buf, data, len);
    t->tx_len = len;
    t->tx_sent = 0U;
    if (t->tx_flush != NULL)
    {
        t->tx_flush(t->tx_buf, t->tx_len);
    }
}
