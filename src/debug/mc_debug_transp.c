/**
 * @file mc_debug_transp.c
 * @brief Transport abstraction layer for the debug subsystem
 *
 * Provides frame boundary detection via magic-byte sequence and length
 * field, buffered outbound transmission with callback flush, and RX state
 * machine reset. Supports UART (magic+length), CAN (multi-frame), and
 * LIN (same as UART) transport types.
 */

#include "mc_debug.h"
#include "mc_port_debug.h"
#include "mc_types.h"
#include <string.h>

/**
 * @brief Initialise a transport instance
 *
 * @param[out] t Transport instance to initialise.
 *   Range: non-NULL pointer to writable mc_debug_transp_t storage.
 * @param[in] port Platform debug port for I/O callbacks.
 *   Range: non-NULL pointer to mc_debug_port_t, or NULL (send is silently skipped).
 * @param[in] type Transport type selection.
 *   Range: one of MC_DEBUG_TRANSP_UART, CAN, LIN, or NONE.
 * @return None.
 *   Range: not applicable.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different t object.
 */
void mc_debug_transp_init(mc_debug_transp_t *t, const mc_debug_port_t *port,
                           mc_debug_transp_type_t type)
{
    if (t == NULL) { return; }
    memset(t, 0, sizeof(*t));
    t->type = type;
    t->initialized = MC_TRUE;
    (void)port;
}

/**
 * @brief Reset the receive buffer state machine
 *
 * Clears accumulated byte count so the transport can start detecting a
 * new frame from scratch. Does not clear the buffer contents.
 *
 * @param[in,out] t Transport instance to reset.
 *   Range: non-NULL pointer to writable mc_debug_transp_t storage.
 * @return None.
 *   Range: not applicable.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different t object.
 */
void mc_debug_transp_reset_rx(mc_debug_transp_t *t)
{
    if (t == NULL) { return; }
    t->rx_len = 0U;
}

/**
 * @brief Push one received byte into the frame-detection state machine
 *
 * Implements the stateful magic-byte + length parsing for inbound data.
 * State transitions:
 *   rx_len=0: expect MAGIC_0, else reject
 *   rx_len=1: expect MAGIC_1, else reset
 *   rx_len=2: store LEN_LO
 *   rx_len=3: store LEN_HI, validate length (must be >= HEADER_SIZE and <= BUF_SIZE)
 *   rx_len>=4: accumulate payload until len reached, then return MC_TRUE
 *
 * @param[in,out] t Transport instance state machine.
 *   Range: non-NULL pointer to writable mc_debug_transp_t storage.
 * @param[in] byte The next received byte.
 *   Range: [0, 255].
 * @return Frame completion flag.
 *   Range: MC_TRUE when a complete frame has been accumulated and is ready
 *          for processing; MC_FALSE otherwise (including NULL input).
 * @par Sync/Async
 *   Synchronous. Designed to be called from ISR (UART RX interrupt).
 * @par Reentrancy
 *   Not reentrant for concurrent calls to the same t from different ISR contexts.
 *   Reentrant when each concurrent call uses a different t object.
 */
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

/**
 * @brief Send a data buffer through the transport's TX flush callback
 *
 * Copies data into the internal TX buffer and calls the platform-level
 * tx_flush callback if registered.
 *
 * @param[in,out] t Transport instance.
 *   Range: non-NULL pointer to writable mc_debug_transp_t storage.
 * @param[in] data Pointer to data to send.
 *   Range: non-NULL pointer to readable buffer of len bytes.
 * @param[in] len Number of bytes to send.
 *   Range: [1, MC_DEBUG_BUF_SIZE]; values exceeding the buffer size are skipped.
 * @return None.
 *   Range: not applicable.
 * @par Sync/Async
 *   Synchronous. The tx_flush callback may use DMA or interrupts.
 * @par Reentrancy
 *   Not reentrant for concurrent calls to the same t.
 *   Reentrant when each concurrent call uses a different t object.
 */
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
