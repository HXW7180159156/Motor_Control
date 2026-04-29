#ifndef MC_PORT_DEBUG_H
#define MC_PORT_DEBUG_H

/** @file mc_port_debug.h @brief Platform hooks for debug communication */

#include <stdint.h>

/**
 * @brief Platform callbacks for debug transport physical I/O
 *
 * Each hook is optional (NULL-checked). If a compiled-in transport has its
 * corresponding hook set to NULL, that transport is silently disabled at runtime.
 *
 * UART hooks are required for FreeMASTER communication.
 * CAN and LIN hooks are reserved for future protocol expansion.
 */
typedef struct
{
    /**
     * @brief Transmit data over UART
     * @param data Buffer to transmit.
     *   Range: non-NULL pointer to readable buffer.
     * @param len  Number of bytes to transmit.
     *   Range: [1, MC_DEBUG_BUF_SIZE].
     */
    void (*uart_tx)(const uint8_t *data, uint16_t len);

    /**
     * @brief Transmit data over CAN
     * @param id   CAN message ID.
     *   Range: standard (11-bit) or extended (29-bit).
     * @param data Buffer to transmit.
     *   Range: non-NULL pointer to readable buffer.
     * @param len  Number of bytes to transmit (max 8 per CAN frame).
     *   Range: [1, 8].
     */
    void (*can_tx)(uint32_t id, const uint8_t *data, uint8_t len);

    /**
     * @brief Transmit data over LIN
     * @param data Buffer to transmit.
     *   Range: non-NULL pointer to readable buffer.
     * @param len  Number of bytes to transmit.
     *   Range: [1, MC_DEBUG_BUF_SIZE].
     */
    void (*lin_tx)(const uint8_t *data, uint16_t len);

    /**
     * @brief Check number of bytes available in UART RX buffer
     * @return Number of available bytes.
     *   Range: [0, UART RX FIFO depth].
     */
    uint16_t (*uart_rx_available)(void);

    /**
     * @brief Read one byte from UART RX buffer
     * @return The received byte.
     *   Range: [0, 255].
     */
    uint8_t  (*uart_rx_read)(void);
} mc_debug_port_t;

#endif /* MC_PORT_DEBUG_H */
