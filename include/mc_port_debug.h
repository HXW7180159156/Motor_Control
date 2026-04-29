#ifndef MC_PORT_DEBUG_H
#define MC_PORT_DEBUG_H

/** @file mc_port_debug.h @brief Platform hooks for debug communication */

#include <stdint.h>

typedef struct
{
    void (*uart_tx)(const uint8_t *data, uint16_t len);
    void (*can_tx)(uint32_t id, const uint8_t *data, uint8_t len);
    void (*lin_tx)(const uint8_t *data, uint16_t len);
    uint16_t (*uart_rx_available)(void);
    uint8_t  (*uart_rx_read)(void);
} mc_debug_port_t;

#endif /* MC_PORT_DEBUG_H */
