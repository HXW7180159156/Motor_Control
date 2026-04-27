#ifndef MC_PORT_FAULT_H
#define MC_PORT_FAULT_H

/** @file mc_port_fault.h @brief Fault input port type definition. */

#include <stdint.h>

/** @brief Fault input from hardware or supervisor logic. */
typedef struct
{
    uint32_t fault_mask; /**< Bitmask of active fault signals. */
} mc_fault_input_t;

#endif /* MC_PORT_FAULT_H */
