#ifndef MC_PORT_RESOLVER_H
#define MC_PORT_RESOLVER_H

/** @file mc_port_resolver.h @brief Resolver port types and interface. */

#include "mc_types.h"

/** @brief Raw resolver sample data. */
typedef struct
{
    int16_t sin_raw; /**< Raw SIN channel sample. */
    int16_t cos_raw; /**< Raw COS channel sample. */
    uint16_t excitation_feedback_raw; /**< Raw excitation feedback sample. */
} mc_resolver_raw_t;

/** @brief Platform resolver interface callbacks. */
typedef struct
{
    void (*excitation_set)(int16_t amplitude, uint16_t frequency_hz); /**< Set resolver excitation amplitude and frequency. */
    void (*sample_fetch)(mc_resolver_raw_t *raw); /**< Fetch raw SIN/COS/excitation samples. */
} mc_resolver_port_t;

#endif /* MC_PORT_RESOLVER_H */
