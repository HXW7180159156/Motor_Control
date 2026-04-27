/** @file mc_port_time.h @brief Timestamp sample type used by time-related ports */

#ifndef MC_PORT_TIME_H
#define MC_PORT_TIME_H

#include <stdint.h>

/** @brief Captured timestamp sample in microseconds */
typedef struct
{
    uint32_t timestamp_us;
} mc_time_sample_t;

#endif /* MC_PORT_TIME_H */
