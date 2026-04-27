#ifndef MC_PLATFORM_BENCH_H
#define MC_PLATFORM_BENCH_H

#include <stdint.h>

typedef struct
{
    uint32_t cycle_count;
    uint32_t max_cycle_count;
} mc_bench_sample_t;

#endif /* MC_PLATFORM_BENCH_H */
