/** @file mc_debug_map.c @brief Variable map manager for debug subsystem */

#include "mc_debug.h"
#include "mc_types.h"
#include <string.h>

#ifndef MC_DEBUG_MAP_SIZE
#define MC_DEBUG_MAP_SIZE (32U)
#endif

static const mc_debug_var_t *g_var_table;
static uint8_t g_var_count;

void mc_debug_map_set_table(const mc_debug_var_t *table, uint8_t count)
{
    g_var_table = table;
    g_var_count = count;
}

const mc_debug_var_t *mc_debug_get_var_table(void)
{
    return g_var_table;
}

uint8_t mc_debug_get_var_count(void)
{
    return g_var_count;
}

void mc_debug_map_collect(uint32_t active_mask, uint32_t timestamp_us,
                           uint8_t *buf, uint16_t *buf_len)
{
    uint8_t var_count = 0U;
    uint16_t pos = MC_DEBUG_HEADER_SIZE;
    uint8_t i;

    memcpy(&buf[0], &timestamp_us, sizeof(uint32_t));
    buf[4] = 0U;

    for (i = 0U; i < g_var_count && i < MC_DEBUG_MAP_SIZE; i++)
    {
        if ((active_mask & (1UL << i)) == 0UL)
        {
            continue;
        }

        if (g_var_table == NULL) { break; }

        const mc_debug_var_t *v = &g_var_table[i];
        buf[pos++] = i;
        buf[pos++] = v->type;
        buf[pos++] = v->size_bytes;
        if (v->addr != NULL)
        {
            memcpy(&buf[pos], v->addr, v->size_bytes);
        }
        pos += v->size_bytes;
        var_count++;
    }

    buf[4] = var_count;
    *buf_len = pos;
}
