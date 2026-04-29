#include "mc_debug.h"
#include "mc_types.h"
#include <string.h>

extern const mc_debug_var_t *mc_debug_get_var_table(void);
extern void mc_debug_map_collect(uint32_t active_mask, uint32_t timestamp_us,
                                  uint8_t *buf, uint16_t *buf_len);

void mc_debug_fm_init(mc_debug_t *dbg)
{
    if (dbg == NULL) { return; }
    (void)dbg;
}

void mc_debug_fm_poll(mc_debug_t *dbg)
{
    (void)dbg;
}
