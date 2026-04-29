/** @file mc_debug_map.c @brief Variable map manager for debug subsystem */

#include "mc_debug.h"
#include "mc_api.h"
#include "mc_types.h"
#include <string.h>

#define MC_DEBUG_INST_TABLE_SIZE (24U)

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

/* === Real variable table built at runtime === */

static mc_debug_var_t g_inst_var_table[MC_DEBUG_INST_TABLE_SIZE];

void mc_debug_set_instance(void *inst)
{
    mc_instance_t *p = (mc_instance_t *)inst;
    uint8_t i = 0U;

    if (p == NULL) { return; }

    g_inst_var_table[i++] = (mc_debug_var_t){ "id_ref",        &p->id_ref,        MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "iq_ref",        &p->iq_ref,        MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "speed_ref_rpm", &p->speed_ref_rpm, MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "v_d",           &p->foc.pmsm_foc.v_dq.d,        MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "v_q",           &p->foc.pmsm_foc.v_dq.q,        MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "i_d",           &p->foc.pmsm_foc.i_dq.d,        MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "i_q",           &p->foc.pmsm_foc.i_dq.q,        MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "duty_a",        &p->foc_last_output.pwm_cmd.duty_a, MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "duty_b",        &p->foc_last_output.pwm_cmd.duty_b, MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "duty_c",        &p->foc_last_output.pwm_cmd.duty_c, MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "active_fault",  &p->diag.active_fault,          MC_DEBUG_VAR_TYPE_UINT32,  4U, MC_DEBUG_FLAG_READABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "active_warn",   &p->diag.active_warning,        MC_DEBUG_VAR_TYPE_UINT32,  4U, MC_DEBUG_FLAG_READABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "id_kp",         &p->cfg.foc.id_pi_cfg.kp,      MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "id_ki",         &p->cfg.foc.id_pi_cfg.ki,      MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "iq_kp",         &p->cfg.foc.iq_pi_cfg.kp,      MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "iq_ki",         &p->cfg.foc.iq_pi_cfg.ki,      MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "speed_kp",      &p->cfg.foc.speed_pi_cfg.kp,   MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "speed_ki",      &p->cfg.foc.speed_pi_cfg.ki,   MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "fw_enable",     &p->cfg.foc.fw_enable,         MC_DEBUG_VAR_TYPE_UINT8,   1U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "fw_min_id",     &p->cfg.foc.fw_min_id,         MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "voltage_limit", &p->cfg.foc.voltage_limit,     MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "iq_limit",      &p->cfg.foc.iq_limit,          MC_DEBUG_VAR_TYPE_FLOAT32, 4U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };
    g_inst_var_table[i++] = (mc_debug_var_t){ "dtc_enable",    &p->cfg.foc.dtc_enable,        MC_DEBUG_VAR_TYPE_UINT8,   1U, MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE };

    mc_debug_map_set_table(g_inst_var_table, i);
}

/* === Collect: packs selected var slots into frame buffer === */

void mc_debug_map_collect(uint32_t active_mask, uint32_t timestamp_us,
                           uint8_t *buf, uint16_t *buf_len)
{
    uint8_t var_count = 0U;
    uint16_t pos = MC_DEBUG_HEADER_SIZE;
    uint8_t i;

    memcpy(&buf[0], &timestamp_us, sizeof(uint32_t));
    buf[4] = 0U;

    for (i = 0U; i < g_var_count && i < 32U; i++)
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
