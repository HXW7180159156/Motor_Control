#ifndef MC_DEBUG_H
#define MC_DEBUG_H

/** @file mc_debug.h @brief Debug and calibration subsystem types */

#include "mc_types.h"

#define MC_DEBUG_VAR_TYPE_FLOAT32   (0U)
#define MC_DEBUG_VAR_TYPE_INT16     (1U)
#define MC_DEBUG_VAR_TYPE_UINT16    (2U)
#define MC_DEBUG_VAR_TYPE_UINT8     (3U)
#define MC_DEBUG_VAR_TYPE_UINT32    (4U)
#define MC_DEBUG_VAR_TYPE_INT32     (5U)

#define MC_DEBUG_FLAG_READABLE       (0x01U)
#define MC_DEBUG_FLAG_WRITABLE       (0x02U)
#define MC_DEBUG_FLAG_RECORDER_ONLY  (0x04U)

#define MC_DEBUG_FRAME_MAGIC_0       (0xDAU)
#define MC_DEBUG_FRAME_MAGIC_1       (0x1AU)
#define MC_DEBUG_HEADER_SIZE         (4U)
#define MC_DEBUG_MAX_VARS_PER_FRAME  (16U)

#ifndef MC_DEBUG_BUF_SIZE
#define MC_DEBUG_BUF_SIZE            (1024U)
#endif

typedef struct
{
    const char *name;
    void       *addr;
    uint8_t     type;
    uint8_t     size_bytes;
    uint8_t     flags;
} mc_debug_var_t;

typedef enum
{
    MC_DEBUG_TRANSP_NONE = 0,
    MC_DEBUG_TRANSP_UART,
    MC_DEBUG_TRANSP_CAN,
    MC_DEBUG_TRANSP_LIN
} mc_debug_transp_type_t;

typedef struct
{
    mc_debug_transp_type_t type;
    mc_bool_t initialized;
    uint8_t  rx_buf[MC_DEBUG_BUF_SIZE];
    uint16_t rx_len;
    uint8_t  tx_buf[MC_DEBUG_BUF_SIZE];
    uint16_t tx_len;
    uint16_t tx_sent;
    void   (*tx_flush)(const uint8_t *data, uint16_t len);
} mc_debug_transp_t;

typedef struct
{
    mc_bool_t      scope_active;
    uint32_t       scope_period_us;
    uint32_t       scope_last_us;
    mc_bool_t      recorder_active;
    uint32_t       recorder_frame_count;
    uint32_t       recorder_total_frames;
    uint32_t       active_var_mask;
    uint32_t       active_var_count;
    mc_debug_transp_t transp;
} mc_debug_t;

#define MC_DEBUG_VAR_RO(name_, ctype_, addr_) \
    { name_, (void*)(addr_), MC_DEBUG_TYPE_ID(ctype_), sizeof(ctype_), MC_DEBUG_FLAG_READABLE }

#define MC_DEBUG_VAR_RW(name_, ctype_, addr_) \
    { name_, (void*)(addr_), MC_DEBUG_TYPE_ID(ctype_), sizeof(ctype_), MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE }

#define MC_DEBUG_TYPE_ID(ctype_) \
    ((sizeof(ctype_) == 4U && ((ctype_)(-1)) < (ctype_)0) ? MC_DEBUG_VAR_TYPE_INT32 : \
     (sizeof(ctype_) == 4U) ? MC_DEBUG_VAR_TYPE_UINT32 : \
     (sizeof(ctype_) == 2U && ((ctype_)(-1)) < (ctype_)0) ? MC_DEBUG_VAR_TYPE_INT16 : \
     (sizeof(ctype_) == 2U) ? MC_DEBUG_VAR_TYPE_UINT16 : \
     (sizeof(ctype_) == 1U && ((ctype_)(-1)) < (ctype_)0) ? MC_DEBUG_VAR_TYPE_UINT8 : \
     (sizeof(ctype_) == sizeof(mc_f32_t)) ? MC_DEBUG_VAR_TYPE_FLOAT32 : 0U)

#endif /* MC_DEBUG_H */
