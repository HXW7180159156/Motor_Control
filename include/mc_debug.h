#ifndef MC_DEBUG_H
#define MC_DEBUG_H

/** @file mc_debug.h @brief Debug and calibration subsystem types and constants */

#include "mc_types.h"

/** @brief Variable type: IEEE 754 single-precision float */
#define MC_DEBUG_VAR_TYPE_FLOAT32   (0U)
/** @brief Variable type: signed 16-bit integer */
#define MC_DEBUG_VAR_TYPE_INT16     (1U)
/** @brief Variable type: unsigned 16-bit integer */
#define MC_DEBUG_VAR_TYPE_UINT16    (2U)
/** @brief Variable type: unsigned 8-bit integer */
#define MC_DEBUG_VAR_TYPE_UINT8     (3U)
/** @brief Variable type: unsigned 32-bit integer */
#define MC_DEBUG_VAR_TYPE_UINT32    (4U)
/** @brief Variable type: signed 32-bit integer */
#define MC_DEBUG_VAR_TYPE_INT32     (5U)

/** @brief Variable is readable (can be observed by host) */
#define MC_DEBUG_FLAG_READABLE       (0x01U)
/** @brief Variable is writable (can be calibrated by host) */
#define MC_DEBUG_FLAG_WRITABLE       (0x02U)
/** @brief Variable appears only in high-speed recorder mode */
#define MC_DEBUG_FLAG_RECORDER_ONLY  (0x04U)

/** @brief First magic byte for frame boundary detection */
#define MC_DEBUG_FRAME_MAGIC_0       (0xDAU)
/** @brief Second magic byte for frame boundary detection */
#define MC_DEBUG_FRAME_MAGIC_1       (0x1AU)
/** @brief Frame header size in bytes (2 magic + 2 length) */
#define MC_DEBUG_HEADER_SIZE         (4U)
/** @brief Maximum number of variables per data frame */
#define MC_DEBUG_MAX_VARS_PER_FRAME  (16U)

/** @brief FreeMASTER GET_INFO command: host queries variable list */
#define MC_DEBUG_FM_CMD_GET_INFO     (0x01U)
/** @brief FreeMASTER READ_VARS command: host reads selected variables */
#define MC_DEBUG_FM_CMD_READ_VARS    (0x03U)
/** @brief FreeMASTER WRITE_VAR command: host writes single variable */
#define MC_DEBUG_FM_CMD_WRITE_VAR    (0x04U)
/** @brief FreeMASTER SCOPE_START command: start periodic streaming */
#define MC_DEBUG_FM_CMD_SCOPE_START  (0x10U)
/** @brief FreeMASTER SCOPE_STOP command: stop periodic streaming */
#define MC_DEBUG_FM_CMD_SCOPE_STOP   (0x11U)
/** @brief FreeMASTER REC_START command: start high-speed recorder */
#define MC_DEBUG_FM_CMD_REC_START    (0x20U)
/** @brief FreeMASTER response frame command byte */
#define MC_DEBUG_FM_CMD_RESPONSE     (0x80U)

#ifndef MC_DEBUG_BUF_SIZE
/** @brief Default debug frame buffer size in bytes */
#define MC_DEBUG_BUF_SIZE            (1024U)
#endif

/**
 * @brief Debug variable registration entry
 *
 * Each entry maps a human-readable name to a typed memory address within
 * the motor control instance. Flags control read/write/recorder capabilities.
 */
typedef struct
{
    const char *name;       /**< Human-readable variable name (ASCIIZ) */
    void       *addr;       /**< Pointer to the variable in memory */
    uint8_t     type;       /**< Type code: MC_DEBUG_VAR_TYPE_* */
    uint8_t     size_bytes; /**< Size in bytes: 1, 2, or 4 */
    uint8_t     flags;      /**< Capability flags: MC_DEBUG_FLAG_* bitmask */
} mc_debug_var_t;

/** @brief Transport type identifiers for frame boundary strategies */
typedef enum
{
    MC_DEBUG_TRANSP_NONE = 0,   /**< No transport configured */
    MC_DEBUG_TRANSP_UART,       /**< UART: magic bytes + length framing */
    MC_DEBUG_TRANSP_CAN,        /**< CAN: first-frame total_len, multi-frame concat */
    MC_DEBUG_TRANSP_LIN         /**< LIN: same framing as UART, obeys schedule table */
} mc_debug_transp_type_t;

/**
 * @brief Transport layer instance state
 *
 * Manages frame boundary detection for inbound data and buffered outbound
 * transmission. Frame format: MAGIC0 MAGIC1 LEN_LO LEN_HI [payload].
 */
typedef struct
{
    mc_debug_transp_type_t type;             /**< Active transport type */
    mc_bool_t initialized;                   /**< Transport has been initialised */
    uint8_t  rx_buf[MC_DEBUG_BUF_SIZE];      /**< Receive buffer for incoming frames */
    uint16_t rx_len;                         /**< Bytes accumulated in rx_buf */
    uint8_t  tx_buf[MC_DEBUG_BUF_SIZE];      /**< Transmit buffer for outgoing frames */
    uint16_t tx_len;                         /**< Bytes queued in tx_buf */
    uint16_t tx_sent;                        /**< Bytes already flushed from tx_buf */
    void   (*tx_flush)(const uint8_t *data, uint16_t len); /**< Platform TX callback */
} mc_debug_transp_t;

/**
 * @brief Debug/calibration subsystem runtime state
 *
 * Tracks scope streaming mode, recorder state, and the active
 * variable mask selected by the host. Lifetime is managed as a
 * field within mc_instance_t.
 */
typedef struct
{
    mc_bool_t      scope_active;        /**< Scope periodic streaming is enabled */
    uint32_t       scope_period_us;     /**< Scope frame interval in microseconds */
    uint32_t       scope_last_us;       /**< Timestamp of last scope frame sent */
    mc_bool_t      recorder_active;     /**< Recorder high-speed capture is active */
    uint32_t       recorder_frame_count;/**< Frames captured in current recorder session */
    uint32_t       recorder_total_frames;/**< Total frames requested for recorder session */
    uint32_t       active_var_mask;     /**< Bitmask of variable slots selected by host */
    uint32_t       active_var_count;    /**< Count of currently active variable slots */
    mc_debug_transp_t transp;           /**< Transport layer state */
} mc_debug_t;

/**
 * @brief Register a read-only debug variable at compile time
 * @param name_  C string literal for the variable name.
 *   Range: any string literal; sent to host during GET_INFO handshake.
 * @param ctype_ C type of the variable.
 *   Range: mc_f32_t, uint32_t, int32_t, uint16_t, int16_t, uint8_t, mc_bool_t.
 * @param addr_  Address expression for the variable.
 *   Range: any lvalue expression that resolves to the variable's memory location.
 */
#define MC_DEBUG_VAR_RO(name_, ctype_, addr_) \
    { name_, (void*)(addr_), MC_DEBUG_TYPE_ID(ctype_), sizeof(ctype_), MC_DEBUG_FLAG_READABLE }

/**
 * @brief Register a read-write (calibratable) debug variable at compile time
 * @param name_  C string literal for the variable name.
 *   Range: any string literal.
 * @param ctype_ C type of the variable.
 *   Range: mc_f32_t, uint32_t, int32_t, uint16_t, int16_t, uint8_t, mc_bool_t.
 * @param addr_  Address expression for the variable.
 *   Range: any lvalue expression that resolves to the variable's memory location.
 */
#define MC_DEBUG_VAR_RW(name_, ctype_, addr_) \
    { name_, (void*)(addr_), MC_DEBUG_TYPE_ID(ctype_), sizeof(ctype_), MC_DEBUG_FLAG_READABLE | MC_DEBUG_FLAG_WRITABLE }

/**
 * @brief Resolve a C type to its debug type code at compile time
 * @param ctype_ The C type to resolve.
 *   Range: one of mc_f32_t, uint32_t, int32_t, uint16_t, int16_t, uint8_t.
 * @return Type code constant (MC_DEBUG_VAR_TYPE_*).
 *   Range: 0..5; defaults to 0 (FLOAT32) for unrecognised types.
 */
#define MC_DEBUG_TYPE_ID(ctype_) \
    ((sizeof(ctype_) == 4U && ((ctype_)(-1)) < (ctype_)0) ? MC_DEBUG_VAR_TYPE_INT32 : \
     (sizeof(ctype_) == 4U) ? MC_DEBUG_VAR_TYPE_UINT32 : \
     (sizeof(ctype_) == 2U && ((ctype_)(-1)) < (ctype_)0) ? MC_DEBUG_VAR_TYPE_INT16 : \
     (sizeof(ctype_) == 2U) ? MC_DEBUG_VAR_TYPE_UINT16 : \
     (sizeof(ctype_) == 1U && ((ctype_)(-1)) < (ctype_)0) ? MC_DEBUG_VAR_TYPE_UINT8 : \
     (sizeof(ctype_) == sizeof(mc_f32_t)) ? MC_DEBUG_VAR_TYPE_FLOAT32 : 0U)

#endif /* MC_DEBUG_H */
