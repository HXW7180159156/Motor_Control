#ifndef MC_CFG_H
#define MC_CFG_H

/** @file mc_cfg.h @brief Build configuration macro flags */

/** @brief Enable BLDC motor support */
#define MC_CFG_ENABLE_BLDC            (1U)
/** @brief Enable PMSM motor support */
#define MC_CFG_ENABLE_PMSM            (1U)
/** @brief Enable float32 arithmetic path */
#define MC_CFG_ENABLE_FLOAT32         (1U)
/** @brief Enable Q31 fixed-point arithmetic path */
#define MC_CFG_ENABLE_Q31             (1U)
/** @brief Enable single-shunt current sensing */
#define MC_CFG_ENABLE_1SHUNT          (1U)
/** @brief Enable dual-shunt current sensing */
#define MC_CFG_ENABLE_2SHUNT          (1U)
/** @brief Enable triple-shunt current sensing */
#define MC_CFG_ENABLE_3SHUNT          (1U)
/** @brief Enable Hall sensor position feedback */
#define MC_CFG_ENABLE_HALL            (1U)
/** @brief Enable encoder position feedback */
#define MC_CFG_ENABLE_ENCODER         (1U)
/** @brief Enable resolver position feedback */
#define MC_CFG_ENABLE_RESOLVER        (1U)
/** @brief Enable sensorless (observer-based) operation */
#define MC_CFG_ENABLE_SENSORLESS      (1U)
/** @brief Enable debug/calibration subsystem (FreeMASTER, UART/CAN/LIN) */
#define MC_CFG_ENABLE_DEBUG           (1U)
/** @brief Enable FreeMASTER protocol engine (requires MC_CFG_ENABLE_DEBUG) */
#define MC_CFG_ENABLE_DEBUG_FM        (1U)
/** @brief Enable UART transport for debug */
#define MC_CFG_ENABLE_DEBUG_UART      (1U)
/** @brief Enable CAN transport for debug */
#define MC_CFG_ENABLE_DEBUG_CAN       (0U)
/** @brief Enable LIN transport for debug */
#define MC_CFG_ENABLE_DEBUG_LIN       (0U)

#endif /* MC_CFG_H */
