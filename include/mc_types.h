#ifndef MC_TYPES_H
#define MC_TYPES_H

/** @file mc_types.h @brief Basic type definitions for motor control library */

#include <stdint.h>
#include <stddef.h>

/** @brief 32-bit floating point type */
typedef float mc_f32_t;
/** @brief 32-bit fixed-point Q31 type */
typedef int32_t mc_q31_t;
/** @brief 16-bit fixed-point Q15 type */
typedef int16_t mc_q15_t;
/** @brief Boolean type (uint8_t) */
typedef uint8_t mc_bool_t;

/** @brief False value for mc_bool_t */
#define MC_FALSE ((mc_bool_t)0U)
/** @brief True value for mc_bool_t */
#define MC_TRUE  ((mc_bool_t)1U)

/** @brief Returns the number of elements in a static array */
#define MC_ARRAY_SIZE(array_) ((size_t)(sizeof(array_) / sizeof((array_)[0])))

#endif /* MC_TYPES_H */
