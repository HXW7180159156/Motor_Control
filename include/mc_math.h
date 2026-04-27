#ifndef MC_MATH_H
#define MC_MATH_H

/** @file mc_math.h @brief Inline math utility functions */

#include "mc_types.h"

/**
 * @brief Clamp a float value within a specified range
 * @param[in] value Input value to clamp.
 *   Range: any `mc_f32_t`; finite values are recommended.
 * @param[in] min_value Minimum allowable value.
 *   Range: any `mc_f32_t`; intended lower bound.
 * @param[in] max_value Maximum allowable value.
 *   Range: any `mc_f32_t`; intended upper bound.
 * @return Clamped value.
 *   Range: [`min_value`, `max_value`] when `min_value <= max_value`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
mc_f32_t mc_math_clamp_f32(mc_f32_t value, mc_f32_t min_value, mc_f32_t max_value);

/**
 * @brief Clamp a Q31 value within a specified range
 * @param[in] value Input value to clamp.
 *   Range: any `mc_q31_t` in [`INT32_MIN`, `INT32_MAX`].
 * @param[in] min_value Minimum allowable value.
 *   Range: any `mc_q31_t`; intended lower bound.
 * @param[in] max_value Maximum allowable value.
 *   Range: any `mc_q31_t`; intended upper bound.
 * @return Clamped Q31 value.
 *   Range: [`min_value`, `max_value`] when `min_value <= max_value`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
mc_q31_t mc_math_clamp_q31(mc_q31_t value, mc_q31_t min_value, mc_q31_t max_value);

/**
 * @brief Convert a float in [-1, 1] to Q31 with saturation
 * @param[in] value Input float value.
 *   Range: any `mc_f32_t`; values outside `[-1.0F, 1.0F]` are saturated.
 * @return Saturated Q31 representation.
 *   Range: [`INT32_MIN`, `INT32_MAX`].
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
mc_q31_t mc_q31_from_f32(mc_f32_t value);

/**
 * @brief Convert Q31 to float in [-1, 1]
 * @param[in] value Input Q31 value.
 *   Range: any `mc_q31_t` in [`INT32_MIN`, `INT32_MAX`].
 * @return Float representation.
 *   Range: approximately `[-1.0F, 1.0F)`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
mc_f32_t mc_q31_to_f32(mc_q31_t value);

/**
 * @brief Add two Q31 values with saturation
 * @param[in] a First operand.
 *   Range: any `mc_q31_t` in [`INT32_MIN`, `INT32_MAX`].
 * @param[in] b Second operand.
 *   Range: any `mc_q31_t` in [`INT32_MIN`, `INT32_MAX`].
 * @return Saturated Q31 sum.
 *   Range: [`INT32_MIN`, `INT32_MAX`].
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
mc_q31_t mc_q31_add_sat(mc_q31_t a, mc_q31_t b);

/**
 * @brief Multiply two Q31 values
 * @param[in] a First operand.
 *   Range: any `mc_q31_t` in [`INT32_MIN`, `INT32_MAX`].
 * @param[in] b Second operand.
 *   Range: any `mc_q31_t` in [`INT32_MIN`, `INT32_MAX`].
 * @return Saturated Q31 product.
 *   Range: [`INT32_MIN`, `INT32_MAX`].
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
mc_q31_t mc_q31_mul(mc_q31_t a, mc_q31_t b);

/**
 * @brief Wrap an angle to the range [-pi, pi] radians
 * @param[in] angle_rad Input angle in radians.
 *   Range: any finite `mc_f32_t` angle.
 * @return Wrapped angle in the range `[-pi, pi]`.
 *   Range: `[-3.1415926536F, 3.1415926536F]`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
mc_f32_t mc_math_wrap_angle_rad(mc_f32_t angle_rad);

/**
 * @brief Apply a first-order low-pass filter (exponential moving average)
 * @param[in] prev_value Previous filtered output value.
 *   Range: any finite `mc_f32_t`.
 * @param[in] input_value Current unfiltered input value.
 *   Range: any finite `mc_f32_t`.
 * @param[in] alpha Filter coefficient.
 *   Range: intended `[0.0F, 1.0F]`; values outside this range extrapolate rather than interpolate.
 * @return Filtered output value.
 *   Range: any `mc_f32_t` representable by `prev_value + alpha * (input_value - prev_value)`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
mc_f32_t mc_math_lpf_f32(mc_f32_t prev_value, mc_f32_t input_value, mc_f32_t alpha);

#endif /* MC_MATH_H */
