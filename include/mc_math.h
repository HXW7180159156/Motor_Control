#ifndef MC_MATH_H
#define MC_MATH_H

/** @file mc_math.h @brief Inline math utility functions */

#include "mc_types.h"

/**
 * @brief Clamp a float value within a specified range
 * @param value Input value to clamp
 * @param min_value Minimum allowable value
 * @param max_value Maximum allowable value
 * @return Clamped value in range [min_value, max_value]
 */
mc_f32_t mc_math_clamp_f32(mc_f32_t value, mc_f32_t min_value, mc_f32_t max_value);

/**
 * @brief Clamp a Q31 value within a specified range
 * @param value Input value to clamp
 * @param min_value Minimum allowable value
 * @param max_value Maximum allowable value
 * @return Clamped Q31 value in range [min_value, max_value]
 */
mc_q31_t mc_math_clamp_q31(mc_q31_t value, mc_q31_t min_value, mc_q31_t max_value);

/**
 * @brief Convert a float in [-1, 1] to Q31 with saturation
 * @param value Input float value
 * @return Saturated Q31 representation
 */
mc_q31_t mc_q31_from_f32(mc_f32_t value);

/**
 * @brief Convert Q31 to float in [-1, 1]
 * @param value Input Q31 value
 * @return Float representation
 */
mc_f32_t mc_q31_to_f32(mc_q31_t value);

/**
 * @brief Add two Q31 values with saturation
 * @param a First operand
 * @param b Second operand
 * @return Saturated Q31 sum
 */
mc_q31_t mc_q31_add_sat(mc_q31_t a, mc_q31_t b);

/**
 * @brief Multiply two Q31 values
 * @param a First operand
 * @param b Second operand
 * @return Saturated Q31 product
 */
mc_q31_t mc_q31_mul(mc_q31_t a, mc_q31_t b);

/**
 * @brief Wrap an angle to the range [-pi, pi] radians
 * @param angle_rad Input angle in radians
 * @return Wrapped angle in the range [-pi, pi]
 */
mc_f32_t mc_math_wrap_angle_rad(mc_f32_t angle_rad);

/**
 * @brief Apply a first-order low-pass filter (exponential moving average)
 * @param prev_value Previous filtered output value
 * @param input_value Current unfiltered input value
 * @param alpha Filter coefficient (0.0 = no update, 1.0 = no filtering)
 * @return Filtered output value
 */
mc_f32_t mc_math_lpf_f32(mc_f32_t prev_value, mc_f32_t input_value, mc_f32_t alpha);

#endif /* MC_MATH_H */
