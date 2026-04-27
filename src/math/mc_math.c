/**
 * @file mc_math.c
 * @brief Math utility functions for motor control
 */

#include "mc_math.h"

#include <limits.h>

/**
 * @brief Clamp a value between minimum and maximum bounds
 * @param value Input value to clamp
 * @param min_value Lower bound
 * @param max_value Upper bound
 * @return Clamped value within [min_value, max_value]
 */
mc_f32_t mc_math_clamp_f32(mc_f32_t value, mc_f32_t min_value, mc_f32_t max_value)
{
    mc_f32_t result = value;

    if (result < min_value)
    {
        result = min_value;
    }
    else if (result > max_value)
    {
        result = max_value;
    }

    return result;
}

mc_q31_t mc_math_clamp_q31(mc_q31_t value, mc_q31_t min_value, mc_q31_t max_value)
{
    mc_q31_t result = value;

    if (result < min_value)
    {
        result = min_value;
    }
    else if (result > max_value)
    {
        result = max_value;
    }

    return result;
}

mc_q31_t mc_q31_from_f32(mc_f32_t value)
{
    mc_f32_t clamped = mc_math_clamp_f32(value, -1.0F, 1.0F);

    if (clamped >= 1.0F)
    {
        return INT32_MAX;
    }

    return (mc_q31_t)(clamped * 2147483648.0F);
}

mc_f32_t mc_q31_to_f32(mc_q31_t value)
{
    return ((mc_f32_t)value) / 2147483648.0F;
}

mc_q31_t mc_q31_add_sat(mc_q31_t a, mc_q31_t b)
{
    int64_t sum = (int64_t)a + (int64_t)b;

    if (sum > (int64_t)INT32_MAX)
    {
        return INT32_MAX;
    }

    if (sum < (int64_t)INT32_MIN)
    {
        return INT32_MIN;
    }

    return (mc_q31_t)sum;
}

mc_q31_t mc_q31_mul(mc_q31_t a, mc_q31_t b)
{
    int64_t product = ((int64_t)a * (int64_t)b) >> 31;

    if (product > (int64_t)INT32_MAX)
    {
        return INT32_MAX;
    }

    if (product < (int64_t)INT32_MIN)
    {
        return INT32_MIN;
    }

    return (mc_q31_t)product;
}

/**
 * @brief Wrap angle to [-pi, pi] radians
 * @param angle_rad Input angle in radians
 * @return Wrapped angle in range [-pi, pi]
 */
mc_f32_t mc_math_wrap_angle_rad(mc_f32_t angle_rad)
{
    mc_f32_t result = angle_rad;
    const mc_f32_t two_pi = 6.2831853072F;
    const mc_f32_t pi = 3.1415926536F;

    while (result > pi)
    {
        result -= two_pi;
    }

    while (result < (-pi))
    {
        result += two_pi;
    }

    return result;
}

/**
 * @brief First-order low-pass filter (exponential moving average)
 * @param prev_value Previous filtered output value
 * @param input_value New input sample
 * @param alpha Filter coefficient (0.0 = no update, 1.0 = no filtering)
 * @return Filtered output value
 */
mc_f32_t mc_math_lpf_f32(mc_f32_t prev_value, mc_f32_t input_value, mc_f32_t alpha)
{
    return prev_value + (alpha * (input_value - prev_value));
}
