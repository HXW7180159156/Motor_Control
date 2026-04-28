#include "unity.h"
#include "mc_constants.h"
#include "mc_control_pi.h"
#include "mc_control_svpwm.h"
#include "mc_math.h"
#include "mc_transform.h"
#include <math.h>
#include <stdint.h>

/* ================================================================
 *  Clarke Transform Vector Tests (golden reference values)
 * ================================================================ */

/* Balanced 3-phase: ia=1.0, ib=-0.5, ic=-0.5 → α=1.0, β=0.0 */
void test_vt_clarke_balanced_positive_a(void)
{
    mc_abc_t abc = {1.0F, -0.5F, -0.5F};
    mc_alphabeta_t ab;
    mc_clarke_run(&abc, &ab);
    TEST_ASSERT_TRUE(fabsf(ab.alpha - 1.0F) < 1e-5F);
    TEST_ASSERT_TRUE(fabsf(ab.beta) < 1e-5F);
}

/* ia=0, ib=1.0, ic=-1.0 → α=0, β≈1.1547 */
void test_vt_clarke_pure_b_phase(void)
{
    mc_abc_t abc = {0.0F, 1.0F, -1.0F};
    mc_alphabeta_t ab;
    mc_clarke_run(&abc, &ab);
    TEST_ASSERT_TRUE(fabsf(ab.alpha) < 1e-5F);
    TEST_ASSERT_TRUE(fabsf(ab.beta - 1.1547005383F) < 1e-4F);
}

/* ================================================================
 *  Park Transform Vector Tests
 * ================================================================ */

/* Pure α=1.0, θ=0 → d=1.0, q=0.0 */
void test_vt_park_single_alpha_angle_zero(void)
{
    mc_alphabeta_t ab = {1.0F, 0.0F};
    mc_dq_t dq;
    mc_park_run(&ab, 0.0F, 1.0F, &dq);
    TEST_ASSERT_TRUE(fabsf(dq.d - 1.0F) < 1e-5F);
    TEST_ASSERT_TRUE(fabsf(dq.q) < 1e-5F);
}

/* Pure α=1.0, θ=90° → d≈0, q≈-1.0 */
void test_vt_park_single_alpha_angle_90(void)
{
    mc_alphabeta_t ab = {1.0F, 0.0F};
    mc_dq_t dq;
    mc_park_run(&ab, 1.0F, 0.0F, &dq);
    TEST_ASSERT_TRUE(fabsf(dq.d) < 1e-5F);
    TEST_ASSERT_TRUE(fabsf(dq.q + 1.0F) < 1e-5F);
}

/* ================================================================
 *  Inverse Park Transform Vector Tests
 * ================================================================ */

/* d=1.0, q=0, θ=0 → α=1.0, β=0 */
void test_vt_ipark_d_axis_angle_zero(void)
{
    mc_dq_t dq = {1.0F, 0.0F};
    mc_alphabeta_t ab;
    mc_ipark_run(&dq, 0.0F, 1.0F, &ab);
    TEST_ASSERT_TRUE(fabsf(ab.alpha - 1.0F) < 1e-5F);
    TEST_ASSERT_TRUE(fabsf(ab.beta) < 1e-5F);
}

/* d=0, q=1.0, θ=0 → α=0, β=1.0 */
void test_vt_ipark_q_axis_angle_zero(void)
{
    mc_dq_t dq = {0.0F, 1.0F};
    mc_alphabeta_t ab;
    mc_ipark_run(&dq, 0.0F, 1.0F, &ab);
    TEST_ASSERT_TRUE(fabsf(ab.alpha) < 1e-5F);
    TEST_ASSERT_TRUE(fabsf(ab.beta - 1.0F) < 1e-5F);
}

/* ================================================================
 *  Park + InvPark Round-trip Vector Tests
 * ================================================================ */

void test_vt_park_ipark_roundtrip(void)
{
    mc_alphabeta_t ab_in = {0.8F, 0.3F};
    mc_dq_t dq;
    mc_alphabeta_t ab_out;
    mc_park_run(&ab_in, 0.5F, 0.8660254038F, &dq);
    mc_ipark_run(&dq, 0.5F, 0.8660254038F, &ab_out);
    TEST_ASSERT_TRUE(fabsf(ab_out.alpha - ab_in.alpha) < 1e-5F);
    TEST_ASSERT_TRUE(fabsf(ab_out.beta - ab_in.beta) < 1e-5F);
}

/* ================================================================
 *  SVPWM Vector Tests
 * ================================================================ */

/* Zero voltage → all duties centered at 0.5 */
void test_vt_svpwm_zero_voltage_centered_duties(void)
{
    mc_alphabeta_t v_ab = {0.0F, 0.0F};
    mc_svpwm_cfg_t cfg = {0.95F, 0.0F, 1.0F};
    mc_pwm_cmd_t pwm = {0};

    mc_svpwm_run(&v_ab, &cfg, &pwm);

    TEST_ASSERT_TRUE(fabsf(pwm.duty_a - 0.5F) < 0.01F);
    TEST_ASSERT_TRUE(fabsf(pwm.duty_b - 0.5F) < 0.01F);
    TEST_ASSERT_TRUE(fabsf(pwm.duty_c - 0.5F) < 0.01F);
    TEST_ASSERT_EQUAL_INT(1U, pwm.valid);
    TEST_ASSERT_TRUE(pwm.sector >= 1U && pwm.sector <= 6U);
}

/* Pure α=0.8 → duties within [0,1] */
void test_vt_svpwm_pure_alpha_duties_in_range(void)
{
    mc_alphabeta_t v_ab = {0.8F, 0.0F};
    mc_svpwm_cfg_t cfg = {0.95F, 0.0F, 1.0F};
    mc_pwm_cmd_t pwm = {0};

    mc_svpwm_run(&v_ab, &cfg, &pwm);

    TEST_ASSERT_TRUE(pwm.duty_a >= 0.0F && pwm.duty_a <= 1.0F);
    TEST_ASSERT_TRUE(pwm.duty_b >= 0.0F && pwm.duty_b <= 1.0F);
    TEST_ASSERT_TRUE(pwm.duty_c >= 0.0F && pwm.duty_c <= 1.0F);
    TEST_ASSERT_TRUE(fabsf(pwm.duty_a + pwm.duty_b + pwm.duty_c - 1.5F) < 0.1F);
}

/* ================================================================
 *  PI Controller Vector Tests
 * ================================================================ */

/* Kp only: output = error * kp */
void test_vt_pi_proportional_only(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {2.0F, 0.0F, -1.0F, 1.0F, -10.0F, 10.0F};
    mc_pi_init(&pi, &cfg);
    mc_f32_t out = mc_pi_run(&pi, 0.5F, 0.01F);
    TEST_ASSERT_TRUE(fabsf(out - 1.0F) < 1e-5F);
}

/* Integrator: output = error * ki * dt (first step) */
void test_vt_pi_integral_first_step(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {0.0F, 10.0F, -1.0F, 1.0F, -10.0F, 10.0F};
    mc_pi_init(&pi, &cfg);
    mc_f32_t out = mc_pi_run(&pi, 0.5F, 0.01F);
    TEST_ASSERT_TRUE(fabsf(out - 0.05F) < 1e-5F);
}

/* PI: step response settles within bounds */
void test_vt_pi_step_response(void)
{
    mc_pi_t pi;
    mc_pi_cfg_t cfg = {0.5F, 10.0F, -0.5F, 0.5F, -2.0F, 2.0F};
    mc_pi_init(&pi, &cfg);
    int i;
    mc_f32_t out = 0.0F;
    for (i = 0; i < 20; i++)
    {
        out = mc_pi_run(&pi, 1.0F, 0.01F);
    }
    TEST_ASSERT_TRUE(out > 1.5F);
    TEST_ASSERT_TRUE(out <= 2.0F);
}

/* ================================================================
 *  Math Utilities Vector Tests
 * ================================================================ */

void test_vt_wrap_angle_zero_stays_zero(void)
{
    TEST_ASSERT_TRUE(fabsf(mc_math_wrap_angle_rad(0.0F)) < 1e-6F);
}

void test_vt_wrap_angle_over_pi(void)
{
    mc_f32_t result = mc_math_wrap_angle_rad(4.0F);
    TEST_ASSERT_TRUE(result > -MC_PI && result <= MC_PI);
}

void test_vt_wrap_angle_negative_large(void)
{
    mc_f32_t result = mc_math_wrap_angle_rad(-7.0F);
    TEST_ASSERT_TRUE(result > -MC_PI && result <= MC_PI);
}

void test_vt_lpf_zero_alpha_no_change(void)
{
    mc_f32_t out = mc_math_lpf_f32(0.5F, 1.0F, 0.0F);
    TEST_ASSERT_TRUE(fabsf(out - 0.5F) < 1e-6F);
}

void test_vt_lpf_full_alpha_instant(void)
{
    mc_f32_t out = mc_math_lpf_f32(0.5F, 1.0F, 1.0F);
    TEST_ASSERT_TRUE(fabsf(out - 1.0F) < 1e-6F);
}

void test_vt_lpf_half_alpha_midpoint(void)
{
    mc_f32_t out = mc_math_lpf_f32(0.0F, 1.0F, 0.5F);
    TEST_ASSERT_TRUE(fabsf(out - 0.5F) < 1e-6F);
}

/* ================================================================
 *  Q31 Conversion Vector Tests
 * ================================================================ */

void test_vt_q31_zero_roundtrip(void)
{
    mc_q31_t q = mc_q31_from_f32(0.0F);
    mc_f32_t f = mc_q31_to_f32(q);
    TEST_ASSERT_TRUE(fabsf(f) < 1e-6F);
}

void test_vt_q31_one_saturates(void)
{
    mc_q31_t q = mc_q31_from_f32(2.0F);
    TEST_ASSERT_EQUAL_INT((mc_q31_t)INT32_MAX, q);
}

void test_vt_q31_neg_one_saturates(void)
{
    mc_q31_t q = mc_q31_from_f32(-2.0F);
    TEST_ASSERT_EQUAL_INT((mc_q31_t)INT32_MIN, q);
}

void test_vt_q31_add_sat_overflow(void)
{
    mc_q31_t result = mc_q31_add_sat(INT32_MAX, 100);
    TEST_ASSERT_EQUAL_INT((mc_q31_t)INT32_MAX, result);
}

void test_vt_q31_mul_max_max(void)
{
    mc_q31_t a = mc_q31_from_f32(1.0F);
    mc_q31_t b = mc_q31_from_f32(1.0F);
    mc_q31_t result = mc_q31_mul(a, b);
    TEST_ASSERT_TRUE(result > (INT32_MAX / 2));
}
