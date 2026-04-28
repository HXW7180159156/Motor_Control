#include "unity.h"

void test_vt_clarke_balanced_positive_a(void);
void test_vt_clarke_pure_b_phase(void);
void test_vt_park_single_alpha_angle_zero(void);
void test_vt_park_single_alpha_angle_90(void);
void test_vt_ipark_d_axis_angle_zero(void);
void test_vt_ipark_q_axis_angle_zero(void);
void test_vt_park_ipark_roundtrip(void);
void test_vt_svpwm_zero_voltage_centered_duties(void);
void test_vt_svpwm_pure_alpha_duties_in_range(void);
void test_vt_pi_proportional_only(void);
void test_vt_pi_integral_first_step(void);
void test_vt_pi_step_response(void);
void test_vt_wrap_angle_zero_stays_zero(void);
void test_vt_wrap_angle_over_pi(void);
void test_vt_wrap_angle_negative_large(void);
void test_vt_lpf_zero_alpha_no_change(void);
void test_vt_lpf_full_alpha_instant(void);
void test_vt_lpf_half_alpha_midpoint(void);
void test_vt_q31_zero_roundtrip(void);
void test_vt_q31_one_saturates(void);
void test_vt_q31_neg_one_saturates(void);
void test_vt_q31_add_sat_overflow(void);
void test_vt_q31_mul_max_max(void);

void setUp(void) {}
void tearDown(void) {}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_vt_clarke_balanced_positive_a);
    RUN_TEST(test_vt_clarke_pure_b_phase);
    RUN_TEST(test_vt_park_single_alpha_angle_zero);
    RUN_TEST(test_vt_park_single_alpha_angle_90);
    RUN_TEST(test_vt_ipark_d_axis_angle_zero);
    RUN_TEST(test_vt_ipark_q_axis_angle_zero);
    RUN_TEST(test_vt_park_ipark_roundtrip);
    RUN_TEST(test_vt_svpwm_zero_voltage_centered_duties);
    RUN_TEST(test_vt_svpwm_pure_alpha_duties_in_range);
    RUN_TEST(test_vt_pi_proportional_only);
    RUN_TEST(test_vt_pi_integral_first_step);
    RUN_TEST(test_vt_pi_step_response);
    RUN_TEST(test_vt_wrap_angle_zero_stays_zero);
    RUN_TEST(test_vt_wrap_angle_over_pi);
    RUN_TEST(test_vt_wrap_angle_negative_large);
    RUN_TEST(test_vt_lpf_zero_alpha_no_change);
    RUN_TEST(test_vt_lpf_full_alpha_instant);
    RUN_TEST(test_vt_lpf_half_alpha_midpoint);
    RUN_TEST(test_vt_q31_zero_roundtrip);
    RUN_TEST(test_vt_q31_one_saturates);
    RUN_TEST(test_vt_q31_neg_one_saturates);
    RUN_TEST(test_vt_q31_add_sat_overflow);
    RUN_TEST(test_vt_q31_mul_max_max);
    return UNITY_END();
}
