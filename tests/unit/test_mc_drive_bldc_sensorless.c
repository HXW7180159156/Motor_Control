#include "unity.h"
#include "mc_drive_bldc_sensorless.h"

void test_default_cfg_has_reasonable_values(void)
{
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    TEST_ASSERT_TRUE(cfg.align_duty > 0.0F);
    TEST_ASSERT_TRUE(cfg.align_time_s > 0.0F);
    TEST_ASSERT_TRUE(cfg.ramp_start_freq_hz > 0.0F);
    TEST_ASSERT_TRUE(cfg.ramp_end_freq_hz > cfg.ramp_start_freq_hz);
    TEST_ASSERT_TRUE(cfg.ramp_time_s > 0.0F);
    TEST_ASSERT_TRUE(cfg.ramp_start_duty > 0.0F);
    TEST_ASSERT_TRUE(cfg.ramp_end_duty > cfg.ramp_start_duty);
    TEST_ASSERT_TRUE(cfg.bemf_threshold_v > 0.0F);
    TEST_ASSERT_TRUE(cfg.advance_angle_deg > 0.0F);
    TEST_ASSERT_TRUE(cfg.advance_angle_deg <= 60.0F);
}

void test_init_sets_idle_phase(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_IDLE, ss.phase);
}

void test_init_null_returns_error(void)
{
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_status_t status = mc_bldc_sensorless_init(NULL, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
}

void test_init_null_cfg_returns_error(void)
{
    mc_bldc_sensorless_t ss;
    mc_status_t status = mc_bldc_sensorless_init(&ss, NULL);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
}

void test_init_rejects_zero_align_time(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.align_time_s = 0.0F;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_bldc_sensorless_init(&ss, &cfg));
}

void test_init_rejects_zero_ramp_start_freq(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_start_freq_hz = 0.0F;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_bldc_sensorless_init(&ss, &cfg));
}

void test_init_rejects_ramp_end_less_than_start(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_end_freq_hz = cfg.ramp_start_freq_hz - 1.0F;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_bldc_sensorless_init(&ss, &cfg));
}

void test_init_rejects_zero_ramp_time(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_time_s = 0.0F;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_bldc_sensorless_init(&ss, &cfg));
}

void test_init_rejects_start_duty_negative(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_start_duty = -0.1F;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_bldc_sensorless_init(&ss, &cfg));
}

void test_init_rejects_start_duty_over_one(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_start_duty = 1.5F;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_bldc_sensorless_init(&ss, &cfg));
}

void test_init_rejects_end_duty_less_than_start(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_end_duty = cfg.ramp_start_duty - 0.1F;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_bldc_sensorless_init(&ss, &cfg));
}

void test_init_rejects_end_duty_over_one(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_end_duty = 1.5F;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_bldc_sensorless_init(&ss, &cfg));
}

void test_init_accepts_end_duty_equal_one(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_end_duty = 1.0F;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, mc_bldc_sensorless_init(&ss, &cfg));
}

void test_init_accepts_end_freq_equal_start(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_end_freq_hz = cfg.ramp_start_freq_hz;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, mc_bldc_sensorless_init(&ss, &cfg));
}

void test_start_transitions_from_idle_to_align(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, mc_bldc_sensorless_start(&ss));
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_ALIGN, ss.phase);
    TEST_ASSERT_TRUE(ss.duty_cmd == cfg.align_duty);
    TEST_ASSERT_TRUE(ss.commutation_step == 0U);
}

void test_start_null_returns_error(void)
{
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, mc_bldc_sensorless_start(NULL));
}

void test_reset_returns_to_idle(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    mc_bldc_sensorless_start(&ss);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.commutation_step = 3U;
    mc_bldc_sensorless_reset(&ss);
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_IDLE, ss.phase);
    TEST_ASSERT_TRUE(ss.commutation_step == 0U);
}

void test_reset_preserves_configuration(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.align_duty = 0.22F;
    mc_bldc_sensorless_init(&ss, &cfg);
    mc_bldc_sensorless_start(&ss);
    ss.speed_pi.integral = 1.0F;
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, mc_bldc_sensorless_reset(&ss));
    TEST_ASSERT_TRUE(ss.cfg.align_duty == 0.22F);
    TEST_ASSERT_TRUE(ss.speed_pi.cfg.kp == cfg.speed_pi_cfg.kp);
    TEST_ASSERT_TRUE(ss.speed_pi.integral == 0.0F);
}

void test_reset_null_returns_error(void)
{
    mc_status_t status = mc_bldc_sensorless_reset(NULL);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
}

void test_idle_produces_zero_pwm(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    mc_status_t status = mc_bldc_sensorless_run(&ss, 0.001F, 0.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_TRUE(pwm_cmd.valid == 0U);
}

void test_align_commutates_and_transitions_to_ramp(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.align_time_s = 0.01F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_ALIGN;
    mc_status_t status = mc_bldc_sensorless_run(&ss, 0.02F, 0.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_OK, status);
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_RAMP_OPEN, ss.phase);
    TEST_ASSERT_TRUE(pwm_cmd.valid == 1U);
}

void test_ramp_open_step_advances_on_timer(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_start_freq_hz = 10.0F;
    cfg.ramp_time_s = 2.0F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RAMP_OPEN;
    mc_bldc_sensorless_run(&ss, 0.02F, 0.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.commutation_step > 0U);
}

void test_run_detects_zc_and_commutates(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.duty_cmd = 0.3F;
    ss.last_zc_period_s = 0.02F;
    ss.commutation_step = 0U;
    ss.step_timer_s = 0.0F;
    ss.zc_detected = MC_FALSE;
    /* Step 0: floating phase = C, voltage 13V > 12V neutral -> ZC */
    mc_bldc_sensorless_run(&ss, 0.001F, 13.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.zc_detected);
    /* Run until commutation delay passes */
    {
        int i;
        for (i = 0; i < 50; i++)
        {
            mc_bldc_sensorless_run(&ss, 0.001F, 13.0F, 24.0F, &pwm_cmd);
        }
    }
    TEST_ASSERT_TRUE(ss.commutation_step == 1U);
}

void test_run_fallback_on_missing_zc(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.duty_cmd = 0.3F;
    ss.last_zc_period_s = 0.02F;
    ss.commutation_step = 0U;
    ss.step_timer_s = 0.0F;
    ss.zc_detected = MC_FALSE;
    {
        int i;
        for (i = 0; i < 35; i++)
        {
            mc_bldc_sensorless_run(&ss, 0.001F, 12.0F, 24.0F, &pwm_cmd);
        }
    }
    TEST_ASSERT_TRUE(ss.commutation_step == 1U);
}

void test_ramp_advances_frequency_over_time(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_start_freq_hz = 5.0F;
    cfg.ramp_end_freq_hz = 20.0F;
    cfg.ramp_time_s = 1.0F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RAMP_OPEN;
    mc_bldc_sensorless_run(&ss, 0.5F, 0.0F, 24.0F, &pwm_cmd);
    {
        mc_f32_t freq_mid = ss.elec_freq_hz;
        mc_bldc_sensorless_run(&ss, 0.5F, 0.0F, 24.0F, &pwm_cmd);
        TEST_ASSERT_TRUE(ss.elec_freq_hz > freq_mid);
    }
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_RUN, ss.phase);
}

void test_floating_phase_map(void)
{
    TEST_ASSERT_EQUAL_INT(2U, mc_bldc_sensorless_floating_phase(0U));
    TEST_ASSERT_EQUAL_INT(1U, mc_bldc_sensorless_floating_phase(1U));
    TEST_ASSERT_EQUAL_INT(0U, mc_bldc_sensorless_floating_phase(2U));
    TEST_ASSERT_EQUAL_INT(2U, mc_bldc_sensorless_floating_phase(3U));
    TEST_ASSERT_EQUAL_INT(1U, mc_bldc_sensorless_floating_phase(4U));
    TEST_ASSERT_EQUAL_INT(0U, mc_bldc_sensorless_floating_phase(5U));
    TEST_ASSERT_EQUAL_INT(0xFF, mc_bldc_sensorless_floating_phase(6U));
}

void test_get_speed_rpm_returns_stored_value(void)
{
    mc_bldc_sensorless_t ss = {{0}};
    ss.mech_speed_rpm = 1500.0F;
    TEST_ASSERT_TRUE(mc_bldc_sensorless_get_speed_rpm(&ss) == 1500.0F);
}

void test_get_speed_rpm_null_returns_zero(void)
{
    TEST_ASSERT_TRUE(mc_bldc_sensorless_get_speed_rpm(NULL) == 0.0F);
}

void test_run_advances_to_run_after_ramp(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_start_freq_hz = 5.0F;
    cfg.ramp_end_freq_hz = 10.0F;
    cfg.ramp_time_s = 0.5F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RAMP_OPEN;
    mc_bldc_sensorless_run(&ss, 0.6F, 0.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_RUN, ss.phase);
}

void test_run_null_pwm_returns_error(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    mc_status_t status = mc_bldc_sensorless_run(&ss, 0.001F, 0.0F, 24.0F, NULL);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
}

void test_run_null_ss_returns_error(void)
{
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_status_t status = mc_bldc_sensorless_run(NULL, 0.001F, 0.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
}

/* --- Edge case / stress tests --- */

void test_zero_dt_does_not_advance_timer(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_ALIGN;
    ss.align_timer_s = 0.0F;
    mc_bldc_sensorless_run(&ss, 0.0F, 0.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.align_timer_s == 0.0F);
}

void test_large_dt_skips_align_directly_to_ramp(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.align_time_s = 0.01F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_ALIGN;
    mc_bldc_sensorless_run(&ss, 1.0F, 0.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_RAMP_OPEN, ss.phase);
}

void test_zero_bus_voltage_no_zc_detected(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.duty_cmd = 0.3F;
    ss.last_zc_period_s = 0.02F;
    /* step 0 (even): ZC when floating > bus/2. bus=0V -> threshold=0V, diff=5.0 > 0 -> ZC expected */
    mc_bldc_sensorless_run(&ss, 0.001F, 5.0F, 0.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.zc_detected);
}

void test_bemf_threshold_blocks_small_crossing(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.bemf_threshold_v = 2.0F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.duty_cmd = 0.3F;
    ss.last_zc_period_s = 0.02F;
    ss.commutation_step = 0U;
    mc_bldc_sensorless_run(&ss, 0.001F, 13.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.zc_detected == MC_FALSE);
}

void test_bemf_threshold_allows_large_crossing(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.bemf_threshold_v = 0.5F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.duty_cmd = 0.3F;
    ss.last_zc_period_s = 0.02F;
    ss.commutation_step = 0U;
    mc_bldc_sensorless_run(&ss, 0.001F, 13.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.zc_detected == MC_TRUE);
}

void test_align_with_already_expired_timer_immediately_transitions(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.align_time_s = 0.5F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_ALIGN;
    ss.align_timer_s = 10.0F;
    mc_bldc_sensorless_run(&ss, 0.001F, 0.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_RAMP_OPEN, ss.phase);
}

void test_ramp_saturates_at_end_values(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.ramp_start_freq_hz = 5.0F;
    cfg.ramp_end_freq_hz = 20.0F;
    cfg.ramp_time_s = 0.1F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RAMP_OPEN;
    /* Run well past ramp_time_s */
    mc_bldc_sensorless_run(&ss, 10.0F, 0.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.elec_freq_hz <= cfg.ramp_end_freq_hz);
    TEST_ASSERT_TRUE(ss.elec_freq_hz >= cfg.ramp_start_freq_hz);
    TEST_ASSERT_TRUE(ss.duty_cmd <= cfg.ramp_end_duty);
    TEST_ASSERT_TRUE(ss.duty_cmd >= cfg.ramp_start_duty);
}

void test_invalid_step_apply_returns_invalid_pwm(void)
{
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_status_t status;
    /* Step 99 is invalid -> should return MC_STATUS_INVALID_ARG and set pwm_cmd.valid = 0 */
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.commutation_step = 99U;
    /* The internal apply_step will receive step=99 through run -> but run clamps step value.
       Actually run uses commutation_step directly in apply_step -> invalid case */
    status = mc_bldc_sensorless_run(&ss, 0.001F, 0.0F, 24.0F, &pwm_cmd);
    /* apply_step with step > 5 returns INVALID_ARG */
    TEST_ASSERT_EQUAL_INT(MC_STATUS_INVALID_ARG, status);
    TEST_ASSERT_TRUE(pwm_cmd.valid == 0U);
}

/* --- Debounce tests --- */

void test_debounce_rejects_single_detection_below_threshold(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.zc_debounce_threshold = 3U;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.duty_cmd = 0.3F;
    ss.last_zc_period_s = 0.02F;
    ss.commutation_step = 0U;
    ss.step_timer_s = 0.0F;
    /* First detection */
    mc_bldc_sensorless_run(&ss, 0.001F, 13.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.zc_detected == MC_FALSE);
    TEST_ASSERT_TRUE(ss.zc_debounce_cnt == 1U);
    /* Second detection */
    mc_bldc_sensorless_run(&ss, 0.001F, 13.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.zc_detected == MC_FALSE);
    TEST_ASSERT_TRUE(ss.zc_debounce_cnt == 2U);
    /* Third detection -> should trigger */
    mc_bldc_sensorless_run(&ss, 0.001F, 13.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.zc_detected == MC_TRUE);
    TEST_ASSERT_TRUE(ss.zc_debounce_cnt == 3U);
}

void test_debounce_resets_on_missing_crossing(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.zc_debounce_threshold = 3U;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.duty_cmd = 0.3F;
    ss.last_zc_period_s = 0.02F;
    ss.commutation_step = 0U;
    ss.step_timer_s = 0.0F;
    /* Two detections */
    mc_bldc_sensorless_run(&ss, 0.001F, 13.0F, 24.0F, &pwm_cmd);
    mc_bldc_sensorless_run(&ss, 0.001F, 13.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.zc_debounce_cnt == 2U);
    /* One miss resets counter */
    mc_bldc_sensorless_run(&ss, 0.001F, 11.0F, 24.0F, &pwm_cmd);
    TEST_ASSERT_TRUE(ss.zc_debounce_cnt == 0U);
}

/* --- Speed PI tests --- */

void test_speed_pi_is_initialized_after_init(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    TEST_ASSERT_TRUE(ss.speed_pi.integral == 0.0F);
    TEST_ASSERT_TRUE(ss.speed_pi.output == 0.0F);
}

void test_speed_step_not_in_run_resets_pi(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_ALIGN;
    ss.speed_pi.integral = 10.0F;
    mc_bldc_sensorless_speed_step(&ss, 1000.0F, 0.01F);
    TEST_ASSERT_TRUE(ss.speed_pi.integral == 0.0F);
}

void test_speed_step_adjusts_duty_in_run(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.speed_pi_cfg.kp = 0.001F;
    cfg.speed_pi_cfg.ki = 0.0F;
    cfg.speed_pi_cfg.output_min = 0.0F;
    cfg.speed_pi_cfg.output_max = 1.0F;
    cfg.speed_pi_cfg.integral_min = -1.0F;
    cfg.speed_pi_cfg.integral_max = 1.0F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.mech_speed_rpm = 500.0F;
    ss.duty_cmd = 0.0F;
    /* speed_ref=1000, feedback=500 -> error=500 -> PI output = 0.001*500 + 0 = 0.5 */
    mc_bldc_sensorless_speed_step(&ss, 1000.0F, 0.01F);
    TEST_ASSERT_TRUE(ss.duty_cmd > 0.49F);
    TEST_ASSERT_TRUE(ss.duty_cmd < 0.51F);
}

void test_speed_step_null_ss_does_nothing(void)
{
    /* Should not crash */
    mc_bldc_sensorless_speed_step(NULL, 1000.0F, 0.01F);
}

void test_speed_step_zero_dt_does_nothing(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.duty_cmd = 0.3F;
    mc_bldc_sensorless_speed_step(&ss, 1000.0F, 0.0F);
    /* PI not updated, duty unchanged */
    TEST_ASSERT_TRUE(ss.duty_cmd == 0.3F);
}

void test_speed_step_negative_dt_does_nothing(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.duty_cmd = 0.3F;
    mc_bldc_sensorless_speed_step(&ss, 1000.0F, -0.01F);
    TEST_ASSERT_TRUE(ss.duty_cmd == 0.3F);
}

/* --- Advance angle tests --- */

void test_advance_angle_default_is_30_degrees(void)
{
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    TEST_ASSERT_TRUE(cfg.advance_angle_deg == 30.0F);
}

void test_advance_angle_affects_commutation_delay_in_advance_step(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.advance_angle_deg = 15.0F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.last_zc_period_s = 0.06F;
    /* Manually advance step to compute commutation_delay_s using the internal function.
       The step advance normally occurs inside run(). Trigger it by going through run() in RUN phase. */
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.commutation_step = 0U;
    ss.step_timer_s = 0.0F;
    ss.zc_timer_s = 0.0F;
    ss.zc_detected = MC_TRUE;
    /* Trigger commutation via zc_timer_s >= commutation_delay_s */
    ss.commutation_delay_s = ss.last_zc_period_s * (cfg.advance_angle_deg / 60.0F);
    ss.zc_timer_s = ss.commutation_delay_s;
    /* This should commutate */
    {
        mc_pwm_cmd_t pwm_cmd = {0};
        mc_bldc_sensorless_run(&ss, 0.001F, 0.0F, 24.0F, &pwm_cmd);
    }
    /* After commutation, step should have advanced and zc_detected reset */
    TEST_ASSERT_EQUAL_INT(1U, ss.commutation_step);
    TEST_ASSERT_TRUE(ss.zc_detected == MC_FALSE);
    /* Verify the new commutation_delay_s uses advance_angle_deg */
    mc_f32_t expected_delay = ss.last_zc_period_s * (cfg.advance_angle_deg / 60.0F);
    mc_f32_t diff = ss.commutation_delay_s - expected_delay;
    TEST_ASSERT_TRUE((diff > -1e-9F) && (diff < 1e-9F));
}

void test_advance_angle_zero_commutates_immediately_after_zc(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.advance_angle_deg = 0.0F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.last_zc_period_s = 0.06F;
    ss.commutation_step = 0U;
    ss.duty_cmd = 0.3F;
    /* Set ZC just detected */
    ss.zc_detected = MC_TRUE;
    ss.step_timer_s = 0.01F;
    ss.zc_timer_s = 0.0F;
    ss.commutation_delay_s = ss.last_zc_period_s * (cfg.advance_angle_deg / 60.0F);
    /* With advance_angle=0, commutation_delay_s=0, so already >= */
    {
        mc_pwm_cmd_t pwm_cmd = {0};
        mc_bldc_sensorless_run(&ss, 0.001F, 0.0F, 24.0F, &pwm_cmd);
    }
    /* Should have commutated immediately */
    TEST_ASSERT_EQUAL_INT(1U, ss.commutation_step);
}

void test_advance_angle_60_waits_full_step_period(void)
{
    mc_bldc_sensorless_t ss;
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.advance_angle_deg = 60.0F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RUN;
    ss.last_zc_period_s = 0.06F;
    ss.commutation_step = 0U;
    ss.duty_cmd = 0.3F;
    ss.zc_detected = MC_TRUE;
    ss.step_timer_s = 0.01F;
    ss.zc_timer_s = 0.0F;
    ss.commutation_delay_s = ss.last_zc_period_s * (cfg.advance_angle_deg / 60.0F);
    /* commutation_delay_s = 0.06 * 1.0 = 0.06. zc_timer_s=0, so NOT yet time to commutate */
    {
        mc_pwm_cmd_t pwm_cmd = {0};
        mc_bldc_sensorless_run(&ss, 0.001F, 0.0F, 24.0F, &pwm_cmd);
    }
    /* Should NOT have commutated yet */
    TEST_ASSERT_EQUAL_INT(0U, ss.commutation_step);
    /* After enough time */
    ss.zc_timer_s = 0.06F;
    {
        mc_pwm_cmd_t pwm_cmd = {0};
        mc_bldc_sensorless_run(&ss, 0.001F, 0.0F, 24.0F, &pwm_cmd);
    }
    TEST_ASSERT_EQUAL_INT(1U, ss.commutation_step);
}

void test_advance_angle_used_at_ramp_to_run_transition(void)
{
    mc_bldc_sensorless_t ss;
    mc_pwm_cmd_t pwm_cmd = {0};
    mc_bldc_sensorless_cfg_t cfg = mc_bldc_sensorless_cfg_default();
    cfg.advance_angle_deg = 45.0F;
    cfg.ramp_start_freq_hz = 10.0F;
    cfg.ramp_end_freq_hz = 10.0F;
    cfg.ramp_end_duty = 0.5F;
    mc_bldc_sensorless_init(&ss, &cfg);
    ss.phase = MC_BLDC_SENSORLESS_RAMP_OPEN;
    mc_bldc_sensorless_run(&ss, 0.001F, 0.0F, 24.0F, &pwm_cmd);
    /* freq reaches end freq -> transitions to RUN. commutation_delay_s should use advance_angle_deg */
    TEST_ASSERT_EQUAL_INT(MC_BLDC_SENSORLESS_RUN, ss.phase);
    mc_f32_t expected_delay = ss.last_zc_period_s * (45.0F / 60.0F);
    TEST_ASSERT_TRUE(ss.commutation_delay_s == expected_delay);
}
