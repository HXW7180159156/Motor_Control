#ifndef MC_DRIVE_BLDC_SENSORLESS_H
#define MC_DRIVE_BLDC_SENSORLESS_H
/** @file mc_drive_bldc_sensorless.h @brief BLDC sensorless drive using BEMF zero-crossing detection */

#include "mc_control_pi.h"
#include "mc_port_pwm.h"
#include "mc_status.h"
#include "mc_types.h"

/**
 * @brief Startup phase for BLDC sensorless drive
 */
typedef enum
{
    MC_BLDC_SENSORLESS_IDLE = 0,    /**< Idle, not running */
    MC_BLDC_SENSORLESS_ALIGN,       /**< Rotor alignment stage */
    MC_BLDC_SENSORLESS_RAMP_OPEN,   /**< Open-loop frequency ramp */
    MC_BLDC_SENSORLESS_RUN          /**< Closed-loop BEMF detection */
} mc_bldc_sensorless_phase_t;

/**
 * @brief BLDC sensorless drive configuration
 */
typedef struct
{
    mc_f32_t align_duty;                /**< Duty cycle during rotor alignment [0,1] */
    mc_f32_t align_time_s;              /**< Alignment duration in seconds */
    mc_f32_t ramp_start_freq_hz;        /**< Starting electrical frequency for open-loop ramp */
    mc_f32_t ramp_end_freq_hz;          /**< Electrical frequency to transition at */
    mc_f32_t ramp_time_s;               /**< Duration of the open-loop ramp in seconds */
    mc_f32_t ramp_start_duty;           /**< Duty cycle at ramp start */
    mc_f32_t ramp_end_duty;             /**< Duty cycle at ramp end */
    mc_f32_t bemf_threshold_v;          /**< Minimum BEMF peak voltage for valid detection */
    mc_f32_t advance_angle_deg;         /**< Commutation advance angle in electrical degrees */
    uint8_t zc_debounce_threshold;      /**< Consecutive ZC samples required (0=disabled) */
    mc_pi_cfg_t speed_pi_cfg;           /**< Speed PI controller config for duty adjustment */
} mc_bldc_sensorless_cfg_t;

/**
 * @brief BLDC sensorless drive runtime state
 */
typedef struct
{
    mc_bldc_sensorless_cfg_t cfg;           /**< Configuration parameters */
    mc_bldc_sensorless_phase_t phase;       /**< Current startup phase */
    uint8_t commutation_step;               /**< Current 6-step sector (0-5) */
    mc_f32_t elec_freq_hz;                  /**< Current estimated electrical frequency */
    mc_f32_t mech_speed_rpm;                /**< Estimated mechanical speed */
    mc_f32_t duty_cmd;                      /**< Current duty cycle command */

    mc_f32_t align_timer_s;                 /**< Alignment phase timer */
    mc_f32_t step_timer_s;                  /**< Time since last commutation */
    mc_f32_t ramp_timer_s;                  /**< Open-loop ramp timer */

    mc_bool_t zc_detected;                  /**< BEMF zero-cross detected in current step */
    mc_f32_t zc_timer_s;                    /**< Time since ZC detection */
    mc_f32_t last_zc_period_s;              /**< Electrical period from last ZC cycle */
    mc_f32_t commutation_delay_s;           /**< 30-degree delay after ZC */

    uint8_t zc_debounce_cnt;                /**< Consecutive ZC detection counter */
    mc_f32_t last_zc_voltage_v;             /**< Floating phase voltage at last ZC */

    mc_pi_t speed_pi;                       /**< Speed PI controller instance */
    mc_pwm_cmd_t last_pwm_cmd;              /**< Most recent PWM output */
} mc_bldc_sensorless_t;

/**
 * @brief Default configuration for BLDC sensorless drive
 * @return Default configuration structure
 */
mc_bldc_sensorless_cfg_t mc_bldc_sensorless_cfg_default(void);

/**
 * @brief Initialise BLDC sensorless drive
 * @param ss  Pointer to sensorless drive instance
 * @param cfg Configuration parameters
 * @return Operation status
 */
mc_status_t mc_bldc_sensorless_init(mc_bldc_sensorless_t *ss, const mc_bldc_sensorless_cfg_t *cfg);

/**
 * @brief Start BLDC sensorless drive from a clean alignment phase
 * @param ss  Pointer to sensorless drive instance
 * @return Operation status
 */
mc_status_t mc_bldc_sensorless_start(mc_bldc_sensorless_t *ss);

/**
 * @brief Reset BLDC sensorless drive to idle state
 * @param ss  Pointer to sensorless drive instance
 * @return Operation status
 */
mc_status_t mc_bldc_sensorless_reset(mc_bldc_sensorless_t *ss);

/**
 * @brief Run one control cycle of the BLDC sensorless drive
 * @param ss                        Pointer to sensorless drive instance
 * @param dt_s                      Sample interval in seconds
 * @param floating_phase_voltage_v  Voltage sample of the currently floating phase
 * @param bus_voltage_v             DC bus voltage
 * @param pwm_cmd                   Output PWM command for the three phases
 * @return Operation status
 */
mc_status_t mc_bldc_sensorless_run(mc_bldc_sensorless_t *ss,
    mc_f32_t dt_s, mc_f32_t floating_phase_voltage_v, mc_f32_t bus_voltage_v,
    mc_pwm_cmd_t *pwm_cmd);

/**
 * @brief Get the floating phase index for the current commutation step
 * @param step  Current commutation step (0-5)
 * @return Phase index (0=A, 1=B, 2=C), or 0xFF on invalid step
 */
uint8_t mc_bldc_sensorless_floating_phase(uint8_t step);

/**
 * @brief Get estimated mechanical speed in RPM
 * @param ss  Pointer to sensorless drive instance
 * @return Estimated mechanical speed in RPM
 */
mc_f32_t mc_bldc_sensorless_get_speed_rpm(const mc_bldc_sensorless_t *ss);

/**
 * @brief Run speed PI control for BLDC sensorless drive
 * @param ss           Pointer to sensorless drive instance
 * @param speed_ref_rpm  Target speed in RPM
 * @param dt_s         Sample interval in seconds
 */
void mc_bldc_sensorless_speed_step(mc_bldc_sensorless_t *ss, mc_f32_t speed_ref_rpm, mc_f32_t dt_s);

#endif /* MC_DRIVE_BLDC_SENSORLESS_H */
