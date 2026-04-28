/**
 * @file mc_sensor_smo.h
 * @brief Sliding Mode Observer (SMO) for PMSM sensorless control
 *
 * The SMO uses a sign-function-based sliding mode current observer to estimate
 * back-EMF, then extracts rotor position via a phase-locked loop (PLL).
 * Compared to the baseline LPF+PLL approach, the SMO offers:
 * - Better noise immunity in mid-to-high speed range
 * - Reduced phase lag from LPF filtering of raw BEMF
 * - Improved robustness to parameter mismatch
 *
 * Typical tuning: k_slide = 1.5..3.0 × max_BEMF, lpf_alpha = 0.3..0.7
 */
#ifndef MC_SENSOR_SMO_H
#define MC_SENSOR_SMO_H

#include "mc_status.h"
#include "mc_transform.h"
#include "mc_types.h"

/**
 * @brief SMO observer configuration parameters
 */
typedef struct
{
    mc_f32_t rs_ohm;               /**< Stator resistance in ohms */
    mc_f32_t ls_h;                 /**< Stator inductance in henries (average Ld/Lq) */
    mc_f32_t pole_pairs;           /**< Number of motor pole pairs */
    mc_f32_t k_slide;              /**< Sliding mode gain [V] (must be > peak BEMF at max speed) */
    mc_f32_t lpf_alpha;            /**< BEMF extraction low-pass filter coefficient (0..1) */
    mc_f32_t min_bemf;             /**< Minimum BEMF magnitude for valid observer output */
    mc_f32_t pll_kp;              /**< PLL proportional gain */
    mc_f32_t pll_ki;              /**< PLL integral gain */
    mc_f32_t lock_bemf;           /**< BEMF magnitude threshold to lock PLL */
    mc_f32_t startup_speed_rad_s;  /**< Open-loop startup target speed [rad/s] */
    mc_f32_t startup_accel_rad_s2; /**< Open-loop startup acceleration [rad/s²] */
    mc_f32_t open_loop_voltage_start; /**< V/f voltage at zero speed */
    mc_f32_t open_loop_voltage_max;   /**< V/f voltage at target startup speed */
} mc_smo_cfg_t;

/**
 * @brief SMO observer runtime state
 */
typedef struct
{
    mc_smo_cfg_t cfg;                   /**< Copied configuration */
    mc_alphabeta_t i_hat_ab;            /**< Estimated alpha-beta current */
    mc_alphabeta_t bemf_ab;             /**< Estimated (filtered) alpha-beta back-EMF */
    mc_alphabeta_t z_ab;                /**< Raw sliding mode correction signal */
    mc_f32_t pll_speed_rad_s;           /**< PLL estimated speed [rad/s] */
    mc_f32_t pll_integrator;            /**< PLL integrator state */
    mc_f32_t elec_angle_rad;            /**< Estimated electrical angle [rad] */
    mc_f32_t mech_speed_rpm;            /**< Estimated mechanical speed [RPM] */
    mc_f32_t open_loop_voltage;         /**< Current V/f voltage during open-loop */
    mc_f32_t bemf_magnitude;            /**< BEMF magnitude for validation */
    uint32_t last_timestamp_us;         /**< Timestamp of previous update [µs] */
    mc_bool_t observer_valid;           /**< Observer output is valid */
    mc_bool_t open_loop_active;         /**< Currently in open-loop startup */
    mc_bool_t pll_locked;               /**< PLL has achieved lock */
    uint8_t open_loop_lock_counter;     /**< Consecutive valid BEMF+angle samples */
    uint8_t pll_lock_counter;           /**< Consecutive PLL lock samples */
    uint8_t pll_unlock_counter;         /**< Consecutive PLL unlock samples */
} mc_smo_state_t;

/**
 * @brief Initialise SMO observer state with configuration
 * @param[out] state SMO state structure to initialise
 * @param[in] cfg SMO configuration parameters
 * @retval MC_STATUS_OK Success
 * @retval MC_STATUS_INVALID_ARG state==NULL, cfg==NULL, or configuration constraint violated
 */
mc_status_t mc_smo_init(mc_smo_state_t *state, const mc_smo_cfg_t *cfg);

/**
 * @brief Update SMO observer with voltage and current measurements
 * @param[in,out] state SMO state (updated in place)
 * @param[in] voltage_ab Alpha-beta voltage vector (applied in prev cycle)
 * @param[in] current_ab Alpha-beta measured current vector
 * @param[in] dt_s Time step [s]
 * @param[in] timestamp_us Microsecond timestamp
 * @retval MC_STATUS_OK Success
 * @retval MC_STATUS_INVALID_ARG NULL pointer or dt_s <= 0
 */
mc_status_t mc_smo_update(mc_smo_state_t *state,
                          const mc_alphabeta_t *voltage_ab,
                          const mc_alphabeta_t *current_ab,
                          mc_f32_t dt_s,
                          uint32_t timestamp_us);

#endif /* MC_SENSOR_SMO_H */
