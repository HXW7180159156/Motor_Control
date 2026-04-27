/**
 * @file mc_sensor_sensorless.h
 * @brief Sensorless (BEMF observer) sensor interface
 */

#ifndef MC_SENSOR_SENSORLESS_H
#define MC_SENSOR_SENSORLESS_H

#include "mc_status.h"
#include "mc_transform.h"
#include "mc_types.h"

/**
 * @brief Sensorless observer configuration parameters
 * @note Open-loop startup exits only after consecutive samples satisfy both `bemf_magnitude >= lock_bemf`
 *       and a bounded angle error check inside the implementation.
 * @note Closed-loop `pll_locked` is also debounced with consecutive in-threshold and out-of-threshold samples.
 * @note `min_bemf` should stay below `lock_bemf`; otherwise the observer may repeatedly invalidate before it can hand off.
 * @note `open_loop_voltage_start` must be non-negative and `open_loop_voltage_max` must be greater than or equal to it.
 * @note These configuration constraints are validated by `mc_sensorless_init()`.
 * @note Large `pll_kp` / `pll_ki` can improve convergence but increase noise sensitivity and lock chatter in this baseline LPF + PLL observer.
 */
typedef struct
{
    mc_f32_t rs_ohm;               /**< Stator resistance in ohms */
    mc_f32_t ls_h;                 /**< Stator inductance in henries */
    mc_f32_t pole_pairs;           /**< Number of motor pole pairs */
    mc_f32_t bemf_filter_alpha;    /**< BEMF low-pass filter coefficient */
    mc_f32_t min_bemf;             /**< Minimum BEMF magnitude for closed-loop operation */
    mc_f32_t pll_kp;              /**< Phase-locked loop proportional gain */
    mc_f32_t pll_ki;              /**< Phase-locked loop integral gain */
    mc_f32_t lock_bemf;           /**< BEMF magnitude threshold to lock observer */
    mc_f32_t startup_speed_rad_s;  /**< Open-loop startup speed in rad/s */
    mc_f32_t startup_accel_rad_s2; /**< Open-loop startup acceleration in rad/s^2 */
    mc_f32_t open_loop_voltage_start; /**< Initial V/f voltage magnitude at zero speed */
    mc_f32_t open_loop_voltage_max;   /**< V/f voltage magnitude at target startup speed */
} mc_sensorless_cfg_t;

/**
 * @brief Sensorless observer state information
 * @note `open_loop_lock_counter`, `pll_lock_counter`, and `pll_unlock_counter` are runtime debounce counters,
 *       not user tuning parameters.
 * @note When `bemf_magnitude` falls below `cfg.min_bemf`, `observer_valid` is cleared immediately, `pll_locked`
 *       is dropped, debounce counters are reset, and closed-loop speed/integrator state are cleared if the observer
 *       had already left open-loop startup.
 */
typedef struct
{
    mc_sensorless_cfg_t cfg;            /**< Sensorless observer configuration */
    mc_alphabeta_t bemf_ab;             /**< Estimated back-EMF in alpha-beta frame */
    mc_alphabeta_t last_current_ab;     /**< Previous current in alpha-beta frame */
    mc_f32_t bemf_magnitude;            /**< Magnitude of estimated back-EMF */
    mc_f32_t pll_speed_rad_s;           /**< PLL estimated speed in rad/s */
    mc_f32_t pll_integrator;            /**< PLL integrator state */
    mc_f32_t elec_angle_rad;            /**< Estimated electrical angle in radians */
    mc_f32_t mech_speed_rpm;            /**< Estimated mechanical speed in RPM */
    uint32_t last_timestamp_us;         /**< Timestamp of previous update in microseconds */
    mc_f32_t open_loop_voltage;         /**< Current V/f voltage magnitude during open-loop startup */
    mc_bool_t observer_valid;           /**< Flag indicating observer estimate is valid */
    mc_bool_t pll_locked;               /**< Flag indicating PLL is locked */
    mc_bool_t open_loop_active;         /**< Flag indicating open-loop startup is active */
    uint8_t open_loop_lock_counter;     /**< Consecutive open-loop samples that satisfy lock criteria */
    uint8_t pll_lock_counter;           /**< Consecutive closed-loop samples that satisfy lock criteria */
    uint8_t pll_unlock_counter;         /**< Consecutive closed-loop samples that violate lock criteria */
} mc_sensorless_state_t;

/**
 * @brief Initialise sensorless observer state with configuration
 * @param state Pointer to sensorless observer state structure
 * @param cfg Pointer to sensorless observer configuration
 * @return MC_OK on success, or an error code
 */
mc_status_t mc_sensorless_init(mc_sensorless_state_t *state, const mc_sensorless_cfg_t *cfg);

/**
 * @brief Update sensorless observer with voltage and current measurements
 * @param state Pointer to sensorless observer state structure
 * @param voltage_ab Voltage vector in alpha-beta frame
 * @param current_ab Current vector in alpha-beta frame
 * @param dt_s Sample interval in seconds
 * @param timestamp_us Current timestamp in microseconds
 * @return MC_OK on success, or an error code
 */
mc_status_t mc_sensorless_update(mc_sensorless_state_t *state,
                                 const mc_alphabeta_t *voltage_ab,
                                 const mc_alphabeta_t *current_ab,
                                 mc_f32_t dt_s,
                                 uint32_t timestamp_us);

#endif /* MC_SENSOR_SENSORLESS_H */
