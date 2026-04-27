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
 * @brief Initialise the sensorless observer state
 * @param[out] state Pointer to sensorless observer state storage.
 *   Range: non-NULL pointer to writable `mc_sensorless_state_t` storage.
 * @param[in] cfg Sensorless observer configuration.
 *   Range: non-NULL pointer to readable `mc_sensorless_cfg_t` storage with:
 *   `pole_pairs > 0.0F`, `ls_h >= 0.0F`, `bemf_filter_alpha` in `(0.0F, 1.0F]`,
 *   `min_bemf >= 0.0F`, `pll_kp >= 0.0F`, `pll_ki >= 0.0F`,
 *   `lock_bemf > min_bemf`, `startup_speed_rad_s >= 0.0F`,
 *   `startup_accel_rad_s2 >= 0.0F`, `open_loop_voltage_start >= 0.0F`, and
 *   `open_loop_voltage_max >= open_loop_voltage_start`.
 * @retval MC_STATUS_OK Initialization completed successfully.
 * @retval MC_STATUS_INVALID_ARG `state == NULL`, `cfg == NULL`, or any configuration constraint above is violated.
 * @note On success, the state is cleared and `open_loop_active` starts as `MC_TRUE`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `state` object.
 */
mc_status_t mc_sensorless_init(mc_sensorless_state_t *state, const mc_sensorless_cfg_t *cfg);

/**
 * @brief Update the sensorless observer with one voltage/current sample pair
 * @param[in,out] state Pointer to sensorless observer state storage.
 *   Range: non-NULL pointer to writable `mc_sensorless_state_t` storage.
 * @param[in] voltage_ab Applied alpha-beta voltage vector.
 *   Range: non-NULL pointer to readable `mc_alphabeta_t` storage.
 * @param[in] current_ab Measured alpha-beta current vector.
 *   Range: non-NULL pointer to readable `mc_alphabeta_t` storage.
 * @param[in] dt_s Sample interval [s].
 *   Range: `dt_s > 0.0F`.
 * @param[in] timestamp_us Caller-supplied timestamp [us].
 *   Range: any `uint32_t`; stored for diagnostics/state tracking.
 * @retval MC_STATUS_OK Update completed successfully.
 * @retval MC_STATUS_INVALID_ARG `state == NULL`, `voltage_ab == NULL`, `current_ab == NULL`, or `dt_s <= 0.0F`.
 * @note When `bemf_magnitude < cfg.min_bemf`, the observer is invalidated, lock/debounce state is cleared, and the function still returns `MC_STATUS_OK`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `state` object. Not reentrant for concurrent writes to the same `state`.
 */
mc_status_t mc_sensorless_update(mc_sensorless_state_t *state,
                                 const mc_alphabeta_t *voltage_ab,
                                 const mc_alphabeta_t *current_ab,
                                 mc_f32_t dt_s,
                                 uint32_t timestamp_us);

#endif /* MC_SENSOR_SENSORLESS_H */
