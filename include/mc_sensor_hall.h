/**
 * @file mc_sensor_hall.h
 * @brief Hall-effect sensor interface
 */

#ifndef MC_SENSOR_HALL_H
#define MC_SENSOR_HALL_H

#include "mc_status.h"
#include "mc_types.h"

/**
 * @brief Hall sensor configuration parameters
 */
typedef struct
{
    uint8_t hall_code_sequence[6];      /**< Ordered sequence of 6 hall codes */
    mc_f32_t elec_angle_table_rad[6];   /**< Electrical angle for each hall sector in radians */
} mc_hall_cfg_t;

/**
 * @brief Hall sensor state information
 */
typedef struct
{
    mc_hall_cfg_t cfg;              /**< Hall sensor configuration */
    uint8_t last_hall_code;         /**< Previous hall sensor code */
    mc_f32_t elec_angle_rad;        /**< Estimated electrical angle in radians */
    mc_f32_t mech_speed_rpm;        /**< Estimated mechanical speed in RPM */
    uint32_t last_transition_us;    /**< Timestamp of last hall transition in microseconds */
} mc_hall_state_t;

/**
 * @brief Initialise hall sensor state with configuration
 * @param[out] state Pointer to hall sensor state structure.
 *   Range: non-NULL pointer to writable `mc_hall_state_t` storage.
 * @param[in] cfg Pointer to hall sensor configuration.
 *   Range: non-NULL pointer to readable `mc_hall_cfg_t` storage.
 * @retval MC_STATUS_OK Initialization completed successfully.
 * @retval MC_STATUS_INVALID_ARG `state == NULL` or `cfg == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `state` object.
 */
mc_status_t mc_hall_init(mc_hall_state_t *state, const mc_hall_cfg_t *cfg);

/**
 * @brief Update hall sensor state with latest code and timestamp
 * @param[in,out] state Pointer to hall sensor state structure.
 *   Range: non-NULL pointer to writable `mc_hall_state_t` storage previously initialized with `mc_hall_init()`.
 * @param[in] hall_code Current Hall sensor code.
 *   Range: any `uint8_t`; the value must match one of the six entries in `state->cfg.hall_code_sequence`.
 * @param[in] timestamp_us Current timestamp in microseconds.
 *   Range: any `uint32_t`; monotonically increasing values are recommended for valid speed estimation.
 * @param[in] pole_pairs Number of motor pole pairs.
 *   Range: `pole_pairs > 0.0F`.
 * @retval MC_STATUS_OK Update completed successfully.
 * @retval MC_STATUS_INVALID_ARG `state == NULL`, `pole_pairs <= 0.0F`, or `hall_code` is not found in the configured Hall sequence.
 * @note Mechanical speed is updated only when a previous transition exists, the timestamp increases, and the Hall code changes.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `state` object. Not reentrant for concurrent writes to the same `state`.
 */
mc_status_t mc_hall_update(mc_hall_state_t *state, uint8_t hall_code, uint32_t timestamp_us, mc_f32_t pole_pairs);

#endif /* MC_SENSOR_HALL_H */
