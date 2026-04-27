/**
 * @file mc_sensor_encoder.h
 * @brief Quadrature encoder sensor interface
 */

#ifndef MC_SENSOR_ENCODER_H
#define MC_SENSOR_ENCODER_H

#include "mc_status.h"
#include "mc_types.h"

/**
 * @brief Encoder configuration parameters
 */
typedef struct
{
    uint32_t counts_per_rev; /**< Encoder counts per full mechanical revolution */
    mc_f32_t pole_pairs;     /**< Number of motor pole pairs */
} mc_encoder_cfg_t;

/**
 * @brief Encoder state information
 */
typedef struct
{
    mc_encoder_cfg_t cfg;           /**< Encoder configuration */
    uint32_t last_count;            /**< Previous encoder count value */
    uint32_t last_timestamp_us;     /**< Timestamp of previous encoder read in microseconds */
    mc_f32_t elec_angle_rad;        /**< Estimated electrical angle in radians */
    mc_f32_t mech_speed_rpm;        /**< Estimated mechanical speed in RPM */
} mc_encoder_state_t;

/**
 * @brief Initialise encoder state with configuration
 * @param[out] state Pointer to encoder state structure.
 *   Range: non-NULL pointer to writable `mc_encoder_state_t` storage.
 * @param[in] cfg Pointer to encoder configuration.
 *   Range: non-NULL pointer to readable `mc_encoder_cfg_t` storage with `counts_per_rev > 0` and `pole_pairs > 0.0F`.
 * @retval MC_STATUS_OK Initialization completed successfully.
 * @retval MC_STATUS_INVALID_ARG `state == NULL`, `cfg == NULL`, `cfg->counts_per_rev == 0`, or `cfg->pole_pairs <= 0.0F`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `state` object.
 */
mc_status_t mc_encoder_init(mc_encoder_state_t *state, const mc_encoder_cfg_t *cfg);

/**
 * @brief Update encoder state with latest count and timestamp
 * @param[in,out] state Pointer to encoder state structure.
 *   Range: non-NULL pointer to writable `mc_encoder_state_t` storage previously initialized with `mc_encoder_init()`.
 * @param[in] encoder_count Current encoder counter value.
 *   Range: any `uint32_t`; the implementation wraps it modulo `state->cfg.counts_per_rev` for angle calculation.
 * @param[in] timestamp_us Current timestamp in microseconds.
 *   Range: any `uint32_t`; monotonically increasing values are recommended for valid speed estimation.
 * @retval MC_STATUS_OK Update completed successfully.
 * @retval MC_STATUS_INVALID_ARG `state == NULL`.
 * @note Mechanical speed is updated only when a previous timestamp exists and `timestamp_us > state->last_timestamp_us`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `state` object. Not reentrant for concurrent writes to the same `state`.
 */
mc_status_t mc_encoder_update(mc_encoder_state_t *state, uint32_t encoder_count, uint32_t timestamp_us);

#endif /* MC_SENSOR_ENCODER_H */
