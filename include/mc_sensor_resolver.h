/**
 * @file mc_sensor_resolver.h
 * @brief Resolver sensor interface
 */

#ifndef MC_SENSOR_RESOLVER_H
#define MC_SENSOR_RESOLVER_H

#include "mc_port_resolver.h"
#include "mc_status.h"
#include "mc_types.h"

/**
 * @brief Resolver configuration parameters
 */
typedef struct
{
    int16_t excitation_amplitude;       /**< Excitation signal amplitude */
    uint16_t excitation_frequency_hz;   /**< Excitation signal frequency in Hz */
    mc_f32_t signal_scale;              /**< Resolver signal scaling factor */
    mc_f32_t min_signal_amplitude;      /**< Minimum valid signal amplitude threshold */
    mc_f32_t pole_pairs;                /**< Number of motor pole pairs */
} mc_resolver_cfg_t;

/**
 * @brief Resolver state information
 */
typedef struct
{
    mc_resolver_cfg_t cfg;          /**< Resolver configuration */
    mc_f32_t sin_component;         /**< Sin component of resolver signal */
    mc_f32_t cos_component;         /**< Cos component of resolver signal */
    mc_f32_t elec_angle_rad;        /**< Estimated electrical angle in radians */
    mc_f32_t mech_speed_rpm;        /**< Estimated mechanical speed in RPM */
    mc_f32_t signal_amplitude;      /**< Measured signal amplitude */
    mc_bool_t signal_valid;         /**< Flag indicating signal is above minimum threshold */
    mc_f32_t last_mech_angle_rad;   /**< Previous mechanical angle in radians */
    uint32_t last_timestamp_us;     /**< Timestamp of previous resolver read in microseconds */
} mc_resolver_state_t;

/**
 * @brief Initialise resolver state with configuration
 * @param[out] state Pointer to resolver state structure.
 *   Range: non-NULL pointer to writable `mc_resolver_state_t` storage.
 * @param[in] cfg Pointer to resolver configuration.
 *   Range: non-NULL pointer to readable `mc_resolver_cfg_t` storage with `pole_pairs > 0.0F` and `signal_scale > 0.0F`.
 * @retval MC_STATUS_OK Initialization completed successfully.
 * @retval MC_STATUS_INVALID_ARG `state == NULL`, `cfg == NULL`, `cfg->pole_pairs <= 0.0F`, or `cfg->signal_scale <= 0.0F`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `state` object.
 */
mc_status_t mc_resolver_init(mc_resolver_state_t *state, const mc_resolver_cfg_t *cfg);

/**
 * @brief Get the resolver excitation command values
 * @param[in] state Pointer to resolver state structure.
 *   Range: non-NULL pointer to readable `mc_resolver_state_t` storage.
 * @param[out] amplitude Pointer to store excitation amplitude.
 *   Range: non-NULL pointer to writable `int16_t` storage.
 * @param[out] frequency_hz Pointer to store excitation frequency in Hz.
 *   Range: non-NULL pointer to writable `uint16_t` storage.
 * @retval MC_STATUS_OK Stored excitation command values were written successfully.
 * @retval MC_STATUS_INVALID_ARG `state == NULL`, `amplitude == NULL`, or `frequency_hz == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant for concurrent read-only calls when each call uses different writable output storage.
 */
mc_status_t mc_resolver_get_excitation_command(const mc_resolver_state_t *state, int16_t *amplitude, uint16_t *frequency_hz);

/**
 * @brief Update resolver state with latest raw data and timestamp
 * @param[in,out] state Pointer to resolver state structure.
 *   Range: non-NULL pointer to writable `mc_resolver_state_t` storage previously initialized with `mc_resolver_init()`.
 * @param[in] raw Pointer to raw resolver data.
 *   Range: non-NULL pointer to readable `mc_resolver_raw_t` storage.
 * @param[in] timestamp_us Current timestamp in microseconds.
 *   Range: any `uint32_t`; monotonically increasing values are recommended for valid speed estimation.
 * @retval MC_STATUS_OK Update completed successfully.
 * @retval MC_STATUS_INVALID_ARG `state == NULL` or `raw == NULL`.
 * @note When the measured resolver signal amplitude falls below `cfg.min_signal_amplitude`, `signal_valid` is cleared and `mech_speed_rpm` is forced to `0.0F`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `state` object. Not reentrant for concurrent writes to the same `state`.
 */
mc_status_t mc_resolver_update(mc_resolver_state_t *state, const mc_resolver_raw_t *raw, uint32_t timestamp_us);

#endif /* MC_SENSOR_RESOLVER_H */
