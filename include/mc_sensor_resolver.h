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
 * @param state Pointer to resolver state structure
 * @param cfg Pointer to resolver configuration
 * @return MC_OK on success, or an error code
 */
mc_status_t mc_resolver_init(mc_resolver_state_t *state, const mc_resolver_cfg_t *cfg);

/**
 * @brief Get the resolver excitation command values
 * @param state Pointer to resolver state structure
 * @param amplitude Pointer to store excitation amplitude
 * @param frequency_hz Pointer to store excitation frequency in Hz
 * @return MC_OK on success, or an error code
 */
mc_status_t mc_resolver_get_excitation_command(const mc_resolver_state_t *state, int16_t *amplitude, uint16_t *frequency_hz);

/**
 * @brief Update resolver state with latest raw data and timestamp
 * @param state Pointer to resolver state structure
 * @param raw Pointer to raw resolver data
 * @param timestamp_us Current timestamp in microseconds
 * @return MC_OK on success, or an error code
 */
mc_status_t mc_resolver_update(mc_resolver_state_t *state, const mc_resolver_raw_t *raw, uint32_t timestamp_us);

#endif /* MC_SENSOR_RESOLVER_H */
