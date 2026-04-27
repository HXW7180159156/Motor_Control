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
 * @param state Pointer to encoder state structure
 * @param cfg Pointer to encoder configuration
 * @return MC_OK on success, or an error code
 */
mc_status_t mc_encoder_init(mc_encoder_state_t *state, const mc_encoder_cfg_t *cfg);

/**
 * @brief Update encoder state with latest count and timestamp
 * @param state Pointer to encoder state structure
 * @param encoder_count Current encoder counter value
 * @param timestamp_us Current timestamp in microseconds
 * @return MC_OK on success, or an error code
 */
mc_status_t mc_encoder_update(mc_encoder_state_t *state, uint32_t encoder_count, uint32_t timestamp_us);

#endif /* MC_SENSOR_ENCODER_H */
