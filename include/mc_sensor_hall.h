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
 * @param state Pointer to hall sensor state structure
 * @param cfg Pointer to hall sensor configuration
 * @return MC_OK on success, or an error code
 */
mc_status_t mc_hall_init(mc_hall_state_t *state, const mc_hall_cfg_t *cfg);

/**
 * @brief Update hall sensor state with latest code and timestamp
 * @param state Pointer to hall sensor state structure
 * @param hall_code Current hall sensor code
 * @param timestamp_us Current timestamp in microseconds
 * @param pole_pairs Number of motor pole pairs
 * @return MC_OK on success, or an error code
 */
mc_status_t mc_hall_update(mc_hall_state_t *state, uint8_t hall_code, uint32_t timestamp_us, mc_f32_t pole_pairs);

#endif /* MC_SENSOR_HALL_H */
