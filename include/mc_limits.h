#ifndef MC_LIMITS_H
#define MC_LIMITS_H

/** @file mc_limits.h @brief Hardware limit structure definition */

#include "mc_types.h"

/** @brief Hardware operating limits for the motor drive */
typedef struct
{
    mc_f32_t bus_voltage_min_v;    /**< Minimum DC bus voltage in volts */
    mc_f32_t bus_voltage_max_v;    /**< Maximum DC bus voltage in volts */
    mc_f32_t phase_current_max_a;  /**< Maximum phase current in amperes */
    mc_f32_t temperature_max_deg_c;/**< Maximum temperature in degrees Celsius */
    mc_f32_t speed_max_rpm;        /**< Maximum motor speed in RPM */
} mc_hw_limits_t;

#endif /* MC_LIMITS_H */
