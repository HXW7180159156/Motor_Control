/**
 * @file mc_diag.h
 * @brief Diagnostic interface for motor control subsystem.
 */

#ifndef MC_DIAG_H
#define MC_DIAG_H

#include "mc_drive_pmsm.h"
#include "mc_status.h"

/**
 * @brief Aggregated diagnostic status of the motor control subsystem.
 */
typedef struct
{
    mc_fault_t active_fault;                                /**< Currently active fault */
    mc_warning_t active_warning;                            /**< Currently active warning */
    uint32_t fault_counter;                                 /**< Count of occurred faults */
    uint32_t warning_counter;                               /**< Count of occurred warnings */
    mc_1shunt_comp_status_t current_comp_status;            /**< Current 1-shunt compensation status */
    mc_bool_t sensorless_observer_valid;                    /**< Sensorless observer validity flag */
    mc_bool_t sensorless_pll_locked;                        /**< Sensorless PLL lock status */
    mc_bool_t sensorless_open_loop_active;                  /**< Sensorless open-loop active flag */
} mc_diag_status_t;

/**
 * @brief Returns the human-readable name of a 1-shunt compensation mode.
 * @param[in] mode The 1-shunt compensation mode enumerator.
 *   Range: any `mc_1shunt_comp_mode_t` value.
 * @return Pointer to a null-terminated constant string describing the mode.
 *   Range: one of `"none"`, `"predict_basic"`, `"predict_high_modulation"`, `"predict_field_weakening"`, or `"unknown"` for unrecognized values.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
const char *mc_diag_1shunt_comp_mode_name(mc_1shunt_comp_mode_t mode);

#endif /* MC_DIAG_H */
