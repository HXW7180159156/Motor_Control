#ifndef MC_STATUS_H
#define MC_STATUS_H

/** @file mc_status.h @brief Status, fault, and warning enumerations */

#include "mc_types.h"

/** @brief General operation status codes */
typedef enum
{
    MC_STATUS_OK = 0,             /**< Operation completed successfully */
    MC_STATUS_ERROR,              /**< Unspecified error occurred */
    MC_STATUS_INVALID_ARG,        /**< Invalid argument passed to function */
    MC_STATUS_INVALID_STATE,      /**< Operation not allowed in current state */
    MC_STATUS_UNSUPPORTED,        /**< Requested feature is not supported */
    MC_STATUS_TIMEOUT             /**< Operation timed out */
} mc_status_t;

/** @brief Fault condition identifiers */
typedef enum
{
    MC_FAULT_NONE = 0,            /**< No fault */
    MC_FAULT_CONFIG,              /**< Configuration error */
    MC_FAULT_OVERCURRENT,         /**< Overcurrent condition detected */
    MC_FAULT_OVERVOLTAGE,         /**< Overvoltage condition detected */
    MC_FAULT_UNDERVOLTAGE,        /**< Undervoltage condition detected */
    MC_FAULT_SENSOR,              /**< Sensor fault detected */
    MC_FAULT_CONTROL,             /**< Control loop fault detected */
    MC_FAULT_INTERNAL             /**< Internal system fault */
} mc_fault_t;

/** @brief Warning condition identifiers */
typedef enum
{
    MC_WARNING_NONE = 0,              /**< No warning */
    MC_WARNING_LIMIT_ACTIVE,          /**< Hardware limit is active */
    MC_WARNING_DERATING_ACTIVE,       /**< Output is being derated */
    MC_WARNING_OBSERVER_UNLOCKED      /**< State observer has lost lock */
} mc_warning_t;

#endif /* MC_STATUS_H */
