/**
 * @file mc_reconstruct_1shunt.h
 * @brief Single-shunt current reconstruction module
 *
 * Provides planning and execution logic for reconstructing three-phase motor
 * currents from a single DC-link shunt resistor measurement.
 */

#ifndef MC_RECONSTRUCT_1SHUNT_H
#define MC_RECONSTRUCT_1SHUNT_H

#include "mc_port_adc.h"
#include "mc_port_pwm.h"
#include "mc_transform.h"

/**
 * @brief Single-shunt configuration parameters
 */
typedef struct
{
    mc_f32_t offset;                   /**< ADC offset correction value */
    mc_f32_t scale;                    /**< Current scaling factor (A/LSB) */
    mc_f32_t min_active_duty;          /**< Minimum active duty cycle for valid measurement */
    mc_f32_t sample_window_margin;     /**< Margin subtracted from sample window (s) */
    mc_f32_t pwm_period_s;             /**< PWM period in seconds */
    mc_f32_t deadtime_s;               /**< Dead-time duration in seconds */
    mc_f32_t adc_aperture_s;           /**< ADC sampling aperture time in seconds */
} mc_1shunt_cfg_t;

/**
 * @brief Identifies which phase edge is used for sampling
 */
typedef enum
{
    MC_1SHUNT_SAMPLE_PHASE_NONE = 0,   /**< No sample phase selected */
    MC_1SHUNT_SAMPLE_PHASE_LEADING,    /**< Sample on the leading edge */
    MC_1SHUNT_SAMPLE_PHASE_TRAILING    /**< Sample on the trailing edge */
} mc_1shunt_sample_phase_t;

/**
 * @brief Identifies which half of the PWM cycle is used
 */
typedef enum
{
    MC_1SHUNT_HALF_CYCLE_NONE = 0,     /**< No half-cycle selected */
    MC_1SHUNT_HALF_CYCLE_UP,           /**< Upper (rising) half-cycle */
    MC_1SHUNT_HALF_CYCLE_DOWN          /**< Lower (falling) half-cycle */
} mc_1shunt_half_cycle_t;

/**
 * @brief Metadata for a single-shunt sampling plan
 */
typedef struct
{
    uint8_t sector;                      /**< Current SVM sector number */
    mc_bool_t sample_valid;              /**< Whether at least one valid sample is available */
    uint8_t sample_count;                /**< Number of valid samples obtained */
    mc_bool_t compensation_required;     /**< Flag indicating dead-time compensation is needed */
    mc_bool_t zero_vector_bias_high;     /**< Bias direction of the zero vector */
    mc_bool_t reorder_required;          /**< Flag indicating phase reordering is needed */
    uint8_t preferred_window;            /**< Index of the preferred sampling window */
    mc_f32_t sample_window_a;            /**< Duration of first sampling window (s) */
    mc_f32_t sample_window_b;            /**< Duration of second sampling window (s) */
    mc_1shunt_sample_phase_t sample_phase_a;   /**< Phase edge for first sample */
    mc_1shunt_sample_phase_t sample_phase_b;   /**< Phase edge for second sample */
    mc_1shunt_half_cycle_t sample_half_a;      /**< Half-cycle for first sample */
    mc_1shunt_half_cycle_t sample_half_b;      /**< Half-cycle for second sample */
    mc_f32_t sample_position_a;          /**< Timing position of first sample (s) */
    mc_f32_t sample_position_b;          /**< Timing position of second sample (s) */
    mc_f32_t sample_half_position_a;     /**< Half-cycle position of first sample (s) */
    mc_f32_t sample_half_position_b;     /**< Half-cycle position of second sample (s) */
    mc_f32_t sample_time_a;              /**< Absolute time of first sample (s) */
    mc_f32_t sample_time_b;              /**< Absolute time of second sample (s) */
    mc_f32_t sample_half_time_a;         /**< Absolute half-cycle time of first sample (s) */
    mc_f32_t sample_half_time_b;         /**< Absolute half-cycle time of second sample (s) */
} mc_1shunt_meta_t;

/**
 * @brief Plan the single-shunt sampling strategy for the given PWM command
 *
 * Analyses the PWM command and configuration to determine sampling windows,
 * phase edges, and compensation requirements.
 *
 * @param[in] pwm_cmd Pointer to the PWM command structure.
 *   Range: readable `mc_pwm_cmd_t` storage or `NULL`.
 * @param[in] cfg Pointer to the single-shunt configuration.
 *   Range: readable `mc_1shunt_cfg_t` storage or `NULL`.
 * @param[out] meta Pointer to the metadata structure to populate with the sampling plan.
 *   Range: writable `mc_1shunt_meta_t` storage or `NULL`.
 * @return None.
 *   Range: not applicable.
 * @note If any pointer is `NULL`, the call has no effect.
 * @note When `cfg->pwm_period_s <= 0.0F`, the planning logic uses an internal fallback period of `1.0F` second-equivalent units.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when concurrent calls do not share writable `meta` storage.
 */
void mc_reconstruct_1shunt_plan(const mc_pwm_cmd_t *pwm_cmd,
                                 const mc_1shunt_cfg_t *cfg,
                                 mc_1shunt_meta_t *meta);

/**
 * @brief Reconstruct three-phase currents from single-shunt ADC samples
 *
 * Uses the pre-computed sampling plan and raw ADC readings to reconstruct
 * the instantaneous phase currents.
 *
 * @param[in] raw Pointer to the raw ADC sample data.
 *   Range: readable `mc_adc_raw_t` storage or `NULL`.
 * @param[in] cfg Pointer to the single-shunt configuration.
 *   Range: readable `mc_1shunt_cfg_t` storage or `NULL`.
 * @param[in] meta Pointer to the sampling plan metadata.
 *   Range: readable `mc_1shunt_meta_t` storage or `NULL`.
 * @param[out] phase_current_abc Pointer to the output structure for reconstructed phase currents.
 *   Range: writable `mc_abc_t` storage or `NULL`.
 * @return None.
 *   Range: not applicable.
 * @note If any pointer is `NULL`, the call has no effect.
 * @note The output is cleared to zero first; when `meta->sample_valid == MC_FALSE`, the output remains zero.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when concurrent calls do not share writable `phase_current_abc` storage.
 */
void mc_reconstruct_1shunt_run(const mc_adc_raw_t *raw,
                               const mc_1shunt_cfg_t *cfg,
                               const mc_1shunt_meta_t *meta,
                               mc_abc_t *phase_current_abc);

#endif /* MC_RECONSTRUCT_1SHUNT_H */
