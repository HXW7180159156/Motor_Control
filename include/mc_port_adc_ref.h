/** @file mc_port_adc_ref.h @brief ADC reference-port mapping and apply helpers */

#ifndef MC_PORT_ADC_REF_H
#define MC_PORT_ADC_REF_H

#include "mc_port_adc_map.h"

/** @brief Reference-port SoC configuration for one logical ADC trigger */
typedef struct
{
    mc_bool_t enable;
    uint8_t compare_index;
    uint8_t soc_index;
    mc_adc_trigger_event_t event;
    mc_adc_trigger_signal_t signal;
    mc_f32_t compare_time_s;
} mc_adc_ref_soc_cfg_t;

/** @brief Aggregated reference-port configuration for up to two SoCs */
typedef struct
{
    uint8_t count;
    mc_adc_ref_soc_cfg_t soc_a;
    mc_adc_ref_soc_cfg_t soc_b;
} mc_adc_ref_apply_cfg_t;

/** @brief Platform callbacks used to apply ADC reference compare and SoC settings */
typedef struct
{
    void (*set_compare)(uint8_t compare_index, mc_f32_t compare_time_s, mc_adc_trigger_event_t event);
    void (*set_soc)(uint8_t soc_index, mc_adc_trigger_signal_t signal, mc_bool_t enable);
} mc_adc_ref_port_t;

/**
 * @brief Build a reference-port apply configuration from a hardware trigger map
 * @param[in] hw_map Hardware trigger map.
 *   Range: readable `mc_adc_hw_trigger_map_t` storage or `NULL`.
 * @param[out] apply_cfg Output configuration for later application.
 *   Range: writable `mc_adc_ref_apply_cfg_t` storage or `NULL`.
 * @return None.
 *   Range: not applicable.
 * @note If `hw_map == NULL` or `apply_cfg == NULL`, the call has no effect.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when concurrent calls do not share writable `apply_cfg` storage.
 */
void mc_adc_ref_build_apply_cfg(const mc_adc_hw_trigger_map_t *hw_map,
                                 mc_adc_ref_apply_cfg_t *apply_cfg);

/**
 * @brief Apply a prepared ADC reference configuration through platform callbacks
 * @param[in] apply_cfg Prepared ADC reference configuration.
 *   Range: readable `mc_adc_ref_apply_cfg_t` storage or `NULL`.
 * @param[in] port Reference-port callbacks.
 *   Range: readable `mc_adc_ref_port_t` storage or `NULL`.
 * @return None.
 *   Range: not applicable.
 * @note If `apply_cfg == NULL` or `port == NULL`, the call has no effect.
 * @note Each callback inside `port` is optional; missing callbacks are skipped.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant from the library side. Callback implementations may impose stricter platform-specific concurrency limits.
 */
void mc_adc_ref_apply(const mc_adc_ref_apply_cfg_t *apply_cfg,
                      const mc_adc_ref_port_t *port);

/**
 * @brief Map and apply a logical ADC trigger plan directly through the reference port
 * @param[in] plan Logical ADC trigger plan.
 *   Range: readable `mc_adc_trigger_plan_t` storage or `NULL`.
 * @param[in] port Reference-port callbacks.
 *   Range: readable `mc_adc_ref_port_t` storage or `NULL`.
 * @return None.
 *   Range: not applicable.
 * @note If `plan == NULL` or `port == NULL`, the call has no effect.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant from the library side. Callback implementations may impose stricter platform-specific concurrency limits.
 */
void mc_adc_ref_apply_plan(const mc_adc_trigger_plan_t *plan,
                            const mc_adc_ref_port_t *port);

#endif /* MC_PORT_ADC_REF_H */
