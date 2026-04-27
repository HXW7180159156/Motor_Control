/** @file mc_port_adc_map.h @brief Logical-to-hardware ADC trigger mapping helpers */

#ifndef MC_PORT_ADC_MAP_H
#define MC_PORT_ADC_MAP_H

#include "mc_port_adc.h"

/** @brief Hardware-facing ADC trigger description */
typedef struct
{
    uint8_t compare_index;
    uint8_t soc_index;
    mc_adc_trigger_event_t event;
    mc_adc_trigger_signal_t signal;
    mc_f32_t compare_time_s;
    mc_f32_t compare_position;
} mc_adc_hw_trigger_t;

/** @brief Hardware trigger map for up to two ADC trigger entries */
typedef struct
{
    uint8_t count;
    mc_adc_hw_trigger_t trigger_a;
    mc_adc_hw_trigger_t trigger_b;
} mc_adc_hw_trigger_map_t;

/**
 * @brief Map a logical ADC trigger plan into a hardware trigger map
 * @param[in] plan Logical ADC trigger plan.
 *   Range: readable `mc_adc_trigger_plan_t` storage or `NULL`.
 * @param[out] hw_map Hardware trigger map output.
 *   Range: writable `mc_adc_hw_trigger_map_t` storage or `NULL`.
 * @return None.
 *   Range: not applicable.
 * @note If `plan == NULL` or `hw_map == NULL`, the call has no effect.
 * @note The current implementation maps only `trigger_a` and `trigger_b`, assigning compare indices `0` and `1` respectively.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when concurrent calls do not share writable `hw_map` storage.
 */
void mc_adc_map_trigger_plan(const mc_adc_trigger_plan_t *plan,
                              mc_adc_hw_trigger_map_t *hw_map);

#endif /* MC_PORT_ADC_MAP_H */
