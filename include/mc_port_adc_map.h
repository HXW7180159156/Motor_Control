#ifndef MC_PORT_ADC_MAP_H
#define MC_PORT_ADC_MAP_H

#include "mc_port_adc.h"

typedef struct
{
    uint8_t compare_index;
    uint8_t soc_index;
    mc_adc_trigger_event_t event;
    mc_adc_trigger_signal_t signal;
    mc_f32_t compare_time_s;
    mc_f32_t compare_position;
} mc_adc_hw_trigger_t;

typedef struct
{
    uint8_t count;
    mc_adc_hw_trigger_t trigger_a;
    mc_adc_hw_trigger_t trigger_b;
} mc_adc_hw_trigger_map_t;

void mc_adc_map_trigger_plan(const mc_adc_trigger_plan_t *plan,
                             mc_adc_hw_trigger_map_t *hw_map);

#endif /* MC_PORT_ADC_MAP_H */
