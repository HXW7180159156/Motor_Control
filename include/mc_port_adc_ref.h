#ifndef MC_PORT_ADC_REF_H
#define MC_PORT_ADC_REF_H

#include "mc_port_adc_map.h"

typedef struct
{
    mc_bool_t enable;
    uint8_t compare_index;
    uint8_t soc_index;
    mc_adc_trigger_event_t event;
    mc_adc_trigger_signal_t signal;
    mc_f32_t compare_time_s;
} mc_adc_ref_soc_cfg_t;

typedef struct
{
    uint8_t count;
    mc_adc_ref_soc_cfg_t soc_a;
    mc_adc_ref_soc_cfg_t soc_b;
} mc_adc_ref_apply_cfg_t;

typedef struct
{
    void (*set_compare)(uint8_t compare_index, mc_f32_t compare_time_s, mc_adc_trigger_event_t event);
    void (*set_soc)(uint8_t soc_index, mc_adc_trigger_signal_t signal, mc_bool_t enable);
} mc_adc_ref_port_t;

void mc_adc_ref_build_apply_cfg(const mc_adc_hw_trigger_map_t *hw_map,
                                mc_adc_ref_apply_cfg_t *apply_cfg);
void mc_adc_ref_apply(const mc_adc_ref_apply_cfg_t *apply_cfg,
                      const mc_adc_ref_port_t *port);
void mc_adc_ref_apply_plan(const mc_adc_trigger_plan_t *plan,
                           const mc_adc_ref_port_t *port);

#endif /* MC_PORT_ADC_REF_H */
