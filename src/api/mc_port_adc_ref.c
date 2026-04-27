/**
 * @file mc_port_adc_ref.c
 * @brief ADC reference port implementation
 */
#include "mc_port_adc_ref.h"

#include "mc_port_adc_map.h"

/**
 * @brief Map a hardware trigger to an ADC reference SoC configuration
 * @param hw_trigger Hardware trigger to map
 * @param soc_cfg Output SoC configuration
 */
static void mc_adc_ref_map_soc(const mc_adc_hw_trigger_t *hw_trigger,
                               mc_adc_ref_soc_cfg_t *soc_cfg)
{
    if ((hw_trigger == NULL) || (soc_cfg == NULL))
    {
        return;
    }

    *soc_cfg = (mc_adc_ref_soc_cfg_t){0};
    if (hw_trigger->signal == MC_ADC_TRIGGER_NONE)
    {
        return;
    }

    soc_cfg->enable = MC_TRUE;
    soc_cfg->compare_index = hw_trigger->compare_index;
    soc_cfg->soc_index = hw_trigger->soc_index;
    soc_cfg->event = hw_trigger->event;
    soc_cfg->signal = hw_trigger->signal;
    soc_cfg->compare_time_s = hw_trigger->compare_time_s;
}

/**
 * @brief Build ADC reference apply configuration from a hardware trigger map
 * @param hw_map Hardware trigger map
 * @param apply_cfg Output apply configuration
 */
void mc_adc_ref_build_apply_cfg(const mc_adc_hw_trigger_map_t *hw_map,
                                mc_adc_ref_apply_cfg_t *apply_cfg)
{
    if ((hw_map == NULL) || (apply_cfg == NULL))
    {
        return;
    }

    *apply_cfg = (mc_adc_ref_apply_cfg_t){0};
    apply_cfg->count = hw_map->count;
    mc_adc_ref_map_soc(&hw_map->trigger_a, &apply_cfg->soc_a);
    mc_adc_ref_map_soc(&hw_map->trigger_b, &apply_cfg->soc_b);
}

/**
 * @brief Apply a single SoC configuration to the ADC reference port
 * @param soc_cfg SoC configuration to apply
 * @param port ADC reference port with set_compare and set_soc callbacks
 */
static void mc_adc_ref_apply_single_soc(const mc_adc_ref_soc_cfg_t *soc_cfg,
                                        const mc_adc_ref_port_t *port)
{
    if ((soc_cfg == NULL) || (port == NULL))
    {
        return;
    }

    if (port->set_compare != NULL)
    {
        port->set_compare(soc_cfg->compare_index, soc_cfg->compare_time_s, soc_cfg->event);
    }

    if (port->set_soc != NULL)
    {
        port->set_soc(soc_cfg->soc_index, soc_cfg->signal, soc_cfg->enable);
    }
}

/**
 * @brief Apply full ADC reference configuration to port (both SoC A and B)
 * @param apply_cfg Apply configuration
 * @param port ADC reference port
 */
void mc_adc_ref_apply(const mc_adc_ref_apply_cfg_t *apply_cfg,
                      const mc_adc_ref_port_t *port)
{
    if ((apply_cfg == NULL) || (port == NULL))
    {
        return;
    }

    mc_adc_ref_apply_single_soc(&apply_cfg->soc_a, port);
    mc_adc_ref_apply_single_soc(&apply_cfg->soc_b, port);
}

/**
 * @brief Apply a logical ADC trigger plan directly to the reference port
 * @param plan Logical trigger plan
 * @param port ADC reference port
 */
void mc_adc_ref_apply_plan(const mc_adc_trigger_plan_t *plan,
                           const mc_adc_ref_port_t *port)
{
    mc_adc_hw_trigger_map_t hw_map;
    mc_adc_ref_apply_cfg_t apply_cfg;

    if ((plan == NULL) || (port == NULL))
    {
        return;
    }

    mc_adc_map_trigger_plan(plan, &hw_map);
    mc_adc_ref_build_apply_cfg(&hw_map, &apply_cfg);
    mc_adc_ref_apply(&apply_cfg, port);
}
