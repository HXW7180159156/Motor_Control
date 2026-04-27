/**
 * @file mc_port_adc_map.c
 * @brief ADC trigger mapping implementation
 */
#include "mc_port_adc_map.h"

/**
 * @brief Map a single logical ADC trigger to a hardware trigger
 * @param trigger Logical trigger configuration
 * @param compare_index PWM compare index for this trigger
 * @param hw_trigger Output hardware trigger
 */
static void mc_adc_map_single_trigger(const mc_adc_trigger_t *trigger,
                                      uint8_t compare_index,
                                      mc_adc_hw_trigger_t *hw_trigger)
{
    if ((trigger == NULL) || (hw_trigger == NULL))
    {
        return;
    }

    *hw_trigger = (mc_adc_hw_trigger_t){0};
    if (trigger->valid == MC_FALSE)
    {
        return;
    }

    hw_trigger->compare_index = compare_index;
    hw_trigger->soc_index = trigger->soc_index;
    hw_trigger->event = trigger->event;
    hw_trigger->signal = trigger->signal;
    hw_trigger->compare_time_s = trigger->time_s;
    hw_trigger->compare_position = trigger->position;
}

/**
 * @brief Map a complete ADC trigger plan to hardware trigger map
 * @param plan Logical trigger plan (up to 2 triggers)
 * @param hw_map Output hardware trigger map
 */
void mc_adc_map_trigger_plan(const mc_adc_trigger_plan_t *plan,
                             mc_adc_hw_trigger_map_t *hw_map)
{
    if ((plan == NULL) || (hw_map == NULL))
    {
        return;
    }

    *hw_map = (mc_adc_hw_trigger_map_t){0};
    hw_map->count = plan->count;
    mc_adc_map_single_trigger(&plan->trigger_a, 0U, &hw_map->trigger_a);
    mc_adc_map_single_trigger(&plan->trigger_b, 1U, &hw_map->trigger_b);
}
