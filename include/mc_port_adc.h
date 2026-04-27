#ifndef MC_PORT_ADC_H
#define MC_PORT_ADC_H

/** @file mc_port_adc.h @brief ADC port abstraction types and calibration API. */

#include "mc_types.h"
#include "mc_status.h"

/** @brief ADC trigger signal selection. */
typedef enum
{
    MC_ADC_TRIGGER_NONE = 0, /**< No trigger. */
    MC_ADC_TRIGGER_1SHUNT_BUS_CURRENT /**< Single-shunt bus current trigger. */
} mc_adc_trigger_signal_t;

/** @brief ADC trigger event edge selection. */
typedef enum
{
    MC_ADC_TRIGGER_EVENT_NONE = 0, /**< No event. */
    MC_ADC_TRIGGER_EVENT_PWM_UP, /**< Trigger on PWM up-count. */
    MC_ADC_TRIGGER_EVENT_PWM_DOWN /**< Trigger on PWM down-count. */
} mc_adc_trigger_event_t;

/** @brief Describes a single ADC trigger configuration. */
typedef struct
{
    mc_bool_t valid; /**< Whether this trigger entry is valid. */
    mc_adc_trigger_signal_t signal; /**< Trigger signal type. */
    mc_adc_trigger_event_t event; /**< Trigger event edge. */
    uint8_t sequence_index; /**< Index in the trigger sequence. */
    uint8_t total_count; /**< Total number of triggers in this sequence. */
    uint8_t soc_index; /**< Start-of-conversion index. */
    uint8_t sector; /**< PWM sector for this trigger. */
    uint8_t preferred_window; /**< Preferred sampling window. */
    mc_bool_t reorder_applied; /**< Whether reordering has been applied. */
    mc_f32_t time_s; /**< Trigger time in seconds. */
    mc_f32_t half_time_s; /**< Half-cycle trigger time in seconds. */
    mc_f32_t position; /**< Electrical position at trigger. */
    mc_f32_t half_position; /**< Electrical position at half-cycle. */
} mc_adc_trigger_t;

/** @brief Complete ADC trigger plan for one PWM period. */
typedef struct
{
    uint8_t count; /**< Number of triggers in this plan. */
    mc_adc_trigger_t trigger_a; /**< First trigger configuration. */
    mc_adc_trigger_t trigger_b; /**< Second trigger configuration. */
} mc_adc_trigger_plan_t;

/** @brief Raw ADC sample values. */
typedef struct
{
    uint16_t phase_a_raw; /**< Raw phase-A current sample. */
    uint16_t phase_b_raw; /**< Raw phase-B current sample. */
    uint16_t phase_c_raw; /**< Raw phase-C current sample. */
    uint16_t bus_voltage_raw; /**< Raw DC bus voltage sample. */
    uint16_t temperature_raw; /**< Raw temperature sensor sample. */
} mc_adc_raw_t;

/**
 * @brief Calibrate a linear ADC channel.
 * @param raw_low Raw ADC value at the low reference point.
 * @param physical_low Physical value at the low reference point.
 * @param raw_high Raw ADC value at the high reference point.
 * @param physical_high Physical value at the high reference point.
 * @param scale Output scale factor (gain).
 * @param offset Output offset (bias).
 * @return MC_STATUS_OK on success, or an error code.
 */
mc_status_t mc_adc_calibrate_linear(uint16_t raw_low, mc_f32_t physical_low,
                                    uint16_t raw_high, mc_f32_t physical_high,
                                    mc_f32_t *scale, mc_f32_t *offset);

#endif /* MC_PORT_ADC_H */
