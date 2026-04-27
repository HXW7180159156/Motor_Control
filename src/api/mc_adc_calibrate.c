/**
 * @file mc_adc_calibrate.c
 * @brief ADC linear calibration implementation
 */
#include "mc_port_adc.h"

/**
 * @brief Compute linear scale and offset from two calibration points
 * @param raw_low Raw ADC value at low calibration point
 * @param physical_low Physical value at low calibration point
 * @param raw_high Raw ADC value at high calibration point
 * @param physical_high Physical value at high calibration point
 * @param scale Output scale factor (physical per raw unit)
 * @param offset Output offset
 * @return MC_STATUS_OK on success, MC_STATUS_INVALID_ARG if scale/offset NULL or raw points equal
 */
mc_status_t mc_adc_calibrate_linear(uint16_t raw_low, mc_f32_t physical_low,
                                    uint16_t raw_high, mc_f32_t physical_high,
                                    mc_f32_t *scale, mc_f32_t *offset)
{
    if ((scale == NULL) || (offset == NULL))
    {
        return MC_STATUS_INVALID_ARG;
    }
    if (raw_high == raw_low)
    {
        return MC_STATUS_INVALID_ARG;
    }

    *scale = (physical_high - physical_low) / ((mc_f32_t)(raw_high - raw_low));
    *offset = physical_low - ((mc_f32_t)raw_low * (*scale));

    return MC_STATUS_OK;
}
