/** @file mc_reconstruct_3shunt.c @brief 3-shunt phase current reconstruction */

#include "mc_constants.h"
#include "mc_reconstruct_3shunt.h"

#include "mc_math.h"

/**
 * @brief Reconstruct phase currents from three shunt ADC samples
 * @param raw Raw ADC sample data (phases A, B, C)
 * @param cfg 3-shunt configuration parameters
 * @param phase_current_abc [out] Reconstructed three-phase currents
 */
void mc_reconstruct_3shunt_run(const mc_adc_raw_t *raw, const mc_3shunt_cfg_t *cfg, mc_abc_t *phase_current_abc)
{
    if ((raw == NULL) || (cfg == NULL) || (phase_current_abc == NULL))
    {
        return;
    }

    phase_current_abc->a = (((mc_f32_t)raw->phase_a_raw) - cfg->offset_a) * cfg->scale_a;
    phase_current_abc->b = (((mc_f32_t)raw->phase_b_raw) - cfg->offset_b) * cfg->scale_b;
    phase_current_abc->c = (((mc_f32_t)raw->phase_c_raw) - cfg->offset_c) * cfg->scale_c;
}

/**
 * @brief Reconstruct phase currents from three shunt ADC samples in Q31
 * @param raw Raw ADC sample data (phases A, B, C)
 * @param cfg 3-shunt Q31 configuration parameters
 * @param phase_current_abc [out] Reconstructed three-phase currents in Q31
 */
void mc_reconstruct_3shunt_q31_run(const mc_adc_raw_t *raw, const mc_3shunt_q31_cfg_t *cfg, mc_abc_q31_t *phase_current_abc)
{
    mc_q31_t raw_a;
    mc_q31_t raw_b;
    mc_q31_t raw_c;

    if ((raw == NULL) || (cfg == NULL) || (phase_current_abc == NULL))
    {
        return;
    }

    raw_a = mc_q31_from_f32((mc_f32_t)raw->phase_a_raw / MC_ADC_FULL_SCALE);
    raw_b = mc_q31_from_f32((mc_f32_t)raw->phase_b_raw / MC_ADC_FULL_SCALE);
    raw_c = mc_q31_from_f32((mc_f32_t)raw->phase_c_raw / MC_ADC_FULL_SCALE);

    phase_current_abc->a = mc_q31_mul(mc_q31_add_sat(raw_a, -cfg->offset_a), cfg->scale_a);
    phase_current_abc->b = mc_q31_mul(mc_q31_add_sat(raw_b, -cfg->offset_b), cfg->scale_b);
    phase_current_abc->c = mc_q31_mul(mc_q31_add_sat(raw_c, -cfg->offset_c), cfg->scale_c);
}
