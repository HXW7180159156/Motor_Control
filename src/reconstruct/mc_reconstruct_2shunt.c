/** @file mc_reconstruct_2shunt.c @brief 2-shunt phase current reconstruction */

#include "mc_reconstruct_2shunt.h"

#include "mc_math.h"

/**
 * @brief Reconstruct phase currents from two shunt ADC samples
 * @param raw Raw ADC sample data (phase A and B)
 * @param cfg 2-shunt configuration parameters
 * @param phase_current_abc [out] Reconstructed three-phase currents
 */
void mc_reconstruct_2shunt_run(const mc_adc_raw_t *raw, const mc_2shunt_cfg_t *cfg, mc_abc_t *phase_current_abc)
{
    mc_f32_t ia;
    mc_f32_t ib;

    if ((raw == NULL) || (cfg == NULL) || (phase_current_abc == NULL))
    {
        return;
    }

    ia = (((mc_f32_t)raw->phase_a_raw) - cfg->offset_a) * cfg->scale_a;
    ib = (((mc_f32_t)raw->phase_b_raw) - cfg->offset_b) * cfg->scale_b;

    phase_current_abc->a = ia;
    phase_current_abc->b = ib;
    phase_current_abc->c = -(ia + ib);
}

void mc_reconstruct_2shunt_q31_run(const mc_adc_raw_t *raw, const mc_2shunt_q31_cfg_t *cfg, mc_abc_q31_t *phase_current_abc)
{
    mc_q31_t ia;
    mc_q31_t ib;
    mc_q31_t raw_a;
    mc_q31_t raw_b;

    if ((raw == NULL) || (cfg == NULL) || (phase_current_abc == NULL))
    {
        return;
    }

    raw_a = mc_q31_from_f32((mc_f32_t)raw->phase_a_raw / 4095.0F);
    raw_b = mc_q31_from_f32((mc_f32_t)raw->phase_b_raw / 4095.0F);
    ia = mc_q31_mul(mc_q31_add_sat(raw_a, -cfg->offset_a), cfg->scale_a);
    ib = mc_q31_mul(mc_q31_add_sat(raw_b, -cfg->offset_b), cfg->scale_b);

    phase_current_abc->a = ia;
    phase_current_abc->b = ib;
    phase_current_abc->c = -mc_q31_add_sat(ia, ib);
}
