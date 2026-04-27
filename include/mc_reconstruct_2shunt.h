#ifndef MC_RECONSTRUCT_2SHUNT_H
#define MC_RECONSTRUCT_2SHUNT_H

#include "mc_port_adc.h"
#include "mc_transform.h"

typedef struct
{
    mc_f32_t offset_a;
    mc_f32_t offset_b;
    mc_f32_t scale_a;
    mc_f32_t scale_b;
} mc_2shunt_cfg_t;

typedef struct
{
    mc_q31_t offset_a;
    mc_q31_t offset_b;
    mc_q31_t scale_a;
    mc_q31_t scale_b;
} mc_2shunt_q31_cfg_t;

void mc_reconstruct_2shunt_run(const mc_adc_raw_t *raw, const mc_2shunt_cfg_t *cfg, mc_abc_t *phase_current_abc);
void mc_reconstruct_2shunt_q31_run(const mc_adc_raw_t *raw, const mc_2shunt_q31_cfg_t *cfg, mc_abc_q31_t *phase_current_abc);

#endif /* MC_RECONSTRUCT_2SHUNT_H */
