/** @file mc_reconstruct_3shunt.h @brief Three-shunt current reconstruction types and API */

#ifndef MC_RECONSTRUCT_3SHUNT_H
#define MC_RECONSTRUCT_3SHUNT_H

#include "mc_port_adc.h"
#include "mc_transform.h"

/** @brief Float32 configuration for three-shunt current reconstruction */
typedef struct
{
    mc_f32_t offset_a;
    mc_f32_t offset_b;
    mc_f32_t offset_c;
    mc_f32_t scale_a;
    mc_f32_t scale_b;
    mc_f32_t scale_c;
} mc_3shunt_cfg_t;

/** @brief Q31 configuration for three-shunt current reconstruction */
typedef struct
{
    mc_q31_t offset_a;
    mc_q31_t offset_b;
    mc_q31_t offset_c;
    mc_q31_t scale_a;
    mc_q31_t scale_b;
    mc_q31_t scale_c;
} mc_3shunt_q31_cfg_t;

/**
 * @brief Reconstruct three-phase currents from three shunt ADC samples
 * @param[in] raw Raw ADC sample data.
 *   Range: readable `mc_adc_raw_t` storage or `NULL`.
 * @param[in] cfg Three-shunt reconstruction configuration.
 *   Range: readable `mc_3shunt_cfg_t` storage or `NULL`.
 * @param[out] phase_current_abc Reconstructed phase currents.
 *   Range: writable `mc_abc_t` storage or `NULL`.
 * @return None.
 *   Range: not applicable.
 * @note If any pointer is `NULL`, the call has no effect.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when concurrent calls do not share writable `phase_current_abc` storage.
 */
void mc_reconstruct_3shunt_run(const mc_adc_raw_t *raw, const mc_3shunt_cfg_t *cfg, mc_abc_t *phase_current_abc);

/**
 * @brief Reconstruct three-phase currents from three shunt ADC samples in Q31
 * @param[in] raw Raw ADC sample data.
 *   Range: readable `mc_adc_raw_t` storage or `NULL`.
 * @param[in] cfg Three-shunt Q31 reconstruction configuration.
 *   Range: readable `mc_3shunt_q31_cfg_t` storage or `NULL`.
 * @param[out] phase_current_abc Reconstructed Q31 phase currents.
 *   Range: writable `mc_abc_q31_t` storage or `NULL`.
 * @return None.
 *   Range: not applicable.
 * @note If any pointer is `NULL`, the call has no effect.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when concurrent calls do not share writable `phase_current_abc` storage.
 */
void mc_reconstruct_3shunt_q31_run(const mc_adc_raw_t *raw, const mc_3shunt_q31_cfg_t *cfg, mc_abc_q31_t *phase_current_abc);

#endif /* MC_RECONSTRUCT_3SHUNT_H */
