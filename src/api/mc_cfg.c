/**
 * @file mc_cfg.c
 * @brief Motor control configuration defaults
 */
#include "mc_cfg.h"

/**
 * @brief Translation unit anchor to ensure MC_CFG_ENABLE symbols are linked
 * @return Sum of MC_CFG_ENABLE_BLDC and MC_CFG_ENABLE_PMSM as int
 */
int mc_cfg_translation_unit_anchor(void)
{
    return (int)(MC_CFG_ENABLE_BLDC + MC_CFG_ENABLE_PMSM);
}
