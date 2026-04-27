#ifndef MC_PLATFORM_FEATURES_H
#define MC_PLATFORM_FEATURES_H

#include "mc_platform_arch.h"

#if (MC_CFG_TARGET_ARCH == MC_ARCH_CORTEX_M0P)
#define MC_PLATFORM_HAS_FPU    (0U)
#define MC_PLATFORM_HAS_DSP    (0U)
#define MC_PLATFORM_HAS_CORDIC (0U)
#else
#define MC_PLATFORM_HAS_FPU    (1U)
#define MC_PLATFORM_HAS_DSP    (1U)
#define MC_PLATFORM_HAS_CORDIC (0U)
#endif

#endif /* MC_PLATFORM_FEATURES_H */
