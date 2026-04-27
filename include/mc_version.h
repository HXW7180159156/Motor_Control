#ifndef MC_VERSION_H
#define MC_VERSION_H

/** @file mc_version.h @brief Version identifiers for the motor control library */

/** @brief Major version number */
#define MC_VERSION_MAJOR (0U)
/** @brief Minor version number */
#define MC_VERSION_MINOR (1U)
/** @brief Patch version number */
#define MC_VERSION_PATCH (0U)

/** @brief Packed 32-bit version (major << 16 | minor << 8 | patch) */
#define MC_VERSION_U32 ((MC_VERSION_MAJOR << 16U) | (MC_VERSION_MINOR << 8U) | MC_VERSION_PATCH)

#endif /* MC_VERSION_H */
