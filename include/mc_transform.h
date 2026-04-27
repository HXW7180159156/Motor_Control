#ifndef MC_TRANSFORM_H
#define MC_TRANSFORM_H

/** @file mc_transform.h @brief Clarke/Park transform structures and functions */

#include "mc_types.h"

/** @brief Three-phase (ABC) stationary frame vector */
typedef struct
{
    mc_f32_t a; /**< Phase A component */
    mc_f32_t b; /**< Phase B component */
    mc_f32_t c; /**< Phase C component */
} mc_abc_t;

typedef struct
{
    mc_q31_t a;
    mc_q31_t b;
    mc_q31_t c;
} mc_abc_q31_t;

/** @brief Two-axis stationary frame (alpha-beta) vector */
typedef struct
{
    mc_f32_t alpha; /**< Alpha-axis component */
    mc_f32_t beta;  /**< Beta-axis component */
} mc_alphabeta_t;

typedef struct
{
    mc_q31_t alpha;
    mc_q31_t beta;
} mc_alphabeta_q31_t;

/** @brief Synchronous rotating frame (direct-quadrature) vector */
typedef struct
{
    mc_f32_t d; /**< Direct-axis (flux) component */
    mc_f32_t q; /**< Quadrature-axis (torque) component */
} mc_dq_t;

typedef struct
{
    mc_q31_t d;
    mc_q31_t q;
} mc_dq_q31_t;

/**
 * @brief Perform Clarke transform: ABC stationary -> alpha-beta stationary
 * @param abc Pointer to ABC frame input vector
 * @param alphabeta Pointer to alpha-beta frame output vector
 */
void mc_clarke_run(const mc_abc_t *abc, mc_alphabeta_t *alphabeta);
void mc_clarke_q31_run(const mc_abc_q31_t *abc, mc_alphabeta_q31_t *alphabeta);

/**
 * @brief Perform Park transform: alpha-beta stationary -> DQ rotating
 * @param alphabeta Pointer to alpha-beta frame input vector
 * @param sin_theta Sine of the rotor electrical angle
 * @param cos_theta Cosine of the rotor electrical angle
 * @param dq Pointer to DQ frame output vector
 */
void mc_park_run(const mc_alphabeta_t *alphabeta, mc_f32_t sin_theta, mc_f32_t cos_theta, mc_dq_t *dq);
void mc_park_q31_run(const mc_alphabeta_q31_t *alphabeta, mc_q31_t sin_theta, mc_q31_t cos_theta, mc_dq_q31_t *dq);

/**
 * @brief Perform inverse Park transform: DQ rotating -> alpha-beta stationary
 * @param dq Pointer to DQ frame input vector
 * @param sin_theta Sine of the rotor electrical angle
 * @param cos_theta Cosine of the rotor electrical angle
 * @param alphabeta Pointer to alpha-beta frame output vector
 */
void mc_ipark_run(const mc_dq_t *dq, mc_f32_t sin_theta, mc_f32_t cos_theta, mc_alphabeta_t *alphabeta);
void mc_ipark_q31_run(const mc_dq_q31_t *dq, mc_q31_t sin_theta, mc_q31_t cos_theta, mc_alphabeta_q31_t *alphabeta);

#endif /* MC_TRANSFORM_H */
