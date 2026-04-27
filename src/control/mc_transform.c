/**
 * @file mc_transform.c
 * @brief Clarke and Park transforms for FOC
 */

#include "mc_transform.h"

#include "mc_math.h"

/**
 * @brief Perform Clarke transform (abc to alpha-beta)
 * @param abc Three-phase input vector (a, b, c)
 * @param alphabeta Output stationary reference frame vector
 */
void mc_clarke_run(const mc_abc_t *abc, mc_alphabeta_t *alphabeta)
{
    if ((abc == NULL) || (alphabeta == NULL))
    {
        return;
    }

    alphabeta->alpha = abc->a;
    alphabeta->beta = (abc->a + (2.0F * abc->b)) * 0.5773502692F;
}

/**
 * @brief Perform Clarke transform using Q31 values (abc to alpha-beta)
 * @param abc Three-phase Q31 input vector (a, b, c)
 * @param alphabeta Output Q31 stationary reference frame vector
 */
void mc_clarke_q31_run(const mc_abc_q31_t *abc, mc_alphabeta_q31_t *alphabeta)
{
    if ((abc == NULL) || (alphabeta == NULL))
    {
        return;
    }

    alphabeta->alpha = abc->a;
    alphabeta->beta = mc_q31_from_f32((mc_q31_to_f32(abc->a) + (2.0F * mc_q31_to_f32(abc->b))) * 0.5773502692F);
}

/**
 * @brief Perform Park transform (alpha-beta to dq rotating frame)
 * @param alphabeta Stationary reference frame input
 * @param sin_theta Sine of rotor electrical angle
 * @param cos_theta Cosine of rotor electrical angle
 * @param dq Output rotating reference frame vector
 */
void mc_park_run(const mc_alphabeta_t *alphabeta, mc_f32_t sin_theta, mc_f32_t cos_theta, mc_dq_t *dq)
{
    if ((alphabeta == NULL) || (dq == NULL))
    {
        return;
    }

    dq->d = (alphabeta->alpha * cos_theta) + (alphabeta->beta * sin_theta);
    dq->q = (alphabeta->beta * cos_theta) - (alphabeta->alpha * sin_theta);
}

/**
 * @brief Perform Park transform using Q31 values (alpha-beta to dq rotating frame)
 * @param alphabeta Q31 stationary reference frame input
 * @param sin_theta Q31 sine of rotor electrical angle
 * @param cos_theta Q31 cosine of rotor electrical angle
 * @param dq Output Q31 rotating reference frame vector
 */
void mc_park_q31_run(const mc_alphabeta_q31_t *alphabeta, mc_q31_t sin_theta, mc_q31_t cos_theta, mc_dq_q31_t *dq)
{
    if ((alphabeta == NULL) || (dq == NULL))
    {
        return;
    }

    dq->d = mc_q31_add_sat(mc_q31_mul(alphabeta->alpha, cos_theta), mc_q31_mul(alphabeta->beta, sin_theta));
    dq->q = mc_q31_add_sat(mc_q31_mul(alphabeta->beta, cos_theta), -mc_q31_mul(alphabeta->alpha, sin_theta));
}

/**
 * @brief Perform inverse Park transform (dq to alpha-beta stationary frame)
 * @param dq Rotating reference frame input
 * @param sin_theta Sine of rotor electrical angle
 * @param cos_theta Cosine of rotor electrical angle
 * @param alphabeta Output stationary reference frame vector
 */
void mc_ipark_run(const mc_dq_t *dq, mc_f32_t sin_theta, mc_f32_t cos_theta, mc_alphabeta_t *alphabeta)
{
    if ((dq == NULL) || (alphabeta == NULL))
    {
        return;
    }

    alphabeta->alpha = (dq->d * cos_theta) - (dq->q * sin_theta);
    alphabeta->beta = (dq->d * sin_theta) + (dq->q * cos_theta);
}

/**
 * @brief Perform inverse Park transform using Q31 values (dq to alpha-beta stationary frame)
 * @param dq Q31 rotating reference frame input
 * @param sin_theta Q31 sine of rotor electrical angle
 * @param cos_theta Q31 cosine of rotor electrical angle
 * @param alphabeta Output Q31 stationary reference frame vector
 */
void mc_ipark_q31_run(const mc_dq_q31_t *dq, mc_q31_t sin_theta, mc_q31_t cos_theta, mc_alphabeta_q31_t *alphabeta)
{
    if ((dq == NULL) || (alphabeta == NULL))
    {
        return;
    }

    alphabeta->alpha = mc_q31_add_sat(mc_q31_mul(dq->d, cos_theta), -mc_q31_mul(dq->q, sin_theta));
    alphabeta->beta = mc_q31_add_sat(mc_q31_mul(dq->d, sin_theta), mc_q31_mul(dq->q, cos_theta));
}
