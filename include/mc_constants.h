#ifndef MC_CONSTANTS_H
#define MC_CONSTANTS_H

/** @file mc_constants.h @brief Named constants for motor control library
 *  @ingroup math
 *
 *  All magic numbers used across the library are defined here to satisfy
 *  MISRA C:2012 Directive 4.14.
 */

#include "mc_types.h"

/* ------------------------------------------------------------------ */
/*  Mathematical constants                                             */
/* ------------------------------------------------------------------ */

/** @brief π */
#define MC_PI (3.1415926536F)
/** @brief 2π */
#define MC_TWO_PI (6.2831853072F)
/** @brief π/2 */
#define MC_PI_OVER_2 (1.5707963268F)
/** @brief Rad/s to RPM conversion factor: 60/(2*PI) */
#define MC_RAD_S_TO_RPM (9.5492965855F)
/** @brief RPM to rad/s conversion factor: 2*PI/60 */
#define MC_RPM_TO_RAD_S (0.1047197551F)
/** @brief √3 / 3 (Clarke transform beta coefficient) */
#define MC_CLARKE_BETA_COEFF (0.5773502692F)
/** @brief √3 / 2 (used in inverse Clarke) */
#define MC_SQRT3_OVER_2 (0.8660254038F)
/** @brief 360°/6 = 60° per commutation step for Hall/6-step */
#define MC_DEG_PER_COMM_STEP (60.0F)
/** @brief Hall sequence length */
#define MC_HALL_STEPS (6U)

/* ------------------------------------------------------------------ */
/*  Q31 fixed-point constants                                          */
/* ------------------------------------------------------------------ */

/** @brief Q31 scaling factor: 2^31 */
#define MC_Q31_SCALE (2147483648.0F)
/** @brief 12-bit ADC full-scale value */
#define MC_ADC_FULL_SCALE (4095.0F)

/* ------------------------------------------------------------------ */
/*  Numeric thresholds / guard values                                  */
/* ------------------------------------------------------------------ */

/** @brief Tiny positive epsilon for float comparisons */
#define MC_EPSILON_F (1e-6F)
/** @brief Tiny epsilon for zero-flux discrimination */
#define MC_EPSILON_FLUX (1e-8F)

/* ------------------------------------------------------------------ */
/*  1-shunt reconstruction constants                                   */
/* ------------------------------------------------------------------ */

/** @brief Common-mode shift for reorder-required case */
#define MC_1SHUNT_SHIFT_REORDER (0.04F)
/** @brief Common-mode shift for non-reorder case */
#define MC_1SHUNT_SHIFT_NORMAL (0.02F)
/** @brief Half-period multiplier */
#define MC_1SHUNT_HALF (0.5F)
/** @brief Sample position midpoint (50% of PWM period) */
#define MC_1SHUNT_MIDPOINT (0.5F)
/** @brief Sum-of-duties threshold for zero-vector bias */
#define MC_1SHUNT_ZERO_VECTOR_DUTY_SUM (1.5F)
/** @brief Minimum sample count for valid 1-shunt measurement */
#define MC_1SHUNT_MIN_SAMPLE_COUNT (2U)
/** @brief Preferred window index for 1-shunt high duty side */
#define MC_1SHUNT_PREFERRED_WINDOW_HIGH (1U)
/** @brief Preferred window index for 1-shunt low duty side */
#define MC_1SHUNT_PREFERRED_WINDOW_LOW (2U)

/* ------------------------------------------------------------------ */
/*  1-shunt prediction / compensation constants                        */
/* ------------------------------------------------------------------ */

/** @brief High modulation threshold (ratio of voltage_limit) */
#define MC_1SHUNT_HIGH_MODULATION_RATIO (0.8F)
/** @brief Prediction weight during high modulation */
#define MC_1SHUNT_PREDICT_WEIGHT_HIGH_MOD (0.6F)
/** @brief Delta limit during high modulation */
#define MC_1SHUNT_DELTA_LIMIT_HIGH_MOD (0.25F)
/** @brief Default prediction weight */
#define MC_1SHUNT_PREDICT_WEIGHT_DEFAULT (1.0F)
/** @brief Default delta limit */
#define MC_1SHUNT_DELTA_LIMIT_DEFAULT (0.5F)
/** @brief Prediction weight multiplier in field weakening */
#define MC_1SHUNT_PREDICT_FW_WEIGHT_MULT (0.7F)
/** @brief Delta limit in field weakening */
#define MC_1SHUNT_DELTA_LIMIT_FW (0.15F)
/** @brief Id threshold for field weakening detection */
#define MC_1SHUNT_FW_ID_THRESHOLD (-0.1F)

/* ------------------------------------------------------------------ */
/*  PMSM sensorless observer constants                                 */
/* ------------------------------------------------------------------ */

/** @brief Default BEMF LPF filter alpha */
#define MC_SENSORLESS_DEFAULT_BEMF_ALPHA (0.2F)
/** @brief Default minimum BEMF magnitude */
#define MC_SENSORLESS_DEFAULT_MIN_BEMF (0.01F)
/** @brief Default PLL proportional gain */
#define MC_SENSORLESS_DEFAULT_PLL_KP (400.0F)
/** @brief Default PLL integral gain */
#define MC_SENSORLESS_DEFAULT_PLL_KI (20000.0F)
/** @brief Lock BEMF-to-min_BEMF ratio */
#define MC_SENSORLESS_DEFAULT_LOCK_BEMF_RATIO (2.0F)
/** @brief Default startup speed */
#define MC_SENSORLESS_DEFAULT_STARTUP_SPEED_RAD_S (80.0F)
/** @brief Default startup acceleration */
#define MC_SENSORLESS_DEFAULT_STARTUP_ACCEL_RAD_S2 (4000.0F)
/** @brief Default open-loop voltage ratio of bus limit */
#define MC_SENSORLESS_DEFAULT_OL_VOLTAGE_RATIO (0.15F)
/** @brief Consecutive samples for open-loop lock */
#define MC_SENSORLESS_OPEN_LOOP_LOCK_SAMPLES (2U)
/** @brief Consecutive samples for PLL lock */
#define MC_SENSORLESS_PLL_LOCK_SAMPLES (2U)
/** @brief Consecutive samples for PLL unlock */
#define MC_SENSORLESS_PLL_UNLOCK_SAMPLES (2U)
/** @brief Counter max value */
#define MC_SENSORLESS_COUNTER_MAX (255U)
/** @brief Open-loop handoff angle error threshold */
#define MC_SENSORLESS_OPEN_LOOP_HANDOFF_ANGLE_ERR_RAD (0.5F)
/** @brief PLL lock angle error threshold */
#define MC_SENSORLESS_PLL_LOCK_ANGLE_ERR_RAD (0.35F)
/** @brief SMO open-loop handoff angle error threshold */
#define MC_SMO_OPEN_LOOP_HANDOFF_ANGLE_ERR_RAD (0.5F)
/** @brief SMO PLL lock angle error threshold */
#define MC_SMO_PLL_LOCK_ANGLE_ERR_RAD (0.35F)

/* ------------------------------------------------------------------ */
/*  Identify / parameter estimation constants                           */
/* ------------------------------------------------------------------ */

/** @brief Minimum positive flux estimate sample [Wb] */
#define MC_IDENTIFY_MIN_FLUX_SAMPLE_WB (1e-6F)
/** @brief Minimum valid flux window duration ratio */
#define MC_IDENTIFY_MIN_VALID_FLUX_WINDOW_RATIO (0.75F)
/** @brief Minimum number of individual flux samples required for trust gating */
#define MC_IDENTIFY_MIN_FLUX_SAMPLES (10U)
/** @brief Maximum allowed coefficient of variation for flux estimate (CV = stddev/mean) */
#define MC_IDENTIFY_MAX_FLUX_CV (0.3F)
/** @brief Default align time [s] */
#define MC_IDENTIFY_DEFAULT_ALIGN_TIME_S (0.05F)
/** @brief Default RS settle time [s] */
#define MC_IDENTIFY_DEFAULT_RS_SETTLE_TIME_S (0.01F)
/** @brief Default RS sample time [s] */
#define MC_IDENTIFY_DEFAULT_RS_SAMPLE_TIME_S (0.005F)
/** @brief Default Ld/Lq pulse time [s] */
#define MC_IDENTIFY_DEFAULT_PULSE_TIME_S (0.002F)
/** @brief Default Lq align time [s] */
#define MC_IDENTIFY_DEFAULT_LQ_ALIGN_TIME_S (0.1F)
/** @brief Default align voltage (ratio of limit) */
#define MC_IDENTIFY_DEFAULT_ALIGN_VOLTAGE (0.1F)
/** @brief Default Rs measurement voltage (ratio of limit) */
#define MC_IDENTIFY_DEFAULT_RS_VOLTAGE (0.15F)
/** @brief Default pulse voltage (ratio of limit) */
#define MC_IDENTIFY_DEFAULT_PULSE_VOLTAGE (0.3F)
/** @brief Default max current [A] */
#define MC_IDENTIFY_DEFAULT_MAX_CURRENT_A (1.0F)
/** @brief Default voltage limit [V] */
#define MC_IDENTIFY_DEFAULT_VOLTAGE_LIMIT (1.0F)

/* ------------------------------------------------------------------ */
/*  BLDC sensorless drive defaults                                     */
/* ------------------------------------------------------------------ */

/** @brief Default align duty */
#define MC_BLDC_SS_DEFAULT_ALIGN_DUTY (0.1F)
/** @brief Default align time [s] */
#define MC_BLDC_SS_DEFAULT_ALIGN_TIME_S (0.5F)
/** @brief Default ramp start frequency [Hz] */
#define MC_BLDC_SS_DEFAULT_RAMP_START_FREQ_HZ (5.0F)
/** @brief Default ramp end frequency [Hz] */
#define MC_BLDC_SS_DEFAULT_RAMP_END_FREQ_HZ (30.0F)
/** @brief Default ramp time [s] */
#define MC_BLDC_SS_DEFAULT_RAMP_TIME_S (1.0F)
/** @brief Default ramp start duty */
#define MC_BLDC_SS_DEFAULT_RAMP_START_DUTY (0.15F)
/** @brief Default ramp end duty */
#define MC_BLDC_SS_DEFAULT_RAMP_END_DUTY (0.4F)
/** @brief Default BEMF threshold [V] */
#define MC_BLDC_SS_DEFAULT_BEMF_THRESHOLD_V (1.0F)
/** @brief Default advance angle [deg] */
#define MC_BLDC_SS_DEFAULT_ADVANCE_ANGLE_DEG (30.0F)
/** @brief Default speed PI kp */
#define MC_BLDC_SS_DEFAULT_SPEED_KP (0.1F)
/** @brief Default speed PI ki */
#define MC_BLDC_SS_DEFAULT_SPEED_KI (1.0F)
/** @brief Default speed PI integral min */
#define MC_BLDC_SS_DEFAULT_SPEED_INT_MIN (-0.5F)
/** @brief Default speed PI integral max */
#define MC_BLDC_SS_DEFAULT_SPEED_INT_MAX (0.5F)
/** @brief Default speed PI output min */
#define MC_BLDC_SS_DEFAULT_SPEED_OUT_MIN (0.0F)
/** @brief Default speed PI output max */
#define MC_BLDC_SS_DEFAULT_SPEED_OUT_MAX (1.0F)
/** @brief Duty clamp ceiling for sensorless speed PI */
#define MC_BLDC_SS_DUTY_MAX_CLAMP (0.95F)
/** @brief Fallback ZC timeout ratio (1.5x period) */
#define MC_BLDC_SS_FALLBACK_ZC_TIMEOUT_RATIO (1.5F)
/** @brief Invalid step sentinel */
#define MC_BLDC_SS_INVALID_STEP (0xFFU)

/* ------------------------------------------------------------------ */
/*  BLDC Hall drive defaults                                           */
/* ------------------------------------------------------------------ */

/** @brief Default duty minimum */
#define MC_BLDC_DEFAULT_DUTY_MIN (0.0F)
/** @brief Default duty maximum */
#define MC_BLDC_DEFAULT_DUTY_MAX (1.0F)

/* ------------------------------------------------------------------ */
/*  Sensor / speed estimation constants                                */
/* ------------------------------------------------------------------ */

/** @brief Microseconds per second */
#define MC_US_PER_SEC (1000000.0F)
/** @brief Seconds per minute */
#define MC_SEC_PER_MIN (60.0F)

#endif /* MC_CONSTANTS_H */
