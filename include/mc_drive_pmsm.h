/**
 * @file mc_drive_pmsm.h
 * @brief PMSM field-oriented control (FOC) drive configuration and execution
 *
 * Defines data structures, configuration types, and API functions for
 * sensorless FOC of permanent magnet synchronous motors.
 */

#ifndef MC_DRIVE_PMSM_H
#define MC_DRIVE_PMSM_H

#include "mc_control_pi.h"
#include "mc_control_svpwm.h"
#include "mc_reconstruct_1shunt.h"
#include "mc_reconstruct_2shunt.h"
#include "mc_reconstruct_3shunt.h"
#include "mc_status.h"
#include "mc_transform.h"

/**
 * @brief Current sensing topology
 */
typedef enum
{
    MC_CURRENT_SENSE_3SHUNT = 0, /**< Three low-side shunt resistors */
    MC_CURRENT_SENSE_2SHUNT,     /**< Two low-side shunt resistors */
    MC_CURRENT_SENSE_1SHUNT      /**< Single-shunt (DC-link) resistor */
} mc_current_sense_type_t;

/**
 * @brief Current sense hardware configuration
 */
typedef struct
{
    mc_current_sense_type_t type; /**< Selected shunt topology */
    union
    {
        mc_3shunt_cfg_t shunt3;   /**< 3-shunt configuration */
        mc_2shunt_cfg_t shunt2;   /**< 2-shunt configuration */
        mc_1shunt_cfg_t shunt1;   /**< 1-shunt configuration */
    } cfg;                        /**< Topology-specific configuration */
} mc_current_sense_cfg_t;

/**
 * @brief 1-shunt current reconstruction compensation mode
 */
typedef enum
{
    MC_1SHUNT_COMP_NONE = 0,                        /**< No compensation */
    MC_1SHUNT_COMP_PREDICT_BASIC,                   /**< Basic predictive compensation */
    MC_1SHUNT_COMP_PREDICT_HIGH_MODULATION,         /**< Predictive compensation for high modulation index */
    MC_1SHUNT_COMP_PREDICT_FIELD_WEAKENING          /**< Predictive compensation during field weakening */
} mc_1shunt_comp_mode_t;

/**
 * @brief Status of 1-shunt current reconstruction compensation
 */
typedef struct
{
    mc_bool_t active;               /**< Compensation is enabled */
    mc_1shunt_comp_mode_t mode;     /**< Active compensation mode */
} mc_1shunt_comp_status_t;

/**
 * @brief PMSM FOC configuration parameters
 */
typedef struct
{
    mc_pi_cfg_t id_pi_cfg;             /**< PI controller gains for d-axis current */
    mc_pi_cfg_t iq_pi_cfg;             /**< PI controller gains for q-axis current */
    mc_svpwm_cfg_t svpwm_cfg;          /**< Space-vector PWM configuration */
    mc_current_sense_cfg_t current_cfg; /**< Current sensing configuration */
    mc_f32_t voltage_limit;            /**< Maximum phase voltage magnitude (V) */
    mc_f32_t bus_voltage_min;          /**< Minimum DC bus voltage for operation (V) */
    mc_f32_t rs_ohm;                   /**< Stator phase resistance (ohm) */
    mc_f32_t ld_h;                     /**< d-axis inductance (H) */
    mc_f32_t lq_h;                     /**< q-axis inductance (H) */
    mc_f32_t flux_wb;                  /**< Permanent magnet flux linkage (Wb) */
    mc_f32_t pole_pairs;               /**< Number of rotor pole pairs */
    mc_bool_t mtpa_enable;             /**< Enable maximum torque per ampere control */
    mc_bool_t fw_enable;               /**< Enable field weakening */
    mc_f32_t fw_kp;                    /**< Field weakening PI proportional gain */
    mc_f32_t fw_ki;                    /**< Field weakening PI integral gain */
    mc_f32_t fw_min_id;                /**< Minimum allowable d-axis current in field weakening (A) */
    mc_f32_t fw_activation_ratio;      /**< Voltage ratio threshold to activate field weakening [0-1] */
} mc_pmsm_foc_cfg_t;

/**
 * @brief PMSM FOC runtime instance state
 */
typedef struct
{
    mc_pi_t id_pi;                     /**< d-axis current PI controller state */
    mc_pi_t iq_pi;                     /**< q-axis current PI controller state */
    mc_pmsm_foc_cfg_t cfg;             /**< Copied configuration parameters */
    mc_abc_t i_abc_last;               /**< Previous phase current sample (for reconstruction) */
    mc_dq_t i_dq;                      /**< Rotor-aligned dq-axis currents */
    mc_dq_t v_dq;                      /**< Rotor-aligned dq-axis voltage demands */
    mc_1shunt_meta_t shunt1_meta;      /**< 1-shunt reconstruction metadata */
    mc_f32_t id_fw_adjustment;         /**< Field weakening d-axis current injection (A) */
    uint8_t last_sector;               /**< Previous SVPWM sector number */
} mc_pmsm_foc_t;

/**
 * @brief Input data for a single FOC execution cycle
 */
typedef struct
{
    mc_adc_raw_t current_raw;       /**< Raw ADC current samples */
    mc_f32_t bus_voltage_v;         /**< Instantaneous DC bus voltage (V) */
    mc_f32_t sin_theta;             /**< Sine of rotor electrical angle */
    mc_f32_t cos_theta;             /**< Cosine of rotor electrical angle */
    mc_f32_t id_ref;                /**< d-axis current reference (A) */
    mc_f32_t iq_ref;                /**< q-axis current reference (A) */
    mc_f32_t dt_s;                  /**< Control loop period (s) */
    mc_f32_t elec_speed_rad_s;      /**< Electrical rotor speed (rad/s) */
} mc_pmsm_foc_input_t;

/**
 * @brief Output data produced by a single FOC execution cycle
 */
typedef struct
{
    mc_abc_t i_abc;                         /**< Reconstructed phase currents (A) */
    mc_alphabeta_t i_ab;                    /**< Stationary-frame alpha-beta currents */
    mc_dq_t i_dq;                           /**< Rotor-aligned dq-axis currents */
    mc_dq_t v_dq;                           /**< Rotor-aligned dq-axis voltage demands */
    mc_alphabeta_t v_ab;                    /**< Stationary-frame alpha-beta voltages */
    mc_pwm_cmd_t pwm_cmd;                   /**< PWM duty-cycle commands */
    mc_adc_trigger_plan_t adc_trigger_plan; /**< ADC sampling trigger timing plan */
    mc_1shunt_comp_status_t current_comp_status; /**< 1-shunt compensation status */
} mc_pmsm_foc_output_t;

/**
 * @brief Initialise a PMSM FOC drive instance
 * @param[out] foc Pointer to FOC instance storage.
 *   Range: non-NULL pointer to writable `mc_pmsm_foc_t` storage.
 * @param[in] cfg Configuration parameters copied into the instance.
 *   Range: non-NULL pointer to readable `mc_pmsm_foc_cfg_t` storage.
 * @retval MC_STATUS_OK Initialization completed successfully.
 * @retval MC_STATUS_INVALID_ARG `foc == NULL` or `cfg == NULL`.
 * @note This function copies the configuration and resets runtime state, but does not validate most numeric configuration ranges.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `foc` object.
 */
mc_status_t mc_pmsm_foc_init(mc_pmsm_foc_t *foc, const mc_pmsm_foc_cfg_t *cfg);

/**
 * @brief Execute one PMSM FOC current-loop cycle
 * @param[in,out] foc Pointer to initialized FOC instance.
 *   Range: non-NULL pointer to writable `mc_pmsm_foc_t` storage.
 * @param[in] in Per-cycle FOC inputs.
 *   Range: non-NULL pointer to readable `mc_pmsm_foc_input_t` storage.
 * @param[out] out Per-cycle FOC outputs.
 *   Range: non-NULL pointer to writable `mc_pmsm_foc_output_t` storage.
 * @retval MC_STATUS_OK Cycle completed successfully.
 * @retval MC_STATUS_INVALID_ARG `foc == NULL`, `in == NULL`, or `out == NULL`.
 * @note Current reconstruction behavior is selected by `foc->cfg.current_cfg.type`; the 1-shunt path also populates `out->adc_trigger_plan` and `out->current_comp_status`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `foc` object and different writable `out` storage. Not reentrant for concurrent writes to the same `foc`.
 */
mc_status_t mc_pmsm_foc_run(mc_pmsm_foc_t *foc, const mc_pmsm_foc_input_t *in, mc_pmsm_foc_output_t *out);

/**
 * @brief Compute the MTPA d-axis current reference
 * @param[in] iq_ref Q-axis current reference [A].
 *   Range: application-defined `mc_f32_t`.
 * @param[in] flux_wb Permanent-magnet flux linkage [Wb].
 *   Range: application-defined `mc_f32_t`.
 * @param[in] ld_h D-axis inductance [H].
 *   Range: application-defined `mc_f32_t`.
 * @param[in] lq_h Q-axis inductance [H].
 *   Range: application-defined `mc_f32_t`.
 * @return Computed MTPA d-axis current reference [A].
 *   Range: application-defined `mc_f32_t`; returns `0.0F` when `lq_h - ld_h` is effectively zero or the internal discriminant is negative.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant.
 */
mc_f32_t mc_pmsm_compute_mtpa_id(mc_f32_t iq_ref, mc_f32_t flux_wb, mc_f32_t ld_h, mc_f32_t lq_h);

#endif /* MC_DRIVE_PMSM_H */
