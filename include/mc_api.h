/**
 * @file mc_api.h
 * @brief Top-level motor control API
 */

#ifndef MC_API_H
#define MC_API_H

#include "mc_cfg.h"
#include "mc_control_foc.h"
#include "mc_identify.h"
#include "mc_drive_bldc.h"
#include "mc_drive_bldc_sensorless.h"
#include "mc_diag.h"
#include "mc_limits.h"
#include "mc_port.h"
#include "mc_port_adc.h"
#include "mc_port_fault.h"
#include "mc_port_pwm.h"
#include "mc_port_resolver.h"
#include "mc_sensor_encoder.h"
#include "mc_sensor_hall.h"
#include "mc_sensor_resolver.h"
#include "mc_sensor_sensorless.h"
#include "mc_status.h"
#include "mc_types.h"

/**
 * @brief Motor control operating modes
 */
typedef enum
{
    MC_MODE_DISABLED = 0,
    MC_MODE_BLDC_HALL,
    MC_MODE_BLDC_SENSORLESS,
    MC_MODE_PMSM_FOC_HALL,
    MC_MODE_PMSM_FOC_ENCODER,
    MC_MODE_PMSM_FOC_RESOLVER,
    MC_MODE_PMSM_FOC_SENSORLESS
} mc_mode_t;

/**
 * @brief Numeric representation modes
 * @note The top-level `MC_NUMERIC_Q31` path is currently limited to
 *       `MC_MODE_PMSM_FOC_HALL` and `MC_MODE_PMSM_FOC_ENCODER`
 *       with `MC_CURRENT_SENSE_2SHUNT` or `MC_CURRENT_SENSE_3SHUNT`,
 *       `mtpa_enable == MC_FALSE`, and `fw_enable == MC_FALSE`.
 */
typedef enum
{
    MC_NUMERIC_FLOAT32 = 0,
    MC_NUMERIC_Q31
} mc_numeric_t;

/**
 * @brief Motor electrical parameters
 */
typedef struct
{
    uint8_t pole_pairs;     /**< Number of motor pole pairs */
    mc_f32_t rs_ohm;        /**< Stator resistance in ohms */
    mc_f32_t ld_h;          /**< D-axis inductance in henries */
    mc_f32_t lq_h;          /**< Q-axis inductance in henries */
    mc_f32_t flux_wb;       /**< Permanent magnet flux linkage in webers */
} mc_motor_params_t;

/**
 * @brief Control loop configuration
 */
typedef struct
{
    uint32_t pwm_frequency_hz;  /**< PWM switching frequency in Hz */
    mc_numeric_t numeric_mode;  /**< Numeric representation mode (float or Q) */
    mc_f32_t current_loop_dt_s; /**< Current control loop sample time in seconds */
} mc_ctrl_cfg_t;

/**
 * @brief Field-oriented control (FOC) algorithm configuration
 */
typedef struct
{
    mc_pi_cfg_t id_pi_cfg;              /**< PI controller configuration for d-axis current */
    mc_pi_cfg_t iq_pi_cfg;              /**< PI controller configuration for q-axis current */
    mc_pi_cfg_t speed_pi_cfg;           /**< PI controller configuration for speed loop */
    mc_svpwm_cfg_t svpwm_cfg;           /**< Space vector PWM configuration */
    mc_current_sense_cfg_t current_cfg; /**< Current sensing configuration */
    mc_f32_t speed_loop_dt_s;           /**< Speed control loop sample time in seconds */
    mc_f32_t iq_limit;                  /**< Q-axis current limit */
    mc_f32_t voltage_limit;             /**< Voltage limit */
    mc_f32_t bus_voltage_scale;         /**< Bus voltage measurement scale factor */
    mc_f32_t bus_voltage_offset;        /**< Bus voltage measurement offset */
    mc_f32_t bus_voltage_min;           /**< Minimum bus voltage threshold */
    mc_bool_t mtpa_enable;              /**< Maximum torque per ampere enable flag */
    mc_bool_t fw_enable;                /**< Field weakening enable flag */
    mc_f32_t fw_kp;                     /**< Field weakening proportional gain */
    mc_f32_t fw_ki;                     /**< Field weakening integral gain */
    mc_f32_t fw_min_id;                 /**< Minimum d-axis current in field weakening */
    mc_f32_t fw_activation_ratio;       /**< Field weakening activation voltage ratio */
} mc_foc_cfg_t;

/**
 * @brief Sensor configuration
 * @note The top-level API binds the instance to one configured `primary_mode`
 *       at initialization time. After `mc_init()` succeeds, `mc_set_mode()`
 *       only accepts that same configured mode.
 */
typedef struct
{
    mc_mode_t primary_mode;             /**< Primary operating mode selection */
    mc_f32_t pole_pairs;                /**< Number of pole pairs */
    mc_hall_cfg_t hall_cfg;             /**< Hall sensor configuration */
    mc_encoder_cfg_t encoder_cfg;       /**< Encoder configuration */
    mc_resolver_cfg_t resolver_cfg;     /**< Resolver configuration */
    mc_sensorless_cfg_t sensorless_cfg; /**< Sensorless control configuration */
    mc_bldc_sensorless_cfg_t bldc_sensorless_cfg; /**< BLDC sensorless configuration */
} mc_sensor_cfg_t;

/**
 * @brief Complete system configuration
 */
typedef struct
{
    mc_motor_params_t motor;        /**< Motor electrical parameters */
    mc_hw_limits_t limits;          /**< Hardware and software limits */
    mc_ctrl_cfg_t control;          /**< Control loop configuration */
    mc_foc_cfg_t foc;               /**< FOC algorithm configuration */
    mc_sensor_cfg_t sensor;         /**< Sensor configuration */
    mc_port_hooks_t hooks;          /**< Platform-specific hook functions */
    mc_resolver_port_t resolver_port; /**< Resolver port configuration */
} mc_system_cfg_t;

/**
 * @brief Fast-rate input data (high-frequency, e.g. current loop rate)
 */
typedef struct
{
    mc_adc_raw_t adc_raw;               /**< Hardware raw ADC samples from current/voltage sensors */
    mc_fault_input_t fault_input;       /**< Hardware fault input signals */
    mc_resolver_raw_t resolver_raw;     /**< Raw resolver signals */
    uint8_t hall_code;                  /**< Hall sensor state code */
    uint32_t timestamp_us;              /**< Input timestamp in microseconds */
} mc_fast_input_t;

/**
 * @brief Medium-rate input data (e.g. speed loop rate)
 */
typedef struct
{
    mc_f32_t speed_feedback_rpm; /**< Speed feedback in RPM */
    uint32_t encoder_count;      /**< Encoder position count */
    uint32_t timestamp_us;       /**< Input timestamp in microseconds */
} mc_medium_input_t;

/**
 * @brief Slow-rate input data (low-frequency monitoring)
 */
typedef struct
{
    mc_f32_t temperature_deg_c;  /**< Motor/driver temperature in degrees Celsius */
    mc_bool_t clear_fault_request; /**< Clear fault flag */
    uint32_t timestamp_us;       /**< Input timestamp in microseconds */
} mc_slow_input_t;

/**
 * @brief Fast-rate output data (PWM commands and ADC trigger plan)
 */
typedef struct
{
    mc_pwm_cmd_t pwm_cmd;                       /**< Output PWM commands for inverter phases */
    mc_adc_trigger_plan_t adc_trigger_plan;     /**< ADC trigger timing plan */
    mc_1shunt_comp_status_t current_comp_status; /**< Single-shunt current reconstruction compensation status */
} mc_fast_output_t;

/**
 * @brief Medium-rate output data (angle, speed, current references)
 */
typedef struct
{
    mc_f32_t elec_angle_rad;  /**< Electrical angle in radians */
    mc_f32_t mech_speed_rpm;  /**< Mechanical speed in RPM */
    mc_f32_t id_ref;          /**< D-axis current reference */
    mc_f32_t iq_ref;          /**< Q-axis current reference */
} mc_medium_output_t;

/**
 * @brief Slow-rate output data (mode and diagnostics)
 */
typedef struct
{
    mc_mode_t mode;           /**< Current operating mode */
    mc_diag_status_t diag;    /**< Diagnostic status */
} mc_slow_output_t;

/**
 * @brief Motor control instance state structure
 */
typedef struct
{
    mc_bool_t initialized;          /**< State flag indicating initialization status */
    mc_bool_t enabled;              /**< State flag indicating enable status */
    mc_bool_t speed_ctrl_enabled;   /**< State flag indicating speed control enabled */
    mc_mode_t mode;                 /**< Current operating mode */
    mc_f32_t speed_ref_rpm;         /**< Speed reference in RPM */
    mc_f32_t torque_ref;            /**< Torque reference */
    mc_f32_t id_ref;                /**< D-axis current reference */
    mc_f32_t iq_ref;                /**< Q-axis current reference */
    mc_system_cfg_t cfg;            /**< Copy of system configuration */
    mc_diag_status_t diag;          /**< Diagnostic status */
    mc_control_foc_t foc;           /**< FOC sub-instance state */
    mc_pi_q31_t foc_speed_pi_q31;   /**< Q31 speed-loop PI for limited Q31 path */
    mc_q31_t iq_ref_q31;            /**< Q31 q-axis current reference for limited Q31 path */
    mc_bldc_hall_t bldc_hall;             /**< BLDC Hall sub-instance state */
    mc_bldc_sensorless_t bldc_sensorless; /**< BLDC sensorless sub-instance state */
    mc_hall_state_t hall;                 /**< Hall sensor sub-instance state */
    mc_encoder_state_t encoder;     /**< Encoder sub-instance state */
    mc_resolver_state_t resolver;   /**< Resolver sub-instance state */
    mc_sensorless_state_t sensorless; /**< Sensorless sub-instance state */
    mc_pmsm_foc_output_t foc_last_output; /**< Most recent FOC cycle output */
    mc_identify_t identify;               /**< Motor parameter identification state */
} mc_instance_t;

/**
 * @brief Initialise a top-level motor-control instance from system configuration
 * @param[out] inst Pointer to instance storage to initialise.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @param[in] cfg System configuration copied into the instance.
 *   Range: non-NULL pointer to readable `mc_system_cfg_t` storage.
 * @retval MC_STATUS_OK Initialization completed successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`, `cfg == NULL`, or a delegated sub-initializer rejects the provided configuration.
 * @retval MC_STATUS_UNSUPPORTED The requested top-level numeric/mode combination is not supported.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object.
 */
mc_status_t mc_init(mc_instance_t *inst, const mc_system_cfg_t *cfg);

/**
 * @brief Reset runtime state while preserving the copied configuration
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @retval MC_STATUS_OK Runtime state was cleared successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`.
 * @note This call clears enable/state-machine/runtime data, restores `mode` to
 *       `cfg.sensor.primary_mode`, and keeps the copied configuration intact.
 * @note If the preserved primary mode is sensorless, the stored sensorless
 *       estimator configuration is reinitialized as part of the reset path.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_reset(mc_instance_t *inst);

/**
 * @brief Enable the motor-control instance
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @retval MC_STATUS_OK Enable request was accepted.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`.
 * @retval MC_STATUS_INVALID_STATE `inst->initialized == MC_FALSE` or the active BLDC sensorless runtime cannot enter its startup sequence.
 * @note In `MC_MODE_BLDC_SENSORLESS`, this call also starts the BLDC sensorless startup state machine.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_start(mc_instance_t *inst);

/**
 * @brief Start motor parameter identification
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @note The current identify path expects the active current-sense configuration to provide reconstructed phase current in amperes.
 * @note The current implementation estimates `Rs`, `Ld`, `Lq`, and a first open-loop `flux_wb` value using time-weighted windows over the actual discrete injected pulse duration, then writes positive results back into the live runtime model.
 * @retval MC_STATUS_OK Identification state was reinitialized and the sequence was started.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`.
 * @retval MC_STATUS_INVALID_STATE `inst->initialized == MC_FALSE`.
 * @par Sync/Async
 *   Synchronous start. The identification sequence itself completes asynchronously across subsequent `mc_fast_step()` calls.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_start_identification(mc_instance_t *inst);

/**
 * @brief Check whether parameter identification has completed
 * @param[in] inst Pointer to motor control instance.
 *   Range: readable `mc_instance_t` storage or `NULL`.
 * @return Identification completion flag.
 *   Range: `MC_TRUE` when `inst != NULL` and the identify state is `MC_IDENTIFY_STATE_COMPLETE`; otherwise `MC_FALSE`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant for concurrent read-only calls.
 */
mc_bool_t mc_is_identification_done(const mc_instance_t *inst);

/**
 * @brief Get the current trusted identification result snapshot
 * @param[in] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to readable `mc_instance_t` storage.
 * @param[out] params Output motor-parameter structure.
 *   Range: non-NULL pointer to writable `mc_motor_params_t` storage.
 * @note Invalid or non-physical `Rs`, `Ld`, and `Lq` estimates are reported as `0.0F`.
 * @note `flux_wb` may remain at the configured seed value when no valid flux estimate samples are collected.
 * @note The current `Ld`, `Lq`, and `flux_wb` windows follow actual discrete injected duration, so results track runtime `dt_s` rather than an idealized continuous pulse width.
 * @note Internal raw identify candidates such as `flux_candidate_wb` are not part of this API and are never returned here.
 * @note This function does not require identification to be complete; it returns the current trusted public snapshot and copies `pole_pairs` from `inst->cfg.motor`.
 * @retval MC_STATUS_OK Parameter snapshot was written successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL` or `params == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant for concurrent read-only calls when each call uses different writable `params` storage.
 */
mc_status_t mc_get_identified_params(const mc_instance_t *inst, mc_motor_params_t *params);

/**
 * @brief Disable the motor-control instance
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @retval MC_STATUS_OK Disable request was accepted.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`.
 * @retval MC_STATUS_INVALID_STATE `inst->initialized == MC_FALSE`.
 * @note In `MC_MODE_BLDC_SENSORLESS`, this call also resets the BLDC sensorless runtime back to idle.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_stop(mc_instance_t *inst);

/**
 * @brief Set the instance operating mode
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @param[in] mode Requested operating mode.
 *   Range: any `mc_mode_t`; after successful initialization only `inst->cfg.sensor.primary_mode` is accepted.
 * @retval MC_STATUS_OK `inst->mode` was updated successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`.
 * @retval MC_STATUS_INVALID_STATE `inst->initialized != MC_FALSE` and `mode != inst->cfg.sensor.primary_mode`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_set_mode(mc_instance_t *inst, mc_mode_t mode);

/**
 * @brief Set the enable state
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @param[in] enable Requested enable flag.
 *   Range: intended `MC_FALSE` or `MC_TRUE`; any nonzero `mc_bool_t` is stored as enabled.
 * @retval MC_STATUS_OK Enable state was updated successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`.
 * @retval MC_STATUS_INVALID_STATE `inst->initialized == MC_FALSE`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_set_enable(mc_instance_t *inst, mc_bool_t enable);

/**
 * @brief Store a mechanical speed reference and enable speed control
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @param[in] speed_ref Mechanical speed reference [RPM].
 *   Range: application-defined `mc_f32_t`; this setter does not clamp or validate the value.
 * @retval MC_STATUS_OK Reference was stored successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`.
 * @note This call only updates reference state and `speed_ctrl_enabled`; it does not require `inst->initialized == MC_TRUE`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_set_speed_ref(mc_instance_t *inst, mc_f32_t speed_ref);

/**
 * @brief Store a torque-producing current request and disable speed control
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @param[in] torque_ref Torque-producing reference value.
 *   Range: application-defined `mc_f32_t`; this setter does not clamp or validate the value.
 * @retval MC_STATUS_OK Reference was stored successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`.
 * @note The current implementation stores `torque_ref`, mirrors it into `iq_ref`, and, when `mtpa_enable != MC_FALSE`, recomputes `id_ref` from the current motor parameters.
 * @note This call only updates reference state and does not require `inst->initialized == MC_TRUE`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_set_torque_ref(mc_instance_t *inst, mc_f32_t torque_ref);

/**
 * @brief Store direct d/q current references and disable speed control
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @param[in] id_ref Direct-axis current reference [A].
 *   Range: application-defined `mc_f32_t`; this setter does not clamp or validate the value.
 * @param[in] iq_ref Quadrature-axis current reference [A].
 *   Range: application-defined `mc_f32_t`; this setter does not clamp or validate the value.
 * @retval MC_STATUS_OK References were stored successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`.
 * @note This call only updates reference state and does not require `inst->initialized == MC_TRUE`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_set_current_ref_dq(mc_instance_t *inst, mc_f32_t id_ref, mc_f32_t iq_ref);

/**
 * @brief Execute one fast-rate control cycle
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @param[in] in Fast-rate input snapshot.
 *   Range: non-NULL pointer to readable `mc_fast_input_t` storage.
 * @param[out] out Fast-rate output snapshot.
 *   Range: non-NULL pointer to writable `mc_fast_output_t` storage.
 * @retval MC_STATUS_OK Cycle completed successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`, `in == NULL`, or `out == NULL`.
 * @retval MC_STATUS_ERROR A delegated position-estimator, observer, or drive-control substep failed.
 * @note When `inst->enabled == MC_FALSE`, this call zeroes `out->pwm_cmd`, `out->adc_trigger_plan`, and `out->current_comp_status` and does not advance the control path.
 * @note When identification is active, this call executes the identify path instead of the normal drive path and applies configured PWM/ADC hooks synchronously before returning.
 * @par Sync/Async
 *   Synchronous. Configured `pwm_apply` and `adc_trigger_apply` hooks are invoked from this call before it returns.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object and different writable `out` storage. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_fast_step(mc_instance_t *inst, const mc_fast_input_t *in, mc_fast_output_t *out);

/**
 * @brief Execute one medium-rate control cycle
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @param[in] in Medium-rate input snapshot.
 *   Range: non-NULL pointer to readable `mc_medium_input_t` storage.
 * @param[out] out Medium-rate output snapshot.
 *   Range: non-NULL pointer to writable `mc_medium_output_t` storage.
 * @retval MC_STATUS_OK Cycle completed successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`, `in == NULL`, or `out == NULL`.
 * @retval MC_STATUS_ERROR A delegated sensor update or speed-loop substep failed.
 * @note When `inst->enabled == MC_FALSE`, sensor outputs may still update, but speed-control references are not advanced.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object and different writable `out` storage. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_medium_step(mc_instance_t *inst, const mc_medium_input_t *in, mc_medium_output_t *out);

/**
 * @brief Execute one slow-rate service cycle
 * @param[in,out] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to writable `mc_instance_t` storage.
 * @param[in] in Slow-rate input snapshot.
 *   Range: non-NULL pointer to readable `mc_slow_input_t` storage.
 * @param[out] out Slow-rate output snapshot.
 *   Range: non-NULL pointer to writable `mc_slow_output_t` storage.
 * @retval MC_STATUS_OK Cycle completed successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL`, `in == NULL`, or `out == NULL`.
 * @note Any nonzero `in->clear_fault_request` clears the stored diagnostic snapshot before `out` is populated.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses a different `inst` object and different writable `out` storage. Not reentrant for concurrent writes to the same `inst`.
 */
mc_status_t mc_slow_step(mc_instance_t *inst, const mc_slow_input_t *in, mc_slow_output_t *out);

/**
 * @brief Get a snapshot of the current mode and diagnostic state
 * @param[in] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to readable `mc_instance_t` storage.
 * @param[out] status Output status structure.
 *   Range: non-NULL pointer to writable `mc_slow_output_t` storage.
 * @retval MC_STATUS_OK Snapshot was written successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL` or `status == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant for concurrent read-only calls when each call uses different writable `status` storage.
 */
mc_status_t mc_get_status(const mc_instance_t *inst, mc_slow_output_t *status);

/**
 * @brief Get the current diagnostic snapshot
 * @param[in] inst Pointer to motor control instance.
 *   Range: non-NULL pointer to readable `mc_instance_t` storage.
 * @param[out] diag Output diagnostic status.
 *   Range: non-NULL pointer to writable `mc_diag_status_t` storage.
 * @retval MC_STATUS_OK Snapshot was written successfully.
 * @retval MC_STATUS_INVALID_ARG `inst == NULL` or `diag == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant for concurrent read-only calls when each call uses different writable `diag` storage.
 */
mc_status_t mc_get_diag(const mc_instance_t *inst, mc_diag_status_t *diag);

/**
 * @brief Get the packed library version number
 * @param[out] version Output version word.
 *   Range: non-NULL pointer to writable `uint32_t` storage.
 * @retval MC_STATUS_OK `*version` was written with `MC_VERSION_U32`.
 * @retval MC_STATUS_INVALID_ARG `version == NULL`.
 * @par Sync/Async
 *   Synchronous.
 * @par Reentrancy
 *   Reentrant when each concurrent call uses different writable `version` storage.
 */
mc_status_t mc_get_version(uint32_t *version);

#endif /* MC_API_H */
