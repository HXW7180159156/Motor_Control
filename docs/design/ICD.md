# Interface Control Document / 接口控制文档

## 中文

### 公共 API
- 由 `mc_api.h` 定义。
- 初始化/复位/使能：`mc_init`、`mc_reset`、`mc_start`、`mc_stop`、`mc_set_enable`
- 模式与给定值：`mc_set_mode`、`mc_set_speed_ref`、`mc_set_torque_ref`、`mc_set_current_ref_dq`
- 周期执行：
  - 快环 `mc_fast_step`（PWM 频率，~10-100 kHz）：ADC 采样/重构 → FOC/BLDC/identify → PWM 输出
  - 中环 `mc_medium_step`（~1-10 kHz）：角度/速度更新、速度环、MTPA 计算
  - 慢环 `mc_slow_step`（~100-1000 Hz）：诊断快照与故障清除请求处理
- 状态读取：`mc_get_status`、`mc_get_diag`、`mc_get_version`
- 参数辨识：`mc_start_identification`、`mc_is_identification_done`、`mc_get_identified_params`
- `mc_set_mode()` 当前采用保守策略：`mc_init()` 成功后仅允许保持已初始化的 `primary_mode`，切到其他 mode 返回 `MC_STATUS_INVALID_STATE`
- `mc_reset()` 当前保留配置与初始化状态，仅清运行态、参考值、PI 状态、诊断与采样元数据
- `mc_start()` 在 `MC_MODE_BLDC_SENSORLESS` 下会公开调用 `mc_bldc_sensorless_start()`，从 `MC_BLDC_SENSORLESS_ALIGN` 进入启动序列
- `mc_start_identification()` 会按当前系统配置重建辨识配置，保留 `cfg.motor.flux_wb` 作为 seed，并清 `1-shunt` 运行元数据
- `mc_get_identified_params()` 仅返回 trusted `Rs` / `Ld` / `Lq` / `flux_wb`；raw candidate 不通过 public API 暴露；无效或非物理 `Rs` / `Ld` / `Lq` 返回 `0.0F`，`flux_wb` 可保留 seed

### 板级接口
- 由 `mc_port*.h` 定义。
- 单电阻 ADC 触发相关接口见：
  - `mc_port_adc.h`
  - `mc_port_adc_map.h`
  - `mc_port_adc_ref.h`

### ADC 校准 (`mc_adc_calibrate`)
- 声明在 `mc_port_adc.h`：`mc_status_t mc_adc_calibrate_linear(mc_adc_raw_t raw1, mc_f32_t phys1, mc_adc_raw_t raw2, mc_f32_t phys2, mc_adc_cal_t *cal);`
- 原理：已知两点 `(raw1, phys1)` 和 `(raw2, phys2)`，计算 `scale = (phys2 - phys1) / (raw2 - raw1)` 和 `offset = phys1 - scale * raw1`
- 输出 `mc_adc_cal_t`：`{mc_f32_t scale; mc_f32_t offset;}`
- 转换：`physical = raw * scale + offset`

### 快环输出 / `mc_fast_output_t`
- `pwm_cmd`：当前周期 PWM 下发命令。
- `adc_trigger_plan`：当前周期 ADC 触发计划（1-shunt/2-shunt/3-shunt 通用）。
- `current_comp_status`：当前周期电流重构补偿状态。
- `current_comp_status.active == MC_TRUE` 表示本周期使用了单电阻无效采样窗口补偿。
- 控制器 disabled 时，`mc_fast_step(...)` 返回全零 PWM/ADC 计划，并清空补偿与无感诊断。
- `identify` 激活时，快环优先执行辨识状态机，并复用当前配置的 `1/2/3-shunt` 电流重构链路；`1-shunt` 下仍会生成 PWM 优化与 ADC trigger 计划。
- `current_comp_status.mode` 当前定义为：
  - `MC_1SHUNT_COMP_NONE`
  - `MC_1SHUNT_COMP_PREDICT_BASIC`
  - `MC_1SHUNT_COMP_PREDICT_HIGH_MODULATION`
  - `MC_1SHUNT_COMP_PREDICT_FIELD_WEAKENING`
- 2-shunt / 3-shunt 当前默认生成一个中点触发（`position = 0.5`, `event = PWM_DOWN`）。

### 中环输出 / `mc_medium_output_t`
- `elec_angle_rad`：估算的电角度
- `mech_speed_rpm`：估算的机械转速
- `id_ref` / `iq_ref`：API 层生成的 d/q 轴电流参考值；`id_ref` 可已包含 API 侧 MTPA 结果，但不含快环内部 FW 负向修正
- 实例 disabled 时，中环仍返回当前状态量，但不推进速度环或 MTPA 更新

### MTPA / 弱磁配置
- MTPA 使能：`mc_foc_cfg_t.mtpa_enable`
- 弱磁使能：`mc_foc_cfg_t.fw_enable`
- 弱磁 PI 系数：`fw_kp`、`fw_ki`、`fw_min_id`、`fw_activation_ratio`
- MTPA 公式：`id_mtpa = flux_wb / (2 * (lq_h - ld_h)) - sqrt(...)`
- MTPA 当前由 API 层接线：`mc_set_torque_ref()` 直接转矩模式下可立即更新 `inst->id_ref`，`mc_medium_step()` 在速度控制使能时按当前 `iq_ref` 再计算一次
- 直接 `mc_set_current_ref_dq()` 路径保持调用者给定的 `id_ref` / `iq_ref`，不会自动重写成 MTPA 值
- 弱磁机制在 `mc_pmsm_foc_run()` 内完成：基于上一周期 Vdq 模长与 `fw_activation_ratio * voltage_limit` 的偏差，输出负向 `id` 修正

### 诊断快照 / `mc_diag_status_t`
- `active_fault` / `active_warning`：当前故障和警告
- `current_comp_status`：保存最近一次 `mc_fast_step(...)` 更新后的单电阻补偿状态。
- `sensorless_observer_valid` / `sensorless_pll_locked` / `sensorless_open_loop_active`：传感器无传感器估算器诊断
- `mc_get_diag(...)` 与 `mc_slow_step(...)` 均返回该快照，便于慢速诊断任务、日志或标定工具读取。
- 当控制器未使能、切到非 FOC 路径、或进入非单电阻补偿路径时，current_comp_status 会清零为 `MC_1SHUNT_COMP_NONE`。
- `mc_diag_1shunt_comp_mode_name(...)` 可将 `current_comp_status.mode` 转成稳定的可读字符串，便于日志和调试界面直接显示。
- PMSM sensorless 路径下，当 `pll_locked == MC_FALSE` 时，`active_warning` 当前置为 `MC_WARNING_OBSERVER_UNLOCKED`。
- BLDC sensorless 路径当前复用 `sensorless_observer_valid` / `sensorless_open_loop_active` 诊断位表示是否已进入 `RUN` 相位；`sensorless_pll_locked` 固定为 `MC_FALSE`。

## English

### Public API
- Defined in `mc_api.h`.
- Init / reset / enable: `mc_init`, `mc_reset`, `mc_start`, `mc_stop`, `mc_set_enable`
- Mode and reference control: `mc_set_mode`, `mc_set_speed_ref`, `mc_set_torque_ref`, `mc_set_current_ref_dq`
- Periodic execution:
  - Fast loop `mc_fast_step` (PWM rate, ~10-100 kHz): ADC sampling/reconstruction -> FOC/BLDC/identify -> PWM
  - Medium loop `mc_medium_step` (~1-10 kHz): angle/speed update, speed loop, MTPA
  - Slow loop `mc_slow_step` (~100-1000 Hz): diagnostic snapshot and fault-clear request handling
- Status access: `mc_get_status`, `mc_get_diag`, `mc_get_version`
- Motor identification: `mc_start_identification`, `mc_is_identification_done`, `mc_get_identified_params`
- `mc_set_mode()` currently uses a conservative policy: after successful `mc_init()`, only the initialized `primary_mode` remains valid; switching to any other mode returns `MC_STATUS_INVALID_STATE`
- `mc_reset()` currently preserves configuration and initialization state while clearing runtime state, references, PI state, diagnostics, and sampling metadata
- `mc_start()` publicly routes BLDC sensorless startup through `mc_bldc_sensorless_start()` and enters `MC_BLDC_SENSORLESS_ALIGN`
- `mc_start_identification()` rebuilds the identify configuration from the current system configuration, keeps `cfg.motor.flux_wb` as the seed, and clears runtime `1-shunt` metadata
- `mc_get_identified_params()` returns trusted `Rs` / `Ld` / `Lq` / `flux_wb` only; raw candidates are never exposed through the public API; invalid or non-physical `Rs` / `Ld` / `Lq` are reported as `0.0F`, and `flux_wb` may remain at the seed value

### Board-Level Interfaces
- Defined in `mc_port*.h`.
- Single-shunt ADC trigger interfaces:
  - `mc_port_adc.h`
  - `mc_port_adc_map.h`
  - `mc_port_adc_ref.h`

### ADC Calibration (`mc_adc_calibrate`)
- Declared in `mc_port_adc.h`: `mc_status_t mc_adc_calibrate_linear(mc_adc_raw_t raw1, mc_f32_t phys1, mc_adc_raw_t raw2, mc_f32_t phys2, mc_adc_cal_t *cal);`
- Principle: Given two points `(raw1, phys1)` and `(raw2, phys2)`, compute `scale = (phys2 - phys1) / (raw2 - raw1)` and `offset = phys1 - scale * raw1`
- Output `mc_adc_cal_t`: `{mc_f32_t scale; mc_f32_t offset;}`
- Conversion: `physical = raw * scale + offset`

### Fast-Loop Output / `mc_fast_output_t`
- `pwm_cmd`: PWM command for the current cycle.
- `adc_trigger_plan`: ADC trigger plan for the current cycle (1-shunt/2-shunt/3-shunt).
- `current_comp_status`: current reconstruction compensation status for the current cycle.
- `current_comp_status.active == MC_TRUE` means the cycle used single-shunt invalid-window compensation.
- When the controller is disabled, `mc_fast_step(...)` returns zeroed PWM/ADC outputs and clears compensation and sensorless diagnostics.
- While `identify` is active, the fast loop runs the identify state machine instead of the normal control path and reuses the configured `1/2/3-shunt` current reconstruction path; `1-shunt` still produces PWM optimization and ADC trigger planning.
- `current_comp_status.mode` currently defines:
  - `MC_1SHUNT_COMP_NONE`
  - `MC_1SHUNT_COMP_PREDICT_BASIC`
  - `MC_1SHUNT_COMP_PREDICT_HIGH_MODULATION`
  - `MC_1SHUNT_COMP_PREDICT_FIELD_WEAKENING`
- 2-shunt / 3-shunt currently default to one midpoint trigger (`position = 0.5`, `event = PWM_DOWN`).

### Medium-Loop Output / `mc_medium_output_t`
- `elec_angle_rad`: estimated electrical angle
- `mech_speed_rpm`: estimated mechanical speed
- `id_ref` / `iq_ref`: API-side d/q-axis current references; `id_ref` may already include API-side MTPA output, but does not include the fast-loop field-weakening adjustment
- When the instance is disabled, the medium loop still reports current state values but does not advance speed-loop or MTPA state

### MTPA / Flux Weakening Configuration
- MTPA enable: `mc_foc_cfg_t.mtpa_enable`
- FW enable: `mc_foc_cfg_t.fw_enable`
- FW PI gains: `fw_kp`, `fw_ki`, `fw_min_id`, `fw_activation_ratio`
- MTPA formula: `id_mtpa = flux_wb / (2 * (lq_h - ld_h)) - sqrt(...)`
- MTPA is currently wired in the API layer: `mc_set_torque_ref()` can update `inst->id_ref` immediately for direct-torque control, and `mc_medium_step()` recomputes it from the current `iq_ref` while speed control is enabled
- Direct `mc_set_current_ref_dq()` preserves the caller-provided `id_ref` / `iq_ref` and does not automatically overwrite them with an MTPA value
- FW is applied inside `mc_pmsm_foc_run()`: a negative `id` adjustment is generated from the previous-cycle Vdq magnitude deviation from `fw_activation_ratio * voltage_limit`

### Diagnostic Snapshot / `mc_diag_status_t`
- `active_fault` / `active_warning`: current fault and warning states
- `current_comp_status`: stores the latest single-shunt compensation status updated by `mc_fast_step(...)`.
- `sensorless_observer_valid` / `sensorless_pll_locked` / `sensorless_open_loop_active`: sensorless estimator diagnostics
- `mc_get_diag(...)` and `mc_slow_step(...)` both expose this snapshot for slower diagnostic tasks, logging, or calibration tools.
- The field is cleared back to `MC_1SHUNT_COMP_NONE` when the controller is disabled, when execution switches to a non-FOC path, or when no single-shunt compensation is active.
- `mc_diag_1shunt_comp_mode_name(...)` converts `current_comp_status.mode` into a stable readable string for logs and debug UIs.
- On the PMSM sensorless path, `active_warning` is currently set to `MC_WARNING_OBSERVER_UNLOCKED` whenever `pll_locked == MC_FALSE`.
- On the BLDC sensorless path, `sensorless_observer_valid` and `sensorless_open_loop_active` are currently reused as phase proxies for whether the drive has reached `RUN`; `sensorless_pll_locked` remains `MC_FALSE`.
