# Interface Control Document / 接口控制文档

## 中文

### 公共 API
- 由 `mc_api.h` 定义。
- 初始化：`mc_init`；启动/停止：`mc_start`/`mc_stop`；复位：`mc_reset`
- 运行模式：
  - 快环 `mc_fast_step`（PWM 频率，~10–100 kHz）：ADC 采样 → FOC/BLDC → PWM 输出
  - 中环 `mc_medium_step`（~1–10 kHz）：角度估算、速度环、MTPA 计算
  - 慢环 `mc_slow_step`（~100–1000 Hz）：温度监测、故障处理、诊断

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
- `current_comp_status.mode` 当前定义为：
  - `MC_1SHUNT_COMP_NONE`
  - `MC_1SHUNT_COMP_PREDICT_BASIC`
  - `MC_1SHUNT_COMP_PREDICT_HIGH_MODULATION`
  - `MC_1SHUNT_COMP_PREDICT_FIELD_WEAKENING`

### 中环输出 / `mc_medium_output_t`
- `elec_angle_rad`：估算的电角度
- `mech_speed_rpm`：估算的机械转速
- `id_ref` / `iq_ref`：d/q 轴电流参考值（含 MTPA 输出）

### MTPA / 弱磁配置
- MTPA 使能：`mc_foc_cfg_t.mtpa_enable`
- 弱磁使能：`mc_foc_cfg_t.fw_enable`
- 弱磁 PI 系数：`fw_kp`、`fw_ki`、`fw_min_id`、`fw_activation_ratio`
- MTPA 公式：`id_mtpa = flux_wb / (2 * (lq_h - ld_h)) - sqrt(...)`
- 弱磁机制：基于 Vdq 模长与 `fw_activation_ratio * voltage_limit` 的偏差 PI 输出负 id

### 诊断快照 / `mc_diag_status_t`
- `active_fault` / `active_warning`：当前故障和警告
- `current_comp_status`：保存最近一次 `mc_fast_step(...)` 更新后的单电阻补偿状态。
- `sensorless_observer_valid` / `sensorless_pll_locked` / `sensorless_open_loop_active`：传感器无传感器估算器诊断
- `mc_get_diag(...)` 与 `mc_slow_step(...)` 均返回该快照，便于慢速诊断任务、日志或标定工具读取。
- 当控制器未使能、切到非 FOC 路径、或进入非单电阻补偿路径时，current_comp_status 会清零为 `MC_1SHUNT_COMP_NONE`。
- `mc_diag_1shunt_comp_mode_name(...)` 可将 `current_comp_status.mode` 转成稳定的可读字符串，便于日志和调试界面直接显示。

## English

### Public API
- Defined in `mc_api.h`.
- Init: `mc_init`; Start/Stop: `mc_start`/`mc_stop`; Reset: `mc_reset`
- Execution rates:
  - Fast loop `mc_fast_step` (PWM rate, ~10-100 kHz): ADC → FOC/BLDC → PWM
  - Medium loop `mc_medium_step` (~1-10 kHz): angle estimation, speed loop, MTPA
  - Slow loop `mc_slow_step` (~100-1000 Hz): temperature monitoring, fault handling, diagnostics

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
- `current_comp_status.mode` currently defines:
  - `MC_1SHUNT_COMP_NONE`
  - `MC_1SHUNT_COMP_PREDICT_BASIC`
  - `MC_1SHUNT_COMP_PREDICT_HIGH_MODULATION`
  - `MC_1SHUNT_COMP_PREDICT_FIELD_WEAKENING`

### Medium-Loop Output / `mc_medium_output_t`
- `elec_angle_rad`: estimated electrical angle
- `mech_speed_rpm`: estimated mechanical speed
- `id_ref` / `iq_ref`: d/q-axis current references (including MTPA output)

### MTPA / Flux Weakening Configuration
- MTPA enable: `mc_foc_cfg_t.mtpa_enable`
- FW enable: `mc_foc_cfg_t.fw_enable`
- FW PI gains: `fw_kp`, `fw_ki`, `fw_min_id`, `fw_activation_ratio`
- MTPA formula: `id_mtpa = flux_wb / (2 * (lq_h - ld_h)) - sqrt(...)`
- FW mechanism: PI output negative id driven by Vdq magnitude deviation from `fw_activation_ratio * voltage_limit`

### Diagnostic Snapshot / `mc_diag_status_t`
- `active_fault` / `active_warning`: current fault and warning states
- `current_comp_status`: stores the latest single-shunt compensation status updated by `mc_fast_step(...)`.
- `sensorless_observer_valid` / `sensorless_pll_locked` / `sensorless_open_loop_active`: sensorless estimator diagnostics
- `mc_get_diag(...)` and `mc_slow_step(...)` both expose this snapshot for slower diagnostic tasks, logging, or calibration tools.
- The field is cleared back to `MC_1SHUNT_COMP_NONE` when the controller is disabled, when execution switches to a non-FOC path, or when no single-shunt compensation is active.
- `mc_diag_1shunt_comp_mode_name(...)` converts `current_comp_status.mode` into a stable readable string for logs and debug UIs.
