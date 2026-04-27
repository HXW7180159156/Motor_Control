# Software Detailed Design - Control / 软件详细设计 - 控制层

## 中文

### FOC 快速电流环（`mc_pmsm_foc_run`）
- 流程：Clarke → Park → PI(id) + PI(iq) → MTPA id 注入 → 弱磁 id 注入 → dq 电压限幅 → InvPark → SVPWM → 1-shunt 规划 → ADC trigger 规划
- 输入：`mc_pmsm_foc_input_t`（含 id_ref / iq_ref，从慢/中环写入）
- 输出：`mc_pmsm_foc_output_t`（含 PWM 命令、ADC trigger 计划、电流重构补偿状态）

### MTPA（最大转矩电流比）
- 函数：`mc_pmsm_compute_mtpa_id(iq_ref, flux_wb, ld_h, lq_h)`
- 公式：`id_mtpa = flux_wb / (2 * (lq_h - ld_h)) - sqrt(flux_wb^2 / (4 * (lq_h - ld_h)^2) + iq_ref^2)`
- 适用：IPM（Ld < Lq），SPM（Ld ≈ Lq）返回 0
- 接线：API 层 `mc_medium_step` 计算 `id_mtpa` 写入 `inst->id_ref`，与指令 `id_ref` 取小

### 弱磁（Field Weakening, FW）
- 机制：基于上一周期限幅后 Vdq 模长与阈值的偏差，PI 积分输出 `id_fw_adjustment`
- 公式：`fw_error = fw_threshold - v_mag`，其中 `fw_threshold = fw_activation_ratio * voltage_limit`
- `fw_output = fw_kp * fw_error + id_fw_adjustment`，钳位 `[fw_min_id, 0]`
- PI 积分项 `id_fw_adjustment` 独立钳位 `[fw_min_id, 0]`，反饱和（anti-windup）通过钳位值回退
- 合成：`effective_id_ref = id_ref - |fw_output|`（即向负方向偏移）
- 可配置：`fw_enable`、`fw_kp`、`fw_ki`、`fw_min_id`、`fw_activation_ratio`

### 1-shunt 无效窗口补偿
- 窗口不足时启用预测补偿，使用简化 PMSM dq 模型
- 补偿模式：None / Basic / High Modulation / Field Weakening
- 诊断：`mc_1shunt_comp_status_t` 暴露当前补偿模式

## English

### FOC Fast Current Loop (`mc_pmsm_foc_run`)
- Pipeline: Clarke → Park → PI(id) + PI(iq) → MTPA id injection → FW id injection → dq voltage limiting → InvPark → SVPWM → 1-shunt planning → ADC trigger planning
- Input: `mc_pmsm_foc_input_t` (including id_ref / iq_ref written by slow/medium loops)
- Output: `mc_pmsm_foc_output_t` (including PWM commands, ADC trigger plan, current reconstruction compensation status)

### MTPA (Maximum Torque Per Ampere)
- Function: `mc_pmsm_compute_mtpa_id(iq_ref, flux_wb, ld_h, lq_h)`
- Formula: `id_mtpa = flux_wb / (2 * (lq_h - ld_h)) - sqrt(flux_wb^2 / (4 * (lq_h - ld_h)^2) + iq_ref^2)`
- Applicable: IPM (Ld < Lq), returns 0 for SPM (Ld ≈ Lq)
- Integration: API layer `mc_medium_step` computes `id_mtpa` into `inst->id_ref`, min'd with commanded `id_ref`

### Field Weakening (FW)
- Mechanism: PI integrator `id_fw_adjustment` driven by deviation of last-cycle limited Vdq magnitude from threshold
- Equation: `fw_error = fw_threshold - v_mag`, where `fw_threshold = fw_activation_ratio * voltage_limit`
- `fw_output = fw_kp * fw_error + id_fw_adjustment`, clamped to `[fw_min_id, 0]`
- PI integrator `id_fw_adjustment` independently clamped to `[fw_min_id, 0]`, anti-windup via clamp back-propagation
- Synthesis: `effective_id_ref = id_ref - |fw_output|` (shifted negative)
- Configurable: `fw_enable`, `fw_kp`, `fw_ki`, `fw_min_id`, `fw_activation_ratio`

### 1-Shunt Invalid-Window Compensation
- Predictive compensation using simplified PMSM dq model when window is insufficient
- Compensation modes: None / Basic / High Modulation / Field Weakening
- Diagnostics: `mc_1shunt_comp_status_t` exposes current compensation mode
