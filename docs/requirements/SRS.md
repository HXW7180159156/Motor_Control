# Software Requirements Specification / 软件需求规格说明

## 编号规则 / Numbering Rule

- 需求编号格式：`REQ-MC-XXYY`
- XX = 域（domain），YY = 域内序号
- Domains: 01 = 工具链/编译器, 02 = 安全, 03 = 控制, 04 = 辨识, 05 = 电流检测, 06 = 传感器, 07 = 数值路径, 08 = 驱动, 09 = 时序, 10 = 精度, 11 = 故障检测, 12 = 标定

---

## 01 — 工具链与构建 / Toolchain & Build

- `REQ-MC-0101` 库应支持 GCC、IAR、Keil (ARMCC)、GHS 工具链。
- `REQ-MC-0102` 库应遵循 C99 语言标准，禁用 GNU 扩展和编译器特异性扩展。
- `REQ-MC-0103` 库应无动态内存分配（无 `malloc`/`free`/`realloc` 调用）。
- `REQ-MC-0104` 库应支持 ARM Cortex-M0+/M4F/M7/M33 和 Cortex-R5F/R52 目标架构。
- `REQ-MC-0105` 库应通过编译器宏适配 GCC `__attribute__`、IAR `_Pragma`、ARMCC `__align`、GHS 等差异化语法。

---

## 02 — 安全 / Safety

- `REQ-MC-0201` 库在检测到 `MC_FAULT_OVERCURRENT` 或 `MC_FAULT_OVERVOLTAGE` 时应立即（同一 fast_step 周期内）将 PWM 输出置零。
- `REQ-MC-0202` 库在传感器信号无效时（如 Resolver 信号低于 `min_signal_amplitude`、Hall 编码不在合法序列中）应将 `observer_valid` 标记为 `MC_FALSE` 并停止角度/速度更新。
- `REQ-MC-0203` 库在 `mc_fast_step` 入口处的所有指针参数（`inst`、`in`、`out`）必须做 NULL 检查，违反时返回 `MC_STATUS_INVALID_ARG`。
- `REQ-MC-0204` 库不得在 `MC_MODE_DISABLED` 或 `inst->enabled == MC_FALSE` 状态下产生非零 PWM 输出。
- `REQ-MC-0205` 库在辨识过程中若电流超过 `id->cfg.max_current_a`，应立即中止辨识并进入 `MC_IDENTIFY_STATE_ERROR`，PWM 归零。
- `REQ-MC-0206` 库的 `mc_slow_step` 应支持 `clear_fault_request` 清除已存储的诊断快照。
- `REQ-MC-0207` 库在母線电压低于 `bus_voltage_min` 时应将有效母线电压钳位至 `bus_voltage_min`，用于电压归一化计算。

---

## 03 — 控制 / Control

- `REQ-MC-0301` 库应提供 PMSM FOC 控制主链；`float32` 路径当前覆盖 Hall、Encoder、Resolver 与首版 Sensorless。
- `REQ-MC-0302` 库应提供 API 级运行时诊断快照，至少包含 `1-shunt` 补偿状态与 PMSM sensorless 的有效/锁定/开环标志。
- `REQ-MC-0303` 库应将 PI 控制器输出和积分项限制在配置的 `output_min`/`output_max` 和 `integral_min`/`integral_max` 范围内。
- `REQ-MC-0304` 库的 SVPWM 模块应将输出电压矢量限制在 `cfg->modulation_limit` 内，并在超限时等比例缩放。
- `REQ-MC-0305` 库的弱磁控制器应在逆变器电压饱和时（`v_mag < fw_threshold`）激活，并通过 PI 控制器逐步减小 d 轴电流，直至达到 `fw_min_id` 限制。
- `REQ-MC-0306` 库的 `mc_set_speed_ref` 和 `mc_set_torque_ref` 应为互相排斥的控制模式：设置速度参考应禁用转矩控制，反之亦然。
- `REQ-MC-0307` 库应支持通过 `mc_set_current_ref_dq` 直接指定 dq 电流参考，绕过速度和转矩控制回路。

---

## 04 — 辨识 / Identification

- `REQ-MC-0401` 库应提供电机参数辨识路径，可输出 `Rs`、`Ld`、`Lq`、`flux_wb`，并仅通过 public API 暴露 trusted 结果。
- `REQ-MC-0402` 辨识的 Rs 估计值应通过多采样点平均计算，无效或非物理（负值）结果应报告为 `0.0F`。
- `REQ-MC-0403` 辨识的 Ld/Lq 估计应从脉冲响应中提取，使用时间加权平均计算，并扣除电阻压降。
- `REQ-MC-0404` 辨识的 `flux_wb` 估计应仅在有效采集窗口足够长（≥ `MC_IDENTIFY_MIN_VALID_FLUX_WINDOW_RATIO` × 配置脉冲时长）后才覆盖配置种子值。
- `REQ-MC-0405` 辨识应遵循离散注入时长的实际积累值来推进各阶段的步数计数，以适应变化的 `dt_s`。
- `REQ-MC-0406` 辨识完成后应将 trusted 结果写回到运行时 FOC 模型（`rs_ohm`, `ld_h`, `lq_h`, `flux_wb`）和 sensorless 模型。

---

## 05 — 电流检测 / Current Sensing

- `REQ-MC-0501` 库应支持单电阻、双电阻、三电阻采样方案。
- `REQ-MC-0502` `identify` 与 PMSM sensorless 路径应复用当前配置的电流重构链路，而不是使用单独硬编码缩放。
- `REQ-MC-0503` 1-shunt 重构应基于 SVPWM 扇区和占空比判断有效采样窗口，并在采样窗口不足一个 PWM 周期内无法获取两相独立电流时使用预测补偿。
- `REQ-MC-0504` 2-shunt 重构应从 A/B 相 ADC 采样计算三相电流，第三相通过基尔霍夫电流定律（`ic = -(ia+ib)`）推导。
- `REQ-MC-0505` 3-shunt 重构应将 A/B/C 三相 ADC 采样分别缩放和偏移后直接输出。
- `REQ-MC-0506` 电流重构输出应以安培为单位，通过 `(raw - offset) * scale` 计算。

---

## 06 — 传感器 / Sensors

- `REQ-MC-0601` PMSM sensorless 路径应提供基线 `LPF + PLL` 观测器，包含开环启动与连续样本 lock/unlock 判据。
- `REQ-MC-0602` PMSM sensorless 初始化应校验 `min_bemf < lock_bemf` 以及开环电压斜坡不倒挂。
- `REQ-MC-0603` PMSM sensorless 观测器应使用 `L * di/dt` 补偿项从反电动势方程中扣除电感电压分量。
- `REQ-MC-0604` Sensorless 开环启动应使用电压-频率斜坡：从 `open_loop_voltage_start` 匀速升至 `open_loop_voltage_max`，频率从 0 升至 `startup_speed_rad_s`。
- `REQ-MC-0605` Sensorless 在 `open_loop_active` 期间应忽略 BEMF 测量角度，使用积分方式推算电气位置。
- `REQ-MC-0606` Sensorless 开环退出应在连续 `MC_SENSORLESS_OPEN_LOOP_LOCK_SAMPLES` 个周期内同时满足 BEMF 幅度 ≥ `lock_bemf` 且角度误差 < `MC_SENSORLESS_OPEN_LOOP_HANDOFF_ANGLE_ERR_RAD` 两个条件。
- `REQ-MC-0607` Hall 传感器应由 `hall_code_sequence` 查找表映射 Hall 码到电气角度，在每次状态变迁时更新转速。
- `REQ-MC-0608` Encoder 传感器应将 raw count 对 `counts_per_rev` 取模后转换为机械角度，再乘以极对数转换为电气角度。
- `REQ-MC-0609` Resolver 传感器应通过 `atan2f(sin, cos)` 计算机械角度，并校验信号幅度 ≥ `min_signal_amplitude` 后标记有效。
- `REQ-MC-0610` 各传感器应在 timestamp 差值大于 0 且有状态变化的条件下更新速度估计。

---

## 07 — 数值路径 / Numeric Paths

- `REQ-MC-0701` 当前顶层 `Q31` 执行路径应仅支持 `PMSM FOC + Hall/Encoder + 2-shunt/3-shunt`；其他组合不在当前支持范围内。
- `REQ-MC-0702` `float32` 路径应覆盖全部控制模式、全部传感器类型和全部电流检测拓扑。
- `REQ-MC-0703` Q31 路径的 PI 控制器、Clarke/Park/InvPark 变换和电流重构应使用饱和算术（`mc_q31_add_sat`、`mc_q31_mul`）防止溢出。
- `REQ-MC-0704` Q31 路径的 `mc_q31_from_f32` 和 `mc_q31_to_f32` 应为有损但可逆的转换（1 个 f32 值映射到 1 个 q31 值）。

---

## 08 — 驱动 / Drive

- `REQ-MC-0801` BLDC sensorless 路径应可通过顶层 `mc_start()` 公开启动。
- `REQ-MC-0802` BLDC Hall 六步换相应基于 `hall_code` 映射到 6 个换相扇区，每个扇区分配 PWM/LOW/OFF 三相模式。
- `REQ-MC-0803` BLDC sensorless 启动过程应为 IDLE→ALIGN→RAMP_OPEN→RUN 四阶段状态机：ALIGN 阶段锁定预定位角，RAMP_OPEN 阶段开环加速，RUN 阶段基于 BEMF 过零检测闭环换相。
- `REQ-MC-0804` BLDC sensorless 的零交叉检测应支持可配置的去抖阈值（`zc_debounce_threshold`），以及基于换相步奇偶校验的上升/下降沿判断。
- `REQ-MC-0805` BLDC sensorless 在 RUN 阶段应支持基于速度 PI 的占空比闭环调节。

---

## 09 — 时序与实时性 / Timing & Real-time

- `REQ-MC-0901` `mc_fast_step`（FOC 电流环）的 WCET（最坏执行时间）在 Cortex-M4F @ 168MHz、启用 FPU 的条件下应 ≤ 30 μs。
- `REQ-MC-0902` `mc_medium_step`（速度环）的 WCET 在同条件下应 ≤ 10 μs。
- `REQ-MC-0903` `mc_slow_step`（监控环）的 WCET 在同条件下应 ≤ 5 μs。
- `REQ-MC-0904` `mc_init`（初始化）应在同步调用中完成，不得启动异步任务或 ISR。
- `REQ-MC-0905` `mc_fast_step` 内的 PWM/ADC 平台钩子调用（`pwm_apply`、`adc_trigger_apply`）应在本函数返回前同步完成。

---

## 10 — 精度 / Accuracy

- `REQ-MC-1001` PMSM FOC 速度闭环稳态误差在额定负载下应 ≤ 1% 额定转速。
- `REQ-MC-1002` Clarke 变换 `mc_clarke_run` 在输入为平衡三相正弦信号时的 β 分量计算误差应 ≤ 1e-4（相对于数学真值）。
- `REQ-MC-1003` Park/InvPark 变换的 round-trip 误差（`Park(InvPark(v))`）应 ≤ 1e-5。
- `REQ-MC-1004` SVPWM 输出的三相占空比之和在任何扇区下应始终 < 3.0，且常见模式偏置后的占空比应分布在 [0, 1] 区间内。
- `REQ-MC-1005` Sensorless 观测器在稳定运行状态下（BEMF 充足）的角度估计误差应 ≤ 10° 电气角。
- `REQ-MC-1006` 参数辨识的 Rs 估计精度应在真实值的 ±20% 以内（受限于开环直流注入法的物理约束）。

---

## 11 — 故障检测与诊断 / Fault Detection & Diagnostics

- `REQ-MC-1101` 库应在辨识过程中检测过流：任一相电流绝对值超过 `max_current_a` 时终止辨识。
- `REQ-MC-1102` 库的 `mc_diag_status_t` 应能编码至少 4 种故障和 2 种警告。
- `REQ-MC-1103` 1-shunt 补偿模式应记录到输出诊断字段中（NONE / PREDICT_BASIC / PREDICT_HIGH_MODULATION / PREDICT_FIELD_WEAKENING）。
- `REQ-MC-1104` 诊断快照可通过 `mc_get_diag` API 在任意时刻读取，不论实例是否 enabled。
- `REQ-MC-1105` Sensorless 观测器在 BEMF 幅度低于 `min_bemf` 时应将 `observer_valid` 置为 `MC_FALSE`，并清零 PLL 锁定状态。
- `REQ-MC-1106` 库应在 `mc_init` 失败后确保 `inst->initialized == MC_FALSE`。

---

## 12 — 标定 / Calibration

- `REQ-MC-1201` 库应提供 `mc_adc_calibrate_linear` 两点线性标定函数，输入两组 (raw, physical) 配对值，输出 scale 和 offset。
- `REQ-MC-1202` 两点标定应拒绝 raw_low == raw_high 的无效输入。
- `REQ-MC-1203` ADC 标定结果应能配置到各电流/电压重构链路中。

---

## English / 英文

### Numbering Rule

- Requirement ID format: `REQ-MC-XXYY`
- XX = domain, YY = sequence within domain
- Domains: 01 = Toolchain/Build, 02 = Safety, 03 = Control, 04 = Identification, 05 = Current Sensing, 06 = Sensors, 07 = Numeric Paths, 08 = Drive, 09 = Timing, 10 = Accuracy, 11 = Fault Detection, 12 = Calibration

---

### 01 — Toolchain & Build

- `REQ-MC-0101` The library shall support GCC, IAR, Keil (ARMCC), and GHS toolchains.
- `REQ-MC-0102` The library shall conform to the C99 language standard, with GNU extensions and compiler-specific extensions disabled.
- `REQ-MC-0103` The library shall not use dynamic memory allocation (no `malloc`/`free`/`realloc` calls).
- `REQ-MC-0104` The library shall support ARM Cortex-M0+/M4F/M7/M33 and Cortex-R5F/R52 target architectures.
- `REQ-MC-0105` The library shall adapt compiler-specific syntax (GCC `__attribute__`, IAR `_Pragma`, ARMCC `__align`, GHS) through compiler-detection macros.

### 02 — Safety

- `REQ-MC-0201` The library shall immediately zero the PWM output (within the same `mc_fast_step` cycle) upon detecting `MC_FAULT_OVERCURRENT` or `MC_FAULT_OVERVOLTAGE`.
- `REQ-MC-0202` The library shall mark `observer_valid` as `MC_FALSE` and stop angle/speed updates when a sensor signal is invalid (e.g., resolver amplitude below `min_signal_amplitude`, Hall code not in the legal sequence).
- `REQ-MC-0203` The library shall NULL-check all pointer parameters (`inst`, `in`, `out`) at the entry of `mc_fast_step` and return `MC_STATUS_INVALID_ARG` on violation.
- `REQ-MC-0204` The library shall not produce non-zero PWM output when in `MC_MODE_DISABLED` or when `inst->enabled == MC_FALSE`.
- `REQ-MC-0205` The library shall abort identification (enter `MC_IDENTIFY_STATE_ERROR`, zero PWM) if the measured current exceeds `id->cfg.max_current_a` during the identification sequence.
- `REQ-MC-0206` The library's `mc_slow_step` shall support clearing the stored diagnostic snapshot via `clear_fault_request`.
- `REQ-MC-0207` The library shall clamp the effective bus voltage to `bus_voltage_min` when the measured bus voltage falls below that threshold, for voltage normalization purposes.

### 03 — Control

- `REQ-MC-0301` The library shall provide a PMSM FOC control path; the current `float32` path covers Hall, encoder, resolver, and a first sensorless implementation.
- `REQ-MC-0302` The library shall provide an API-level runtime diagnostic snapshot including at least `1-shunt` compensation status and PMSM sensorless valid / locked / open-loop flags.
- `REQ-MC-0303` The library shall clamp PI controller output and integral term within the configured `output_min`/`output_max` and `integral_min`/`integral_max` ranges.
- `REQ-MC-0304` The library's SVPWM module shall limit the output voltage vector within `cfg->modulation_limit` and scale proportionally when exceeded.
- `REQ-MC-0305` The library's field-weakening controller shall activate when the inverter voltage saturates (`v_mag < fw_threshold`) and gradually reduce the d-axis current through a PI controller, down to the `fw_min_id` limit.
- `REQ-MC-0306` The library's `mc_set_speed_ref` and `mc_set_torque_ref` shall be mutually exclusive control modes: setting a speed reference shall disable torque control, and vice versa.
- `REQ-MC-0307` The library shall support direct dq current reference setting via `mc_set_current_ref_dq`, bypassing speed and torque control loops.

### 04 — Identification

- `REQ-MC-0401` The library shall provide a motor-parameter identification path for `Rs`, `Ld`, `Lq`, and `flux_wb`, and shall expose trusted results only through the public API.
- `REQ-MC-0402` The identification Rs estimate shall be computed from multi-sample averaging; invalid or non-physical (negative) results shall be reported as `0.0F`.
- `REQ-MC-0403` The identification Ld/Lq estimates shall be extracted from pulse response using time-weighted averaging with resistive voltage drop subtraction.
- `REQ-MC-0404` The identification `flux_wb` estimate shall replace the configured seed value only when the valid sampling window is sufficiently long (≥ `MC_IDENTIFY_MIN_VALID_FLUX_WINDOW_RATIO` × configured pulse duration).
- `REQ-MC-0405` Identification shall advance step counts based on the actual accumulated discrete injection duration, adapting to variable `dt_s`.
- `REQ-MC-0406` Upon completion, identification shall write trusted results back into the runtime FOC model (`rs_ohm`, `ld_h`, `lq_h`, `flux_wb`) and sensorless model.

### 05 — Current Sensing

- `REQ-MC-0501` The library shall support 1-shunt, 2-shunt, and 3-shunt current sensing.
- `REQ-MC-0502` The `identify` and PMSM sensorless paths shall reuse the configured current reconstruction chain instead of using separate hard-coded scaling.
- `REQ-MC-0503` 1-shunt reconstruction shall determine valid sampling windows based on SVPWM sector and duty cycles, and shall use predictive compensation when two independent phase currents cannot be obtained within a single PWM period.
- `REQ-MC-0504` 2-shunt reconstruction shall derive three-phase currents from A/B phase ADC samples, computing the third phase via Kirchhoff's current law (`ic = -(ia+ib)`).
- `REQ-MC-0505` 3-shunt reconstruction shall directly output A/B/C phase ADC samples after applying individual scale and offset per channel.
- `REQ-MC-0506` Current reconstruction output shall be in amperes, computed as `(raw - offset) * scale`.

### 06 — Sensors

- `REQ-MC-0601` The PMSM sensorless path shall provide a baseline `LPF + PLL` observer with open-loop startup and consecutive-sample lock / unlock criteria.
- `REQ-MC-0602` PMSM sensorless initialization shall validate `min_bemf < lock_bemf` and a non-inverted open-loop voltage ramp.
- `REQ-MC-0603` The PMSM sensorless observer shall use `L * di/dt` compensation to subtract the inductive voltage component from the back-EMF equation.
- `REQ-MC-0604` Sensorless open-loop startup shall use a voltage-frequency ramp: linearly increasing from `open_loop_voltage_start` to `open_loop_voltage_max`, frequency from 0 to `startup_speed_rad_s`.
- `REQ-MC-0605` During `open_loop_active`, the sensorless observer shall ignore BEMF-measured angle and use integration to estimate the electrical position.
- `REQ-MC-0606` Sensorless open-loop exit shall require both BEMF magnitude ≥ `lock_bemf` and angle error < `MC_SENSORLESS_OPEN_LOOP_HANDOFF_ANGLE_ERR_RAD` for at least `MC_SENSORLESS_OPEN_LOOP_LOCK_SAMPLES` consecutive cycles.
- `REQ-MC-0607` The Hall sensor shall map Hall codes to electrical angles via a `hall_code_sequence` lookup table and update speed on each state transition.
- `REQ-MC-0608` The encoder sensor shall convert the raw count modulo `counts_per_rev` to mechanical angle, then multiply by pole pairs to obtain electrical angle.
- `REQ-MC-0609` The resolver sensor shall compute mechanical angle via `atan2f(sin, cos)` and validate signal amplitude ≥ `min_signal_amplitude` before marking as valid.
- `REQ-MC-0610` Each sensor shall update speed estimates only when the timestamp delta is positive and a state change has occurred.

### 07 — Numeric Paths

- `REQ-MC-0701` The current top-level `Q31` execution path shall be limited to `PMSM FOC + Hall/Encoder + 2-shunt/3-shunt`; other combinations are outside the current supported scope.
- `REQ-MC-0702` The `float32` path shall cover all control modes, all sensor types, and all current sensing topologies.
- `REQ-MC-0703` The Q31 path PI controllers, Clarke/Park/InvPark transforms, and current reconstruction shall use saturating arithmetic (`mc_q31_add_sat`, `mc_q31_mul`) to prevent overflow.
- `REQ-MC-0704` The Q31 path `mc_q31_from_f32` and `mc_q31_to_f32` shall be lossy but invertible conversions (one f32 value maps to one q31 value).

### 08 — Drive

- `REQ-MC-0801` The BLDC sensorless path shall be startable through the top-level `mc_start()` API.
- `REQ-MC-0802` BLDC Hall six-step commutation shall map `hall_code` to 6 commutation sectors, each assigning PWM/LOW/OFF to the three phases.
- `REQ-MC-0803` The BLDC sensorless startup sequence shall follow a four-stage state machine IDLE→ALIGN→RAMP_OPEN→RUN: ALIGN locks the pre-positioning angle, RAMP_OPEN accelerates open-loop, RUN commutates closed-loop based on BEMF zero-crossing detection.
- `REQ-MC-0804` BLDC sensorless zero-crossing detection shall support a configurable debounce threshold (`zc_debounce_threshold`) and rising/falling edge discrimination based on commutation step parity.
- `REQ-MC-0805` BLDC sensorless in RUN phase shall support duty-cycle closed-loop regulation via a speed PI controller.

### 09 — Timing & Real-time

- `REQ-MC-0901` `mc_fast_step` (FOC current loop) WCET on Cortex-M4F @ 168MHz with FPU enabled shall be ≤ 30 μs.
- `REQ-MC-0902` `mc_medium_step` (speed loop) WCET under the same conditions shall be ≤ 10 μs.
- `REQ-MC-0903` `mc_slow_step` (monitoring loop) WCET under the same conditions shall be ≤ 5 μs.
- `REQ-MC-0904` `mc_init` shall complete synchronously and shall not start asynchronous tasks or ISRs.
- `REQ-MC-0905` The PWM/ADC platform hook calls (`pwm_apply`, `adc_trigger_apply`) within `mc_fast_step` shall complete synchronously before the function returns.

### 10 — Accuracy

- `REQ-MC-1001` PMSM FOC speed closed-loop steady-state error at rated load shall be ≤ 1% of rated speed.
- `REQ-MC-1002` Clarke transform `mc_clarke_run` beta-component computation error with balanced three-phase sinusoidal input shall be ≤ 1e-4 relative to the mathematical true value.
- `REQ-MC-1003` Park/InvPark round-trip error (`Park(InvPark(v))`) shall be ≤ 1e-5.
- `REQ-MC-1004` SVPWM output three-phase duty sum across all sectors shall always be < 3.0, and common-mode shifted duties shall fall within [0, 1].
- `REQ-MC-1005` Sensorless observer angle estimation error in steady state (adequate BEMF) shall be ≤ 10° electrical.
- `REQ-MC-1006` Parameter identification Rs estimate accuracy shall be within ±20% of the true value (limited by the physical constraints of the open-loop DC injection method).

### 11 — Fault Detection & Diagnostics

- `REQ-MC-1101` The library shall detect overcurrent during identification: if any phase current absolute value exceeds `max_current_a`, identification shall be terminated.
- `REQ-MC-1102` The library's `mc_diag_status_t` shall be capable of encoding at least 4 fault types and 2 warning types.
- `REQ-MC-1103` 1-shunt compensation mode shall be recorded in the output diagnostic field (NONE / PREDICT_BASIC / PREDICT_HIGH_MODULATION / PREDICT_FIELD_WEAKENING).
- `REQ-MC-1104` The diagnostic snapshot shall be retrievable via `mc_get_diag` API at any time, regardless of whether the instance is enabled.
- `REQ-MC-1105` The sensorless observer shall set `observer_valid` to `MC_FALSE` and clear PLL lock state when BEMF magnitude falls below `min_bemf`.
- `REQ-MC-1106` The library shall ensure `inst->initialized == MC_FALSE` after a failed `mc_init` call.

### 12 — Calibration

- `REQ-MC-1201` The library shall provide `mc_adc_calibrate_linear`, a two-point linear calibration function accepting two (raw, physical) pairs and outputting scale and offset.
- `REQ-MC-1202` Two-point calibration shall reject the invalid input case where `raw_low == raw_high`.
- `REQ-MC-1203` ADC calibration results shall be configurable into each current/voltage reconstruction chain.
