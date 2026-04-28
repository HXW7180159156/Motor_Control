# FMEA / 失效模式与影响分析

## 分层索引 / Layer Index

- `FMEA-MC-01xx` — 平台层 Platform (PWM, ADC, Fault, Timer)
- `FMEA-MC-02xx` — 数学层 Math (clamp, wrap, LPF, Q31)
- `FMEA-MC-03xx` — 控制层 Control (Clarke/Park, PI, SVPWM, Speed Loop)
- `FMEA-MC-04xx` — 估算层 Estimator (Hall, Encoder, Resolver, Sensorless)
- `FMEA-MC-05xx` — 重构层 Reconstruct (1/2/3-shunt)
- `FMEA-MC-06xx` — 驱动层 Drive (PMSM FOC, BLDC Hall, BLDC Sensorless)
- `FMEA-MC-07xx` — API 层 API (Lifecycle, Mode, Identify, Diag)

---

## 中文

### 平台层 / Platform

- `FMEA-MC-0101` PWM 输出未更新（平台 `pwm_apply` 钩子失效）→ 电机持续以旧占空比运行，可能导致过流或失去控制。
- `FMEA-MC-0102` ADC 采样值卡滞（传感器断线或硬件故障）→ 三相电流重构为恒值，FOC 闭环失控，电机可能骤停或超速。
- `FMEA-MC-0103` 故障信号处理延迟（`fault_signal` 未及时调用）→ 过流/过压故障未及时传播至控制层，保护逻辑失效。
- `FMEA-MC-0104` 定时器回绕或溢出（`get_time_us` 返回异常值）→ 速度估算出现跳变，Hall/Encoder 转速突变。

### 数学层 / Math

- `FMEA-MC-0201` `mc_math_clamp_f32` 输入 `min > max` → 钳位逻辑反转，输出值可能超出预期范围。
- `FMEA-MC-0202` Q31 乘法溢出未饱和（`mc_q31_mul` 缺少溢出检查）→ INT32_MAX 回绕到负值，控制量反转。
- `FMEA-MC-0203` `mc_math_wrap_angle_rad` 在大角度输入时循环次数过多 → WCET 不可预测，可能违反实时性约束。
- `FMEA-MC-0204` LPF 系数 alpha 超出 [0, 1] 范围 → 滤波器发散或负反馈，BEMF 估计震荡。

### 控制层 / Control

- `FMEA-MC-0301` PI 积分饱和（windup）后长时间不恢复 → 速度超调/震荡，电机可能失步。
- `FMEA-MC-0302` SVPWM 扇区检测错误（电压矢量在扇区边界附近抖动）→ 占空比跳变，电流谐波增大。
- `FMEA-MC-0303` SVPWM 调制深度超限未裁剪 → 占空比 > 1.0 或 < 0.0，PWM 寄存器写入异常值。
- `FMEA-MC-0304` Park 变换 sin/cos 传入未归一化值 → dq 电流计算错误，FOC 解耦失效。
- `FMEA-MC-0305` 速度 PI 在低速/零速时积分累积 → 启动时给定过大转矩指令，可能过流。

### 估算层 / Estimator

- `FMEA-MC-0401` Hall 传感器编码不在合法序列中 → `mc_hall_find_index` 失败，角度/速度不再更新，保持 stale 值。
- `FMEA-MC-0402` Encoder 计数溢出（Z 相缺失或计数回绕）→ 机械角度跳变，速度尖峰。
- `FMEA-MC-0403` Resolver 信号幅度过低（`signal_amplitude < min_signal_amplitude`）→ 角度计算噪声大，FOC 抖动。
- `FMEA-MC-0404` Sensorless BEMF 幅度不足时仍计算角度 → 随机角度估计，电机失步。
- `FMEA-MC-0405` Sensorless PLL 相位锁定后 BEMF 突降 → 如果 unlock 检测不及时，角度累积误差持续增大。
- `FMEA-MC-0406` Sensorless `dt_s` 过大导致 `di/dt` 计算精度不足 → 电感补偿项偏差大，BEMF 估计偏移。

### 重构层 / Reconstruct

- `FMEA-MC-0501` 单电阻采样窗口失配 → 电流估计失真。（`FMEA-MC-0001`）
- `FMEA-MC-0502` 1-shunt 预测补偿在高速时模型失配 → 补偿电流与实际电流偏差增大，FOC 控制精度下降。
- `FMEA-MC-0503` 2-shunt 重构第三相计算因浮点舍入误差导致 `ia+ib+ic ≠ 0` → 零序电流分量引入控制回路。
- `FMEA-MC-0504` 3-shunt 单相 ADC 通道故障（offset 漂移）→ 该相电流估计产生直流偏置。

### 驱动层 / Drive

- `FMEA-MC-0601` PMSM FOC 弱磁控制器 PI 积分无限负向漂移 → d 轴电流过度负向，可能退磁永磁体。
- `FMEA-MC-0602` PMSM FOC 母线电压过低时未钳位 → 电压归一化输出超 1.0，SVPWM 饱和。
- `FMEA-MC-0603` BLDC Hall 换相扇区映射错误 → 电机反转或扭矩为零。
- `FMEA-MC-0604` BLDC sensorless 零交叉检测在重载时 BEMF 波形畸变 → 过零检测延迟，换相滞后。
- `FMEA-MC-0605` BLDC sensorless 启动 RAMP_OPEN 在阻力大时失步 → 电机无法进入闭环，持续开环震荡。

### API 层 / API

- `FMEA-MC-0701` `mc_init` 失败后 `initialized` 状态未正确清除 → 后续调用可能操作未初始化的传感器/PI 状态。
- `FMEA-MC-0702` 模式切换时未完全重置旧模式运行时数据 → 传感器 stale 值被新控制模式使用。
- `FMEA-MC-0703` `mc_start_identification` 在实例 disabled 时被调用 → 辨识在无 PWM 输出状态下空转，无法完成。
- `FMEA-MC-0704` 辨识 trust gate 未能有效过滤非物理值 → 错误的 `Rs`/`Ld`/`Lq` 写回运行时模型，FOC 控制异常。
- `FMEA-MC-0705` `mc_fast_step` 在 disabled 状态下未将 output 清零 → 上位机读到 stale PWM 状态作决策。

---

## English

### Platform Layer

- `FMEA-MC-0101` PWM output not updated (platform `pwm_apply` hook failure) → Motor continues running at old duty cycle, potentially causing overcurrent or loss of control.
- `FMEA-MC-0102` ADC sample stuck (sensor disconnection or hardware fault) → Three-phase current reconstruction yields constant values, FOC closed-loop becomes uncontrolled, motor may stall or over-speed.
- `FMEA-MC-0103` Fault signal handling delayed (`fault_signal` not called promptly) → Overcurrent/overvoltage faults not propagated to control layer, protection logic disabled.
- `FMEA-MC-0104` Timer wraparound or overflow (`get_time_us` returns abnormal value) → Speed estimation glitch, Hall/Encoder speed spikes.

### Math Layer

- `FMEA-MC-0201` `mc_math_clamp_f32` receives `min > max` → Clamp logic inverted, output may exceed expected bounds.
- `FMEA-MC-0202` Q31 multiplication overflow not saturated → INT32_MAX wraps to negative, control quantity sign inverted.
- `FMEA-MC-0203` `mc_math_wrap_angle_rad` loops excessively on large inputs → Unbounded WCET, violating real-time constraints.
- `FMEA-MC-0204` LPF coefficient alpha outside [0, 1] → Filter diverges or exhibits negative feedback, BEMF estimate oscillates.

### Control Layer

- `FMEA-MC-0301` PI integrator windup not recovered promptly → Speed overshoot/oscillation, potential loss of synchronism.
- `FMEA-MC-0302` SVPWM sector detection error (voltage vector dithering near sector boundary) → Duty cycle glitches, increased current harmonics.
- `FMEA-MC-0303` SVPWM modulation depth exceeded without clamping → Duty > 1.0 or < 0.0, abnormal PWM register values.
- `FMEA-MC-0304` Park transform receives unnormalized sin/cos → dq current calculation error, FOC decoupling invalid.
- `FMEA-MC-0305` Speed PI integrator accumulates at low/zero speed → Excessive torque command at startup, potential overcurrent.

### Estimator Layer

- `FMEA-MC-0401` Hall sensor code not in legal sequence → `mc_hall_find_index` fails, angle/speed not updated, stale values retained.
- `FMEA-MC-0402` Encoder count overflow (Z-phase missing or count wraparound) → Mechanical angle jump, speed spike.
- `FMEA-MC-0403` Resolver signal amplitude too low (`signal_amplitude < min_signal_amplitude`) → Noisy angle computation, FOC jitter.
- `FMEA-MC-0404` Sensorless BEMF magnitude insufficient but angle still computed → Random angle estimate, loss of synchronism.
- `FMEA-MC-0405` Sensorless PLL locked then BEMF drops abruptly → If unlock detection is delayed, cumulative angle error grows continuously.
- `FMEA-MC-0406` Sensorless `dt_s` too large causing `di/dt` accuracy loss → Inductance compensation deviates, BEMF estimate biased.

### Reconstruct Layer

- `FMEA-MC-0501` 1-shunt sampling window mismatch → Current estimation distortion. (was `FMEA-MC-0001`)
- `FMEA-MC-0502` 1-shunt predictive compensation model mismatch at high speed → Compensated vs actual current deviation grows, FOC accuracy degrades.
- `FMEA-MC-0503` 2-shunt third phase calculation floating-point rounding causes `ia+ib+ic ≠ 0` → Zero-sequence current introduced into control loop.
- `FMEA-MC-0504` 3-shunt single ADC channel fault (offset drift) → DC bias in that phase's current estimate.

### Drive Layer

- `FMEA-MC-0601` PMSM FOC field-weakening PI integrator drifts negatively without bound → Excessive negative d-axis current, potential permanent magnet demagnetization.
- `FMEA-MC-0602` PMSM FOC bus voltage below minimum not clamped → Voltage normalization output exceeds 1.0, SVPWM saturates.
- `FMEA-MC-0603` BLDC Hall commutation sector mapping error → Motor reverses or produces zero torque.
- `FMEA-MC-0604` BLDC sensorless zero-crossing detection under heavy load BEMF waveform distortion → ZC detection delayed, commutation lagging.
- `FMEA-MC-0605` BLDC sensorless RAMP_OPEN loses synchronism under high load → Motor unable to enter closed-loop, sustained open-loop oscillation.

### API Layer

- `FMEA-MC-0701` `mc_init` failure does not correctly clear `initialized` state → Subsequent calls may operate on uninitialized sensor/PI state.
- `FMEA-MC-0702` Mode switch does not fully reset old mode runtime data → Stale sensor values consumed by new control mode.
- `FMEA-MC-0703` `mc_start_identification` called while instance is disabled → Identification idles without PWM output, cannot complete.
- `FMEA-MC-0704` Identify trust gate fails to filter non-physical values → Erroneous `Rs`/`Ld`/`Lq` written back to runtime model, FOC abnormal.
- `FMEA-MC-0705` `mc_fast_step` does not zero output when disabled → Host reads stale PWM state for decision-making.
