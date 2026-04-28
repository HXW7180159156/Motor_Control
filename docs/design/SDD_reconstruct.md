# Software Detailed Design - Reconstruct / 软件详细设计 - 重构层

## 中文

### 概述

电流重构层负责将 ADC 原始值转换为以安培为单位的三相电流。三种电流检测拓扑共享统一接口模式：`mc_reconstruct_Nshunt_run(raw, cfg, [meta], &i_abc)`。

### 3-shunt 重构

每个 ADC 通道独立标定后直接输出三相电流。

伪代码：
```
RECONSTRUCT_3SHUNT(raw, cfg, i_abc):
    i_abc.a = (raw.phase_a - cfg.offset_a) * cfg.scale_a
    i_abc.b = (raw.phase_b - cfg.offset_b) * cfg.scale_b
    i_abc.c = (raw.phase_c - cfg.offset_c) * cfg.scale_c
```

### 2-shunt 重构

A/B 相直接标定，C 相由基尔霍夫电流定律推导。

伪代码：
```
RECONSTRUCT_2SHUNT(raw, cfg, i_abc):
    ia = (raw.phase_a - cfg.offset_a) * cfg.scale_a
    ib = (raw.phase_b - cfg.offset_b) * cfg.scale_b
    i_abc.a = ia
    i_abc.b = ib
    i_abc.c = -(ia + ib)
```

### 1-shunt 重构

1-shunt 是最复杂的重构链路，分为两步：

**Step 1: 采样窗口规划 (`mc_reconstruct_1shunt_plan`)**

```
PLAN_1SHUNT(pwm_cmd, cfg, meta):
    meta.sector = pwm_cmd.sector
    根据扇区确定 duty_x, duty_y (两个活跃相的占空比)
    threshold = cfg.min_active_duty + cfg.sample_window_margin

    // 判断窗口有效性
    active_duty_min = min(duty_x, duty_y)
    meta.sample_valid = (active_duty_min > threshold)
    meta.sample_count = (duty_x > threshold) + (duty_y > threshold)
    meta.compensation_required = (meta.sample_count < 2)
    meta.reorder_required = meta.compensation_required

    // 零点矢量偏置判断
    meta.zero_vector_bias_high = (duty_a + duty_b + duty_c < 1.5)
    meta.preferred_window = (duty_x >= duty_y) ? 1 : 2

    // 采样相位与位置
    对每个有效窗口：
      采样相位 = (duty >= 0.5) ? TRAILING : LEADING
      采样位置 = LEADING ? duty_low : (duty - margin_half)

    // 相位重排（窗口不足时）
    if reorder_required:
      将两路采样都排列在低零矢量侧或高零矢量侧
      确保两路采样在同一个半周期内

    // 最终化半周期/时间元数据
    FINALIZE_SAMPLE(phase, pwm_period, position, ...)
```

**Step 2: 电流重构 (`mc_reconstruct_1shunt_run`)**

```
RECONSTRUCT_1SHUNT(raw, cfg, meta, i_abc):
    i_bus = (raw.phase_a - cfg.offset) * cfg.scale
    i_abc = {0, 0, 0}

    if not meta.sample_valid: return  // 窗口无效时返回零

    根据 meta.sector 映射 i_bus 到两相：
      sector=1: i_abc.a = i_bus,  i_abc.b = -i_bus
      sector=2: i_abc.a = i_bus,  i_abc.c = -i_bus
      sector=3: i_abc.b = i_bus,  i_abc.c = -i_bus
      sector=4: i_abc.b = i_bus,  i_abc.a = -i_bus
      sector=5: i_abc.c = i_bus,  i_abc.a = -i_bus
      sector=6: i_abc.c = i_bus,  i_abc.b = -i_bus
```

### 1-shunt 预测补偿

当采样窗口不足（只有一个有效窗口）时，驱动层调用预测补偿：

```
PREDICT_1SHUNT(foc, in, i_abc, comp_status):
    预测权重 = 1.0, delta_limit = 0.5

    if modulation_index > 0.8 * voltage_limit:
        预测权重 = 0.6, delta_limit = 0.25  // 高调制模式
    if id_ref < -0.1:
        预测权重 *= 0.7, delta_limit = 0.15  // 弱磁模式

    // PMSM dq 模型预测
    delta_id = (vd - Rs*id + ωe*Lq*iq) / Ld * dt
    delta_iq = (vq - Rs*iq - ωe*(Ld*id + flux)) / Lq * dt

    delta_id = CLAMP(delta_id * 预测权重, ±delta_limit)
    delta_iq = CLAMP(delta_iq * 预测权重, ±delta_limit)

    predicted_id = id + delta_id
    predicted_iq = iq + delta_iq
    InvPark → i_abc  // dq 预测值反变换到三相

    comp_status = {active=TRUE, mode=当前模式}
```

## English

### Overview

The current reconstruction layer converts raw ADC values to three-phase currents in amperes. All three current sensing topologies share a unified interface: `mc_reconstruct_Nshunt_run(raw, cfg, [meta], &i_abc)`.

### 3-Shunt Reconstruction

Each ADC channel is independently calibrated and directly outputs three-phase current.

Pseudo-code:
```
RECONSTRUCT_3SHUNT(raw, cfg, i_abc):
    i_abc.a = (raw.phase_a - cfg.offset_a) * cfg.scale_a
    i_abc.b = (raw.phase_b - cfg.offset_b) * cfg.scale_b
    i_abc.c = (raw.phase_c - cfg.offset_c) * cfg.scale_c
```

### 2-Shunt Reconstruction

Phases A/B are calibrated directly; phase C is derived from Kirchhoff's current law.

Pseudo-code:
```
RECONSTRUCT_2SHUNT(raw, cfg, i_abc):
    ia = (raw.phase_a - cfg.offset_a) * cfg.scale_a
    ib = (raw.phase_b - cfg.offset_b) * cfg.scale_b
    i_abc.a = ia
    i_abc.b = ib
    i_abc.c = -(ia + ib)
```

### 1-Shunt Reconstruction

Single-shunt is the most complex reconstruction chain, executed in two steps:

**Step 1: Sampling Window Planning (`mc_reconstruct_1shunt_plan`)**

```
PLAN_1SHUNT(pwm_cmd, cfg, meta):
    meta.sector = pwm_cmd.sector
    Determine duty_x, duty_y per sector (duties of the two active phases)
    threshold = cfg.min_active_duty + cfg.sample_window_margin

    // Window validity
    active_duty_min = min(duty_x, duty_y)
    meta.sample_valid = (active_duty_min > threshold)
    meta.sample_count = (duty_x > threshold) + (duty_y > threshold)
    meta.compensation_required = (meta.sample_count < 2)
    meta.reorder_required = meta.compensation_required

    // Zero-vector bias
    meta.zero_vector_bias_high = (duty_a + duty_b + duty_c < 1.5)
    meta.preferred_window = (duty_x >= duty_y) ? 1 : 2

    // Sample phase & position
    For each valid window:
      sample_phase = (duty >= 0.5) ? TRAILING : LEADING
      sample_position = LEADING ? duty_low : (duty - margin_half)

    // Phase reorder (when insufficient windows)
    if reorder_required:
      Arrange both samples on the low or high zero-vector side
      Ensure both samples fall within the same half-cycle

    // Finalize half-cycle / timing metadata
    FINALIZE_SAMPLE(phase, pwm_period, position, ...)
```

**Step 2: Current Reconstruction (`mc_reconstruct_1shunt_run`)**

```
RECONSTRUCT_1SHUNT(raw, cfg, meta, i_abc):
    i_bus = (raw.phase_a - cfg.offset) * cfg.scale
    i_abc = {0, 0, 0}

    if not meta.sample_valid: return

    Map i_bus to two phases per sector:
      sector=1: i_abc.a = i_bus,  i_abc.b = -i_bus
      sector=2: i_abc.a = i_bus,  i_abc.c = -i_bus
      ...
```

### 1-Shunt Predictive Compensation

When the sampling window is insufficient (only one valid window), the drive layer invokes predictive compensation:

```
PREDICT_1SHUNT(foc, in, i_abc, comp_status):
    prediction_weight = 1.0, delta_limit = 0.5

    if modulation_index > 0.8 * voltage_limit:
        prediction_weight = 0.6, delta_limit = 0.25  // High modulation
    if id_ref < -0.1:
        prediction_weight *= 0.7, delta_limit = 0.15  // Field weakening

    // PMSM dq model prediction
    delta_id = (vd - Rs*id + ωe*Lq*iq) / Ld * dt
    delta_iq = (vq - Rs*iq - ωe*(Ld*id + flux)) / Lq * dt

    delta_id = CLAMP(delta_id * prediction_weight, ±delta_limit)
    delta_iq = CLAMP(delta_iq * prediction_weight, ±delta_limit)

    predicted_id = id + delta_id
    predicted_iq = iq + delta_iq
    InvPark → i_abc  // Inverse transform dq prediction to three-phase

    comp_status = {active=TRUE, mode=current mode}
```
