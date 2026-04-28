# Motor Control Library — User Guide / 使用指南

**Version**: 0.1.0

---

## 目录

1. [Quick Start / 快速上手](#1-quick-start)
2. [Use Case: PMSM FOC with Encoder / PMSM FOC + 编码器](#2-pmsm-foc-encoder)
3. [Use Case: PMSM FOC Sensorless (SMO) / PMSM FOC 无感 (SMO)](#3-pmsm-foc-sensorless-smo)
4. [Use Case: BLDC Hall / BLDC Hall 六步换相](#4-bldc-hall)
5. [Use Case: Motor Parameter Identification / 电机参数辨识](#5-identify)
6. [Use Case: Auto-Tune PI from Identified Params / 辨识后自动调谐](#6-auto-tune)
7. [Tuning Guide / 调优指南](#7-tuning-guide)
8. [Troubleshooting / 故障排查](#8-troubleshooting)
9. [Checklist / 集成检查清单](#9-checklist)

---

## 1. Quick Start / 快速上手

### 1.1 Minimal PMSM FOC Setup / 最小化 PMSM FOC 配置

```c
#include "mc_api.h"

static mc_instance_t g_mc;

/* --- 平台钩子实现 --- */
static uint32_t my_get_time_us(void) {
    return SysTick->VAL;  /* 替换为你的硬件定时器 */
}

static void my_pwm_apply(const void *cmd) {
    const mc_pwm_cmd_t *pwm = (const mc_pwm_cmd_t *)cmd;
    TIM1->CCR1 = (uint32_t)(pwm->duty_a * TIM1->ARR);  /* 替换为你的 PWM 外设 */
    TIM1->CCR2 = (uint32_t)(pwm->duty_b * TIM1->ARR);
    TIM1->CCR3 = (uint32_t)(pwm->duty_c * TIM1->ARR);
}

void motor_control_init(void) {
    mc_system_cfg_t cfg = {0};

    /* 1. 电机参数 */
    cfg.motor.pole_pairs = 4;
    cfg.motor.rs_ohm = 0.5f;
    cfg.motor.ld_h = 0.001f;
    cfg.motor.lq_h = 0.0012f;
    cfg.motor.flux_wb = 0.01f;

    /* 2. 硬件限制 */
    cfg.limits.bus_voltage_max_v = 24.0f;
    cfg.limits.phase_current_max_a = 10.0f;
    cfg.limits.speed_max_rpm = 5000.0f;

    /* 3. 控制环参数 */
    cfg.control.pwm_frequency_hz = 20000;
    cfg.control.numeric_mode = MC_NUMERIC_FLOAT32;
    cfg.control.current_loop_dt_s = 0.00005f;  /* 50µs */

    /* 4. FOC 配置 */
    cfg.foc.svpwm_cfg.modulation_limit = 0.95f;
    cfg.foc.svpwm_cfg.duty_min = 0.0f;
    cfg.foc.svpwm_cfg.duty_max = 1.0f;
    cfg.foc.current_cfg.type = MC_CURRENT_SENSE_2SHUNT;
    cfg.foc.current_cfg.cfg.shunt2.offset_a = 0.0f;
    cfg.foc.current_cfg.cfg.shunt2.scale_a = 1.0f;
    cfg.foc.current_cfg.cfg.shunt2.offset_b = 0.0f;
    cfg.foc.current_cfg.cfg.shunt2.scale_b = 1.0f;
    cfg.foc.speed_loop_dt_s = 0.001f;   /* 1ms */
    cfg.foc.iq_limit = 5.0f;
    cfg.foc.voltage_limit = 24.0f;
    cfg.foc.bus_voltage_scale = 1.0f;
    cfg.foc.bus_voltage_min = 5.0f;
    cfg.foc.mtpa_enable = MC_FALSE;
    cfg.foc.fw_enable = MC_FALSE;

    /* 5. 电流环 PI（先用保守值） */
    cfg.foc.id_pi_cfg = (mc_pi_cfg_t){0.5f, 50.0f, -0.5f, 0.5f, -1.0f, 1.0f};
    cfg.foc.iq_pi_cfg = (mc_pi_cfg_t){0.5f, 50.0f, -0.5f, 0.5f, -1.0f, 1.0f};
    cfg.foc.speed_pi_cfg = (mc_pi_cfg_t){0.05f, 2.0f, -0.5f, 0.5f, -5.0f, 5.0f};

    /* 6. 传感器 */
    cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER;
    cfg.sensor.encoder_cfg.counts_per_rev = 4096;
    cfg.sensor.encoder_cfg.pole_pairs = 4.0f;

    /* 7. 平台钩子 */
    cfg.hooks.pwm_apply = my_pwm_apply;
    cfg.hooks.adc_trigger_apply = NULL;
    cfg.hooks.fault_signal = NULL;
    cfg.hooks.get_time_us = my_get_time_us;

    /* 8. 初始化 */
    mc_status_t s = mc_init(&g_mc, &cfg);
    if (s != MC_STATUS_OK) {
        /* 错误处理 */
        return;
    }
}

void motor_control_start(void) {
    mc_start(&g_mc);
    mc_set_speed_ref(&g_mc, 1000.0f);   /* 目标 1000 RPM */
}

/* PWM 中断中调用 */
void pwm_interrupt_handler(void) {
    mc_fast_input_t in;
    mc_fast_output_t out;
    read_adc_values(&in.adc_raw);        /* 读取 ADC */
    in.timestamp_us = my_get_time_us();
    mc_fast_step(&g_mc, &in, &out);       /* 执行快速环 */
    my_pwm_apply(&out.pwm_cmd);           /* 更新 PWM */
}

/* 1ms 定时中断中调用 */
void timer_1ms_handler(void) {
    mc_medium_input_t in;
    mc_medium_output_t out;
    in.speed_feedback_rpm = get_encoder_speed();   /* 读取编码器速度 */
    in.encoder_count = get_encoder_count();
    in.timestamp_us = my_get_time_us();
    mc_medium_step(&g_mc, &in, &out);
}

/* 10ms 主循环中调用 */
void main_monitor_loop(void) {
    mc_slow_input_t in;
    mc_slow_output_t out;
    in.temperature_deg_c = read_temperature();
    in.clear_fault_request = MC_FALSE;
    in.timestamp_us = my_get_time_us();
    mc_slow_step(&g_mc, &in, &out);

    if (out.diag.active_fault != MC_FAULT_NONE) {
        emergency_stop();
    }
}
```

---

## 2. Use Case: PMSM FOC with Encoder / PMSM FOC + 编码器

### 2.1 Configuration Steps / 配置步骤

1. 设置 `cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_ENCODER`
2. 配置编码器参数：`counts_per_rev`（每周计数）、`pole_pairs`
3. 在 `mc_medium_input_t` 中每周期传入 `encoder_count` 和 `speed_feedback_rpm`
4. 编码器速度由库内部自动计算（`mc_encoder_update`）

### 2.2 Speed Control vs Torque Control / 速度控制 vs 转矩控制

```c
/* 速度模式 */
mc_set_speed_ref(&g_mc, 1500.0f);   // 启动速度闭环

/* 转矩模式 */
mc_set_torque_ref(&g_mc, 2.0f);    // 切换到转矩控制

/* 直接电流模式 */
mc_set_current_ref_dq(&g_mc, 0.0f, 3.0f);  // Id=0, Iq=3A
```

### 2.3 Enabling Field Weakening / 启用弱磁

```c
cfg.foc.fw_enable = MC_TRUE;
cfg.foc.fw_kp = 1.0f;
cfg.foc.fw_ki = 10.0f;
cfg.foc.fw_min_id = -2.0f;                  /* 最大允许负向 Id */
cfg.foc.fw_activation_ratio = 0.9f;          /* 电压利用率 > 90% 时激活 */
```

**Note**: 弱磁仅在电压饱和时激活。对于 SPM 电机 (`Ld ≈ Lq`)，弱磁效果有限；对于 IPM 电机 (`Ld < Lq`)，弱磁效果显著。

---

## 3. Use Case: PMSM FOC Sensorless (SMO) / PMSM FOC 无感 (SMO)

### 3.1 Configuration / 配置

```c
cfg.sensor.primary_mode = MC_MODE_PMSM_FOC_SMO;

/* SMO 复用 sensorless_cfg 参数入口 */
cfg.sensor.sensorless_cfg.bemf_filter_alpha = 0.5f;   /* SMO LPF alpha */
cfg.sensor.sensorless_cfg.min_bemf = 0.01f;
cfg.sensor.sensorless_cfg.pll_kp = 400.0f;
cfg.sensor.sensorless_cfg.pll_ki = 20000.0f;
cfg.sensor.sensorless_cfg.lock_bemf = 0.02f;           /* 必须 > min_bemf */
cfg.sensor.sensorless_cfg.startup_speed_rad_s = 80.0f;
cfg.sensor.sensorless_cfg.startup_accel_rad_s2 = 4000.0f;
cfg.sensor.sensorless_cfg.open_loop_voltage_max = 0.15f * cfg.foc.voltage_limit;
cfg.sensor.sensorless_cfg.open_loop_voltage_start = 0.0f;
```

**Important**: 未填入的字段（≤0）会自动使用保守默认值。

### 3.2 Startup Behavior / 启动行为

SMO 启动分为三个阶段：

1. **开环加速** — 电压/频率 (V/f) 斜坡从 0 到 `startup_speed_rad_s`，电压从 `open_loop_voltage_start` 到 `open_loop_voltage_max`
2. **切换判断** — 连续 2 个周期同时满足：BEMF ≥ `lock_bemf` 且角度误差 < 0.5 rad
3. **闭环锁定** — PLL 接管角度跟踪，连续 2 个周期角度误差 < 0.35 rad 后标记 `pll_locked`

### 3.3 SMO vs LPF+PLL Selection / 选择指南

| 场景 | 推荐 |
|---|---|
| M0+ / 低算力 MCU、原型验证 | LPF+PLL (`MC_MODE_PMSM_FOC_SENSORLESS`) |
| M4F/M7 量产、中高速、噪声环境 | SMO (`MC_MODE_PMSM_FOC_SMO`) |
| 低速重载启动 | 两者都需要手动优化开环参数 |

---

## 4. Use Case: BLDC Hall / BLDC Hall 六步换相

### 4.1 Configuration / 配置

```c
cfg.sensor.primary_mode = MC_MODE_BLDC_HALL;

/* Hall 换相表 — 6 个 Hall 编码对应 6 个扇区 */
cfg.sensor.hall_cfg.hall_code_sequence[0] = 1;   /* Hall=1 → 扇区1 */
cfg.sensor.hall_cfg.hall_code_sequence[1] = 5;   /* Hall=5 → 扇区2 */
cfg.sensor.hall_cfg.hall_code_sequence[2] = 4;   /* Hall=4 → 扇区3 */
cfg.sensor.hall_cfg.hall_code_sequence[3] = 6;   /* Hall=6 → 扇区4 */
cfg.sensor.hall_cfg.hall_code_sequence[4] = 2;   /* Hall=2 → 扇区5 */
cfg.sensor.hall_cfg.hall_code_sequence[5] = 3;   /* Hall=3 → 扇区6 */

/* 每个 Hall 状态对应的电气角度 [rad] */
cfg.sensor.hall_cfg.elec_angle_table_rad[0] = 0.0f;        /* 0° */
cfg.sensor.hall_cfg.elec_angle_table_rad[1] = 1.047f;      /* 60° */
cfg.sensor.hall_cfg.elec_angle_table_rad[2] = 2.094f;      /* 120° */
cfg.sensor.hall_cfg.elec_angle_table_rad[3] = 3.142f;      /* 180° */
cfg.sensor.hall_cfg.elec_angle_table_rad[4] = 4.189f;      /* 240° */
cfg.sensor.hall_cfg.elec_angle_table_rad[5] = 5.236f;      /* 300° */
cfg.sensor.pole_pairs = 2.0f;
```

### 4.2 Hall Code Input / Hall 编码输入

```c
void hall_isr(void) {
    mc_fast_input_t in;
    mc_fast_output_t out;

    in.hall_code = read_hall_gpio();     /* 读取 3 路 Hall GPIO → 1-6 */
    in.timestamp_us = my_get_time_us();

    mc_fast_step(&g_mc, &in, &out);
    my_pwm_apply(&out.pwm_cmd);
}
```

---

## 5. Use Case: Motor Parameter Identification / 电机参数辨识

### 5.1 Full Identify Sequence / 完整辨识流程

```c
/* 步骤 1: 初始化 */
mc_init(&g_mc, &cfg);

/* 步骤 2: 启动辨识（异步） */
mc_start_identification(&g_mc);

/* 步骤 3: 循环执行 fast_step 直到完成 */
while (!mc_is_identification_done(&g_mc)) {
    mc_fast_input_t in;
    mc_fast_output_t out;
    read_adc_values(&in.adc_raw);
    in.timestamp_us = my_get_time_us();
    mc_fast_step(&g_mc, &in, &out);
    my_pwm_apply(&out.pwm_cmd);
}

/* 步骤 4: 获取结果 */
mc_motor_params_t params;
mc_get_identified_params(&g_mc, &params);
printf("Rs=%.4fΩ  Ld=%.6fH  Lq=%.6fH  flux=%.4fWb\n",
       params.rs_ohm, params.ld_h, params.lq_h, params.flux_wb);
```

### 5.2 Expected Results / 预期结果

| Parameter | Typical Range | Trust Gate |
|---|---|---|
| Rs | 0.01 ~ 10 Ω | > 0 且 isfinite |
| Ld | 0.0001 ~ 0.1 H | > 0 且 isfinite |
| Lq | 0.0001 ~ 0.1 H | > 0 且 isfinite |
| flux_wb | 0.001 ~ 0.5 Wb | CV ≤ 30%，≥ 10 个有效样本，时间窗口 ≥ 75% 脉冲时长 |

### 5.3 Required Conditions / 必要条件

- 电机转子自由旋转（辨识过程中有轻微振动，不会持续转动）
- 母线电压稳定
- 电流检测必须已标定（scale/offset 正确）
- `mc_fast_step` 以固定频率调用

---

## 6. Use Case: Auto-Tune PI from Identified Params / 辨识后自动调谐

### 6.1 Workflow / 工作流

```c
/* 1. 先辨识电机参数 */
mc_start_identification(&g_mc);
while (!mc_is_identification_done(&g_mc)) {
    mc_fast_step(&g_mc, &in, &out);
}

/* 2. 自动调谐 PI */
mc_auto_tune_pi(&g_mc, 20.0f, 10.0f);
/*  bandwidth_divider=20 → 电流环带宽 = 2π*20kHz/20 ≈ 6283 rad/s ≈ 1kHz
 *  speed_ratio=10 → 速度环带宽 = 1kHz/10 = 100Hz
 */

/* 3. 验证 PI 增益 */
printf("Id PI: Kp=%.2f Ki=%.2f\n",
       g_mc.cfg.foc.id_pi_cfg.kp, g_mc.cfg.foc.id_pi_cfg.ki);
printf("Iq PI: Kp=%.2f Ki=%.2f\n",
       g_mc.cfg.foc.iq_pi_cfg.kp, g_mc.cfg.foc.iq_pi_cfg.ki);
printf("Speed PI: Kp=%.4f Ki=%.4f\n",
       g_mc.cfg.foc.speed_pi_cfg.kp, g_mc.cfg.foc.speed_pi_cfg.ki);
```

### 6.2 Bandwidth Divider Guidelines / 带宽分频器选择

| divider | Current Loop BW | Speed Loop BW | Characteristics |
|---|---|---|---|
| 10 | ~2 kHz | ~200 Hz | 激进 — 快速响应，噪声敏感 |
| 20 | ~1 kHz | ~100 Hz | 平衡 — 推荐大多数场景 |
| 40 | ~500 Hz | ~50 Hz | 保守 — 慢响应，噪声鲁棒 |

### 6.3 Manual Fine-Tuning After Auto-Tune / 自动调谐后手动微调

```c
/* 增加 Kp → 更快响应，但可能超调 */
g_mc.cfg.foc.iq_pi_cfg.kp *= 1.5f;

/* 增加 Ki → 更快消除静差，但可能震荡 */
g_mc.cfg.foc.speed_pi_cfg.ki *= 2.0f;

/* 重新应用手动调整后的参数 */
mc_control_foc_set_speed_pi(&g_mc.foc, &g_mc.cfg.foc.speed_pi_cfg, g_mc.cfg.foc.iq_limit);
```

---

## 7. Tuning Guide / 调优指南

### 7.1 Sensorless Observer Tuning / 无感观测器调优

#### 启动参数

| 参数 | 太小 | 太大 |
|---|---|---|
| `startup_speed_rad_s` | 开环阶段过长，BEMF 不足无法锁定 | 启动电流大，切换冲击大 |
| `startup_accel_rad_s2` | 启动慢 | 电流冲击，可能过流 |
| `open_loop_voltage_max` | BEMF 不足 | 电流过大 |

**经验法则**:
- `startup_speed_rad_s` ≈ 额定速度的 10~20%
- `open_loop_voltage_max` ≈ `voltage_limit * 0.1~0.2`
- `lock_bemf` ≈ `min_bemf * 2~5`

#### PLL 参数

| 参数 | 推荐范围 | 说明 |
|---|---|---|
| `pll_kp` | 100 ~ 800 | 过小：慢收敛；过大：噪声放大 |
| `pll_ki` | 5000 ~ 50000 | 过小：静差大；过大：超调 |
| `bemf_filter_alpha` | 0.2 ~ 0.7 | 过小：滞后大；过大：噪声大 |

**经验公式** (LPF+PLL):
```
pll_kp = 2 * ω_n     (ω_n = 期望 PLL 带宽，通常 200~400 rad/s)
pll_ki = ω_n²
```

### 7.2 Current Loop PI Tuning / 电流环 PI 调优

**Step 1**: 使用 `mc_auto_tune_current_pi()` 获取初始值。

**Step 2**: 观察阶跃响应 — 如果振荡，减小 Kp 或 Ki；如果响应慢，增大 Kp。

**Step 3**: 设置积分限幅：
```
integral_limit = voltage_limit * 0.5 / Rs   (或不超过 current_limit)
output_limit   = ±voltage_limit
```

### 7.3 Speed Loop PI Tuning / 速度环 PI 调优

**Step 1**: 先用 auto-tune 获取初始值。

**Step 2**: 在电流环稳定工作后，逐步增加速度环增益。

**Step 3**: 观察速度阶跃响应：
- 一次超调 < 10%，然后稳定 → OK
- 持续振荡 → 减小 Kp/Ki
- 无超调但响应慢 → 增大 Kp

### 7.4 Dead-Time Compensation Tuning / 死区补偿调优

```c
/* 启用死区补偿 */
cfg.foc.dtc_enable = MC_TRUE;

/* 设置死区时间 [ns]（从逆变器数据手册获取） */
cfg.foc.dtc_deadtime_ns = 500.0f;   /* 典型值 200~2000 ns */
```

**效果检查**: 低速轻载时（< 10% 额定速度），电流波形应为正弦；若存在平台状失真（零交越钳位），增加 deadtime_ns 设定值。

---

## 8. Troubleshooting / 故障排查

### 8.1 `mc_init` Returns Error / `mc_init` 返回错误

| Cause | Solution |
|---|---|
| `inst == NULL` 或 `cfg == NULL` | 检查指针有效性 |
| `MC_STATUS_UNSUPPORTED` | Q31 路径仅支持 Hall/Encoder + 2/3-shunt；检查数值模式组合 |
| Sensorless/SMO cfg 不合法 | 检查 `min_bemf < lock_bemf`，`open_loop_voltage_max ≥ start` |
| Hall 序列未配置 | 填写完整的 6 个 `hall_code_sequence` |
| Encoder `counts_per_rev == 0` | 设置编码器线数 |
| Resolver `signal_scale ≤ 0` | 设置信号缩放因子 |

### 8.2 Motor Doesn't Start / 电机不启动

| Cause | Solution |
|---|---|
| 未调用 `mc_start()` | 在 init 后调用 |
| `inst->enabled == MC_FALSE` | 检查是否在其他地方调用了 `mc_stop()` |
| PWM 钩子未注册 | 设置 `cfg.hooks.pwm_apply` |
| Hall 编码不正确 | 用示波器验证 Hall 信号序列 |
| BLDC sensorless 配置不合法 | 检查 `ramp_start_freq_hz > 0` 且 `ramp_end_freq ≥ ramp_start_freq` |

### 8.3 Sensorless Observer Fails to Lock / 无感观测器无法锁定

| Cause | Solution |
|---|---|
| `open_loop_voltage_max` 太小 | 增大到 `0.2 * voltage_limit` |
| `startup_speed_rad_s` 太大 | 减小到额定速度的 15% |
| `lock_bemf` 太大 | 减小到 `min_bemf * 2` |
| `min_bemf` 太大 | 减小到 `0.005 * voltage_limit` |
| 负载太重 | 减小启动负载或增大 `open_loop_voltage_max` |
| `bemf_filter_alpha` 太小 | 增大到 0.5~0.7 |

**诊断**: 检查 `inst->sensorless.open_loop_active` 是否为 FALSE（已退出开环）。若始终为 TRUE，说明 BEMF 不足或角度误差太大。

### 8.4 Current Waveform Distorted / 电流波形失真

| Cause | Solution |
|---|---|
| 死区未补偿 | 启用 `dtc_enable`，设置正确的 `dtc_deadtime_ns` |
| ADC offset 错误 | 重新标定 `offset_a/b/c` |
| 1-shunt window invalid | 减小 `min_active_duty`，增大 PWM 频率 |
| PI Kp 太小 | 增大 Kp |

### 8.5 Motor Overheats / 电机过热

| Cause | Solution |
|---|---|
| Id 过大（非 MTPA 运行） | IPM 电机启用 MTPA (`mtpa_enable = MC_TRUE`) |
| 弱磁 Id 过负 | 检查 `fw_min_id` 设置 |
| 开关频率太低 | 增大 PWM 频率 |

### 8.6 Identification Produces 0.0 / 辨识结果全零

| Cause | Solution |
|---|---|
| 电流检测未标定 | 确保 `scale` 和 `offset` 使输出单位为安培 |
| `max_current_a` 太小 | 增大到额定电流的 50% |
| 母线电压太低 | 检查 `bus_voltage_min` |
| 电机被锁定/堵转 | 确保转子可自由转动 |

---

## 9. Checklist / 集成检查清单

### Phase 1: Initial Integration / 初步集成

- [ ] 实现 `pwm_apply` 平台钩子（将占空比写入定时器比较寄存器）
- [ ] 实现 `get_time_us` 平台钩子（µs 级时间戳）
- [ ] 配置 `mc_motor_params_t`（极对数，Rs, Ld, Lq, flux_wb）
- [ ] 配置 `mc_hw_limits_t`（电压/电流/温度/速度限制）
- [ ] 选择 `mc_mode_t` 运行模式
- [ ] 配置对应传感器参数
- [ ] 调用 `mc_init()` → 检查返回值 `MC_STATUS_OK`
- [ ] 调用 `mc_start()` → 在 ISR 中调用 `mc_fast_step`

### Phase 2: Basic Operation / 基本运行

- [ ] 电机能启动并能调速
- [ ] PWM 输出在 0~100% 范围内
- [ ] `mc_fast_step` 在 PWM 中断中每次调用一次
- [ ] `mc_medium_step` 以正确频率调用（通常 10x 低于 fast_step）
- [ ] 无过流故障触发

### Phase 3: Optimization / 优化

- [ ] 执行 `mc_start_identification` → `mc_auto_tune_pi`
- [ ] 手动微调 PI 增益（观察阶跃响应）
- [ ] 启用死区补偿（若电流波形有零交越失真）
- [ ] 启用弱磁控制（若需要扩展速度范围）
- [ ] 传感器参数调优（Encoder 线数、Hall 序列、Resolver 激磁）
- [ ] 无感观测器参数调优（开环启动参数 + PLL 增益）

### Phase 4: Production Readiness / 量产准备

- [ ] 通过 `mc_get_diag` 持续监控故障/警告
- [ ] 在故障发生时调用 `mc_stop` 并进入安全状态
- [ ] 验证所有 FMEA 条目均有对策
- [ ] MISRA C:2012 静态分析通过（`cmake --build build --target cppcheck`）
- [ ] 全量单元测试通过（`ctest --test-dir build`）
- [ ] 极端工况测试（过温、欠压、过压、堵转）

---

## References / 参考文档

| Document | Path |
|---|---|
| User Manual (API Reference) | `docs/UM_USER_MANUAL.md` |
| Software Requirements | `docs/requirements/SRS.md` |
| Architecture Design | `docs/design/SAD.md` |
| Detailed Design (Control) | `docs/design/SDD_control.md` |
| Detailed Design (Drive) | `docs/design/SDD_drive.md` |
| Detailed Design (Estimator) | `docs/design/SDD_estimator.md` |
| Detailed Design (Math) | `docs/design/SDD_math.md` |
| Detailed Design (Reconstruct) | `docs/design/SDD_reconstruct.md` |
| Detailed Design (Platform) | `docs/design/SDD_platform.md` |
| HARA | `docs/safety/HARA.md` |
| FMEA | `docs/safety/FMEA.md` |
| Safety Case | `docs/safety/SAFETY_CASE.md` |
| Development Plan | `docs/process/DEVELOPMENT_PLAN.md` |
| Coding Standard | `docs/process/CODING_STANDARD.md` |
