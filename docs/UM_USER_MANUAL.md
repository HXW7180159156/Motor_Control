# Motor Control Library — User Manual / 用户手册

**Version**: 0.1.0 | **Language**: C99 | **License**: BSD 3-Clause

---

## 1. Overview / 概述

Motor Control Library 是一个面向汽车嵌入式 MCU 的可移植 C99 电机控制算法库，支持 PMSM FOC 和 BLDC 无刷直流电机的完整控制链路。

### Key Features / 核心特性

| Feature | Description |
|---|---|
| PMSM FOC | 磁场定向控制，含 Clarke/Park/InvPark 变换、PI 电流环、SVPWM、弱磁、MTPA |
| BLDC | 六步换相，Hall 有感和 BEMF 无感两种模式 |
| Sensorless | LPF+PLL 基线观测器 + SMO 滑模观测器 |
| Identify | 电机参数自动辨识（Rs, Ld, Lq, flux_wb），含 CV 一致性检验 trust gate |
| Current Sense | 1-shunt / 2-shunt / 3-shunt 电流检测与重构，1-shunt 含预测补偿 |
| Auto-Tune | 从辨识参数自动计算电流环和速度环 PI 增益 |
| DTC | 逆变器死区时间补偿 |
| Dual Numeric | float32 全库覆盖 + Q31 定点路径 |
| Safety | ISO 26262 友好流程，MISRA C:2012 基线，完整 HARA→SG→TSR→HSI 证据链 |
| Platform | ARM Cortex-M0+/M4F/M7/M33/R5F/R52，GCC/IAR/Keil/GHS 工具链 |

### Architecture / 架构

```
┌─────────────────────────────────────────────┐
│  api     — mc_init, mc_start, mc_fast_step │
├─────────────────────────────────────────────┤
│  drive   — PMSM FOC, BLDC Hall, BLDC SS     │
├─────────────────────────────────────────────┤
│  reconstruct — 1/2/3-shunt current sensing  │
├─────────────────────────────────────────────┤
│  estimator — Hall, Encoder, Resolver,       │
│              Sensorless(LPF+PLL), SMO       │
├─────────────────────────────────────────────┤
│  control — Clarke/Park/InvPark, PI, SVPWM   │
├─────────────────────────────────────────────┤
│  math    — clamp, wrap, LPF, Q31 helpers   │
├─────────────────────────────────────────────┤
│  platform — PWM, ADC, Fault, Timer hooks    │
└─────────────────────────────────────────────┘
```

---

## 2. Data Types / 数据类型

### 2.1 Numeric Types / 数值类型

| Type | C Type | Range | Use |
|---|---|---|---|
| `mc_f32_t` | `float` | IEEE 754 | float32 计算路径 |
| `mc_q31_t` | `int32_t` | [-1.0, 1.0) | Q31 定点计算 |
| `mc_q15_t` | `int16_t` | [-1.0, 1.0) | 保留 |
| `mc_bool_t` | `uint8_t` | {0, 1} | 布尔值 |

```c
#define MC_FALSE ((mc_bool_t)0U)
#define MC_TRUE  ((mc_bool_t)1U)
```

### 2.2 Vector Types / 向量类型

```c
typedef struct { mc_f32_t a, b, c; } mc_abc_t;          // 三相静止坐标系
typedef struct { mc_f32_t alpha, beta; } mc_alphabeta_t;  // αβ 静止坐标系
typedef struct { mc_f32_t d, q; } mc_dq_t;               // dq 旋转坐标系

// Q31 variants
typedef struct { mc_q31_t a, b, c; } mc_abc_q31_t;
typedef struct { mc_q31_t alpha, beta; } mc_alphabeta_q31_t;
typedef struct { mc_q31_t d, q; } mc_dq_q31_t;
```

---

## 3. Status Codes / 状态码

| Code | Value | Meaning |
|---|---|---|
| `MC_STATUS_OK` | 0 | 操作成功 |
| `MC_STATUS_ERROR` | 1 | 未指定错误 |
| `MC_STATUS_INVALID_ARG` | 2 | 无效参数（NULL 指针等） |
| `MC_STATUS_INVALID_STATE` | 3 | 当前状态不允许该操作 |
| `MC_STATUS_UNSUPPORTED` | 4 | 请求的功能未支持 |
| `MC_STATUS_TIMEOUT` | 5 | 操作超时 |

### Fault Codes / 故障码

| Code | Meaning |
|---|---|
| `MC_FAULT_NONE` | 无故障 |
| `MC_FAULT_CONFIG` | 配置错误 |
| `MC_FAULT_OVERCURRENT` | 过流 |
| `MC_FAULT_OVERVOLTAGE` | 过压 |
| `MC_FAULT_UNDERVOLTAGE` | 欠压 |
| `MC_FAULT_SENSOR` | 传感器故障 |
| `MC_FAULT_CONTROL` | 控制回路故障 |
| `MC_FAULT_INTERNAL` | 内部系统故障 |

### Warning Codes / 警告码

| Code | Meaning |
|---|---|
| `MC_WARNING_NONE` | 无警告 |
| `MC_WARNING_LIMIT_ACTIVE` | 硬件限制激活 |
| `MC_WARNING_DERATING_ACTIVE` | 输出降额 |
| `MC_WARNING_OBSERVER_UNLOCKED` | 观测器失锁 |

---

## 4. Operating Modes / 运行模式

| Mode | Description | Sensor Required |
|---|---|---|
| `MC_MODE_DISABLED` | 禁用 | — |
| `MC_MODE_BLDC_HALL` | BLDC Hall 六步换相 | Hall 传感器 |
| `MC_MODE_BLDC_SENSORLESS` | BLDC BEMF 过零检测 | — |
| `MC_MODE_PMSM_FOC_HALL` | PMSM FOC + Hall | Hall 传感器 |
| `MC_MODE_PMSM_FOC_ENCODER` | PMSM FOC + 编码器 | 增量编码器 |
| `MC_MODE_PMSM_FOC_RESOLVER` | PMSM FOC + 旋变 | Resolver |
| `MC_MODE_PMSM_FOC_SENSORLESS` | PMSM FOC + LPF+PLL 观测器 | — |
| `MC_MODE_PMSM_FOC_SMO` | PMSM FOC + SMO 观测器 | — |

---

## 5. Configuration Structures / 配置结构体

### 5.1 Motor Parameters / 电机参数 (`mc_motor_params_t`)

```c
typedef struct {
    uint8_t pole_pairs;     // 极对数
    mc_f32_t rs_ohm;        // 定子电阻 [Ω]
    mc_f32_t ld_h;          // d 轴电感 [H]
    mc_f32_t lq_h;          // q 轴电感 [H]
    mc_f32_t flux_wb;       // 永磁体磁链 [Wb]
} mc_motor_params_t;
```

### 5.2 FOC Configuration / FOC 配置 (`mc_foc_cfg_t`)

```c
typedef struct {
    mc_pi_cfg_t id_pi_cfg;                // d 轴电流 PI
    mc_pi_cfg_t iq_pi_cfg;                // q 轴电流 PI
    mc_pi_cfg_t speed_pi_cfg;             // 速度环 PI
    mc_svpwm_cfg_t svpwm_cfg;             // SVPWM 配置
    mc_current_sense_cfg_t current_cfg;   // 电流检测配置
    mc_f32_t speed_loop_dt_s;             // 速度环采样时间 [s]
    mc_f32_t iq_limit;                    // q 轴电流限制 [A]
    mc_f32_t voltage_limit;               // 电压限制 [V]
    mc_f32_t bus_voltage_scale;           // 母线电压测量缩放
    mc_f32_t bus_voltage_offset;          // 母线电压测量偏移
    mc_f32_t bus_voltage_min;             // 最小母线电压 [V]
    mc_bool_t mtpa_enable;                // 启用 MTPA
    mc_bool_t fw_enable;                  // 启用弱磁
    mc_f32_t fw_kp;                       // 弱磁 Kp
    mc_f32_t fw_ki;                       // 弱磁 Ki
    mc_f32_t fw_min_id;                   // 弱磁最小 Id [A]
    mc_f32_t fw_activation_ratio;         // 弱磁激活电压比
    mc_bool_t dtc_enable;                 // 启用死区补偿
    mc_f32_t dtc_deadtime_ns;             // 死区时间 [ns]
} mc_foc_cfg_t;
```

### 5.3 PI Configuration / PI 配置 (`mc_pi_cfg_t`)

```c
typedef struct {
    mc_f32_t kp;            // 比例增益
    mc_f32_t ki;            // 积分增益
    mc_f32_t integral_min;  // 积分下限
    mc_f32_t integral_max;  // 积分上限
    mc_f32_t output_min;    // 输出下限
    mc_f32_t output_max;    // 输出上限
} mc_pi_cfg_t;
```

### 5.4 SVPWM Configuration / SVPWM 配置 (`mc_svpwm_cfg_t`)

```c
typedef struct {
    mc_f32_t modulation_limit;  // 调制深度限制 [0-1]，推荐 0.95
    mc_f32_t duty_min;          // 占空比下限 [0]，通常 0.0
    mc_f32_t duty_max;          // 占空比上限 [1]，通常 1.0
} mc_svpwm_cfg_t;
```

### 5.5 Current Sense Configuration / 电流检测配置 (`mc_current_sense_cfg_t`)

```c
typedef struct {
    mc_current_sense_type_t type;  // MC_CURRENT_SENSE_1SHUNT / 2SHUNT / 3SHUNT
    union {
        mc_1shunt_cfg_t shunt1;    // 1-shunt: scale, offset, pwm_period_s, min_active_duty, ...
        mc_2shunt_cfg_t shunt2;    // 2-shunt: offset_a/b, scale_a/b
        mc_3shunt_cfg_t shunt3;    // 3-shunt: offset_a/b/c, scale_a/b/c
    } cfg;
} mc_current_sense_cfg_t;
```

### 5.6 Sensor Configuration / 传感器配置 (`mc_sensor_cfg_t`)

```c
typedef struct {
    mc_mode_t primary_mode;              // 主模式选择
    mc_f32_t pole_pairs;                 // 极对数
    mc_hall_cfg_t hall_cfg;              // Hall 传感器配置
    mc_encoder_cfg_t encoder_cfg;        // 编码器配置
    mc_resolver_cfg_t resolver_cfg;      // 旋变配置
    mc_sensorless_cfg_t sensorless_cfg;  // 无感观测器配置（LPF+PLL 和 SMO 共用）
    mc_bldc_sensorless_cfg_t bldc_sensorless_cfg;  // BLDC 无感配置
} mc_sensor_cfg_t;
```

### 5.7 Hardware Limits / 硬件限制 (`mc_hw_limits_t`)

```c
typedef struct {
    mc_f32_t bus_voltage_min_v;     // 最小母线电压 [V]
    mc_f32_t bus_voltage_max_v;     // 最大母线电压 [V]
    mc_f32_t phase_current_max_a;   // 最大相电流 [A]
    mc_f32_t temperature_max_deg_c; // 最高温度 [°C]
    mc_f32_t speed_max_rpm;         // 最高转速 [RPM]
} mc_hw_limits_t;
```

### 5.8 Platform Hooks / 平台钩子 (`mc_port_hooks_t`)

```c
typedef struct {
    void (*pwm_apply)(const void *cmd);                    // 应用 PWM 输出
    void (*adc_trigger_apply)(const mc_adc_trigger_plan_t *plan);  // 应用 ADC 触发
    void (*fault_signal)(uint32_t fault_code);             // 上报故障
    uint32_t (*get_time_us)(void);                         // 获取 µs 时间戳
} mc_port_hooks_t;
```

---

## 6. API Reference / API 参考

### 6.1 Lifecycle / 生命周期

#### `mc_init`
```c
mc_status_t mc_init(mc_instance_t *inst, const mc_system_cfg_t *cfg);
```
初始化电机控制实例。拷贝配置、初始化传感器和驱动子系统。

**Parameters**:
- `inst` — 实例存储（需非 NULL）
- `cfg` — 系统配置（需非 NULL）

**Returns**: `MC_STATUS_OK` 成功，`MC_STATUS_INVALID_ARG` 参数无效，`MC_STATUS_UNSUPPORTED` 不支持的数值/模式组合。

**Note**: 初始化失败后 `inst->initialized == MC_FALSE`。

---

#### `mc_start`
```c
mc_status_t mc_start(mc_instance_t *inst);
```
使能电机控制实例。BLDC sensorless 模式下还会启动对齐序列。

**Returns**: `MC_STATUS_OK` 成功，`MC_STATUS_INVALID_ARG` inst==NULL，`MC_STATUS_INVALID_STATE` 未初始化或 BLDC sensorless 无法启动。

---

#### `mc_stop`
```c
mc_status_t mc_stop(mc_instance_t *inst);
```
禁用电机控制实例。BLDC sensorless 模式会复位到 IDLE。

---

#### `mc_reset`
```c
mc_status_t mc_reset(mc_instance_t *inst);
```
复位运行时状态，保留配置。清除 enable/mode/速度/转矩/PI/传感器状态。

---

#### `mc_set_mode`
```c
mc_status_t mc_set_mode(mc_instance_t *inst, mc_mode_t mode);
```
设置运行模式。初始化后仅接受 `primary_mode`。

**Returns**: `MC_STATUS_INVALID_STATE` 当尝试切换到非配置模式。

---

#### `mc_set_enable`
```c
mc_status_t mc_set_enable(mc_instance_t *inst, mc_bool_t enable);
```
直接设置 enable 标志。

---

### 6.2 Control Reference / 控制参考值

#### `mc_set_speed_ref`
```c
mc_status_t mc_set_speed_ref(mc_instance_t *inst, mc_f32_t speed_ref);
```
设置机械速度参考 [RPM] 并启用速度控制（禁用转矩控制）。

---

#### `mc_set_torque_ref`
```c
mc_status_t mc_set_torque_ref(mc_instance_t *inst, mc_f32_t torque_ref);
```
设置转矩参考值，禁用速度控制。若 MTPA 启用则计算对应的 id_ref。

---

#### `mc_set_current_ref_dq`
```c
mc_status_t mc_set_current_ref_dq(mc_instance_t *inst, mc_f32_t id_ref, mc_f32_t iq_ref);
```
直接设置 dq 电流参考 [A]，绕过速度和转矩回路。

---

### 6.3 Control Loop Execution / 控制回路执行

#### `mc_fast_step`
```c
mc_status_t mc_fast_step(mc_instance_t *inst, const mc_fast_input_t *in, mc_fast_output_t *out);
```
执行一次快速控制循环（电流环 / 辨识步进）。频率通常等于 PWM 频率。

**Input** (`mc_fast_input_t`):
```c
typedef struct {
    mc_adc_raw_t adc_raw;           // ADC 原始值（电流/电压）
    mc_fault_input_t fault_input;   // 硬件故障输入
    mc_resolver_raw_t resolver_raw; // 旋变原始信号
    uint8_t hall_code;              // Hall 编码
    uint32_t timestamp_us;          // 时间戳 [µs]
} mc_fast_input_t;
```

**Output** (`mc_fast_output_t`):
```c
typedef struct {
    mc_pwm_cmd_t pwm_cmd;                       // PWM 输出命令
    mc_adc_trigger_plan_t adc_trigger_plan;     // ADC 触发计划
    mc_1shunt_comp_status_t current_comp_status; // 1-shunt 补偿状态
} mc_fast_output_t;
```

**Important**: 若 `inst->enabled == MC_FALSE`，output 全部清零。

---

#### `mc_medium_step`
```c
mc_status_t mc_medium_step(mc_instance_t *inst, const mc_medium_input_t *in, mc_medium_output_t *out);
```
执行一次中速控制循环（速度环 / 传感器更新）。频率通常为电流环的 1/N。

**Input**:
```c
typedef struct {
    mc_f32_t speed_feedback_rpm;    // 速度反馈 [RPM]
    uint32_t encoder_count;         // 编码器计数
    uint32_t timestamp_us;          // 时间戳 [µs]
} mc_medium_input_t;
```

**Output**:
```c
typedef struct {
    mc_f32_t elec_angle_rad;    // 电气角度 [rad]
    mc_f32_t mech_speed_rpm;    // 机械速度 [RPM]
    mc_f32_t id_ref;            // d 轴电流参考 [A]
    mc_f32_t iq_ref;            // q 轴电流参考 [A]
} mc_medium_output_t;
```

---

#### `mc_slow_step`
```c
mc_status_t mc_slow_step(mc_instance_t *inst, const mc_slow_input_t *in, mc_slow_output_t *out);
```
执行一次慢速服务循环（诊断/故障处理/温度监控）。

**Input**:
```c
typedef struct {
    mc_f32_t temperature_deg_c;       // 温度 [°C]
    mc_bool_t clear_fault_request;    // 清除故障请求
    uint32_t timestamp_us;            // 时间戳 [µs]
} mc_slow_input_t;
```

**Output**:
```c
typedef struct {
    mc_mode_t mode;         // 当前模式
    mc_diag_status_t diag;  // 诊断状态
} mc_slow_output_t;
```

---

### 6.4 Diagnostics / 诊断

#### `mc_get_status`
```c
mc_status_t mc_get_status(const mc_instance_t *inst, mc_slow_output_t *status);
```
获取当前模式和诊断状态快照。

---

#### `mc_get_diag`
```c
mc_status_t mc_get_diag(const mc_instance_t *inst, mc_diag_status_t *diag);
```
获取详细诊断快照，包含：
```c
typedef struct {
    mc_fault_t active_fault;                // 当前故障
    mc_warning_t active_warning;            // 当前警告
    uint32_t fault_counter;                  // 故障计数
    uint32_t warning_counter;                // 警告计数
    mc_1shunt_comp_status_t current_comp_status;  // 1-shunt 补偿状态
    mc_bool_t sensorless_observer_valid;     // 观测器有效标志
    mc_bool_t sensorless_pll_locked;         // PLL 锁定标志
    mc_bool_t sensorless_open_loop_active;   // 开环运行标志
} mc_diag_status_t;
```

---

#### `mc_get_version`
```c
mc_status_t mc_get_version(uint32_t *version);
```
获取打包的版本号。格式：`(MAJOR << 16) | (MINOR << 8) | PATCH`。

---

### 6.5 Parameter Identification / 参数辨识

#### `mc_start_identification`
```c
mc_status_t mc_start_identification(mc_instance_t *inst);
```
启动电机参数辨识序列（异步完成，需后续调用 `mc_fast_step`）。

**辨识流程**:
```
ALIGN → RS_SETTLE → RS_MEASURE → LD_INJECT → LD_MEASURE
→ LQ_ALIGN → LQ_INJECT → LQ_MEASURE → COMPLETE
```

---

#### `mc_is_identification_done`
```c
mc_bool_t mc_is_identification_done(const mc_instance_t *inst);
```
检查辨识是否完成。返回 `MC_TRUE` 当状态为 `COMPLETE`。

---

#### `mc_get_identified_params`
```c
mc_status_t mc_get_identified_params(const mc_instance_t *inst, mc_motor_params_t *params);
```
获取 trusted 辨识结果。不要求辨识已完成——返回当前可信值。

**Note**: 无效的 Rs/Ld/Lq 返回 `0.0F`；flux_wb 无有效样本时保留种子值。

---

### 6.6 Auto-Tune / 自动调谐

#### `mc_auto_tune_pi`
```c
mc_status_t mc_auto_tune_pi(mc_instance_t *inst, mc_f32_t bandwidth_divider, mc_f32_t speed_ratio);
```
从辨识后的电机参数自动计算最优 PI 增益并写入实例配置。

**Parameters**:
- `bandwidth_divider` — 电流环带宽 = `2π·f_pwm / divider`。推荐 20（平衡），10（激进），40（保守）
- `speed_ratio` — 速度环带宽 = 电流环带宽 / ratio。推荐 10

**公式**:
- `Kp_d = ω_c * Ld`，`Ki_d = ω_c * Rs`
- `Kp_q = ω_c * Lq`，`Ki_q = ω_c * Rs`
- `Kp_speed = ω_s * Ld * 0.5`，`Ki_speed = ω_s * Kp_speed * 0.25`

---

### 6.7 PI Controller / PI 控制器

```c
void mc_pi_init(mc_pi_t *pi, const mc_pi_cfg_t *cfg);
void mc_pi_reset(mc_pi_t *pi);
mc_f32_t mc_pi_run(mc_pi_t *pi, mc_f32_t error, mc_f32_t dt_s);
```

---

### 6.8 Transforms / 坐标变换

```c
void mc_clarke_run(const mc_abc_t *abc, mc_alphabeta_t *ab);
void mc_park_run(const mc_alphabeta_t *ab, mc_f32_t sin, mc_f32_t cos, mc_dq_t *dq);
void mc_ipark_run(const mc_dq_t *dq, mc_f32_t sin, mc_f32_t cos, mc_alphabeta_t *ab);
```

Clarke: `α = a`, `β = (a + 2b) / √3`

Park: `d = α·cosθ + β·sinθ`, `q = β·cosθ - α·sinθ`

InvPark: `α = d·cosθ - q·sinθ`, `β = d·sinθ + q·cosθ`

---

### 6.9 SVPWM / 空间矢量 PWM

```c
void mc_svpwm_run(const mc_alphabeta_t *v_ab, const mc_svpwm_cfg_t *cfg, mc_pwm_cmd_t *pwm);
```
将 αβ 电压矢量转换为三相 PWM 占空比。自动施加共模偏置，检测扇区。

---

### 6.10 Sensorless Observers / 无感观测器

| Observer | Function | Technique |
|---|---|---|
| LPF+PLL | `mc_sensorless_init` / `mc_sensorless_update` | BEMF 直接估计 + 一阶 LPF + PLL |
| SMO | `mc_smo_init` / `mc_smo_update` | 滑模电流观测器 + sign 函数 + LPF + PLL |

**Observer Comparison**:

| 特性 | LPF+PLL | SMO |
|---|---|---|
| 噪声抑制 | 中 | 高 |
| 相位滞后 | 较大 | 小 |
| 收敛速度 | 慢 | 快 |
| 计算开销 | 低 | 中 |
| 推荐场景 | 原型验证、低算力 MCU | 量产、中高速 |

---

### 6.11 Current Reconstruction / 电流重构

```c
void mc_reconstruct_1shunt_plan(const mc_pwm_cmd_t *pwm, const mc_1shunt_cfg_t *cfg, mc_1shunt_meta_t *meta);
void mc_reconstruct_1shunt_run(const mc_adc_raw_t *raw, const mc_1shunt_cfg_t *cfg, const mc_1shunt_meta_t *meta, mc_abc_t *i_abc);
void mc_reconstruct_2shunt_run(const mc_adc_raw_t *raw, const mc_2shunt_cfg_t *cfg, mc_abc_t *i_abc);
void mc_reconstruct_3shunt_run(const mc_adc_raw_t *raw, const mc_3shunt_cfg_t *cfg, mc_abc_t *i_abc);
```

1-shunt: `i_bus = (raw - offset) * scale` → 按扇区映射到两相。

2-shunt: `ia = (raw_a - off_a) * scale_a`, `ib = (raw_b - off_b) * scale_b`, `ic = -(ia+ib)`。

3-shunt: 每相独立标定后直接输出。

---

### 6.12 Sensor Interfaces / 传感器接口

| Sensor | Init | Update |
|---|---|---|
| Hall | `mc_hall_init(state, cfg)` | `mc_hall_update(state, code, ts, pole_pairs)` |
| Encoder | `mc_encoder_init(state, cfg)` | `mc_encoder_update(state, count, ts)` |
| Resolver | `mc_resolver_init(state, cfg)` | `mc_resolver_update(state, raw, ts)` |

---

### 6.13 BLDC Drive / BLDC 驱动

```c
mc_status_t mc_bldc_hall_init(mc_bldc_hall_t *drive, const mc_bldc_hall_cfg_t *cfg);
mc_status_t mc_bldc_hall_run(mc_bldc_hall_t *drive, uint8_t hall_code, mc_f32_t duty_cmd, mc_pwm_cmd_t *pwm);

mc_status_t mc_bldc_sensorless_init(mc_bldc_sensorless_t *ss, const mc_bldc_sensorless_cfg_t *cfg);
mc_status_t mc_bldc_sensorless_start(mc_bldc_sensorless_t *ss);
mc_status_t mc_bldc_sensorless_run(mc_bldc_sensorless_t *ss, mc_f32_t dt_s, mc_f32_t floating_v, mc_f32_t bus_v, mc_pwm_cmd_t *pwm);
void mc_bldc_sensorless_speed_step(mc_bldc_sensorless_t *ss, mc_f32_t speed_ref_rpm, mc_f32_t dt_s);
```

---

### 6.14 PMSM FOC Drive / PMSM FOC 驱动

```c
mc_status_t mc_pmsm_foc_init(mc_pmsm_foc_t *foc, const mc_pmsm_foc_cfg_t *cfg);
mc_status_t mc_pmsm_foc_run(mc_pmsm_foc_t *foc, const mc_pmsm_foc_input_t *in, mc_pmsm_foc_output_t *out);
mc_f32_t mc_pmsm_compute_mtpa_id(mc_f32_t iq_ref, mc_f32_t flux_wb, mc_f32_t ld_h, mc_f32_t lq_h);
```

---

### 6.15 Math Utilities / 数学工具

```c
mc_f32_t mc_math_clamp_f32(mc_f32_t v, mc_f32_t min, mc_f32_t max);
mc_f32_t mc_math_wrap_angle_rad(mc_f32_t angle);
mc_f32_t mc_math_lpf_f32(mc_f32_t prev, mc_f32_t input, mc_f32_t alpha);

mc_q31_t mc_q31_from_f32(mc_f32_t value);
mc_f32_t mc_q31_to_f32(mc_q31_t value);
mc_q31_t mc_q31_add_sat(mc_q31_t a, mc_q31_t b);
mc_q31_t mc_q31_mul(mc_q31_t a, mc_q31_t b);
```

---

## 7. Platform Integration / 平台集成

### 7.1 Required Hooks / 必需钩子

```c
mc_port_hooks_t hooks = {
    .pwm_apply = my_pwm_apply,             // 必须
    .adc_trigger_apply = my_adc_trigger,   // 可选（无 NULL 则跳过）
    .fault_signal = my_fault_handler,      // 可选
    .get_time_us = my_get_time_us,         // 必须（用于速度估算）
};
```

### 7.2 PWM Output / PWM 输出

`out->pwm_cmd` 结构体：
```c
typedef struct {
    mc_f32_t duty_a, duty_b, duty_c;   // 三相占空比 [0-1]
    uint8_t phase_mode_a, b, c;         // MC_PWM_PHASE_PWM/HIGH/LOW/OFF
    mc_f32_t common_mode_shift;          // 共模偏置
    uint8_t sector;                      // SVPWM 扇区 [1-6]
    uint8_t valid;                       // 1 = 有效输出
} mc_pwm_cmd_t;
```

### 7.3 ADC Input / ADC 输入

`in->adc_raw` 结构体：
```c
typedef struct {
    uint16_t phase_a_raw, phase_b_raw, phase_c_raw;  // 相电流 ADC 值
    uint16_t bus_voltage_raw;                          // 母线电压 ADC 值
} mc_adc_raw_t;
```

### 7.4 Scheduling Example / 调度示例

```c
// 在 PWM 中断中
void PWM_ISR(void) {
    mc_fast_input_t fast_in;
    mc_fast_output_t fast_out;

    read_adc(&fast_in.adc_raw);
    fast_in.timestamp_us = get_timer_us();

    mc_fast_step(&inst, &fast_in, &fast_out);
    apply_pwm(&fast_out.pwm_cmd);
}

// 在定时器中断中（例如每 1ms）
void TIMER_ISR_1ms(void) {
    mc_medium_input_t med_in;
    mc_medium_output_t med_out;

    med_in.speed_feedback_rpm = get_speed();
    med_in.timestamp_us = get_timer_us();

    mc_medium_step(&inst, &med_in, &med_out);
}

// 在主循环中（例如每 10ms）
void main_loop(void) {
    mc_slow_input_t slow_in;
    mc_slow_output_t slow_out;

    slow_in.temperature_deg_c = read_temperature();
    slow_in.timestamp_us = get_timer_us();

    mc_slow_step(&inst, &slow_in, &slow_out);

    if (slow_out.diag.active_fault != MC_FAULT_NONE) {
        handle_fault(slow_out.diag);
    }
}
```

---

## 8. Build System / 构建系统

### Requirements
- CMake ≥ 3.20
- C99 编译器（MSVC, GCC, IAR, Keil, GHS）

### Build Commands

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

### Build Options

| Option | Default | Description |
|---|---|---|
| `MC_BUILD_TESTS` | ON | 构建单元测试 |
| `MC_RUN_CPPCHECK` | ON | 运行 MISRA 静态分析 |

### Static Analysis

```sh
cmake --build build --target cppcheck
```

---

## 9. Version / 版本

```c
#define MC_VERSION_MAJOR (0U)
#define MC_VERSION_MINOR (1U)
#define MC_VERSION_PATCH (0U)
#define MC_VERSION_U32   ((MAJOR << 16) | (MINOR << 8) | PATCH)
```

---

## 10. Compliance / 合规说明

| Standard | Status | Reference |
|---|---|---|
| MISRA C:2012 | 基线已建立，强制规则 0 违规 | `cppcheck_suppressions.txt` |
| ISO 26262 | HARA→SG→TSR→HSI 证据链完整 | `docs/safety/` |
| C99 | 严格遵从，无 GNU 扩展 | `CMakeLists.txt` |
| Doxygen | 全部公共 API 有注释 | `include/` |
