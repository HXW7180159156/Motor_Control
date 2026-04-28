# Hazard Analysis and Risk Assessment / 危害分析与风险评估

## Item Definition / 相关项定义

| Field | Description |
|---|---|
| Item | Motor Control Library (mc_*) — 可移植 C99 电机控制算法库 |
| Functions | PMSM FOC 控制、BLDC 控制、电机参数辨识、电流检测与重构、传感器信号处理 |
| Intended use | 汽车牵引电机、助力转向、制动助力、冷却风扇/水泵的嵌入式 MCU 控制软件 |
| Target ASIL | 根据集成系统不同，最高可达 ASIL D |

### 假定使用场景 / Assumed Use Cases

| Scenario | Application | Max ASIL |
|---|---|---|
| SC-01 | EV 主驱动牵引电机 | ASIL C |
| SC-02 | 电动助力转向 (EPS) | ASIL D |
| SC-03 | 电子制动助力 (e-Booster) | ASIL D |
| SC-04 | 冷却风扇/水泵 | ASIL B |

---

## Hazard Analysis / 危害分析

### 危害分类 / Hazard Classification

| HAZ-ID | 危害描述 | 场景 | Severity | Exposure | Controllability | ASIL |
|---|---|---|---|---|---|---|
| HAZ-01 | 电机意外全速正转（正向扭矩驱动） | SC-01 | S3 | E4 | C2 | ASIL C |
| HAZ-02 | 电机意外反转 | SC-01 | S3 | E3 | C2 | ASIL B |
| HAZ-03 | 电机失控加速（超过指令速度） | SC-01 | S3 | E3 | C1 | ASIL C |
| HAZ-04 | 电机意外制动/锁轴（负扭矩） | SC-01 | S3 | E3 | C2 | ASIL B |
| HAZ-05 | 电机扭矩输出与指令反向 | SC-02 | S3 | E4 | C3 | ASIL D |
| HAZ-06 | 转向辅助力矩消失 | SC-02 | S3 | E4 | C3 | ASIL D |
| HAZ-07 | 制动助力消失 | SC-03 | S3 | E4 | C3 | ASIL D |
| HAZ-08 | 电机过温未保护导致退磁/起火 | SC-01 | S2 | E3 | C3 | ASIL B |
| HAZ-09 | 电流检测失效导致 FOC 控制异常 | All | S3 | E3 | C2 | ASIL C |

### Severity 等级定义 / Severity Scale

| Class | Description |
|---|---|
| S0 | No injury |
| S1 | Light to moderate injury |
| S2 | Severe injury (life-threatening, survival probable) |
| S3 | Fatal or life-threatening (survival uncertain) |

### Exposure 等级定义 / Exposure Scale

| Class | Description |
|---|---|
| E1 | Very low probability |
| E2 | Low probability |
| E3 | Medium probability |
| E4 | High probability |

### Controllability 等级定义 / Controllability Scale

| Class | Description |
|---|---|
| C1 | Simply controllable |
| C2 | Normally controllable |
| C3 | Difficult to control or uncontrollable |

---

## Safety Goals / 安全目标

| SG-ID | 安全目标 | ASIL | 来源 HAZ | 安全状态 | FTTI |
|---|---|---|---|---|---|
| SG-01 | 电机不得输出与指令方向相反的扭矩 | ASIL D | HAZ-05 | PWM 置零 | 100ms |
| SG-02 | 电机不得在未收到指令时产生自主运动 | ASIL C | HAZ-01, HAZ-07 | PWM 置零 | 200ms |
| SG-03 | 电机转速不得超过指令值的 120%（失控加速防护） | ASIL C | HAZ-03 | PWM 置零 | 150ms |
| SG-04 | 电流/电压传感器故障时不得推导出错误的控制输出 | ASIL C | HAZ-09 | 标记传感器失效，冻结控制 | 50ms |
| SG-05 | 电机辨识参数异常不得写入运行时模型 | ASIL B | — | 保留种子值，reject 异常结果 | — |

### FTTI 说明
FTTI = Fault Tolerant Time Interval，即从故障发生到进入安全状态的最大允许时间。

---

## Technical Safety Requirements / 技术安全需求

| TSR-ID | 技术安全需求 | 关联 SG | ASIL | 验证方法 |
|---|---|---|---|---|
| TSR-01 | `mc_fast_step` 入口处检查 `inst`、`in`、`out` 非 NULL，任一为 NULL 返回 `MC_STATUS_INVALID_ARG` | SG-01, SG-02 | ASIL D | 单元测试：NULL arg 注入 |
| TSR-02 | 检测 `inst->enabled == MC_FALSE` 时 PWM 输出必须为全零 | SG-02 | ASIL C | 集成测试：disabled 状态 zero PWM |
| TSR-03 | 检测 `MC_FAULT_OVERCURRENT` / `MC_FAULT_OVERVOLTAGE` 时同周期内 PWM 置零 | SG-01, SG-02 | ASIL D | 故障注入测试 |
| TSR-04 | Sensorless 观测器 BEMF 幅度 < `min_bemf` 时将 `observer_valid` 置 `MC_FALSE`，禁止控制环使用观测角度 | SG-04 | ASIL C | 集成测试：低 BEMF 注入 |
| TSR-05 | Resolver 信号幅度 < `min_signal_amplitude` 时标记 `signal_valid = MC_FALSE`，角度冻结 | SG-04 | ASIL C | 单元测试：低信号注入 |
| TSR-06 | 辨识状态机在电流 > `max_current_a` 时进入 `MC_IDENTIFY_STATE_ERROR`，PWM 归零 | SG-05 | ASIL B | 集成测试：过流注入 |
| TSR-07 | 辨识结果 Rs/Ld/Lq 非正/非有限值时报告 `0.0F`，禁止写入运行时模型 | SG-05 | ASIL B | 单元测试：无效结果注入 |
| TSR-08 | `mc_init` 失败后必须保证 `inst->initialized == MC_FALSE` | SG-01, SG-02 | ASIL D | 单元测试：init 失败状态检查 |
| TSR-09 | 快速环路 `mc_fast_step` 在辨识激活时执行辨识路径，完成后将 trusted 结果写回 FOC/sensorless 模型 | SG-05 | ASIL B | 集成测试：辨识完成检查 |
| TSR-10 | 1-shunt 补偿模式必须记录到诊断结构体 `mc_1shunt_comp_status_t` 中 | SG-04 | ASIL C | 集成测试：诊断字段检查 |

---

## Hardware-Software Interface (HSI) / 软硬件接口

| HSI-ID | 接口 | 方向 | 安全相关属性 |
|---|---|---|---|
| HSI-01 | PWM 输出端口 (`pwm_apply`) | SW → HW | 故障时必须在同一个控制周期内输出零占空比 |
| HSI-02 | ADC 采样输入 (`adc_raw`) | HW → SW | 原始值需经过标定转换（scale/offset），禁止直接使用 raw 值作为控制输入 |
| HSI-03 | 故障信号 (`fault_signal`) | HW → SW | 过流/过压硬件保护信号应优先于软件判断，延迟 ≤ 10μs |
| HSI-04 | 定时器 (`get_time_us`) | HW → SW | 溢出/回绕需在速度估算中容忍，时间戳单调递增非必需但推荐 |
| HSI-05 | Resolver 激磁 (`excitation_set`) | SW → HW | 激磁幅值/频率与 resolver 硬件匹配，避免过激磁导致信号失真 |
| HSI-06 | 看门狗 | HW ↔ SW | 平台层看门狗刷新应在 `mc_fast_step` 完成后由集成者控制 |
