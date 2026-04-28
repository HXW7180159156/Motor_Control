# Safety Case / 安全案例

## Executive Summary / 执行摘要

本安全案例针对 **Motor Control Library (v0.1.0)** 在汽车牵引电机、电动助力转向 (EPS) 和电子制动助力 (e-Booster) 场景下的功能安全性进行论证。安全分析遵循 ISO 26262-6:2018 框架，覆盖从危害分析到软件验证的完整生命周期。

目标 ASIL：**最高 ASIL D**（EPS/e-Booster），基础 ASIL C（牵引电机）。

---

## 1. Item Definition / 相关项定义

| Field | Value |
|---|---|
| Item | Motor Control Library — 可移植 C99 电机控制算法库 |
| Functions | PMSM FOC、BLDC、参数辨识、电流重构、传感器处理 |
| Architecture | 7 层分层架构 (platform → math → control → estimator → reconstruct → drive → api) |
| Platforms | ARM Cortex-M0+/M4F/M7/M33, Cortex-R5F/R52 |
| Compilers | GCC, IAR, Keil, GHS |
| Safety Standard | ISO 26262-6:2018, MISRA C:2012 |

详细见 `docs/safety/HARA.md`。

---

## 2. Hazard Analysis & Risk Assessment / 危害分析与风险评估

| HAZ-ID | Hazard | Scenario | S | E | C | ASIL |
|---|---|---|---|---|---|---|
| HAZ-01 | 电机意外全速正转 | SC-01 (牵引) | S3 | E4 | C2 | **ASIL C** |
| HAZ-03 | 电机失控加速 | SC-01 (牵引) | S3 | E3 | C1 | **ASIL C** |
| HAZ-05 | 扭矩输出与指令反向 | SC-02 (EPS) | S3 | E4 | C3 | **ASIL D** |
| HAZ-06 | 转向助力消失 | SC-02 (EPS) | S3 | E4 | C3 | **ASIL D** |
| HAZ-07 | 制动助力消失 | SC-03 (Brake) | S3 | E4 | C3 | **ASIL D** |
| HAZ-09 | 电流检测失效 | All | S3 | E3 | C2 | **ASIL C** |

完整 HARA 见 `docs/safety/HARA.md`。

---

## 3. Safety Goals / 安全目标

| SG | Safety Goal | ASIL | Safe State | FTTI |
|---|---|---|---|---|
| SG-01 | 电机不得输出与指令方向相反的扭矩 | **ASIL D** | PWM 全零 | 100ms |
| SG-02 | 电机不得在未收到指令时自主运动 | ASIL C | PWM 全零 | 200ms |
| SG-03 | 电机转速不得超过指令值的 120% | ASIL C | PWM 全零 | 150ms |
| SG-04 | 传感器故障时不得推导错误控制输出 | ASIL C | 标记失效，冻结 | 50ms |
| SG-05 | 辨识参数异常不得写入运行时模型 | ASIL B | 保留种子值 | — |

---

## 4. Technical Safety Requirements / 技术安全需求

| TSR | Requirement | SG | ASIL | Verification |
|---|---|---|---|---|
| TSR-01 | fast_step 入口 NULL 检查 | SG-01, SG-02 | D | Unit: null arg 注入 |
| TSR-02 | disabled 状态 PWM 全零 | SG-02 | C | Integration: disabled test |
| TSR-03 | 过流/过压同周期 PWM 置零 | SG-01, SG-02 | D | Fault injection |
| TSR-04 | Sensorless BEMF < min 标记无效 | SG-04 | C | Integration: low BEMF |
| TSR-05 | Resolver 低幅值标记失效 | SG-04 | C | Unit: low signal |
| TSR-06 | 辨识过流进入 ERROR | SG-05 | B | Fault injection: overcurrent |
| TSR-07 | 辨识非物理值 reject | SG-05 | B | Unit: invalid result |
| TSR-08 | init 失败保证 initialized=FALSE | SG-01, SG-02 | D | Unit: fail state |
| TSR-09 | 辨识 trusted 结果写回模型 | SG-05 | B | Integration: completion |
| TSR-10 | 1-shunt 补偿模式记录到诊断 | SG-04 | C | Integration: diag fields |

---

## 5. Safety Mechanisms / 安全机制

### 5.1 Fault Detection / 故障检测

| Mechanism | Implementation | Coverage |
|---|---|---|
| NULL pointer guard | 所有 public API 入口 `if (inst==NULL) return MC_STATUS_INVALID_ARG` | TSR-01, TSR-08 |
| PWM zero on fault | `mc_fast_step` disabled path → `(mc_pwm_cmd_t){0}` | TSR-02, TSR-03 |
| Sensor validity | 各传感器 `signal_valid` / `observer_valid` 标志位 | TSR-04, TSR-05 |
| Overcurrent during identify | `if |i| > max_current_a → MC_IDENTIFY_STATE_ERROR` | TSR-06 |
| Trust gate for identify | `mc_identify_positive_estimate_or_zero` + `isfinite` check | TSR-07 |
| Diagnostic snapshot | `mc_diag_status_t` 包含 `active_fault`, `active_warning`, `current_comp_status` | TSR-10 |

### 5.2 Fault Reaction / 故障反应

| Fault | Reaction | Timing |
|---|---|---|
| `MC_FAULT_OVERCURRENT` | PWM 全零 | 同一 fast_step 周期 |
| `MC_FAULT_OVERVOLTAGE` | PWM 全零 | 同一 fast_step 周期 |
| Sensor invalid | `observer_valid = MC_FALSE`，速度清零 | 下一个 fast_step |
| Identify overcurrent | `MC_IDENTIFY_STATE_ERROR`，PWM 归零 | 即时 |
| Init failure | `initialized = MC_FALSE`，禁止后续操作 | 即时 |

---

## 6. FMEA Coverage / FMEA 覆盖

FMEA 覆盖全部 7 层架构，共 **30 项失效模式**：

| Layer | Entries | Key Failure Modes |
|---|---|---|
| Platform | 4 | PWM 未更新，ADC 卡滞，Fault 延迟，Timer 溢出 |
| Math | 4 | clamp 反转，Q31 溢出未饱和，wrap 无界，LPF 发散 |
| Control | 5 | PI 积分饱和，SVPWM 扇区抖动，Park 未归一化 sin/cos |
| Estimator | 6 | Hall 非法编码，Encoder 溢出，Resolver 低幅度，Sensorless unlock 延迟 |
| Reconstruct | 4 | 1-shunt 窗口失配，预测补偿模型失配，3-shunt 漂移 |
| Drive | 5 | 弱磁积分负漂，母线欠压未钳位，Hall 扇区错误，ZC 检测延迟 |
| API | 5 | init 失败未清除，模式切换残留，辨识 trust gate 失效 |

详细 FMEA 见 `docs/safety/FMEA.md`。

---

## 7. Verification & Validation / 验证与确认

### 7.1 Test Coverage Summary / 测试覆盖汇总

| Test Level | Suites | Cases | Type |
|---|---|---|---|
| Unit Test | 22 | ~80 | 单元功能验证 |
| Integration Test | 6 | 17 | 子系统集成验证 |
| Fault Injection | 3 | 10 | 安全需求验证 |
| Vector (Golden Ref) | 1 | 23 | 算法精度验证 |
| Bench (Performance) | 1 | 7 | 实时性验证 |
| **Total** | **34** | **~137** | — |

### 7.2 Safety Requirement Coverage / 安全需求覆盖

| TSR | Unit | Integration | Fault | Vector |
|---|---|---|---|---|
| TSR-01 (NULL guard) | ✓ | — | ✓ | — |
| TSR-02 (disabled PWM) | ✓ | ✓ | — | — |
| TSR-03 (fault PWM zero) | — | — | ✓ | — |
| TSR-04 (low BEMF) | ✓ | ✓ | ✓ | — |
| TSR-05 (low resolver) | ✓ | — | ✓ | — |
| TSR-06 (identify OC) | — | ✓ | ✓ | — |
| TSR-07 (non-physical reject) | ✓ | — | — | — |
| TSR-08 (init fail state) | ✓ | — | ✓ | — |
| TSR-09 (trusted write-back) | — | ✓ | — | — |
| TSR-10 (1shunt diag) | ✓ | ✓ | — | — |

覆盖率：**10/10 TSR 均有验证覆盖** ✓

---

## 8. Residual Risk Assessment / 残余风险评估

| Risk | Severity | Residual after Mitigation | Rationale |
|---|---|---|---|
| MCU 硬件故障导致 PWM 持续输出 | Catastrophic | **需外部 SBC/看门狗** | 软件库本身无法检测 MCU 锁死；平台层需集成外部硬件看门狗 |
| 编译器优化导致安全代码被移除 | High | **低** | `volatile` 未用于安全关键变量；需添加 Memory Barrier 或使用 `MC_ALIGN` 保护 |
| 识别 flux_wb 精度不足 | Low | **低** | 当前开环电压平衡法为原型级实现；偏差记录 DEV-D02 已说明量产需死区补偿 |
| 标准数学库函数未认证 | Medium | **低**（已记录偏差） | DEV-005 要求量产阶段替换为认证数学库 |
| FFI (免于干扰) 未证明 | Medium | **中** | 多实例场景下内存隔离未形式化证明 |

### 残余风险结论

| ASIL Level | Residual Risk Acceptable? |
|---|---|
| ASIL B | ✓ Yes |
| ASIL C | ✓ Yes（需外部看门狗） |
| ASIL D | ⚠ Conditional（需补充 FFI 证明 + 认证数学库 + Memory Barrier） |

---

## 9. Compliance Summary / 合规摘要

| ISO 26262 Clause | Requirement | Status | Reference |
|---|---|---|---|
| ISO 26262-3: Item Definition | ✗ | ✓ | `HARA.md` §1 |
| ISO 26262-3: HARA | ✗ | ✓ | `HARA.md` §2-3 |
| ISO 26262-3: Safety Goals | ✗ | ✓ | `HARA.md` §4 |
| ISO 26262-4: TSR | ✗ | ✓ | `HARA.md` §5 |
| ISO 26262-4: HSI | ✗ | ✓ | `HARA.md` §6 |
| ISO 26262-6: Software Safety Requirements | ✗ | ✓ | `SRS.md` §02 |
| ISO 26262-6: Software Architecture | ✗ | ✓ | `SAD.md` |
| ISO 26262-6: Software Unit Design | ✗ | ✓ | `SDD_*.md` (6 docs) |
| ISO 26262-6: Software Unit Test | ✗ | ✓ | 22 unit test suites |
| ISO 26262-6: Software Integration Test | ✗ | ✓ | 6 integration test suites |
| ISO 26262-6: Software Safety Validation | ✗ | ✓ | Fault injection + Vector tests |
| ISO 26262-9: ASIL Decomposition | ✗ | — | Not applied (single-point design) |
| ISO 26262-9: FFI Analysis | ✗ | ⚠ | **Not completed** — required for ASIL D |
| ISO 26262-8: Supporting Processes | ✗ | ✓ | `CODING_STANDARD.md`, `MISRA_COMPLIANCE_PLAN.md`, `RELEASE_CHECKLIST.md` |

---

## 10. Conclusions / 结论

### Achievements / 成果

1. 建立了完整的 HARA → SG → TSR → HSI → Test 安全生命周期证据链
2. 42 条软件需求中的 7 条安全需求全部有验证覆盖
3. 10 条 TSR 全部被单元/集成/故障注入/向量测试覆盖
4. FMEA 覆盖全 7 层架构 30 项失效模式
5. MISRA C:2012 基线已建立，8 条偏差记录说明

### Gaps / 差距

| Gap | Required for ASIL | Action |
|---|---|---|
| FFI 分析 | ASIL D | 多实例内存隔离的形式化证明 |
| 认证数学库 | ASIL C/D | 替换 `<math.h>` 为 CMSIS-DSP 或同等认证库 |
| 外部看门狗集成 | ASIL C/D | 平台层 `pwm_apply` 需集成硬件看门狗刷新 |
| Memory Barrier 保护安全关键变量 | ASIL D | 关键状态变量添加编译器/CPU 内存屏障 |
| 编译器全矩阵 CI | ASIL B+ | GCC/IAR/Keil/GHS 实际编译验证 |
| 硬件在环 (HIL) 测试 | ASIL D | 真实 MCU 平台上的故障注入验证 |

### Overall Statement / 总体声明

Motor Control Library v0.1.0 在当前的 M1-M3 开发阶段已建立了符合 ISO 26262 要求的软件安全开发基础。对于 **ASIL B** 应用场景，当前证据链可以支撑安全论证。对于 **ASIL C/D** 应用，需完成上述差距项（特别是 FFI 分析、认证数学库和外部看门狗）后方可完整论证。
