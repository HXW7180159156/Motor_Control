# Development Plan / 开发计划

## 阶段概览 / Phase Overview

```
M1 收尾           M2 深化                M3 量产准备
(2~3 周)          (4~6 周)              (6~8 周)
────●────────────────●─────────────────────●──→
  代码质量归一    安全需求+集成测试      认证证据链闭合
```

---

## M1 收尾阶段 — 代码基线达标 / Code Baseline Qualification

### M1.1 消除魔数 / Eliminate Magic Numbers

- 新建 `include/mc_constants.h`，集中定义数学常数与算法默认参数。
- 范围：全部 15+ 处裸浮点字面量。
- 验证：编译通过 + 全量单元测试无回归。

### M1.2 消除重复代码 / Eliminate Duplicate Code

- 删除 `mc_control_foc_clamp`、`mc_pi_clamp`、`mc_reconstruct_1shunt_clamp`。
- 统一调用 `mc_math_clamp_f32`。
- 验证：`ctest` 全量通过。

### M1.3 修复浮点比较违规 / Fix Float Comparison Violations

- 配置值校验改用区间判断（如 `if (value < 1e-6F)` 替代 `if (value <= 0.0F)`）。
- 范围：`mc_api.c`、`mc_control_foc.c`、`mc_drive_pmsm.c`、`mc_math.c`。
- 注意：不宜用容差的无条件浮动比较保留不变，走偏差记录。

### M1.4 配置 MISRA 静态检查 / Configure MISRA Static Analysis

- CMake 集成 cppcheck，配置 MISRA C:2012 规则集。
- CI 中可自动运行。

### M1.5 Doxygen 格式统一 / Unify Doxygen Format

- 统一 `@file` 注释格式。
- 为新文件（如 `mc_constants.h`）补充 `@ingroup` 分组。

### M1.6 架构缺陷修复 / Fix Architectural Issues

- `mc_drive_pmsm.c:454-455` 重复调用 `mc_pmsm_optimize_1shunt_pwm` 添加注释或修正。
- `mc_sensor_sensorless.c:86-88` 添加 dt_s 上限检查。
- 各模块 NaN/Inf 保护统一化。

### M1.7 Changelog 更新 / Update Changelog

- 记录 M1.1~M1.6 所有变更。

### M1 验收标准 / M1 Acceptance Criteria

| 项 | 标准 |
|---|---|
| 魔数 | 源代码中 `0.0F`/`1.0F` 以外的裸浮点字面量归零 |
| 重复代码 | 4 个 clamp 归一到 `mc_math_clamp_f32` |
| MISRA 基线 | `cppcheck --addon=misra` 0 个强制违规 |
| Doxygen | 所有新增/修改文件格式一致 |
| 单元测试 | 22 套全量通过，无回归 |

---

## M2 深化阶段 — 安全需求+集成测试 / Safety Requirements + Integration Tests

### M2.1 SRS 需求扩充 / Expand SRS

- SRS 从 10 条扩充至 40~50 条，覆盖安全目标、时序、精度、故障检测、标定、传感器、辨识。
- 同步更新 RTM。

### M2.2 功能安全基础 / Functional Safety Foundation

- Item Definition 定义系统边界与应用场景。
- HARA Lite 简化版危害分析，派生 Safety Goals 及 ASIL。
- TSR 派生技术安全需求。
- FMEA 扩展至 30+ 条目。

### M2.3 集成测试实现 / Integration Test Implementation

- `tests/integration/` 实现 6 套集成测试，覆盖 PMSM FOC、Sensorless、BLDC Hall、BLDC Sensorless、辨识、三步调度。

### M2.4 SDD 文档深化 / Deepen SDD Documentation

- 为 6 份 SDD 补充状态机图、时序图、伪代码、数值精度分析。

### M2.5 性能基准测试 / Performance Benchmark Tests

- `tests/bench/` 实现 3 项基准测试：fast_step、transform、PI。

### M2 验收标准 / M2 Acceptance Criteria

| 项 | 标准 |
|---|---|
| SRS | 需求 40+ 条，RTM 追溯完整 |
| FMEA | 30+ 条目，覆盖所有层级 |
| 集成测试 | 6+ 套，全量通过 |
| SDD | 每篇均有状态机图/时序图/伪代码 |
| 性能 | 关键路径 WCET 有测量数据 |

---

## M3 量产准备阶段 — 认证证据链闭合 / Certification Evidence Closure

### M3.1 完整安全案例 / Complete Safety Case

- HARA → Safety Goals → FSR → TSR → HSI 完整证据链。
- 安全案例总结报告 `docs/safety/SAFETY_CASE.md`。
- FFI（免于干扰）分析。

### M3.2 故障注入测试 / Fault Injection Tests

- `tests/fault/` 实现 10+ 故障注入场景（传感器信号丢失、跳变、电源故障、PWM 故障）。

### M3.3 向量测试 / Vector Tests

- Golden Reference Model 对比（Clarke/Park/InvPark/SVPWM/PI vs MATLAB）。

### M3.4 MISRA 完整合规 / Full MISRA Compliance

- 强制规则 0 违规，建议规则全部有偏差记录。

### M3.5 编译器矩阵验证 / Compiler Matrix Verification

- CI 中 GCC + IAR + Keil + GHS 编译通过。

### M3 验收标准 / M3 Acceptance Criteria

| 项 | 标准 |
|---|---|
| 安全案例 | HARA→SG→TSR→HSI 证据链完整 |
| 故障注入 | 10+ 场景符合安全目标 |
| 向量测试 | 与 MATLAB 参考误差 < 1e-5 |
| MISRA | 强制 0 违规 |
| 编译器 | 全矩阵编译通过 |

---

## 人力资源估算 / Effort Estimation

| 阶段 | 总人天 | 建议角色 |
|---|---|---|
| M1 收尾 | 5.5 人天 | 1 名嵌入式 C 开发 |
| M2 深化 | 18 人天 | 1 名开发 + 0.5 名功能安全工程师 |
| M3 量产 | 17 人天 | 1 名开发 + 1 名功能安全工程师 + 0.5 名测试工程师 |
| 合计 | ~40 人天 | — |

---

## 关键里程碑 / Key Milestones

```
Week 1-2    M1.1~M1.6 代码质量归一      → M1 基线冻结
Week 3-4    M2.1 SRS 扩充 + M2.2 HARA   → 安全需求基线
Week 5-6    M2.3 集成测试 + M2.4 SDD     → 设计基线
Week 7-8    M2.5 性能测试                → M2 阶段门
Week 9-12   M3.1 安全案例 + M3.2 故障注入 → 安全证据基线
Week 13-14  M3.3 向量测试 + M3.4 MISRA   → 合规基线
Week 15-16  M3.5 编译器矩阵 + 总结评审    → 发布候选
```
