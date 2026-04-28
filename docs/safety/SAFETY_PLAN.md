# Safety Plan / 安全计划

## 中文 / Chinese

### 安全开发生命周期 / Safety Development Lifecycle

| Phase | 活动 | 产出 | 状态 |
|---|---|---|---|
| 1. Item Definition | 定义相关项和边界 | `HARA.md` § Item Definition | ✓ |
| 2. HARA | 危害分析与风险评估 | `HARA.md` — 9 项 HAZ, 5 条 SG | ✓ |
| 3. Functional Safety Concept | 功能安全概念（SG + FSR） | `HARA.md` § Safety Goals → TSR 派生 | ✓ |
| 4. Technical Safety Concept | 技术安全概念（TSR + HSI） | `HARA.md` § TSR (10 条) + HSI (6 条) | ✓ |
| 5. Software Safety Requirements | 软件安全需求 | `SRS.md` 02-Safety 域 (7 条需求) | ✓ |
| 6. Software Architecture | 软件架构设计 | `SAD.md` 7 层分层架构 | ✓ |
| 7. Software Unit Design & Implementation | 软件单元设计与实现 | 22 套单元测试 + `mc_constants.h` MISRA 基础 | ✓ |
| 8. Software Integration Test | 软件集成测试 | `tests/integration/` 6 套 17 用例 | ✓ |
| 9. Safety Validation | 安全验证 | 故障注入测试 + 向量测试 (TBD) | ⇨ |
| 10. Safety Case | 安全案例 | `SAFETY_CASE.md` (TBD) | ⇨ |

### 当前安全活动状态 / Current Safety Activity Status

- ISO 26262 开发流程已建立骨架，覆盖 HARA → SG → TSR → HSI 全链条。
- 42 条软件需求中，7 条为安全相关需求（`REQ-MC-02xx`），均有 FMEA + 测试覆盖。
- FMEA 已覆盖全部 7 层架构的 30 种失效模式。
- 10 条 TSR 与软件需求双向追溯。
- cppcheck 静态分析已集成。

### 待完成 / To Be Completed

1. 故障注入测试（`tests/fault/`）
2. 向量测试（vs MATLAB/SIMULINK Golden Reference）
3. 完整安全案例报告（`SAFETY_CASE.md`）
4. MISRA C:2012 强制规则全合规
5. 编译器矩阵 (GCC/IAR/Keil/GHS) CI 验证

---

## English / 英文

### Safety Development Lifecycle

| Phase | Activity | Output | Status |
|---|---|---|---|
| 1. Item Definition | Define item and boundaries | `HARA.md` § Item Definition | ✓ |
| 2. HARA | Hazard Analysis and Risk Assessment | `HARA.md` — 9 HAZ, 5 SGs | ✓ |
| 3. Functional Safety Concept | Safety concept (SG + FSR) | `HARA.md` § Safety Goals → TSR derivation | ✓ |
| 4. Technical Safety Concept | Technical safety concept (TSR + HSI) | `HARA.md` § TSR (10 items) + HSI (6 items) | ✓ |
| 5. Software Safety Requirements | Software safety requirements | `SRS.md` 02-Safety domain (7 reqs) | ✓ |
| 6. Software Architecture | Software architecture design | `SAD.md` 7-layer architecture | ✓ |
| 7. Software Unit Design & Implementation | Unit design and implementation | 22 unit test suites + `mc_constants.h` MISRA baseline | ✓ |
| 8. Software Integration Test | Integration testing | `tests/integration/` 6 suites 17 cases | ✓ |
| 9. Safety Validation | Safety validation | Fault injection + Vector tests (TBD) | ⇨ |
| 10. Safety Case | Safety case | `SAFETY_CASE.md` (TBD) | ⇨ |

### Current Safety Activity Status

- ISO 26262 development workflow skeleton established, covering HARA → SG → TSR → HSI.
- 7 out of 42 software requirements are safety-related (`REQ-MC-02xx`), with FMEA + test coverage.
- FMEA covers 30 failure modes across all 7 architectural layers.
- 10 TSRs are bidirectionally traced to software requirements.
- cppcheck static analysis integrated.

### To Be Completed

1. Fault injection tests (`tests/fault/`)
2. Vector tests (vs MATLAB/SIMULINK Golden Reference)
3. Full safety case report (`SAFETY_CASE.md`)
4. Full MISRA C:2012 mandatory rule compliance
5. Compiler matrix (GCC/IAR/Keil/GHS) CI verification
