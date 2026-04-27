# Unit Test Specification / 单元测试规格说明

## 中文

- 测试编号格式：`TST-MC-XXXX`
- 当前单元测试框架：Unity
- 当前自动化入口：CMake / CTest 目标 `test_mc_api`

### 当前覆盖范围

- API 生命周期与模式约束：`mc_init()` 失败回滚、`mc_set_mode()` 保守切模态、`mc_reset()` / `mc_stop()` 运行态语义、disabled 中环不推进速度环
- PMSM FOC / BLDC Hall / BLDC sensorless / PMSM sensorless 的顶层 API 管线
- `1/2/3-shunt` 电流重构、`1-shunt` 补偿状态与诊断字符串
- `identify` 公式与状态机：
  - `Ld/Lq` 按窗口平均电流扣除 `Rs` 压降
  - 变 `dt` 与实际离散脉冲时长
  - `flux_wb` 时间加权、有效窗口门限、seed 保留
  - candidate / trusted 分层与非物理结果拒绝
- PMSM sensorless 观测器：
  - `L * di/dt` 补偿
  - low-BEMF invalidation 语义
  - 连续样本 handoff / lock / unlock
  - `min_bemf < lock_bemf` 与开环电压斜坡约束校验
- 首版 Q31：helper、Clarke/Park/InvPark、SVPWM、`2/3-shunt` 重构，以及顶层支持矩阵拦截

### 当前限制

- 当前仓库未提供独立 `CMock` mock suite；接口隔离主要通过轻量输入夹具与 API 级回归完成
- 工具链兼容性、板级 ADC/PWM 外设联调和硬件在环不属于当前单元测试范围

## English

- Test ID format: `TST-MC-XXXX`
- Current unit test framework: Unity
- Current automated entry point: CMake / CTest target `test_mc_api`

### Current Coverage

- API lifecycle and mode constraints: `mc_init()` rollback on failure, conservative `mc_set_mode()`, runtime semantics of `mc_reset()` / `mc_stop()`, and disabled medium-loop behavior
- Top-level API pipelines for PMSM FOC, BLDC Hall, BLDC sensorless, and PMSM sensorless
- `1/2/3-shunt` current reconstruction, `1-shunt` compensation status, and diagnostic mode strings
- `identify` formulas and state machine:
  - `Ld/Lq` estimation using window-average current for `Rs` drop compensation
  - variable `dt` and actual discrete pulse duration
  - `flux_wb` time-weighting, valid-window thresholding, and seed retention
  - candidate / trusted layering and non-physical estimate rejection
- PMSM sensorless observer:
  - `L * di/dt` compensation
  - low-BEMF invalidation behavior
  - consecutive-sample handoff / lock / unlock behavior
  - validation of `min_bemf < lock_bemf` and open-loop voltage ramp constraints
- Initial Q31 path: helpers, Clarke/Park/InvPark, SVPWM, `2/3-shunt` reconstruction, and top-level support-matrix gating

### Current Limits

- The repository does not currently provide a separate `CMock` mock suite; isolation mainly comes from lightweight fixtures and API-level regressions
- Toolchain compatibility, board-level ADC/PWM integration, and hardware-in-the-loop are outside the current unit-test scope
