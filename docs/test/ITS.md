# Integration Test Specification / 集成测试规格说明

## 中文

- 当前集成测试风格用例复用 Unity 可执行文件 `test_mc_api`，尚未拆成独立 integration test target

### 当前覆盖范围

- 顶层 API 生命周期：`mc_init`、`mc_start`、`mc_stop`、`mc_reset`、`mc_set_mode`
- PMSM FOC 端到端路径：Encoder、Hall、Resolver、Sensorless
- BLDC 端到端路径：Hall 六步换相、Sensorless public start path 与 medium-step 速度 PI
- `identify` 结果回写：
  - FOC runtime model 更新
  - sensorless runtime model 更新
  - public path 仅应用 trusted 结果，不应用 raw candidate
- `identify` 与 PMSM sensorless 共用 `1/2/3-shunt` 重构输入路径
- 诊断快照贯通：`mc_fast_step(...)`、`mc_get_diag(...)`、`mc_slow_step(...)`
- PWM/ADC trigger hook 调用路径

### 当前限制

- 当前没有独立板级集成、外设驱动联调或 HIL 测试规范
- 传感器与功率级时序仍以软件夹具输入为主，不覆盖真实中断/采样抖动环境

## English

- Current integration-style coverage is implemented inside the shared Unity executable `test_mc_api`; there is no separate integration-test target yet

### Current Coverage

- Top-level API lifecycle: `mc_init`, `mc_start`, `mc_stop`, `mc_reset`, and `mc_set_mode`
- End-to-end PMSM FOC paths: encoder, Hall, resolver, and sensorless
- End-to-end BLDC paths: Hall six-step commutation, sensorless public start path, and medium-step speed PI behavior
- `identify` result application:
  - FOC runtime model updates
  - sensorless runtime model updates
  - trusted public results are applied while raw candidates are not
- Shared `1/2/3-shunt` reconstruction input paths for `identify` and PMSM sensorless
- Diagnostic propagation through `mc_fast_step(...)`, `mc_get_diag(...)`, and `mc_slow_step(...)`
- PWM / ADC trigger hook invocation paths

### Current Limits

- There is currently no separate board-level integration, peripheral bring-up, or HIL test specification
- Sensor and power-stage timing is still exercised mainly through software fixtures rather than real interrupt / sampling jitter environments
