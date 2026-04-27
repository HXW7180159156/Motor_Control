# Software Architecture Design / 软件架构设计

## 中文

### 分层

1. `platform`
   - 板级抽象层，承接 PWM、ADC、fault、resolver 等端口与 trigger map。
2. `math`
   - 通用数值与基础数学能力，如 clamp、wrap、LPF、`float32/Q31` helper。
3. `control`
   - Clarke/Park/InvPark、PI、SVPWM 以及外环速度控制。
4. `estimator`
   - Hall、Encoder、Resolver、PMSM sensorless 观测器。
5. `reconstruct`
   - `1/2/3-shunt` 电流重构与 `1-shunt` 采样窗口元数据。
6. `drive`
   - PMSM FOC 驱动、BLDC Hall 六步换相、BLDC sensorless 启动/换相。
7. `api`
   - 顶层实例生命周期、模式约束、fast/medium/slow 调度、诊断快照与 `identify` 编排。

### 当前主链

- PMSM FOC：`api -> estimator + control + drive + reconstruct -> platform`
- PMSM identify：`api -> reconstruct -> identify state machine -> SVPWM/PWM -> platform`
- BLDC Hall：`api -> estimator(hall) + drive -> platform`
- BLDC sensorless：`api -> drive(sensorless) -> platform`，其中 `mc_medium_step()` 可对 `duty_cmd` 运行速度 PI

### 当前架构约束

- 顶层 `MC_NUMERIC_Q31` 当前仅支持 `PMSM FOC + Hall/Encoder + 2/3-shunt`
- `mc_set_mode()` 在初始化完成后保持保守单模式策略，不支持任意 runtime 切模态
- `identify` 的 raw candidate 保持在实现内部，穿过 API 边界的仅是 trusted 结果

## English

### Layers

1. `platform`
   - board abstraction for PWM, ADC, fault, resolver, and trigger-map hooks.
2. `math`
   - common numeric utilities such as clamp, wrap, LPF, and `float32/Q31` helpers.
3. `control`
   - Clarke/Park/InvPark, PI control, SVPWM, and outer-loop speed control.
4. `estimator`
   - Hall, encoder, resolver, and PMSM sensorless observers.
5. `reconstruct`
   - `1/2/3-shunt` current reconstruction and `1-shunt` sampling metadata.
6. `drive`
   - PMSM FOC drive, BLDC Hall six-step commutation, and BLDC sensorless startup/commutation.
7. `api`
   - top-level lifecycle, mode constraints, fast/medium/slow scheduling, diagnostics, and `identify` orchestration.

### Current Main Paths

- PMSM FOC: `api -> estimator + control + drive + reconstruct -> platform`
- PMSM identify: `api -> reconstruct -> identify state machine -> SVPWM/PWM -> platform`
- BLDC Hall: `api -> estimator(hall) + drive -> platform`
- BLDC sensorless: `api -> drive(sensorless) -> platform`, with `mc_medium_step()` able to run duty-based speed PI

### Current Architectural Constraints

- Top-level `MC_NUMERIC_Q31` is currently limited to `PMSM FOC + Hall/Encoder + 2/3-shunt`
- `mc_set_mode()` keeps a conservative single-mode policy after initialization and does not support arbitrary runtime mode switching
- `identify` raw candidates remain internal implementation details; only trusted results cross the API boundary
