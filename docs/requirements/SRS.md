# Software Requirements Specification / 软件需求规格说明

## 中文

### 编号规则

- 需求编号格式：`REQ-MC-XXXX`

### 当前范围需求

- `REQ-MC-0101` 库应支持 GCC、IAR、Keil、GHS 工具链。
- `REQ-MC-0301` 库应提供 PMSM FOC 控制主链；`float32` 路径当前覆盖 Hall、Encoder、Resolver 与首版 Sensorless。
- `REQ-MC-0302` 库应提供 API 级运行时诊断快照，至少包含 `1-shunt` 补偿状态与 PMSM sensorless 的有效/锁定/开环标志。
- `REQ-MC-0401` 库应提供电机参数辨识路径，可输出 `Rs`、`Ld`、`Lq`、`flux_wb`，并仅通过 public API 暴露 trusted 结果。
- `REQ-MC-0501` 库应支持单电阻、双电阻、三电阻采样方案。
- `REQ-MC-0502` `identify` 与 PMSM sensorless 路径应复用当前配置的电流重构链路，而不是使用单独硬编码缩放。
- `REQ-MC-0601` PMSM sensorless 路径应提供基线 `LPF + PLL` 观测器，包含开环启动与连续样本 lock/unlock 判据。
- `REQ-MC-0602` PMSM sensorless 初始化应校验 `min_bemf < lock_bemf` 以及开环电压斜坡不倒挂。
- `REQ-MC-0701` 当前顶层 `Q31` 执行路径应仅支持 `PMSM FOC + Hall/Encoder + 2-shunt/3-shunt`；其他组合不在当前支持范围内。
- `REQ-MC-0801` BLDC sensorless 路径应可通过顶层 `mc_start()` 公开启动。

## English

### Numbering Rule

- Requirement ID format: `REQ-MC-XXXX`

### Current Scoped Requirements

- `REQ-MC-0101` The library shall support GCC, IAR, Keil, and GHS toolchains.
- `REQ-MC-0301` The library shall provide a PMSM FOC control path; the current `float32` path covers Hall, encoder, resolver, and a first sensorless implementation.
- `REQ-MC-0302` The library shall provide an API-level runtime diagnostic snapshot including at least `1-shunt` compensation status and PMSM sensorless valid / locked / open-loop flags.
- `REQ-MC-0401` The library shall provide a motor-parameter identification path for `Rs`, `Ld`, `Lq`, and `flux_wb`, and shall expose trusted results only through the public API.
- `REQ-MC-0501` The library shall support 1-shunt, 2-shunt, and 3-shunt current sensing.
- `REQ-MC-0502` The `identify` and PMSM sensorless paths shall reuse the configured current reconstruction chain instead of using separate hard-coded scaling.
- `REQ-MC-0601` The PMSM sensorless path shall provide a baseline `LPF + PLL` observer with open-loop startup and consecutive-sample lock / unlock criteria.
- `REQ-MC-0602` PMSM sensorless initialization shall validate `min_bemf < lock_bemf` and a non-inverted open-loop voltage ramp.
- `REQ-MC-0701` The current top-level `Q31` execution path shall be limited to `PMSM FOC + Hall/Encoder + 2-shunt/3-shunt`; other combinations are outside the current supported scope.
- `REQ-MC-0801` The BLDC sensorless path shall be startable through the top-level `mc_start()` API.
