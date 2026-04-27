# Vector Test Specification / 向量测试规格说明

## 中文

- 当前向量/数值一致性检查复用 `test_mc_api`，尚未拆成独立 vector corpus 或 golden-data 工具链

### 当前覆盖范围

- `mc_q31_from_f32` / `mc_q31_to_f32` / 饱和加法 / 乘法 等 helper
- `Clarke`、`Park`、`InvPark` 的 `float32` / `Q31` 基本一致性与 round-trip
- `SVPWM` 在 `float32` / `Q31` 下的有效占空比输出
- `2-shunt` / `3-shunt` 的 Q31 电流重构
- 顶层 `Q31` 支持矩阵：
  - 允许 `PMSM FOC + Hall/Encoder + 2-shunt/3-shunt`
  - 拒绝当前未支持组合

### 当前限制

- 目前没有独立的大规模离线向量集、MATLAB/NumPy golden reference 或误差统计报告
- `1-shunt` 顶层 Q31 路径当前不在支持矩阵内

## English

- Current vector / numeric-consistency checks are embedded in `test_mc_api`; there is no separate vector corpus or golden-data toolchain yet

### Current Coverage

- `mc_q31_from_f32` / `mc_q31_to_f32` / saturating add / multiply helpers
- Basic `float32` / `Q31` consistency and round-trip checks for `Clarke`, `Park`, and `InvPark`
- Valid duty generation from `SVPWM` in both `float32` and `Q31`
- Q31 current reconstruction for `2-shunt` / `3-shunt`
- Top-level `Q31` support matrix:
  - allowed: `PMSM FOC + Hall/Encoder + 2-shunt/3-shunt`
  - rejected: currently unsupported combinations

### Current Limits

- There is currently no standalone large offline vector set, MATLAB/NumPy golden reference, or formal error-statistics report
- The top-level `1-shunt` Q31 path is not in the current supported matrix
