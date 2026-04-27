# Software Detailed Design - Reconstruct / 软件详细设计 - 重构层

## 中文

- 3-shunt：`mc_reconstruct_3shunt_run(...)` 基于每相 `offset + scale` 直接重构三相电流
- 2-shunt：`mc_reconstruct_2shunt_run(...)` 重构 A/B 相并按 `ia + ib + ic = 0` 推导 C 相
- 1-shunt：由 `mc_reconstruct_1shunt_plan(...)` 先生成采样窗口与元数据，再由 `mc_reconstruct_1shunt_run(...)` 结合 ADC 原始值完成三相重构
- `mc_1shunt_meta_t` 当前保存：
  - 有效窗口数量与时序位置
  - 采样沿/半周信息
  - zero-vector bias / 相位重排需求
  - 是否需要预测补偿
- 当前 `PMSM FOC` 快环、`identify` 路径和 `PMSM sensorless` 输入路径都复用同一套 `1/2/3-shunt` 重构逻辑
- Q31 当前仅提供 `2-shunt` / `3-shunt` 重构实现；顶层 Q31 支持矩阵仍限制在 `PMSM FOC + Hall/Encoder + 2/3-shunt`

## English

- 3-shunt: `mc_reconstruct_3shunt_run(...)` directly reconstructs all three phase currents from per-phase `offset + scale`
- 2-shunt: `mc_reconstruct_2shunt_run(...)` reconstructs phases A/B and derives phase C from `ia + ib + ic = 0`
- 1-shunt: `mc_reconstruct_1shunt_plan(...)` first produces sampling windows and metadata, then `mc_reconstruct_1shunt_run(...)` reconstructs phase current from raw ADC data and that metadata
- `mc_1shunt_meta_t` currently stores:
  - valid window count and timing positions
  - sample-edge / half-cycle information
  - zero-vector bias and phase-reorder requirements
  - whether predictive compensation is required
- The current `PMSM FOC` fast loop, `identify` path, and `PMSM sensorless` input path all reuse the same configured `1/2/3-shunt` reconstruction logic
- Q31 reconstruction is currently provided only for `2-shunt` / `3-shunt`; the top-level Q31 support matrix remains limited to `PMSM FOC + Hall/Encoder + 2/3-shunt`
