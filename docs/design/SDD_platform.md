# Software Detailed Design - Platform / 软件详细设计 - 平台层

## 中文

- 编译器兼容宏
- 架构特征开关
- benchmark 抽象

### 1-shunt ADC 触发映射

- 平台层将单电阻采样触发拆分为三层数据模型：
  - 算法层：`mc_adc_trigger_plan_t`
  - 硬件映射层：`mc_adc_hw_trigger_map_t`
  - 参考板级应用层：`mc_adc_ref_apply_cfg_t`
- 算法层由 FOC/重构模块生成，表达“采几次、在哪个 PWM 半周期采样、相对位置与绝对时间”。
- 硬件映射层由 `mc_adc_map_trigger_plan(...)` 生成，表达“使用哪个 compare 槽、哪个 ADC SOC 槽、由 PWM 上/下计数事件触发”。
- 参考板级应用层由 `mc_adc_ref_build_apply_cfg(...)` 生成，表达“需要真正下发到板级驱动的两条 SOC 配置记录”。

### 板级集成步骤

- 板级 `adc_trigger_apply(...)` 建议按以下顺序工作：
  - 调用 `mc_adc_map_trigger_plan(...)`
  - 调用 `mc_adc_ref_build_apply_cfg(...)`
  - 根据 `soc_a` / `soc_b` 的 `enable` 状态配置或关闭 ADC 触发
- `compare_index` 建议映射到定时器 compare 通道编号。
- `soc_index` 建议映射到 ADC 注入队列或 SOC A/B 槽位编号。
- `event` 建议映射到：
  - `MC_ADC_TRIGGER_EVENT_PWM_UP` -> PWM 上计数匹配事件
  - `MC_ADC_TRIGGER_EVENT_PWM_DOWN` -> PWM 下计数匹配事件
- `compare_time_s` 可直接用于：
  - 先按 PWM 时钟频率换算 compare tick
  - 再写入定时器 compare 寄存器
- 若目标 MCU 只能接受归一化位置而不是秒级时间，可改用 `compare_position` 或由 `time_s / pwm_period_s` 反算。

### 推荐约定

- `soc_a` 优先绑定第一采样点，`soc_b` 绑定第二采样点。
- `count == 1` 时，不要求第二个 SOC 槽无效，但板级可选择：
  - 保留第二槽用于调试/镜像触发
  - 或在 `enable == MC_FALSE` 时关闭第二槽
- `reorder_applied == MC_TRUE` 时，板级不应再自行交换采样顺序，避免与算法层重复重排。
- 若上位诊断或标定任务需要感知单电阻补偿模式，可通过 `mc_fast_output_t.current_comp_status` 获取当前周期状态，或通过 `mc_diag_status_t.current_comp_status` 获取最近一次快环持久化快照。

### 参考适配对象

- Cortex-M 高级定时器 + injected ADC
- Cortex-M 通用定时器 + regular ADC external trigger
- C2000 ePWM + ADC SOC A/B
- R5F/R52 GPT/EPWM + ADC queue

### ADC 校准（mc_adc_calibrate_linear）

- 函数：`mc_status_t mc_adc_calibrate_linear(mc_adc_raw_t raw1, mc_f32_t phys1, mc_adc_raw_t raw2, mc_f32_t phys2, mc_adc_cal_t *cal)`
- 声明在 `mc_port_adc.h`，实现在 `src/api/mc_adc_calibrate.c`
- 原理：两点校准法，已知两个校准点 `(raw1, phys1)` 和 `(raw2, phys2)`，计算线性系数
- 公式：`scale = (phys2 - phys1) / (raw2 - raw1)`，`offset = phys1 - scale * raw1`
- 输出：`mc_adc_cal_t` 包含 `scale` 和 `offset`
- 转换：`physical = raw * scale + offset`
- 典型用途：电流采样通道、电压采样通道的增益和偏置校准
- 板级集成：通常在初始化阶段调用，存储校准系数，然后在快环中应用

## English

- Compiler compatibility macros
- Architecture feature switches
- Benchmark abstraction

### ADC Calibration (`mc_adc_calibrate_linear`)

- Function: `mc_status_t mc_adc_calibrate_linear(mc_adc_raw_t raw1, mc_f32_t phys1, mc_adc_raw_t raw2, mc_f32_t phys2, mc_adc_cal_t *cal)`
- Declared in `mc_port_adc.h`, implemented in `src/api/mc_adc_calibrate.c`
- Principle: Two-point calibration, given known points `(raw1, phys1)` and `(raw2, phys2)`, compute linear coefficients
- Formula: `scale = (phys2 - phys1) / (raw2 - raw1)`, `offset = phys1 - scale * raw1`
- Output: `mc_adc_cal_t` containing `scale` and `offset`
- Conversion: `physical = raw * scale + offset`
- Typical usage: Gain and offset calibration for current and voltage sense channels
- Board integration: Typically called at init to store calibration coefficients, then applied in fast loop

### 1-shunt ADC Trigger Mapping

- The platform layer splits single-shunt trigger handling into three data models:
  - Algorithm layer: `mc_adc_trigger_plan_t`
  - Hardware mapping layer: `mc_adc_hw_trigger_map_t`
  - Reference board-apply layer: `mc_adc_ref_apply_cfg_t`
- The algorithm layer is produced by the FOC/reconstruction path and describes sample count, PWM half-cycle, normalized position, and absolute time.
- The hardware mapping layer is produced by `mc_adc_map_trigger_plan(...)` and describes compare slot, ADC SOC slot, and PWM up/down trigger event.
- The reference board-apply layer is produced by `mc_adc_ref_build_apply_cfg(...)` and describes the final pair of SOC configuration records for board code.

### Board Integration Flow

- A board-level `adc_trigger_apply(...)` implementation is expected to:
  - call `mc_adc_map_trigger_plan(...)`
  - call `mc_adc_ref_build_apply_cfg(...)`
  - enable or disable ADC trigger slots based on `soc_a` / `soc_b`
- `compare_index` should map to a timer compare channel.
- `soc_index` should map to an ADC injected queue slot or SOC A/B slot.
- `event` should map to:
  - `MC_ADC_TRIGGER_EVENT_PWM_UP` -> PWM up-count compare event
  - `MC_ADC_TRIGGER_EVENT_PWM_DOWN` -> PWM down-count compare event
- `compare_time_s` can be converted into compare ticks using the local PWM timer clock.
- If the target MCU consumes normalized positions instead of seconds, board code may use normalized position data instead.

### Recommended Conventions

- Bind `soc_a` to the first sample point and `soc_b` to the second sample point.
- When `count == 1`, board code may either:
  - keep the second slot for diagnostics or mirrored triggering
  - or disable it when `enable == MC_FALSE`
- When `reorder_applied == MC_TRUE`, board code should not re-swap the trigger order.
- If higher-level diagnostics or calibration tooling needs visibility into the active single-shunt compensation mode, it can use `mc_fast_output_t.current_comp_status` for the current cycle or `mc_diag_status_t.current_comp_status` for the latest persisted fast-loop snapshot.

### Reference Targets

- Cortex-M advanced timer + injected ADC
- Cortex-M general timer + regular ADC external trigger
- C2000 ePWM + ADC SOC A/B
- R5F/R52 GPT/EPWM + ADC queue

### Debug Transport Integration / 调试传输层集成

调试子系统通过分离的平台钩子 `mc_debug_port_t` 支持三个物理传输层。

**文件**: `include/mc_port_debug.h`

**钩子结构**:

| Hook | 方向 | 职责 |
|---|---|---|
| `uart_tx(data, len)` | MCU→PC | 发送数据块到 UART |
| `uart_rx_available()` | PC→MCU | 查询 UART RX FIFO 中的可用字节数 |
| `uart_rx_read()` | PC→MCU | 从 UART RX FIFO 读取一个字节 |
| `can_tx(id, data, len)` | MCU→PC | 发送 CAN 帧（每帧 ≤ 8 字节；多帧由传输层拼接） |
| `lin_tx(data, len)` | MCU→PC | 发送 LIN 帧（帧格式同 UART，服从 LIN Schedule Table） |

**帧边界策略**:

| 传输层 | 帧边界 | 最大帧长 | 典型速率 |
|---|---|---|---|
| UART | 魔术字节 `0xDA 0x1A` + 2 字节 little-endian 长度 | 256 字节 | 115200–3M baud |
| CAN | 标准帧 8 字节/帧；首帧传输 total_len；多帧拼接 | 256 字节 | 1 Mbps |
| LIN | 同 UART（本质是串行）；服从 LIN Schedule Table | 256 字节 | 20 kbps |

**板级集成步骤**:

- 板级 `uart_rx_available()` 和 `uart_rx_read()` 在 UART RX ISR 中调用 `mc_debug_fm_feed_rx()`
- 板级 `uart_tx()` 由传输层的 `tx_flush` 回调调用，可直接使用 DMA 或中断
- 所有钩子均可为空—相应的传输层被静默禁用

**参考适配对象**:

- Cortex-M UART/USART + DMA
- Cortex-M CAN/FDCAN 外设（标准帧）用于调试
- Cortex-M UART 用于 LIN（通过 LIN 收发器）
