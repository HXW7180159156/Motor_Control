# Deviation Record / 偏离记录

## 中文 / Chinese

### MISRA C:2012 偏离列表

| 偏离 ID | 规则 | 描述 | 理由 | 范围 |
|---|---|---|---|---|
| DEV-001 | Dir 4.14 | 魔数 `0.0F`、`1.0F` 作为初始值/边界值保留在代码中 | 这些值代表"零"和"一"，意义自明且不会因不同上下文产生歧义。已在 `mc_constants.h` 中消除所有领域特异性魔数。 | 全库 |
| DEV-002 | Rule 13.4 | 部分配置值校验使用 `< 0.0F`、`<= 0.0F` 等浮点比较 | 这些比较用于检测"未配置"状态（默认值为 `0.0F`），是故意的边界值检测而非连续量比较。容差比较在此场景下不适用。 | `mc_api.c`, `mc_sensor_sensorless.c`, `mc_identify.c`, `mc_drive_bldc_sensorless.c` 的配置验证路径 |
| DEV-003 | Rule 11.3 | `void (*pwm_apply)(const void *cmd)` 使用 `const void *` 作为平台抽象 | 平台钩子函数需要接收不同平台的 PWM 数据结构，`const void *` 是 C 中实现接口抽象的惯用方式。调用侧保证类型安全。 | `mc_port.h` |
| DEV-004 | Rule 10.8 | C99 复合字面量 `(mc_instance_t){0}` 用于结构体零初始化 | 这是 ISO C99 标准特性，比 `memset` 更安全（类型感知），且不会误清零 padding 导致未定义行为。MSVC/GCC/IAR/Keil/GHS 均支持。 | 全库 |
| DEV-005 | Rule 21.6 | 使用 `<math.h>` 中的 `sqrtf`、`sinf`、`cosf`、`atan2f`、`fabsf`、`ceilf`、`isfinite` | 这些是 ISO C 标准库函数，在 ARM CMSIS-DSP 或编译器提供的安全认证数学库中有认证等价替代。当前阶段使用标准库作为原型实现，量产阶段将替换为认证数学库。 | `mc_sensor_sensorless.c`, `mc_drive_pmsm.c`, `mc_control_svpwm.c`, `mc_identify.c` |
| DEV-006 | Rule 15.7 | `mc_identify_run` 中 `switch` 的某些 `case` 以 `break` 结束而非 `if..else if` 无 default 默认 | 辨识状态机枚举已严格定义所有可能状态，default 不可能到达。`break` 语法符合 MISRA 单出口原则。 | `mc_identify.c` |

### 非 MISRA 设计偏离

| 偏离 ID | 描述 | 理由 |
|---|---|---|
| DEV-D01 | `mc_instance_t` 为大结构体（包含所有子系统状态） | 实现简单性和确定性内存布局优先于"按需分配"。静态分配避免了动态内存的复杂性。 |
| DEV-D02 | 辨识中的 flux_wb 估计采用开环电压平衡法而非逆变器非线性补偿 | 这是首版实现。量产级方案需要逆变器死区补偿和饱和效应建模。 |

---

## English / 英文

### MISRA C:2012 Deviation List

| Dev ID | Rule | Description | Justification | Scope |
|---|---|---|---|---|
| DEV-001 | Dir 4.14 | `0.0F` / `1.0F` retained as initial/boundary values | These represent "zero" and "one" with self-evident semantics. All domain-specific magic numbers eliminated via `mc_constants.h`. | Library-wide |
| DEV-002 | Rule 13.4 | Float comparisons `< 0.0F` / `<= 0.0F` in config validation paths | These detect "unconfigured" states (default is `0.0F`); tolerance-based comparison is inappropriate for boundary detection. | Config validation in `mc_api.c`, `mc_sensor_sensorless.c`, `mc_identify.c`, `mc_drive_bldc_sensorless.c` |
| DEV-003 | Rule 11.3 | `void (*pwm_apply)(const void *cmd)` uses `const void *` for platform abstraction | Platform hooks must accept different PWM data structures; `const void *` is the idiomatic C abstraction mechanism. Type safety ensured at call site. | `mc_port.h` |
| DEV-004 | Rule 10.8 | C99 compound literals `(mc_instance_t){0}` for struct zero-init | ISO C99 standard feature; safer than `memset` (type-aware), avoids undefined behavior from padding zeroing. Supported by MSVC/GCC/IAR/Keil/GHS. | Library-wide |
| DEV-005 | Rule 21.6 | `<math.h>` functions: `sqrtf`, `sinf`, `cosf`, `atan2f`, `fabsf`, `ceilf`, `isfinite` | ISO C standard library; certified equivalents exist in ARM CMSIS-DSP or safety-certified math libraries. Prototype phase uses standard library; production will replace with certified math. | `mc_sensor_sensorless.c`, `mc_drive_pmsm.c`, `mc_control_svpwm.c`, `mc_identify.c` |
| DEV-006 | Rule 15.7 | `switch` in `mc_identify_run` ends cases with `break`; no unreachable `default` | Identify state machine enum strictly defines all states; `default` is unreachable. `break` follows MISRA single-exit principle. | `mc_identify.c` |

### Non-MISRA Design Deviations

| Dev ID | Description | Justification |
|---|---|---|
| DEV-D01 | `mc_instance_t` is a large struct containing all subsystem states | Simplicity of implementation and deterministic memory layout prioritized over per-mode allocation. Static allocation avoids dynamic memory complexity. |
| DEV-D02 | Identify flux_wb uses open-loop voltage-balance method without inverter non-linearity compensation | First prototype implementation. Production-grade solution requires dead-time compensation and saturation modeling. |
