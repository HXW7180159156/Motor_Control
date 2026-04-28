# Software Detailed Design - Math / 软件详细设计 - 数学层

## 中文

### 数值类型与精度

| 类型 | C 类型 | 范围 | 精度 | 用途 |
|---|---|---|---|---|
| `mc_f32_t` | `float` | IEEE 754 single | ~7 位十进制 | 全库 float32 路径 |
| `mc_q31_t` | `int32_t` | [-1.0, 1.0) | 2^-31 ≈ 4.66e-10 | 定点 PID、变换、重构 |
| `mc_q15_t` | `int16_t` | [-1.0, 1.0) | 2^-15 ≈ 3.05e-5 | 保留（未当前使用） |
| `mc_bool_t` | `uint8_t` | {0, 1} | — | 布尔逻辑（避免 C99 _Bool 对齐不一致） |

### clamp_f32 / clamp_q31

- 接口：`mc_math_clamp_f32(value, min, max)` / `mc_math_clamp_q31(value, min, max)`
- 算法：if-else 分支比较，无浮点容差
- 用途：全库统一的钳位原语，消除 5 处重复实现
- 注意：调用方保证 `min <= max`，否则结果不可预测

### Q31 转换与饱和算术

`mc_q31_from_f32(v)` / `mc_q31_to_f32(q)` — 有损但可逆的 float↔Q31 转换。

伪代码：
```
Q31_FROM_F32(v):
    clamped = CLAMP(v, -1.0, 1.0)
    if clamped >= 1.0 → return INT32_MAX       // 饱和
    return (int32_t)(clamped * 2^31)

Q31_TO_F32(q):
    return (float)q / 2^31
```

`mc_q31_add_sat(a, b)` — 饱和加法：
```
SUM = (int64_t)a + (int64_t)b
if SUM > INT32_MAX → INT32_MAX
if SUM < INT32_MIN → INT32_MIN
return (mc_q31_t)SUM
```

`mc_q31_mul(a, b)` — 饱和乘法：
```
PRODUCT = (int64_t)a * (int64_t)b >> 31
if PRODUCT > INT32_MAX → INT32_MAX
if PRODUCT < INT32_MIN → INT32_MIN
return (mc_q31_t)PRODUCT
```

### 角度包装 (wrap_angle_rad)

伪代码：
```
WRAP_ANGLE(angle):
    while angle >  π: angle -= 2π
    while angle < -π: angle += 2π
    return angle
```

- 输入：任意有限角度 [rad]
- 输出：[-π, π]
- WCET：与输入成正比（不适用于无界输入）

### 一阶低通滤波器 (lpf_f32)

伪代码：
```
LPF(prev, input, alpha):
    return prev + alpha * (input - prev)
```

- α=0：保持（无更新）
- α=1：直通（无滤波）
- α=0.5：半衰

## English

### Numeric Types & Precision

| Type | C type | Range | Precision | Use |
|---|---|---|---|---|
| `mc_f32_t` | `float` | IEEE 754 single | ~7 decimal digits | Full library float32 path |
| `mc_q31_t` | `int32_t` | [-1.0, 1.0) | 2^-31 ≈ 4.66e-10 | Fixed-point PID, transforms, reconstruction |
| `mc_q15_t` | `int16_t` | [-1.0, 1.0) | 2^-15 ≈ 3.05e-5 | Reserved (not currently used) |
| `mc_bool_t` | `uint8_t` | {0, 1} | — | Boolean logic (avoids C99 _Bool alignment inconsistency) |

### clamp_f32 / clamp_q31

- Interface: `mc_math_clamp_f32(value, min, max)` / `mc_math_clamp_q31(value, min, max)`
- Algorithm: if-else branch comparison, no float tolerance
- Purpose: Unified clamp primitive used library-wide, eliminating 5 duplicate implementations
- Note: Caller must guarantee `min <= max`, otherwise behavior is undefined

### Q31 Conversion & Saturating Arithmetic

`mc_q31_from_f32(v)` / `mc_q31_to_f32(q)` — lossy but invertible float↔Q31 conversion.

Pseudo-code:
```
Q31_FROM_F32(v):
    clamped = CLAMP(v, -1.0, 1.0)
    if clamped >= 1.0 → return INT32_MAX
    return (int32_t)(clamped * 2^31)

Q31_TO_F32(q):
    return (float)q / 2^31
```

`mc_q31_add_sat(a, b)` — saturating addition:
```
SUM = (int64_t)a + (int64_t)b
if SUM > INT32_MAX → INT32_MAX
if SUM < INT32_MIN → INT32_MIN
return (mc_q31_t)SUM
```

`mc_q31_mul(a, b)` — saturating multiplication:
```
PRODUCT = (int64_t)a * (int64_t)b >> 31
if PRODUCT > INT32_MAX → INT32_MAX
if PRODUCT < INT32_MIN → INT32_MIN
return (mc_q31_t)PRODUCT
```

### Angle Wrapping (wrap_angle_rad)

Pseudo-code:
```
WRAP_ANGLE(angle):
    while angle >  π: angle -= 2π
    while angle < -π: angle += 2π
    return angle
```

- Input: any finite angle [rad]
- Output: [-π, π]
- WCET: proportional to input magnitude (unbounded inputs not suitable)

### First-Order Low-Pass Filter (lpf_f32)

Pseudo-code:
```
LPF(prev, input, alpha):
    return prev + alpha * (input - prev)
```

- α=0: hold (no update)
- α=1: passthrough (no filtering)
- α=0.5: half-life decay
