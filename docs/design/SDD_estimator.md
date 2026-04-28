# Software Detailed Design - Estimator / 软件详细设计 - 估算层

## 中文

- 霍尔、编码器、旋变、无感观测器

### 无感观测器

- 当前无感 PMSM FOC 路径为首版基线实现。
- 观测器输入使用上一个控制周期的 `v_ab` 与当前重构 `i_ab`，按 `e = v - R * i - L * di/dt` 估计反电势。
- 反电势经一阶低通后，通过 `atan2f(...)` 形成测量角度。
- 角度测量再进入离散 PLL：
  - `pll_kp` 用于比例校正
  - `pll_ki` 用于积分校正
- 开环启动阶段：
  - `pll_speed_rad_s` 按 `startup_accel_rad_s2` 加速，直到 `startup_speed_rad_s`
  - `open_loop_voltage` 在 `open_loop_voltage_start` 到 `open_loop_voltage_max` 之间按速度比例线性上升
  - `mc_fast_step()` 在 `open_loop_active` 期间会把 FOC 输出电压模长限制到 `open_loop_voltage`
- 当反电势幅值低于 `min_bemf` 时，观测器输出无效并清零速度。
- 若此时已经离开开环，当前实现还会同步清零 PLL 速度/积分状态，并复位 lock/unlock debounce 计数器，避免沿用过期闭环状态。
- 开环启动阶段只有在连续 2 个样本同时满足 `bemf_magnitude >= lock_bemf` 且角度误差小于当前实现阈值 `0.5 rad` 时，才退出 `open_loop_active`。
- 闭环 `pll_locked` 同样采用连续 2 个样本 debounce：角度误差连续小于 `0.35 rad` 才重新锁定，连续超出阈值才解锁，避免单样本抖动反复翻转。
- 当前配置入口为 `mc_system_cfg_t.sensor.sensorless_cfg`，已开放：
  - `bemf_filter_alpha`
  - `min_bemf`
  - `pll_kp`
  - `pll_ki`
  - `lock_bemf`
  - `startup_speed_rad_s`
  - `startup_accel_rad_s2`
  - `open_loop_voltage_start`
  - `open_loop_voltage_max`
- `rs_ohm`、`ls_h`、`pole_pairs` 由 `mc_init()` 从电机参数自动注入；其中 `ls_h` 当前取 `(Ld + Lq) / 2`。
- 当 `sensorless_cfg` 留空或给出非正值时，`mc_init()` 当前会注入保守默认值后再调用 `mc_sensorless_init()`：
  - `bemf_filter_alpha = 0.2`
  - `min_bemf = 0.01`
  - `pll_kp = 400`
  - `pll_ki = 20000`
  - `lock_bemf = 2 * min_bemf`
  - `startup_speed_rad_s = 80`
  - `startup_accel_rad_s2 = 4000`
  - `open_loop_voltage_max = 0.15 * voltage_limit`
  - `open_loop_voltage_start < 0` 时夹到 `0`
- 当前调参边界建议：
  - `min_bemf` 应低于 `lock_bemf`，否则观测器可能在准备切出开环前先被判无效。
  - `bemf_filter_alpha` 过小会增加相位滞后并推迟锁定，过大则会放大噪声与 lock/unlock 抖动。
  - `pll_kp` / `pll_ki` 过大时会更快跟踪，但更容易在噪声、参数失配或低速弱反电势下出现 chatter。
  - `startup_speed_rad_s` / `startup_accel_rad_s2` / 开环电压斜坡需要与电机反电势建立速度匹配；过低会迟迟无法满足 `lock_bemf`，过高会增大切换时角误差。
- 上述 `min_bemf < lock_bemf` 以及开环电压斜坡不倒挂的约束，当前已在 `mc_sensorless_init()` 中做参数校验。

### SMO (滑模观测器)

Sliding Mode Observer 是 LPF+PLL 观测器的生产级替代方案。通过滑模符号函数驱动电流误差收敛，提取反电势后经 PLL 计算位置。

**文件**: `src/estimator/mc_sensor_smo.c`, `include/mc_sensor_smo.h`

**算法步骤**:

```
1. SMO 电流观测器:
   i_err = i_hat - i_measured
   z = k_slide * sign(i_err)                         // 滑模校正项
   di_hat/dt = (v - Rs*i_hat + z) / Ls               // 前向 Euler 离散化

2. 反电势提取 (一阶 LPF):
   bemf = LPF(z, lpf_alpha)

3. PLL 锁相:
   bemf_mag = |bemf_ab|
   if bemf_mag < min_bemf: observer_valid = FALSE
   measured_angle = atan2(bemf_beta, bemf_alpha) - π/2
   angle_error = measured_angle - elec_angle
   speed = PLL_integrator + pll_kp * angle_error
   PLL_integrator += pll_ki * angle_error * dt
   elec_angle += speed * dt

4. 开环启动: 同上 (V/f 斜坡 → lock_bemf handoff)
```

**调参指南**:

| 参数 | 范围建议 | 说明 |
|---|---|---|
| `k_slide` | 1.5~3.0 × 最大 BEMF | 过小则收敛慢，过大则抖振加剧 |
| `lpf_alpha` | 0.3~0.7 | 过低延迟大，过高滤波不足 |
| `pll_kp` | 100~800 | 同上 |
| `pll_ki` | 5000~50000 | 同上 |

**与 LPF+PLL 的对比**:

| 特性 | LPF+PLL | SMO |
|---|---|---|
| 噪声免疫 | 中（LPF 直接过滤测量 BEMF） | 高（滑模切换天然滤除高频噪声） |
| 相位滞后 | 较大（LPF 延迟） | 小（LPF 仅过滤 z 信号，信号路径短） |
| 参数鲁棒性 | 取决于 LPF α | 取决于 k_slide 余量 |
| 计算开销 | 低 | 中（多一次电流观测器积分） |
| 收敛速度 | 慢于 SMO | 快（滑模切换快速收敛） |

**模式选择**: `MC_MODE_PMSM_FOC_SMO` 在 API 层使用 `mc_api.c` 中的 SMO 专用路径。当前 SMO 与 LPF+PLL 共享 `mc_system_cfg_t.sensor.sensorless_cfg` 配置入口，API 层在 init 时自动将 sensorless 配置映射为 SMO 参数（k_slide 默认取 3×lock_bemf，lpf_alpha 取 0.5）。

### 后续增强方向

- 扩展反电势观测器 (model-based EEMF)
- 启动阶段开环/半闭环切换
- 低速区锁相与有效性诊断增强

## English

- Hall, encoder, resolver, and sensorless observers

### Sensorless Observer

- The current sensorless PMSM FOC path is a first baseline implementation.
- The observer uses previous-cycle `v_ab` together with current reconstructed `i_ab` and estimates back-EMF with `e = v - R * i - L * di/dt`.
- Back-EMF is filtered by a first-order low-pass filter, and `atan2f(...)` provides the measured electrical angle.
- The measured angle is then tracked by a discrete PLL using:
  - `pll_kp` for proportional correction
  - `pll_ki` for integral correction
- During open-loop startup:
  - `pll_speed_rad_s` ramps with `startup_accel_rad_s2` until `startup_speed_rad_s`
  - `open_loop_voltage` rises linearly from `open_loop_voltage_start` to `open_loop_voltage_max`
  - `mc_fast_step()` limits the applied FOC voltage magnitude to `open_loop_voltage` while `open_loop_active`
- When back-EMF magnitude falls below `min_bemf`, the observer is marked invalid and speed is cleared to zero.
- If this happens after open-loop handoff, the current implementation also clears PLL speed/integrator state and resets lock/unlock debounce counters so closed-loop state is not reused after invalidation.
- Open-loop startup exits only after 2 consecutive samples satisfy both `bemf_magnitude >= lock_bemf` and the current implementation's `0.5 rad` bounded angle-error condition.
- Closed-loop `pll_locked` also uses a 2-sample debounce: in-threshold samples below `0.35 rad` relock, and repeated out-of-threshold samples unlock.
- The current configuration entry point is `mc_system_cfg_t.sensor.sensorless_cfg`, which exposes:
  - `bemf_filter_alpha`
  - `min_bemf`
  - `pll_kp`
  - `pll_ki`
  - `lock_bemf`
  - `startup_speed_rad_s`
  - `startup_accel_rad_s2`
  - `open_loop_voltage_start`
  - `open_loop_voltage_max`
- `rs_ohm`, `ls_h`, and `pole_pairs` are injected automatically by `mc_init()` from motor parameters; `ls_h` currently uses `(Ld + Lq) / 2`.
- When `sensorless_cfg` leaves fields unset or non-positive, `mc_init()` currently injects conservative defaults before calling `mc_sensorless_init()`:
  - `bemf_filter_alpha = 0.2`
  - `min_bemf = 0.01`
  - `pll_kp = 400`
  - `pll_ki = 20000`
  - `lock_bemf = 2 * min_bemf`
  - `startup_speed_rad_s = 80`
  - `startup_accel_rad_s2 = 4000`
  - `open_loop_voltage_max = 0.15 * voltage_limit`
  - negative `open_loop_voltage_start` is clamped to `0`
- Current tuning guidance:
  - Keep `min_bemf` below `lock_bemf`, or the observer can invalidate before closed-loop handoff completes.
  - Very small `bemf_filter_alpha` adds phase lag and slows lock acquisition; very large values pass more noise and can increase lock chatter.
  - Larger `pll_kp` / `pll_ki` improve response speed but reduce robustness to noise, parameter mismatch, and weak low-speed back-EMF.
  - `startup_speed_rad_s`, `startup_accel_rad_s2`, and the open-loop voltage ramp should be coordinated with the motor's actual back-EMF build-up; overly aggressive values can increase handoff angle error.
- The `min_bemf < lock_bemf` relationship and the non-inverted open-loop voltage ramp constraints are now enforced by `mc_sensorless_init()`.

### Planned Enhancements

- add SMO or extended back-EMF observer variants
- add open-loop / semi-closed-loop startup handoff
- improve low-speed lock and observer validity diagnostics
