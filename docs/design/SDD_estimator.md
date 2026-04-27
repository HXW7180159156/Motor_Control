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

### 后续增强方向

- SMO 或扩展反电势观测器
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
