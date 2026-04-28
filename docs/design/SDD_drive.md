# Software Detailed Design - Drive / 软件详细设计 - 驱动层

## 中文

### 架构

驱动层是算法库最上层（仅次于 API 层），负责将控制算法、传感器估计、电流重构串联为完整的控制回路。

### BLDC Hall 六步换相

```
BLDC_HALL_RUN(drive, hall_code, duty_cmd, pwm):
    clamped = CLAMP(duty_cmd, drive.cfg.duty_min, drive.cfg.duty_max)
    根据 hall_code 映射扇区：
      hall=1→扇区1(A=H,B=L,C=OFF), hall=5→扇区2(A=H,C=L,B=OFF),
      hall=4→扇区3(B=H,A=OFF,C=L), hall=6→扇区4(B=H,A=L,C=OFF),
      hall=2→扇区5(C=H,A=L,B=OFF), hall=3→扇区6(C=H,A=OFF,B=L)
    drive.last_hall_code = hall_code
    drive.last_pwm_cmd = pwm
```

### BLDC Sensorless 启动与运行

**状态机**:

```
IDLE ──(mc_start)──→ ALIGN ──(timer≥align_time)──→ RAMP_OPEN
                                                          │
                                                    (freq≥ramp_end)
                                                          ↓
                        RUN ←──(zc_detect ── advance_angle_delay)──┤
```

**状态详解**:

| 状态 | 行为 | 换相来源 |
|---|---|---|
| `IDLE` | PWM 全零输出 | — |
| `ALIGN` | 固定第一扇区、固定 `align_duty`，持续 `align_time_s` | 定时 |
| `RAMP_OPEN` | 频率从 `ramp_start_freq` 斜坡到 `ramp_end_freq`，占空比同步斜坡 | 定时 (1/(freq×6)) |
| `RUN` | BEMF 过零检测驱动换相，支持去抖 | BEMF 过零 + advance_angle |

**BEMF 过零检测**:

```
DETECT_ZC(ss, floating_voltage, bus_voltage):
    threshold = bus_voltage * 0.5
    diff = floating_voltage - threshold

    if |diff| < bemf_threshold_v: return FALSE  // 欠幅 → 无效

    根据换相步奇偶确定过零方向：
      偶数步: crossing = (diff > 0)
      奇数步: crossing = (diff < 0)

    if zc_debounce_threshold == 0: return crossing  // 无去抖

    // 连续样本去抖
    if crossing: zc_debounce_cnt++, 达到阈值后返回 TRUE
    else: zc_debounce_cnt = 0, 返回 FALSE
```

**速度 PI 调节 (`mc_bldc_sensorless_speed_step`)**:

```
SPEED_STEP(ss, speed_ref, dt):
    if phase != RUN: reset PI, return
    error = speed_ref - ss.mech_speed_rpm
    duty_pi = PI_RUN(error, dt)
    ss.duty_cmd = CLAMP(duty_pi, ramp_start_duty, 0.95)
```

### PMSM FOC 驱动

**FOC 快速电流环** (`mc_pmsm_foc_run`):

```
PMSM_FOC_RUN(foc, in, out):
    // 1. 电流重构
    RECONSTRUCT_CURRENT(foc, in, &out.i_abc, &out.comp_status)

    // 2. Clarke + Park 变换
    CLARKE(out.i_abc, &out.i_ab)
    PARK(out.i_ab, sinθ, cosθ, &out.i_dq)

    // 3. 弱磁 id 修正
    if fw_enable:
        v_mag = sqrt(vd² + vq²)  // 上一周期限幅后电压
        fw_threshold = voltage_limit * fw_activation_ratio
        fw_error = fw_threshold - v_mag
        id_fw_adjustment += fw_ki * fw_error * dt  // PI 积分
        CLAMP(id_fw_adjustment, fw_min_id, 0)
        fw_output = fw_kp * fw_error + id_fw_adjustment
        CLAMP(fw_output, fw_min_id, 0)
        effective_id_ref = CLAMP(id_ref + fw_output, fw_min_id, ∞)

    // 4. 电流 PI
    out.v_dq.d = PI_RUN(id_pi, effective_id_ref - out.i_dq.d, dt)
    out.v_dq.q = PI_RUN(iq_pi, iq_ref - out.i_dq.q, dt)

    // 5. 电压限幅
    LIMIT_VOLTAGE(foc, &out.v_dq)

    // 6. InvPark
    IPARK(out.v_dq, sinθ, cosθ, &out.v_ab)

    // 7. 母线电压归一化
    NORMALIZE_VOLTAGE(foc, in, &out.v_ab)

    // 8. 1-shunt PWM 优化 + SVPWM
    if current_cfg.type == 1SHUNT:
        PLAN_1SHUNT_FROM_VOLTAGE(foc, &out.v_ab, &out.pwm_cmd)
        OPTIMIZE_1SHUNT_PWM(foc, &out.pwm_cmd)     // pass 1: 电压预览
    SVPWM_RUN(&out.v_ab, &svpwm_cfg, &out.pwm_cmd)
    if current_cfg.type == 1SHUNT:
        OPTIMIZE_1SHUNT_PWM(foc, &out.pwm_cmd)     // pass 2: 最终占空比

    // 9. ADC 触发计划
    PLAN_ADC_TRIGGER(foc, &shunt1_meta, &out.trigger_plan)

    // 10. 状态保存
    foc.i_dq = out.i_dq;  foc.v_dq = out.v_dq;  foc.i_abc_last = out.i_abc
```

**1-shunt PWM 优化** (`mc_pmsm_optimize_1shunt_pwm`):

```
OPTIMIZE_1SHUNT_PWM(foc, pwm):
    if current_cfg.type != 1SHUNT: return

    // 共模偏置
    shift = reorder_required ? 0.04 : 0.02
    pwm.common_mode_shift = zero_vector_bias_high ? shift : -shift

    // 相位重排
    if reorder_required:
        将活跃相设置为 PWM 模式
        两路不活跃相设置为 HIGH（高零矢量）或 LOW（低零矢量）
```

## English

### Architecture

The drive layer is the uppermost layer (just below API), responsible for orchestrating control algorithms, sensor estimates, and current reconstruction into a complete control loop.

### BLDC Hall Six-Step Commutation

```
BLDC_HALL_RUN(drive, hall_code, duty_cmd, pwm):
    clamped = CLAMP(duty_cmd, drive.cfg.duty_min, drive.cfg.duty_max)
    Map hall_code to sector:
      hall=1→sector1(A=H,B=L,C=OFF), hall=5→sector2(A=H,C=L,B=OFF),
      hall=4→sector3(B=H,A=OFF,C=L), hall=6→sector4(B=H,A=L,C=OFF),
      hall=2→sector5(C=H,A=L,B=OFF), hall=3→sector6(C=H,A=OFF,B=L)
    drive.last_hall_code = hall_code
    drive.last_pwm_cmd = pwm
```

### BLDC Sensorless Startup & Run

**State Machine**:

```
IDLE ──(mc_start)──→ ALIGN ──(timer≥align_time)──→ RAMP_OPEN
                                                          │
                                                    (freq≥ramp_end)
                                                          ↓
                        RUN ←──(zc_detect ── advance_angle_delay)──┤
```

**State Details**:

| State | Behavior | Commutation Source |
|---|---|---|
| `IDLE` | Zero PWM output | — |
| `ALIGN` | Fixed first sector, fixed `align_duty`, for `align_time_s` | Timer |
| `RAMP_OPEN` | Frequency ramps from `ramp_start_freq` to `ramp_end_freq`, duty follows same ramp | Timer (1/(freq×6)) |
| `RUN` | BEMF zero-crossing detection triggers commutation with debounce | BEMF ZC + advance_angle |

**BEMF Zero-Crossing Detection**:

```
DETECT_ZC(ss, floating_voltage, bus_voltage):
    threshold = bus_voltage * 0.5
    diff = floating_voltage - threshold

    if |diff| < bemf_threshold_v: return FALSE  // Below threshold → invalid

    Determine crossing direction by commutation step parity:
      even step: crossing = (diff > 0)
      odd step:  crossing = (diff < 0)

    if zc_debounce_threshold == 0: return crossing

    // Consecutive-sample debounce
    if crossing: zc_debounce_cnt++, return TRUE when threshold reached
    else: zc_debounce_cnt = 0, return FALSE
```

**Speed PI Regulation (`mc_bldc_sensorless_speed_step`)**:

```
SPEED_STEP(ss, speed_ref, dt):
    if phase != RUN: reset PI, return
    error = speed_ref - ss.mech_speed_rpm
    duty_pi = PI_RUN(error, dt)
    ss.duty_cmd = CLAMP(duty_pi, ramp_start_duty, 0.95)
```

### PMSM FOC Drive

**FOC Fast Current Loop** (`mc_pmsm_foc_run`):

```
PMSM_FOC_RUN(foc, in, out):
    // 1. Current reconstruction
    RECONSTRUCT_CURRENT(foc, in, &out.i_abc, &out.comp_status)

    // 2. Clarke + Park
    CLARKE(out.i_abc, &out.i_ab)
    PARK(out.i_ab, sinθ, cosθ, &out.i_dq)

    // 3. FW id adjustment
    if fw_enable:
        v_mag = sqrt(vd² + vq²)  // Previous cycle limited voltage
        fw_threshold = voltage_limit * fw_activation_ratio
        fw_error = fw_threshold - v_mag
        id_fw_adjustment += fw_ki * fw_error * dt
        CLAMP(id_fw_adjustment, fw_min_id, 0)
        fw_output = fw_kp * fw_error + id_fw_adjustment
        CLAMP(fw_output, fw_min_id, 0)
        effective_id_ref = CLAMP(id_ref + fw_output, fw_min_id, ∞)

    // 4. Current PI
    out.v_dq.d = PI_RUN(id_pi, effective_id_ref - out.i_dq.d, dt)
    out.v_dq.q = PI_RUN(iq_pi, iq_ref - out.i_dq.q, dt)

    // 5. Voltage limit
    LIMIT_VOLTAGE(foc, &out.v_dq)

    // 6. InvPark
    IPARK(out.v_dq, sinθ, cosθ, &out.v_ab)

    // 7. Bus voltage normalization
    NORMALIZE_VOLTAGE(foc, in, &out.v_ab)

    // 8. 1-shunt PWM optimization + SVPWM
    if current_cfg.type == 1SHUNT:
        PLAN_1SHUNT_FROM_VOLTAGE(foc, &out.v_ab, &out.pwm_cmd)
        OPTIMIZE_1SHUNT_PWM(foc, &out.pwm_cmd)     // pass 1: voltage preview
    SVPWM_RUN(&out.v_ab, &svpwm_cfg, &out.pwm_cmd)
    if current_cfg.type == 1SHUNT:
        OPTIMIZE_1SHUNT_PWM(foc, &out.pwm_cmd)     // pass 2: final duties

    // 9. ADC trigger plan
    PLAN_ADC_TRIGGER(foc, &shunt1_meta, &out.trigger_plan)

    // 10. State save
    foc.i_dq = out.i_dq;  foc.v_dq = out.v_dq;  foc.i_abc_last = out.i_abc
```

**1-Shunt PWM Optimization** (`mc_pmsm_optimize_1shunt_pwm`):

```
OPTIMIZE_1SHUNT_PWM(foc, pwm):
    if current_cfg.type != 1SHUNT: return

    // Common-mode shift
    shift = reorder_required ? 0.04 : 0.02
    pwm.common_mode_shift = zero_vector_bias_high ? shift : -shift

    // Phase reordering
    if reorder_required:
        Set active phase to PWM mode
        Set two inactive phases to HIGH (high zero-vector) or LOW (low zero-vector)
```
