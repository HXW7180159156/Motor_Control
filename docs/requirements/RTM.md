# Requirements Traceability Matrix / 需求追踪矩阵

| Requirement | Scope Summary | Design / Interface References | Verification References |
|---|---|---|---|
| **01 — 工具链与构建** |||
| `REQ-MC-0101` | GCC/IAR/Keil/GHS toolchains | `README.md`, `mc_platform_compiler.h` | CI or manual build coverage |
| `REQ-MC-0102` | C99 standard, no extensions | `CMakeLists.txt`, `CODING_STANDARD.md` | Compiler flag audit |
| `REQ-MC-0103` | No dynamic memory | `CODING_STANDARD.md` | Code review, cppcheck `no-malloc` |
| `REQ-MC-0104` | Cortex-M/R target architectures | `mc_platform_arch.h`, `mc_platform_features.h` | Build matrix |
| `REQ-MC-0105` | Compiler macro adaptation | `mc_platform_compiler.h` | Per-compiler CI |
| **02 — 安全** |||
| `REQ-MC-0201` | PWM zero on overcurrent/overvoltage | `mc_api.c` fast_step disable path, `mc_diag.h` | `test_mc_api.c` fault tests, fault injection |
| `REQ-MC-0202` | Sensor invalid → stop updates | `mc_sensor_resolver.c`, `mc_sensor_hall.c` | `test_mc_sensor_*.c` |
| `REQ-MC-0203` | NULL pointer check on fast_step entry | `mc_api.c` mc_fast_step | `test_mc_api.c` null-arg tests |
| `REQ-MC-0204` | No PWM in DISABLED mode | `mc_api.c` mc_fast_step, `mc_drive_*.c` | `test_mc_api.c` |
| `REQ-MC-0205` | Overcurrent abort during identify | `mc_identify.c` mc_identify_run | `test_mc_identify.c` overcurrent tests |
| `REQ-MC-0206` | Clear fault via slow_step | `mc_api.c` mc_slow_step | `test_mc_api.c` fault-clear tests |
| `REQ-MC-0207` | Bus voltage clamp below min | `mc_drive_pmsm.c` mc_pmsm_normalize_voltage | `test_mc_drive_pmsm.c` |
| **03 — 控制** |||
| `REQ-MC-0301` | PMSM FOC control path | `SAD.md`, `SDD_control.md`, `mc_api.h` | `test_mc_api.c`, `ITS.md` |
| `REQ-MC-0302` | API diagnostics snapshot | `ICD.md`, `mc_diag.h`, `mc_api.h` | `test_mc_api.c`, `test_mc_diag.c` |
| `REQ-MC-0303` | PI output/integral clamp | `mc_control_pi.c` | `test_mc_control_pi.c` |
| `REQ-MC-0304` | SVPWM voltage limit | `mc_control_svpwm.c` | `test_mc_control_svpwm.c` |
| `REQ-MC-0305` | Field weakening activation | `mc_drive_pmsm.c` mc_pmsm_foc_run | `test_mc_drive_pmsm.c` |
| `REQ-MC-0306` | Speed/torque mutual exclusion | `mc_api.c` mc_set_speed_ref / mc_set_torque_ref | `test_mc_api.c` |
| `REQ-MC-0307` | Direct dq current reference | `mc_api.c` mc_set_current_ref_dq | `test_mc_api.c` |
| **04 — 辨识** |||
| `REQ-MC-0401` | Trusted-only identify results | `ICD.md`, `mc_api.h`, `mc_identify.h` | `test_mc_identify.c`, `test_mc_api.c` |
| `REQ-MC-0402` | Rs averaging, reject negative | `mc_identify.c` RS_MEASURE state | `test_mc_identify.c` |
| `REQ-MC-0403` | Ld/Lq pulse response with R-drop | `mc_identify.c` compute inductance | `test_mc_identify.c` |
| `REQ-MC-0404` | flux_wb window ratio gate | `mc_identify.c` finalize flux | `test_mc_identify.c` flux tests |
| `REQ-MC-0405` | Discrete dt_s for step counting | `mc_identify.c` compute_steps | `test_mc_identify.c` variable dt tests |
| `REQ-MC-0406` | Write trusted results to runtime | `mc_api.c` apply_identified_params | `test_mc_api.c` identify completion tests |
| **05 — 电流检测** |||
| `REQ-MC-0501` | 1/2/3-shunt support | `SDD_drive.md`, `SDD_reconstruct.md` | `test_mc_reconstruct_*.c` |
| `REQ-MC-0502` | Shared reconstruction path | `ICD.md`, `mc_api.h` | `test_mc_api.c`, `ITS.md` |
| `REQ-MC-0503` | 1shunt predictive compensation | `mc_reconstruct_1shunt.c`, `mc_drive_pmsm.c` | `test_mc_reconstruct_1shunt.c` |
| `REQ-MC-0504` | 2shunt KCL third phase | `mc_reconstruct_2shunt.c` | `test_mc_reconstruct_2shunt.c` |
| `REQ-MC-0505` | 3shunt direct output | `mc_reconstruct_3shunt.c` | `test_mc_reconstruct_3shunt.c` |
| `REQ-MC-0506` | Current in amperes via (raw-offset)*scale | `mc_reconstruct_*.c` | `test_mc_reconstruct_*.c` |
| **06 — 传感器** |||
| `REQ-MC-0601` | Sensorless LPF+PLL baseline | `SDD_estimator.md`, `mc_sensor_sensorless.h` | `test_mc_sensor_sensorless.c` |
| `REQ-MC-0602` | Sensorless config validation | `mc_sensor_sensorless.c` init | `test_mc_sensor_sensorless.c` |
| `REQ-MC-0603` | L*di/dt compensation | `mc_sensor_sensorless.c` update | `test_mc_sensor_sensorless.c` inductance tests |
| `REQ-MC-0604` | Voltage-frequency ramp | `mc_sensor_sensorless.c` open_loop path | `test_mc_sensor_sensorless.c` |
| `REQ-MC-0605` | Open-loop angle integration | `mc_sensor_sensorless.c` open_loop_active | `test_mc_sensor_sensorless.c` |
| `REQ-MC-0606` | Consecutive-sample lock gate | `mc_sensor_sensorless.c` lock counter logic | `test_mc_sensor_sensorless.c` lock/unlock tests |
| `REQ-MC-0607` | Hall code→angle lookup | `mc_sensor_hall.c` | `test_mc_sensor_hall.c` |
| `REQ-MC-0608` | Encoder count→angle | `mc_sensor_encoder.c` | `test_mc_sensor_encoder.c` |
| `REQ-MC-0609` | Resolver atan2f + amplitude check | `mc_sensor_resolver.c` | `test_mc_sensor_resolver.c` |
| `REQ-MC-0610` | Speed update on state change only | `mc_sensor_*.c` timestamp checks | `test_mc_sensor_*.c` |
| **07 — 数值路径** |||
| `REQ-MC-0701` | Q31 limited support matrix | `README.md`, `mc_api.h` | `test_mc_api.c`, `VTS.md` |
| `REQ-MC-0702` | float32 full coverage | `mc_cfg.h` MC_CFG_ENABLE_FLOAT32 | `test_mc_api.c` mode matrix tests |
| `REQ-MC-0703` | Q31 saturating arithmetic | `mc_math.c` q31_add_sat, q31_mul | `test_mc_math.c` |
| `REQ-MC-0704` | Q31↔f32 lossy reversible | `mc_math.c` q31_from/to_f32 | `test_mc_math.c` |
| **08 — 驱动** |||
| `REQ-MC-0801` | BLDC sensorless public start | `ICD.md`, `mc_drive_bldc_sensorless.h` | `test_mc_api.c`, `test_mc_drive_bldc_sensorless.c` |
| `REQ-MC-0802` | BLDC Hall 6-step commutation | `mc_drive_bldc.c` | `test_mc_drive_bldc.c` |
| `REQ-MC-0803` | BLDC SS 4-stage state machine | `mc_drive_bldc_sensorless.c` | `test_mc_drive_bldc_sensorless.c` |
| `REQ-MC-0804` | ZC debounce and parity detection | `mc_drive_bldc_sensorless.c` detect_zc | `test_mc_drive_bldc_sensorless.c` |
| `REQ-MC-0805` | BLDC SS speed PI closed-loop | `mc_drive_bldc_sensorless.c` speed_step | `test_mc_drive_bldc_sensorless.c` speed PI tests |
| **09 — 时序** |||
| `REQ-MC-0901` | fast_step WCET ≤ 30μs | `mc_api.c` mc_fast_step | `bench_mc_fast_step.c` (planned) |
| `REQ-MC-0902` | medium_step WCET ≤ 10μs | `mc_api.c` mc_medium_step | `bench_mc_medium_step.c` (planned) |
| `REQ-MC-0903` | slow_step WCET ≤ 5μs | `mc_api.c` mc_slow_step | `bench_mc_slow_step.c` (planned) |
| `REQ-MC-0904` | mc_init synchronous | `mc_api.c` mc_init | `test_mc_api.c` init tests |
| `REQ-MC-0905` | Platform hooks synchronous | `mc_api.c` hooks calls | `test_mc_api.c` stub capture |
| **10 — 精度** |||
| `REQ-MC-1001` | FOC speed error ≤ 1% | `mc_control_foc.c` | Integration test |
| `REQ-MC-1002` | Clarke beta error ≤ 1e-4 | `mc_transform.c` | `test_mc_transform.c` |
| `REQ-MC-1003` | Park round-trip ≤ 1e-5 | `mc_transform.c` | `test_mc_transform.c` round-trip tests |
| `REQ-MC-1004` | SVPWM duty sum < 3.0, in [0,1] | `mc_control_svpwm.c` | `test_mc_control_svpwm.c` |
| `REQ-MC-1005` | Sensorless angle error ≤ 10° | `mc_sensor_sensorless.c` | Vector test vs MATLAB |
| `REQ-MC-1006` | Identify Rs error ≤ ±20% | `mc_identify.c` | `test_mc_identify.c` |
| **11 — 故障检测** |||
| `REQ-MC-1101` | Identify overcurrent abort | `mc_identify.c` current check | `test_mc_identify.c` |
| `REQ-MC-1102` | diag_status 4 faults + 2 warnings | `mc_diag.h` enums | `test_mc_diag.c` |
| `REQ-MC-1103` | 1shunt comp mode in diag | `mc_diag.c`, `mc_drive_pmsm.c` | `test_mc_api.c`, `test_mc_diag.c` |
| `REQ-MC-1104` | get_diag any time | `mc_api.c` mc_get_diag | `test_mc_api.c` |
| `REQ-MC-1105` | observer_valid on low BEMF | `mc_sensor_sensorless.c` | `test_mc_sensor_sensorless.c` |
| `REQ-MC-1106` | init failure → initialized==FALSE | `mc_api.c` mc_init | `test_mc_api.c` |
| **12 — 标定** |||
| `REQ-MC-1201` | adc_calibrate_linear | `mc_adc_calibrate.c`, `mc_port_adc.h` | `test_mc_adc_calibrate.c` |
| `REQ-MC-1202` | Reject equal raw points | `mc_adc_calibrate.c` | `test_mc_adc_calibrate.c` |
| `REQ-MC-1203` | Calibration configurable to chains | `mc_port_adc_ref.c` | `test_mc_port_adc_ref.c` |
