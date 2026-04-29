# Changelog / 变更记录

## Unreleased / 未发布

### Added / 新增

- `include/mc_constants.h`: named constants for all math, Q31, 1-shunt, sensorless, identify, BLDC defaults and thresholds (MISRA C:2012 Dir 4.14)
- `docs/process/DEVELOPMENT_PLAN.md`: phased development roadmap through M1/M2/M3
- `cppcheck.json` / `cppcheck_suppressions.txt`: MISRA static analysis configuration
- CMake `cppcheck` custom target for automated static analysis (`MC_RUN_CPPCHECK` option)
- `isfinite()` NaN/Inf guard in PMSM sensorless observer BEMF calculation
- Documented rationale for second `mc_pmsm_optimize_1shunt_pwm` call in PMSM FOC run
- SRS expanded from 10 to 42 requirements across 12 domains (safety, timing, accuracy, fault, calibration etc.)
- FMEA expanded from 1 to 30 entries covering all 7 architectural layers
- RTM updated with full requirement-to-design-to-test traceability
- 6 integration test suites (18 test cases) covering PMSM FOC, sensorless, BLDC Hall, BLDC sensorless, identify, and scheduling

### Changed / 修改

- Consolidated 5 duplicate `clamp` implementations into single `mc_math_clamp_f32` across all modules
- Replaced 40+ magic numbers with named constants from `mc_constants.h` across all source files
- Bilingual docs standardized across SRS, RTM, FMEA, HARA, SAFETY_PLAN, DEVIATION_RECORD
- Updated Safety Plan to reflect HARA→SG→TSR→HSI lifecycle completion

### Added / 新增 (M3 量产准备)

- `docs/safety/HARA.md`: item definition, 9 hazards, 5 safety goals (up to ASIL D), 10 TSRs, 6 HSI entries
- `docs/safety/SAFETY_PLAN.md`: rewritten with 10-phase safety development lifecycle status
- `docs/safety/DEVIATION_RECORD.md`: 6 MISRA deviations + 2 design deviations with justifications
- Fault injection tests (`tests/fault/`): 4 suites, 10 test cases covering sensor faults, reconstruct faults, and lifecycle faults
- Vector golden reference tests (`tests/vector/`): 23 test cases validating Clarke/Park/InvPark/SVPWM/PI/Q31 math against known reference values
- CMake targets: `test_mc_fault`, `test_mc_vector`
- Total test coverage: 22 unit + 6 integration + 4 fault + 1 vector = 33 test suites, ~105 test cases

### Changed / 修改

- Consolidated 5 duplicate `clamp` implementations into single `mc_math_clamp_f32` across all modules
- Replaced 40+ magic numbers with named constants from `mc_constants.h` across all source files
- `mc_platform_compiler.h`: added `MC_INLINE`, `MC_RESTRICT`, `MC_ALIGN`, `MC_UNUSED`, `MC_STATIC_ASSERT` macros for multi-compiler portability
- BLDC sensorless: added `MC_BLDC_SS_INVALID_STEP` sentinel constant instead of bare `0xFF`
- Initialized `freq` and `duty` locals in `mc_bldc_sensorless_run` to suppress MSVC C4700

### Fixed / 修复

- None

### Added / 新增 (Debug/Calibration)

- `src/debug/mc_debug_map.c`: compile-time variable registration with runtime address resolution; 23 motor control variables exposed to host via pointer-deref collection in FreeMASTER frames
- `src/debug/mc_debug_fm.c`: FreeMASTER protocol engine implementing 7 command handlers (GET_INFO, READ_VARS, WRITE_VAR, SCOPE_START/STOP, REC_START, RESPONSE); scope timer for periodic auto-streaming
- `src/debug/mc_debug_transp.c`: transport abstraction layer with magic-byte frame boundary detection (0xDA 0x1A + 2-byte little-endian length), buffered TX with callback flush, ISR-safe RX state machine
- `include/mc_debug.h`: public types (`mc_debug_var_t`, `mc_debug_transp_t`, `mc_debug_t`), FreeMASTER command constants, `MC_DEBUG_VAR_RO/RW` registration macros, `MC_DEBUG_TYPE_ID` ctype resolver
- `include/mc_port_debug.h`: platform debug hooks (`uart_tx`, `can_tx`, `lin_tx`, `uart_rx_available`, `uart_rx_read`)
- Compile-time configuration macros: `MC_CFG_ENABLE_DEBUG`, `MC_CFG_ENABLE_DEBUG_FM`, `MC_CFG_ENABLE_DEBUG_UART/CAN/LIN` (in `mc_cfg.h`)
- Debug subsystem auto-polls via `mc_fast_step()` — integrated into `mc_instance_t.debug` field, zero intrusion when `MC_CFG_ENABLE_DEBUG=0`
- 23 real variables exposed: id_ref, iq_ref, speed_ref_rpm, v_d, v_q, i_d, i_q, duty_a/b/c, active_fault, active_warn, id_kp/ki, iq_kp/ki, speed_kp/ki, fw_enable, fw_min_id, voltage_limit, iq_limit, dtc_enable
- 14 unit test cases: variable map (4), transport (4), protocol engine (4), end-to-end (2)
- Full Doxygen coverage on all 6 new source/header files following existing `mc_control_pi.h` convention

### Known / 已知

- Initial M1 repository skeleton.
- Debug subsystem: FreeMASTER connection not yet validated on real hardware; variable table currently supports PMSM FOC path only (BLDC variables for future expansion).
