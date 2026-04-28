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

### Known / 已知

- Initial M1 repository skeleton.
