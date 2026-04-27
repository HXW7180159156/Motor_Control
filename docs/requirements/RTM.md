# Requirements Traceability Matrix / 需求追踪矩阵

| Requirement | Scope Summary | Design / Interface References | Verification References |
| --- | --- | --- | --- |
| `REQ-MC-0101` | GCC / IAR / Keil / GHS toolchains | `README.md`, `docs/requirements/SRS.md` | Manual or CI toolchain coverage not documented yet |
| `REQ-MC-0301` | PMSM FOC control path | `docs/design/SAD.md`, `docs/design/SDD_control.md`, `docs/design/SDD_drive.md`, `docs/design/SDD_estimator.md`, `include/mc_api.h` | `tests/unit/test_mc_api.c`, `docs/test/ITS.md` |
| `REQ-MC-0302` | API diagnostics snapshot | `docs/design/ICD.md`, `include/mc_diag.h`, `include/mc_api.h` | `tests/unit/test_mc_api.c`, `tests/unit/test_mc_diag.c`, `docs/test/UTS.md` |
| `REQ-MC-0401` | Trusted-only identify public results | `docs/design/ICD.md`, `include/mc_api.h`, `include/mc_identify.h` | `tests/unit/test_mc_identify.c`, `tests/unit/test_mc_api.c`, `docs/test/UTS.md` |
| `REQ-MC-0501` | 1-shunt / 2-shunt / 3-shunt sensing | `docs/design/SDD_drive.md`, `docs/design/SDD_reconstruct.md`, `include/mc_reconstruct_1shunt.h`, `include/mc_reconstruct_2shunt.h`, `include/mc_reconstruct_3shunt.h` | `tests/unit/test_mc_reconstruct_1shunt.c`, `tests/unit/test_mc_reconstruct_2shunt.c`, `tests/unit/test_mc_reconstruct_3shunt.c`, `docs/test/UTS.md` |
| `REQ-MC-0502` | Shared reconstruction path for identify and sensorless | `docs/design/ICD.md`, `docs/design/SDD_reconstruct.md`, `include/mc_api.h` | `tests/unit/test_mc_api.c`, `docs/test/ITS.md` |
| `REQ-MC-0601` | Baseline PMSM sensorless LPF + PLL observer | `docs/design/SDD_estimator.md`, `include/mc_sensor_sensorless.h`, `include/mc_api.h` | `tests/unit/test_mc_sensor_sensorless.c`, `tests/unit/test_mc_api.c`, `docs/test/UTS.md`, `docs/test/ITS.md` |
| `REQ-MC-0602` | Sensorless configuration constraints | `docs/design/SDD_estimator.md`, `include/mc_sensor_sensorless.h` | `tests/unit/test_mc_sensor_sensorless.c`, `tests/unit/test_mc_api.c`, `docs/test/UTS.md` |
| `REQ-MC-0701` | Limited top-level Q31 support matrix | `README.md`, `docs/design/SDD_reconstruct.md`, `include/mc_api.h` | `tests/unit/test_mc_api.c`, `docs/test/UTS.md`, `docs/test/VTS.md` |
| `REQ-MC-0801` | Public BLDC sensorless start path | `docs/design/ICD.md`, `docs/design/SDD_drive.md`, `include/mc_api.h`, `include/mc_drive_bldc_sensorless.h` | `tests/unit/test_mc_api.c`, `tests/unit/test_mc_drive_bldc_sensorless.c`, `docs/test/ITS.md` |
