# Changelog / 变更记录

## Unreleased / 未发布

### Added / 新增

- FW (field weakening) kp term wiring: proportional term now acts on current-cycle voltage magnitude error before PI integration (`src/drive/mc_drive_pmsm.c`)
- ADC calibration function: `mc_adc_calibrate_linear` two-point calibration (`src/api/mc_adc_calibrate.c`, `include/mc_port_adc.h`)
- 2-shunt / 3-shunt ADC trigger plan: default single trigger at PWM midpoint (position=0.5, event=PWM_DOWN) in `mc_pmsm_foc_run`
- Doxygen comments on public headers: `mc_api.h`, `mc_drive_pmsm.h`, `mc_reconstruct_1shunt.h`, `mc_diag.h`

### Changed / 修改

- Redesigned FW calculation order: fw_kp proportional term computed before PI integration, using last-cycle limited Vdq (`foc->v_dq`)
- Updated SDD documents: `SDD_control.md` (MTPA/FW/1shunt compensation), `SDD_drive.md` (current sense/ADC trigger), `SDD_platform.md` (ADC calibration), `ICD.md` (all new interfaces)
- Updated 3SHUNT ADC trigger plan assertions in `test_mc_api.c` to match new default trigger behavior

### Fixed / 修复

- None

### Known / 已知

- Initial M1 repository skeleton.
