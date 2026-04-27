# Review Status

## Completed In This Review Batch

- `mc_api` lifecycle handling was tightened so initialization only succeeds after mode-specific setup completes.
- `mc_set_mode()` now rejects switching into modes that were never initialized.
- `mc_medium_step()` no longer advances speed-control state while the instance is disabled.
- `MC_MODE_BLDC_SENSORLESS` now has a public start path through `mc_start()`.
- `mc_bldc_sensorless_reset()` now preserves configuration while clearing runtime state.
- `bemf_threshold_v` is now used by the BLDC sensorless zero-crossing path.
- The initial supported `MC_NUMERIC_Q31` execution path was added for top-level PMSM FOC with Hall or encoder feedback and 2-shunt or 3-shunt current sensing.
- Public docs were updated to describe the supported Q31 feature matrix conservatively.
- `identify` results now propagate back into the live FOC and sensorless runtime models.
- `identify` now produces a first real `flux_wb` estimate instead of exposing an unwired output field.
- `flux_wb` estimation now filters invalid samples, averages valid samples, and preserves the configured seed when no valid samples are collected.
- `identify` now uses the configured 1-shunt, 2-shunt, or 3-shunt current reconstruction path instead of hard-coded alpha-beta scaling.
- `identify` now integrates with the 1-shunt sampling-plan and ADC trigger generation path.
- The PMSM sensorless observer input path now uses the configured current-sense reconstruction path instead of hard-coded alpha-beta scaling.
- `mc_identify` measure-state behavior was tightened so `LD_MEASURE` and `LQ_MEASURE` better match their runtime semantics.
- Non-physical negative `Rs`, `Ld`, and `Lq` identify estimates are now rejected before they can propagate into the runtime model.
- `identify` inductance and flux windows now use time-weighted averaging over the actual discrete injected duration instead of idealized fixed-width averaging.
- `identify` now tracks internal raw candidates for `Rs`, `Ld`, `Lq`, and `flux_wb`, while public result APIs continue to expose trusted values only.
- `identify` flux accumulation now rejects q-axis current samples that oppose the applied excitation direction, so rebound samples do not inflate the first open-loop `flux_wb` estimate.
- The PMSM sensorless observer now requires consecutive qualifying samples before leaving open-loop startup or toggling PLL lock state.
- Public docs and headers now describe the PMSM sensorless observer's lock/unlock debounce assumptions and baseline tuning limits.
- `mc_sensorless_init()` now enforces the documented `min_bemf < lock_bemf` and non-inverted open-loop voltage-ramp constraints.

## Current Limits

- The current `flux_wb` estimate is still a first open-loop voltage-balance approximation, not a production-grade identification routine.
- `flux_wb` now requires most of the measure window to contain accepted samples before it replaces the configured seed, so short valid fragments are intentionally ignored.
- `mc_get_identified_params()` may report `0.0F` for `Rs`, `Ld`, or `Lq` when the corresponding estimate is invalid or non-physical.
- `mc_get_identified_params()` may report the configured `flux_wb` seed when the flux estimation window yields no valid samples.
- Internal identify candidate fields are runtime-only implementation details and are intentionally not exposed by the public result APIs.
- `identify` timing still follows the controller's discrete `dt_s`, so high jitter or coarse loop timing can still bias the estimate even though the windows now track actual injected duration.
- The top-level Q31 path remains intentionally limited to PMSM FOC with Hall or encoder feedback and 2-shunt or 3-shunt current sensing.
- The PMSM sensorless observer still uses simple LPF + PLL logic with lightweight debounce; it is more robust than before, but not yet a heavily tuned production observer.

## Next Candidates

1. Improve the physical quality of the `identify` formulas and measurement windows beyond the current conservative guards.
2. Expand documentation for the PMSM sensorless observer lock/unlock assumptions and current tuning limits.
3. Revisit whether additional Q31 combinations should be supported, or keep the current support matrix explicit and narrow.
