# Motor Control Library / 电机控制算法库

## Overview / 概述

This repository hosts a portable C99 motor control library skeleton for automotive-grade MCU targets.

该仓库提供一个面向车规 MCU 的可移植 C99 电机控制算法库骨架。

## Scope / 范围

- BLDC and PMSM control / BLDC 与 PMSM 控制
- Hall, encoder, resolver, and sensorless support / 霍尔、编码器、旋变与无感支持
- 1-shunt, 2-shunt, and 3-shunt current sensing / 单电阻、双电阻、三电阻采样
- GCC, IAR, Keil, and GHS toolchains / GCC、IAR、Keil 与 GHS 工具链
- MISRA C:2012 and ISO 26262 friendly process / MISRA C:2012 与 ISO 26262 友好流程

## Build / 构建

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## Repository Layout / 目录结构

- `include/`: public headers / 公共头文件
- `src/`: library sources / 库源码
- `tests/`: unit, vector, integration, and bench tests / 单元、向量、集成与性能测试
- `docs/`: bilingual project documents / 中英双语项目文档
- `toolchains/`: compiler entry files / 编译器入口文件

## Status / 当前状态

Implemented baseline control and estimator paths now include PMSM FOC with Hall, encoder, resolver, and a first sensorless back-EMF LPF + PLL observer, plus BLDC Hall commutation. The PMSM sensorless observer currently uses consecutive-sample debounce for open-loop exit and PLL lock/unlock decisions, and its documented configuration constraints are now enforced during `mc_sensorless_init()`. Motor parameter identification now writes identified `Rs`, `Ld`, `Lq`, and `flux_wb` back into the live runtime model. The initial Q31 path is currently limited to top-level PMSM FOC operation with Hall or encoder feedback and 2-shunt or 3-shunt current sensing.

Current identify behavior is intentionally conservative: `current_ab` is expected to arrive in amperes through the configured current-sense reconstruction path, `align_voltage` / `rs_voltage` / `pulse_voltage` are command ratios scaled by `voltage_limit`, `Ld` / `Lq` pulse windows and the final `flux_wb` window use time-weighted averaging over the actual discrete injected duration, internal raw candidates are tracked for `Rs` / `Ld` / `Lq` / `flux_wb` before trust gates promote them, public results still expose only trusted values, invalid or non-physical `Rs` / `Ld` / `Lq` estimates are reported as `0.0F`, and `flux_wb` may remain at its configured seed when the flux sampling window yields no valid estimate.

当前已实现基础控制与估算路径，包括 PMSM FOC 的霍尔、编码器、旋变，以及首版无感反电势 LPF + PLL 观测器，同时已具备 BLDC Hall 六步换相。电机参数辨识现已把 `Rs`、`Ld`、`Lq` 和 `flux_wb` 回写到运行时模型。首版 Q31 路径当前仅支持顶层 PMSM FOC 的霍尔或编码器反馈，以及双电阻或三电阻采样。

当前 `identify` 行为采取保守策略：`current_ab` 需要先通过当前配置的电流采样重建并以安培为单位输入，`align_voltage` / `rs_voltage` / `pulse_voltage` 是按 `voltage_limit` 缩放的命令比例，`Ld` / `Lq` 脉冲窗口以及最终 `flux_wb` 窗口都按实际离散注入时长做时间加权平均，`Rs` / `Ld` / `Lq` / `flux_wb` 都会先记录内部 raw candidate 再经过 trust gate 提升为公开结果，对外接口仍只暴露 trusted 值，无效或非物理的 `Rs` / `Ld` / `Lq` 会返回 `0.0F`，而 `flux_wb` 在采样窗口没有得到有效估计时会保留配置中的 seed 值。
