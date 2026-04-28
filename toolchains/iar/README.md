# IAR Toolchain Build / IAR 工具链构建

## Prerequisites / 前提

- IAR Embedded Workbench for Arm ≥ 9.x
- `iccarm` in PATH

## Build / 构建

```bat
rem toolchains\iar\build.bat
iccarm.exe ^
    --cpu Cortex-M4F ^
    --fpu VFPv4_sp ^
    --dlib_config full ^
    -DMC_CFG_TARGET_ARCH=2 ^
    -I include ^
    -o build_iar\motor_control.out ^
    src\api\*.c src\math\*.c src\control\*.c src\reconstruct\*.c src\estimator\*.c src\drive\*.c
```

## Compiler-Specific Notes / 编译器特定说明

| Feature | IAR Support |
|---|---|
| `inline` | `#pragma inline` or `__inline` |
| `restrict` | `__restrict` |
| `_Alignas` | `_Pragma("data_alignment=")` |
| C99 compound literal | Supported |
| `__attribute__((aligned))` | Use `#pragma data_alignment` instead |

## Status / 状态

- **Planned**: project skeleton only, build not yet validated.
