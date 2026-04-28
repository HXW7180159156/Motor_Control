# GHS (Green Hills) Toolchain Build / GHS 工具链构建

## Prerequisites / 前提

- Green Hills MULTI IDE ≥ 7.x
- `ccarm` in PATH

## Build / 构建

```bat
rem toolchains\ghs\build.bat
ccarm.exe ^
    -cpu=cortex-m4 ^
    -fpu=vfpv4_sp_d16 ^
    -Ogeneral ^
    -DMC_CFG_TARGET_ARCH=2 ^
    -I include ^
    -c src\api\*.c src\math\*.c src\control\*.c src\reconstruct\*.c src\estimator\*.c src\drive\*.c

cxarm.exe -r build_ghs\motor_control.lib *.o
```

## Compiler-Specific Notes / 编译器特定说明

| Feature | GHS Support |
|---|---|
| `inline` | `inline` |
| `restrict` | `__restrict` |
| `_Alignas` | `__attribute__((aligned(n)))` |
| C99 compound literal | Supported |
| `__attribute__((unused))` | Supported |

## Status / 状态

- **Planned**: project skeleton only, build not yet validated.
