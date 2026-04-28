# Keil (ARMCC) Toolchain Build / Keil 工具链构建

## Prerequisites / 前提

- Keil MDK-ARM ≥ 5.36
- `armcc` / `armclang` in PATH

## Build / 构建

```bat
rem toolchains\keil\build.bat
armclang.exe ^
    --target=arm-arm-none-eabi ^
    -mcpu=cortex-m4 ^
    -mfpu=fpv4-sp-d16 ^
    -mfloat-abi=hard ^
    -O2 ^
    -DMC_CFG_TARGET_ARCH=2 ^
    -I include ^
    -c src\api\*.c src\math\*.c src\control\*.c src\reconstruct\*.c src\estimator\*.c src\drive\*.c

armar.exe -r build_keil\motor_control.lib *.o
```

## Compiler-Specific Notes / 编译器特定说明

| Feature | ARMCC / ARMCLANG Support |
|---|---|
| `inline` | `__inline` |
| `restrict` | `__restrict` |
| `_Alignas` | `__align(n)` |
| C99 compound literal | Supported |
| `__attribute__((unused))` | Supported |

## Status / 状态

- **Planned**: project skeleton only, build not yet validated.
