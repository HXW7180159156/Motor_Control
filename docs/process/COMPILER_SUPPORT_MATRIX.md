# Compiler Support Matrix / 编译器支持矩阵

| Toolchain / 工具链 | Status / 状态 | Build Script | Notes / 说明 |
|---|---|---|---|
| GCC (arm-none-eabi) | Planned | `toolchains/gcc/toolchain.cmake` | CMake native support, DWT cycle counter |
| IAR EWARM | Planned | `toolchains/iar/build.bat` | Requires `_Pragma` for alignment |
| Keil MDK-ARM (ARMCLANG) | Planned | `toolchains/keil/build.bat` | ARM Compiler 6 (LLVM-based) |
| GHS MULTI | Planned | `toolchains/ghs/build.bat` | ASIL D certified compiler |

## Compiler Adaptation Matrix / 编译器适配矩阵

| Macro / 宏 | GCC | IAR | Keil (ARMCC) | GHS |
|---|---|---|---|---|
| `MC_INLINE` | `inline` | `inline` | `__inline` | `inline` |
| `MC_RESTRICT` | `restrict` | `restrict` | `__restrict` | `restrict` |
| `MC_ALIGN(n)` | `__attribute__((aligned(n)))` | `_Pragma("data_alignment=n")` | `__align(n)` | `__attribute__((aligned(n)))` |
| `MC_UNUSED` | `__attribute__((unused))` | `__root` | `__attribute__((unused))` | `__attribute__((unused))` |
| C99 compound literal | ✓ | ✓ | ✓ | ✓ |
| `stdint.h` | ✓ | ✓ | ✓ | ✓ |
| `math.h` (single-precision) | ✓ | ✓ | ✓ | ✓ |

## CI Plan / CI 计划

| Toolchain | Priority | Target |
|---|---|---|
| GCC | P0 | GitHub Actions, `arm-none-eabi-gcc` container |
| IAR | P1 | Local/self-hosted runner with IAR license |
| Keil | P1 | Local/self-hosted runner with Keil license |
| GHS | P2 | Contact GHS for evaluation license |

## Current Status / 当前状态

- **MSVC x64 (host build)**: primary development compiler, 0 errors ✓
- **GCC (host)**: tested via Ninja generator in `build_ninja_x86gcc/`
- **GCC (arm-none-eabi)**: toolchain file ready, pending CI hardware
- **IAR / Keil / GHS**: build scripts ready, pending toolchain installation
