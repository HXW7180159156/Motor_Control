# GCC Toolchain Build / GCC 工具链构建

## Prerequisites / 前提

- `arm-none-eabi-gcc` (GNU Arm Embedded Toolchain) ≥ 10.3
- CMake ≥ 3.20

## Build / 构建

```sh
cmake -S . -B build_gcc \
    -DCMAKE_TOOLCHAIN_FILE=toolchains/gcc/toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build_gcc
```

## Toolchain File / 工具链文件

```cmake
# toolchains/gcc/toolchain.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O2 -ffunction-sections -fdata-sections" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nosys.specs -Wl,--gc-sections" CACHE STRING "" FORCE)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
```

## Status / 状态

- **Planned**: toolchain file exists, build not yet validated in CI.
