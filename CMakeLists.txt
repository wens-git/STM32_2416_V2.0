cmake_minimum_required(VERSION 3.22)

# 未显式指定工具链时，默认使用仓库内的 ARM GCC 工具链配置。
# 该设置必须位于 project() 之前，否则 CMake 可能先按本机编译器检测工程。
# 顶层工程负责统一选择 ARM GCC 工具链，并将固件目录作为唯一构建目标。
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/Firmware/cmake/gcc-arm-none-eabi.cmake)
endif()

project(STM32F407_2416_INIT)

# 固件目录包含芯片启动文件、HAL 驱动、业务源码和最终镜像生成规则。
# 固件目录内部维护源文件、芯片定义、链接脚本和镜像生成规则。
add_subdirectory(Firmware)
