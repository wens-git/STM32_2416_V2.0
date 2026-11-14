cmake_minimum_required(VERSION 3.22)

# 固件使用 C11；保留 GNU 扩展以兼容 STM32 HAL 和启动代码中的扩展语法。
# 固件统一使用 GNU C11；保留 GNU 扩展以兼容 CMSIS/HAL 的编译属性。
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 未传入构建类型时使用 Debug，便于归档工程开箱后直接调试。
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME init_2416_f407)
# 生成 compile_commands.json，供 clangd 等代码分析工具读取。
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 支持从 Firmware 目录单独配置工程时自动选择交叉编译工具链。
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)
endif()

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

set(FIRMWARE_DIST_DIR "${CMAKE_SOURCE_DIR}/dist")
file(MAKE_DIRECTORY "${FIRMWARE_DIST_DIR}")

# 同时启用 C 与汇编：启动文件 startup_stm32f407xx.s 需要汇编器参与构建。
enable_language(C ASM)
add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE)

# 业务层和板级扩展源码。CubeMX 生成的基础源码及 HAL 驱动由
# cmake/stm32cubemx/CMakeLists.txt 集中维护，避免同一文件被重复编译。
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 项目自研业务、板级适配和运动控制实现。
    Core/Src/IODEF.c
    Core/Src/Dependence/bsp_uart.c
    Core/Src/Dependence/bsp_uart485.c
    Core/Src/Dependence/bsp_iwdg.c
    Core/Src/Dependence/input.c
    Core/Src/Dependence/modbus_rtu.c
    Core/Src/Dependence/pwm_motor.c
    Core/Src/Dependence/soft_pwm.c
    Core/Src/Dependence/motor_axis.c
    Core/Src/Dependence/nvram.c
    Core/Src/Dependence/nvram_auto.c
    Core/Src/Dependence/scheduler.c
    Core/Src/Dependence/plc_timer.c
    Core/Src/Dependence/error_code.c
    Core/Src/app/app.c
    Core/Src/app/axis/axis_manager.c
    Core/Src/app/business/business_app.c
    Core/Src/app/debug/motor_debug_service.c
    Core/Src/app/rs485_service.c
    Core/Src/app/rs232_service.c
    Core/Src/app/rs232_freeport_example.c
    Core/Src/tasks/main_task.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE)

# ARM 裸机工程不链接宿主机默认库；移除工具链探测中可能出现的 ob 库。
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# stm32cubemx 目标集中提供启动文件、HAL 驱动、CMSIS 和芯片编译定义。
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

# ELF 链接成功后同步生成烧录常用的 HEX/BIN 文件，并输出段大小统计。
# 链接后同时生成调试、烧录和体积分析所需产物。
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${FIRMWARE_DIST_DIR}"
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    BYPRODUCTS
        "${FIRMWARE_DIST_DIR}/${CMAKE_PROJECT_NAME}.hex"
        "${FIRMWARE_DIST_DIR}/${CMAKE_PROJECT_NAME}.bin"
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${FIRMWARE_DIST_DIR}/${CMAKE_PROJECT_NAME}.hex"
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${FIRMWARE_DIST_DIR}/${CMAKE_PROJECT_NAME}.bin"
    COMMENT "Refreshing firmware images in dist"
    VERBATIM
)

add_custom_target(${CMAKE_PROJECT_NAME}_images ALL
    DEPENDS ${CMAKE_PROJECT_NAME}
)
