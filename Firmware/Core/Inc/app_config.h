/**
 * @file    app_config.h
 * @brief   全局系统配置头文件 — 定义时钟频率、调度器参数、任务周期及临界区保护宏
 * @details 本文件是整个工程的"参数中心"，所有与系统节拍、时钟、调度器相关的可调参数
 *          均集中于此，便于统一维护和全局检索。任何模块若需获取系统级参数，
 *          只需包含此文件即可，避免硬编码魔法数字散落在各源文件中。
 *
 *          【修改注意事项】
 *          - SCHEDULER_HW_TICK_US 必须与 SysTick / 硬件定时器的实际周期严格一致；
 *          - 修改 SCHEDULER_TICK_MS 会影响所有任务的周期计算和软件定时器精度；
 *          - SCHEDULER_MAX_TASKS 决定了静态分配的任务槽位数量，增大会增加 RAM 占用。
 */

#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

/* ---- 标准库类型头文件 ---- */
#include <stdint.h>   /* 提供 uint8_t, uint16_t, uint32_t 等定宽整型 */
#include <stdbool.h>  /* 提供 bool / true / false 布尔类型 */
#include <stddef.h>   /* 提供 NULL, size_t 等基础定义 */

/* ---- HAL 库全局头文件 ---- */
#include "stm32f4xx_hal.h"  /* 包含 STM32F4 HAL 驱动的所有外设定义及 CMSIS 核心接口 */

/* ==========================================================================
 * 第 1 节: 系统时钟配置 (System Clock Configuration)
 * ==========================================================================
 * STM32F407VET6 在 VOS = Scale1 模式下最高支持 168 MHz 主频。
 * 本工程使用 HSI (16MHz) 经过 PLL 倍频得到 168MHz SYSCLK:
 *   PLLM=16, PLLN=336, PLLP=2 => 16 / 16 * 336 / 2 = 168 MHz
 * 该宏定义的值必须与 SystemClock_Config() 中实际配置的主频完全一致，
 * 否则所有依赖时钟计算的模块 (如 UART 波特率、定时器分频) 均会出错。
 */
#define SYS_CLK_FREQ  168000000UL   /* 系统主时钟频率，单位: Hz (168 MHz) */

/* ==========================================================================
 * 第 2 节: 调度器核心配置 (Scheduler Core Configuration)
 * ==========================================================================
 * 本工程采用协作式调度器 (Cooperative Scheduler) 架构:
 *   - 硬件定时器 (SysTick) 以固定周期产生中断，作为"硬件节拍"；
 *   - 调度器内部通过分频器将硬件节拍转换为"应用节拍" (App Tick)；
 *   - 各任务的周期以"应用节拍"为单位进行配置。
 *
 * 时序关系:
 *   硬件中断周期 = SCHEDULER_HW_TICK_US (微秒)
 *   应用节拍周期 = SCHEDULER_TICK_MS (毫秒)
 *   分频系数     = SCHEDULER_TICK_MS * 1000 / SCHEDULER_HW_TICK_US
 *   例: 1ms * 1000 / 1000us = 1 (即每 1 次硬件中断产生 1 个应用节拍)
 */

/**
 * @def   SCHEDULER_HW_TICK_US
 * @brief 硬件定时器中断周期，单位: 微秒 (us)
 * @note  HAL 库默认配置 SysTick 为 1ms (即 1000us)。
 *        若将来改用 TIM6 / TIM7 等通用定时器产生更高频率的中断，
 *        需同步修改此值以保持调度器分频计算正确。
 */
#define SCHEDULER_HW_TICK_US  1000  /* 1000us = 1ms，与 SysTick 默认配置匹配 */

/**
 * @def   SCHEDULER_TICK_MS
 * @brief 应用层调度节拍周期，单位: 毫秒 (ms)
 * @note  所有通过 scheduler_add() 注册的任务，其 period 参数的单位均为
 *        "应用节拍"，实际执行周期 = period × SCHEDULER_TICK_MS。
 *        例: scheduler_add(Task_Main, 10) => 实际周期 = 10 × 1ms = 10ms
 */
#define SCHEDULER_TICK_MS     1     /* 每 1ms 产生一个应用层调度节拍 */

/**
 * @def   SCHEDULER_MAX_TASKS
 * @brief 调度器支持的最大任务数量
 * @note  任务控制块 (TCB) 以静态数组形式分配在 .bss 段中。
 *        每个 TCB 占用约 16 字节 (指针 + 3 × uint32 + bool)。
 *        若任务数量不足，scheduler_add() 将返回 0xFF 表示注册失败。
 */
#define SCHEDULER_MAX_TASKS   15     /* 最多同时注册 15 个调度任务 */

/* ==========================================================================
 * 第 3 节: 应用任务周期配置 (Application Task Period Configuration)
 * ==========================================================================
 * 以下宏以毫秒为单位定义各任务的执行周期，供 main.c 中计算对应的
 * 调度节拍数: period_ticks = TASK_xxx_PERIOD_MS / SCHEDULER_TICK_MS
 */
#define TASK_APP_PERIOD_MS      10    /* 主业务任务周期: 10ms  → scheduler_add 中 period = 10/1 = 10 个应用节拍 */
#define TASK_MODBUS_PERIOD_MS   5     /* Modbus 通信轮询任务周期: 5ms → 确保帧处理及时性 (新增) */
#define TASK_DISPLAY_PERIOD_MS  100   /* 显示/辅助任务周期: 100ms → scheduler_add 中 period = 100/1 = 100 个应用节拍 (新增) */
#define TASK_WDT_PERIOD_MS      500   /* 看门狗喂狗任务周期: 500ms → scheduler_add 中 period = 500/1 = 500 个应用节拍 */
#define SYSTEM_IWDG_TIMEOUT_MS  8000U /* 覆盖 Sector 7 擦除期间的最长主循环停顿。 */

/* ==========================================================================
 * 第 3.5 节: 调试开关 (Debug Switches)
 * ==========================================================================
 * 2416 版沿用 1612 的应用分层:
 * - app/business: 正常业务扩展点
 * - app/debug:    Modbus 电机调试服务
 * 通过以下宏统一控制调试功能开关，保持工程结构和现有 1612 项目一致。
 */

/* printf 重定向开关: 调试阶段默认开启，量产时可注释掉。 */
#ifndef DEBUG_PRINTF
/* 仅当 USART1 未用于 Modbus 时，才允许在构建参数中定义 DEBUG_PRINTF。 */
#endif

#ifndef AXIS_ALLOW_UNPROTECTED_MOTION
/* Bench/debug build only. Production must bind and require both limit inputs. */
#define AXIS_ALLOW_UNPROTECTED_MOTION  1
#endif

#ifndef UART_BAUDRATE
#define UART_BAUDRATE  115200U
#endif

/* 调试服务开关: 默认启用 app/debug/motor_debug_service。 */
#ifndef APP_DEBUG_SERVICE_ENABLE
#define APP_DEBUG_SERVICE_ENABLE  1
#endif

/* NVRAM 存储使能: 启用 Flash 定位点存储 */
#ifndef NVRAM_ENABLE
#define NVRAM_ENABLE  1
#endif

/* Separate generic persistent-variable store debounce (milliseconds). */
#ifndef NVRAM_AUTO_SAVE_DELAY_MS
#define NVRAM_AUTO_SAVE_DELAY_MS  5000U
#endif

/* 链路测试开关: 默认关闭，避免和 Modbus 共用串口时互相干扰。 */
#ifndef UART_LINK_TEST_ENABLE
#define UART_LINK_TEST_ENABLE  0
#endif

/* 链路测试周期 (ms)，仅 UART_LINK_TEST_ENABLE=1 时生效。 */
#ifndef UART_LINK_TEST_PERIOD_MS
#define UART_LINK_TEST_PERIOD_MS  100
#endif

/* ==========================================================================
 * 第 4 节: 原子操作临界区保护宏 (Atomic / Critical Section Macros)
 * ==========================================================================
 * 在裸机 (Bare-metal) 环境下，通过关闭全局中断来实现临界区保护。
 *
 * 工作原理:
 *   ATOMIC_ENTER() — 先保存当前 PRIMASK 寄存器的值，再执行 __disable_irq()
 *                     关闭所有可屏蔽中断。保存原始值是为了支持嵌套调用:
 *                     如果调用前中断已经处于关闭状态，退出时不会错误地重新打开。
 *   ATOMIC_EXIT()  — 将之前保存的 PRIMASK 值写回，恢复中断使能状态。
 *
 * 使用示例:
 *   ATOMIC_ENTER();
 *   // ... 访问共享变量 ...
 *   ATOMIC_EXIT();
 *
 * @warning  临界区内的代码应尽量简短，避免长时间关闭中断影响实时性。
 *           若需在同一函数中使用多对 ATOMIC_ENTER/EXIT，应将每对包裹在
 *           独立的 { } 代码块中，避免变量名冲突。
 */
#define ATOMIC_ENTER()  uint32_t _atomic_primask_save_ = __get_PRIMASK(); __disable_irq()
#define ATOMIC_EXIT()   __set_PRIMASK(_atomic_primask_save_)


/* ==========================================================================
 * RS485 configuration (independent from USART1 / RS232)
 * ==========================================================================
 * USART3: PC10(TX) / PC11(RX), DE/RE = PB14
 * Default protocol: Modbus RTU slave, same register map as RS232.
 * Set RS485_PROTOCOL_MODE to RS485_PROTO_FREEPORT for raw serial frames.
 */
#ifndef RS485_ENABLE
#define RS485_ENABLE               1
#endif

#ifndef RS485_BAUDRATE
#define RS485_BAUDRATE             115200U
#endif

#define RS485_PROTO_MODBUS         1
#define RS485_PROTO_FREEPORT       2

#ifndef RS485_PROTOCOL_MODE
#define RS485_PROTOCOL_MODE        RS485_PROTO_MODBUS
#endif

#ifndef RS485_DE_GPIO_PORT
#define RS485_DE_GPIO_PORT         GPIOB
#endif

#ifndef RS485_DE_PIN
#define RS485_DE_PIN               GPIO_PIN_14
#endif

/* Modbus slave address used by the RS485 port. Kept equal to the RS232
 * default so the same host tool can be moved to the USB-RS485 adapter. */
#ifndef RS485_SLAVE_ADDR
#define RS485_SLAVE_ADDR           1U
#endif
/* ==========================================================================
 * RS232 / USART1 protocol selection
 * ==========================================================================
 * RS232_PROTO_MODBUS   : USART1 keeps the built-in Modbus RTU slave (default).
 * RS232_PROTO_FREEPORT : USART1 raw frames go to RS232_FreePort_OnFrame() and
 *                        the USART1 Modbus slave is disabled at the same time.
 *
 * Independent of RS485_PROTOCOL_MODE, so both ports can run different
 * protocols at the same time, e.g. RS485 = MODBUS (host tool) together with
 * RS232 = FREEPORT (private device protocol).
 */
#define RS232_PROTO_MODBUS         1
#define RS232_PROTO_FREEPORT       2

#ifndef RS232_PROTOCOL_MODE
#define RS232_PROTOCOL_MODE        RS232_PROTO_MODBUS
#endif

/* Reference echo callback in rs232_freeport_example.c.
 * 1 = compile it (only effective in RS232_PROTO_FREEPORT mode).
 * Set to 0 once you provide your own RS232_FreePort_OnFrame(). */
#ifndef RS232_FREEPORT_EXAMPLE
#define RS232_FREEPORT_EXAMPLE     1
#endif

/* ==========================================================================
 * RS232 / USART1 runtime protocol switching
 * ==========================================================================
 * RS232_RUNTIME_PROTOCOL_SWITCH = 1 keeps BOTH the Modbus RTU slave and the
 * free-port path resident in the same image and selects between them at run
 * time through holding register 220 (written through the RS485 port, which is
 * always reachable). No rebuild or reflash is needed to move USART1 between
 * the debug host tool and a custom device protocol.
 *
 * RS232_RUNTIME_PROTOCOL_SWITCH = 0 (default) keeps the historical
 * compile-time-only behaviour: RS232_PROTOCOL_MODE decides at build time and
 * only one USART1 protocol is linked in.
 *
 * Register 220 (read/write, both ports):
 *   0 = USART1 Modbus RTU slave (debug host tool)
 *   1 = USART1 free port       (custom device protocol)
 * The value is persisted to NVRAM and restored after power-up.
 * ========================================================================== */
/* Default: on when the debug service (register map) is present.  The selector
 * lives in motor_debug_service.c, so a build without it must fall back to the
 * compile-time RS232_PROTOCOL_MODE constant. */
#ifndef RS232_RUNTIME_PROTOCOL_SWITCH
#if APP_DEBUG_SERVICE_ENABLE
#define RS232_RUNTIME_PROTOCOL_SWITCH  1
#else
#define RS232_RUNTIME_PROTOCOL_SWITCH  0
#endif
#endif

/* Runtime switching stores its selector inside the debug service, so the two
 * switches are coupled: enable the debug service or fall back to compile-time
 * selection. */
#if RS232_RUNTIME_PROTOCOL_SWITCH && !APP_DEBUG_SERVICE_ENABLE
#error "RS232_RUNTIME_PROTOCOL_SWITCH requires APP_DEBUG_SERVICE_ENABLE=1"
#endif
#endif /* __APP_CONFIG_H__ */
