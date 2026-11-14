/**
 * @file    scheduler.h
 * @brief   协作式多任务调度器 (Cooperative Scheduler) 头文件 — STM32 优化版
 * @details 本模块实现了一个轻量级的时间触发协作式调度器，核心思想如下:
 *
 *          1) 硬件定时器 (SysTick) 在固定周期 (1ms) 产生中断，驱动 scheduler_hw_tick_isr()；
 *          2) 内部分频器将硬件节拍转换为应用节拍 (默认 1ms)，每当应用节拍到达时，
 *             扫描所有任务的倒计时器，将到期任务标记为"就绪" (is_ready = true)；
 *          3) 主循环中的 scheduler_run() 检查就绪标志，依次执行就绪任务并重载倒计时器。
 *
 *          该架构避免了抢占式 RTOS 的栈空间开销和复杂的优先级反转问题，
 *          非常适合资源有限的工业控制场景。
 *
 *          此外，通过 scheduler_register_hook() 可注册一个高频钩子函数，
 *          该钩子在每次硬件中断中直接调用（不经分频），适用于步进电机脉冲驱动、
 *          输入消抖扫描等对时间精度要求较高的场合。
 *
 * @note    使用 32 位计数器，避免了 51 单片机 16 位定时器在长周期运行时的溢出问题。
 */

#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include "app_config.h"  /* 引入系统配置: HW_TICK_US, TICK_MS, MAX_TASKS, ATOMIC 宏 */
#include <stdint.h>      /* uint8_t, uint32_t 等定宽整型 */
#include <stdbool.h>     /* bool / true / false 布尔类型 */

/**
 * @brief 任务函数指针类型定义
 * @note  所有注册到调度器的任务函数必须符合此原型: void func(void)
 */
typedef void (*TaskFunc)(void);

/**
 * @brief 任务控制块 (Task Control Block, TCB)
 * @details 每个注册的任务对应一个 TCB 实例，由调度器以静态数组方式管理。
 *          结构体成员说明:
 *          - func:     任务入口函数指针，为 NULL 表示该槽位未使用
 *          - period:   任务的执行周期，单位为应用节拍数 (App Ticks)
 *          - counter:  内部倒计时器，每个应用节拍递减 1，减至 0 时触发就绪
 *          - is_ready: 就绪标志位，由 ISR 中的 scheduler_tick() 置位，
 *                      由主循环中的 scheduler_run() 清除。
 *                      因跨越 ISR 与主循环两个执行上下文，必须声明为 volatile。
 */
typedef struct
{
    TaskFunc func;          /**< 任务函数的入口地址 (NULL = 空闲槽位) */
    uint32_t period;        /**< 任务的执行周期 (单位: 应用节拍数, App Ticks) */
    uint32_t counter;       /**< 内部倒计时器 (每个应用节拍递减 1) */
    volatile bool is_ready; /**< 就绪标志位 (ISR 写, 主循环读, 必须 volatile) */
} Task_t;

/**
 * @brief  初始化调度器
 * @note   清零所有任务槽位，复位分频器计数，必须在注册任何任务之前调用。
 */
void scheduler_init(void);

/**
 * @brief  向调度器注册一个新任务
 * @param  func    任务函数指针 (不可为 NULL)
 * @param  period  任务执行周期，单位为应用节拍数 (App Ticks)
 *                 实际毫秒周期 = period × SCHEDULER_TICK_MS
 * @return 成功返回任务 ID (0 ~ MAX_TASKS-1)；失败返回 0xFF
 * @note   任务按注册顺序获得递增 ID，注册后立即参与调度。
 */
uint8_t scheduler_add(TaskFunc func, uint32_t period);

/**
 * @brief  动态修改已注册任务的执行周期
 * @param  id      任务 ID (由 scheduler_add() 返回)
 * @param  period  新的执行周期 (单位: 应用节拍数)
 * @note   新周期将在当前倒计时结束后的下一个周期生效。
 */
void scheduler_set_period(uint8_t id, uint32_t period);

/**
 * @brief  硬件节拍中断服务入口
 * @details 必须在硬件定时器中断 (如 SysTick_Handler) 中调用。
 *          每次调用完成以下工作:
 *          1) 执行已注册的高频钩子函数 (如步进电机驱动、输入扫描等)；
 *          2) 对内部分频器计数器加 1，达到分频阈值时递增系统节拍并扫描任务就绪状态。
 * @note   调用频率 = 1 / SCHEDULER_HW_TICK_US (默认 1kHz)。
 */
void scheduler_hw_tick_isr(void);

/**
 * @brief  注册高频硬件节拍钩子函数
 * @param  hook 钩子函数指针 (NULL 表示取消注册)
 * @note   钩子在 ISR 上下文中执行，必须极短且不可阻塞。
 *         典型用途: 步进电机脉冲驱动、输入消抖扫描等。
 */
void scheduler_register_hook(void (*hook)(void));

/**
 * @brief  主循环调度入口
 * @details 在 while(1) 主循环中反复调用。扫描所有任务的 is_ready 标志，
 *          若为 true，则清除标志、重载倒计时器、执行任务函数。
 *          各任务按注册顺序依次执行，不支持抢占。
 */
void scheduler_run(void);

/**
 * @brief  获取当前应用层系统节拍计数
 * @return 自系统启动以来累计的应用节拍数 (32 位无符号，约 49.7 天溢出@1ms 节拍)
 * @note   内部通过 ATOMIC 临界区保护，确保在主循环中读取时不会被 ISR 打断。
 */
uint32_t scheduler_get_ticks(void);

#endif /* __SCHEDULER_H__ */
