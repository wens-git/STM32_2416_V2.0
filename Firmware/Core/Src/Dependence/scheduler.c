/**
 * @file    scheduler.c
 * @brief   协作式多任务调度器 (Cooperative Scheduler) 源文件
 * @details 本文件实现了调度器的核心逻辑，包括:
 *          - 任务注册与管理 (静态数组 + 计数器)
 *          - 硬件节拍到应用节拍的分频转换
 *          - ISR 驱动的任务就绪标记
 *          - 主循环轮询执行就绪任务
 *          - 高频钩子函数回调机制
 *
 *          执行流程示意:
 *          ┌─────────────────────────────────────────────────────┐
 *          │ SysTick (1ms) ──► scheduler_hw_tick_isr()          │
 *          │   ├─ g_tickHook()          ← 高频钩子 (每 1ms)     │
 *          │   └─ 分频计数到达 ──► scheduler_tick()             │
 *          │       └─ 扫描所有任务 counter, 到期者 is_ready=true │
 *          │                                                     │
 *          │ main while(1) ──► scheduler_run()                  │
 *          │   └─ 遍历 is_ready==true 的任务:                    │
 *          │       ├─ is_ready = false                           │
 *          │       ├─ counter = period (重载)                    │
 *          │       └─ func() 执行任务                            │
 *          └─────────────────────────────────────────────────────┘
 */

#include "scheduler.h"
#include "stm32f4xx_hal.h"  /* STM32F407 HAL 驱动头文件 */

/* ==========================================================================
 * 私有静态变量定义 (模块内部，外部不可直接访问)
 * ========================================================================== */

/** @brief 任务控制块数组 — 以静态方式预分配 MAX_TASKS 个任务槽位 */
static Task_t g_tasks[SCHEDULER_MAX_TASKS];

/** @brief 当前已注册的任务数量 (同时也是下一个可用的任务 ID) */
static uint8_t g_taskCount = 0;

/**
 * @brief 应用层系统节拍计数器 (自启动以来的累计应用节拍数)
 * @note  该变量在 ISR 中递增、在主循环中读取，必须声明为 volatile
 *        以防止编译器优化掉对它的多次读取。
 */
static volatile uint32_t g_system_ticks = 0;

/**
 * @brief 高频硬件节拍钩子函数指针
 * @note  每次硬件中断均调用 (不经分频)，适用于步进电机脉冲、输入消抖等
 *        对时间精度要求高的场合。NULL 表示未注册。
 */
static void (*g_tickHook)(void) = NULL;

/**
 * @brief 分频器当前计数值
 * @details 每次硬件中断递增 1，达到 g_scheduler_divider_max 时
 *          产生一个应用节拍并复位为 0。
 */
static uint32_t g_scheduler_divider = 0;

/**
 * @brief 分频器阈值 (硬件节拍数 → 1 个应用节拍)
 * @details 在 scheduler_init() 中根据配置参数计算:
 *          g_scheduler_divider_max = (SCHEDULER_TICK_MS × 1000) / SCHEDULER_HW_TICK_US
 *          例: (1ms × 1000) / 1000us = 1
 */
static uint32_t g_scheduler_divider_max = 0;

/* ==========================================================================
 * 初始化与任务注册
 * ========================================================================== */

/**
 * @brief  初始化调度器 — 清零所有任务槽位并计算分频阈值
 * @note   必须在注册任何任务之前调用，且仅调用一次。
 */
void scheduler_init(void)
{
    uint8_t i;

    /* 遍历所有任务槽位，将函数指针清空、就绪标志复位 */
    for (i = 0; i < SCHEDULER_MAX_TASKS; i++)
    {
        g_tasks[i].func = NULL;       /* 标记为空闲槽位 */
        g_tasks[i].is_ready = false;  /* 未就绪 */
    }

    g_taskCount = 0;       /* 已注册任务数归零 */
    g_system_ticks = 0;    /* 系统节拍计数器归零 */
    
    g_scheduler_divider = 0;  /* 分频器当前计数归零 */

    /* 计算分频阈值: 多少次硬件中断等于一个应用节拍 */
    /* 公式: (应用节拍周期_ms × 1000) / 硬件中断周期_us */
    g_scheduler_divider_max = (SCHEDULER_TICK_MS * 1000) / SCHEDULER_HW_TICK_US;
}

/**
 * @brief  向调度器注册一个新任务
 * @param  func   任务入口函数指针 (不允许为 NULL)
 * @param  period 执行周期，单位: 应用节拍数 (实际毫秒 = period × SCHEDULER_TICK_MS)
 * @return 任务 ID (0 ~ MAX_TASKS-1)，注册失败返回 0xFF
 */
uint8_t scheduler_add(TaskFunc func, uint32_t period)
{
    uint8_t id;

    /* 参数合法性检查: 函数指针非空 且 任务槽位未满 */
    if (func == NULL || g_taskCount >= SCHEDULER_MAX_TASKS)
    {
        return 0xFF;  /* 注册失败 */
    }

    id = g_taskCount++;  /* 分配任务 ID 并递增计数器 (先用后加) */

    g_tasks[id].func = func;       /* 绑定任务函数 */
    g_tasks[id].period = period;   /* 设置执行周期 */
    g_tasks[id].counter = period;  /* 初始倒计时器设为一个完整周期 (首次触发需等待一个完整周期) */
    g_tasks[id].is_ready = false;  /* 初始状态为未就绪 */

    return id;  /* 返回分配的任务 ID */
}

/**
 * @brief  注册高频硬件节拍钩子函数
 * @param  hook 钩子函数指针 (NULL 表示取消注册)
 * @note   钩子在 ISR 上下文中执行，必须极短且不可阻塞。
 */
void scheduler_register_hook(void (*hook)(void))
{
    g_tickHook = hook;  /* 直接赋值，下次硬件中断时即生效 */
}

/* ==========================================================================
 * 核心调度逻辑 (ISR 端)
 * ========================================================================== */

/**
 * @brief  应用节拍处理 — 扫描并更新所有任务的倒计时器
 * @details 由 scheduler_hw_tick_isr() 在分频条件满足时调用。
 *          遍历已注册的任务列表，对每个任务的 counter 递减:
 *          - counter > 0: 继续递减，尚未到期
 *          - counter == 0: 将 is_ready 置为 true，标记任务就绪
 *          注意: 此时不重载 counter，重载操作由 scheduler_run() 在执行任务后完成，
 *          这确保了即使任务执行时间较长，也不会错误地累加延迟。
 */
static void scheduler_tick(void)
{
    uint8_t i;

    for (i = 0; i < g_taskCount; i++)
    {
        if (g_tasks[i].func)  /* 仅处理已注册的有效任务 (函数指针非空) */
        {
            if (g_tasks[i].counter > 0)  /* counter > 0: 尚未到期, 需继续递减 */
            {
                if (--g_tasks[i].counter == 0)     /* 递减后恰好为 0: 该任务本轮到期 */
                {
                    g_tasks[i].is_ready = true;    /* 置位就绪标志, 主循环中 scheduler_run() 将执行此任务 */
                }
            }
        }
    }
}

/**
 * @brief  硬件定时器中断服务入口 — 由 SysTick_Handler (或 TIMx 中断) 调用
 * @details 每次硬件中断 (默认每 1ms) 执行以下操作:
 *          1) 调用高频钩子 (如步进电机脉冲驱动、输入消抖扫描)；
 *          2) 分频计数器递增，达到阈值时产生一个应用节拍并扫描任务就绪。
 */
void scheduler_hw_tick_isr(void)
{
    /* 第一步: 执行高频钩子 (每次硬件中断都调用，不经分频) */
    if (g_tickHook)
    {
        g_tickHook();  /* 调用已注册的钩子函数 (如 Tick_Hook) */
    }

    /* 第二步: 分频计数，判断是否到达一个应用节拍 */
    if (++g_scheduler_divider >= g_scheduler_divider_max)
    {
        g_scheduler_divider = 0;  /* 分频计数器复位 */
        g_system_ticks++;         /* 应用层系统节拍计数器递增 */
        scheduler_tick();         /* 扫描所有任务的倒计时器，标记就绪 */
    }
}

/* ==========================================================================
 * 核心调度逻辑 (主循环端)
 * ========================================================================== */

/**
 * @brief  主循环调度入口 — 在 while(1) 中反复调用
 * @details 遍历所有任务，若 is_ready == true:
 *          1) 清除就绪标志 (is_ready = false)
 *          2) 重载倒计时器 (counter = period) — 为下一个周期做准备
 *          3) 执行任务函数 (func())
 *          任务按注册顺序依次检查和执行，属于协作式调度 (非抢占)。
 */
void scheduler_run(void)
{
    uint8_t i;

    for (i = 0; i < g_taskCount; i++)
    {
        if (g_tasks[i].is_ready)
        {
            g_tasks[i].is_ready = false;       /* 清除就绪标志，防止重复执行 */
            g_tasks[i].counter = g_tasks[i].period;  /* 重载倒计时器 */
            g_tasks[i].func();                 /* 执行任务函数 (在主循环上下文中) */
        }
    }
}

/* ==========================================================================
 * 辅助接口
 * ========================================================================== */

/**
 * @brief  获取当前应用层系统节拍计数 (线程安全)
 * @return 自系统启动以来累计的应用节拍数
 * @note   使用临界区保护以确保在主循环中读取 32 位变量时不会被 ISR 打断
 *         (虽然 Cortex-M3 对 32 位对齐访问是原子的，但加保护更加严谨)。
 */
uint32_t scheduler_get_ticks(void)
{
    uint32_t t;

    ATOMIC_ENTER();        /* 关闭全局中断，进入临界区 */
    t = g_system_ticks;    /* 读取节拍计数器当前值 */
    ATOMIC_EXIT();         /* 恢复中断状态，退出临界区 */

    return t;
}

/**
 * @brief  动态修改已注册任务的执行周期
 * @param  id     任务 ID (由 scheduler_add 返回值确定)
 * @param  period 新的执行周期 (单位: 应用节拍数)
 * @note   仅当 id 合法时执行修改，新周期将在当前倒计时结束后的下一轮生效。
 */
void scheduler_set_period(uint8_t id, uint32_t period)
{
    if (id < g_taskCount)         /* 校验任务 ID 合法性 */
        g_tasks[id].period = period;  /* 更新周期值 */
}
