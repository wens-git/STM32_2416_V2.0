/**
 * @file    fsm.h
 * @brief   有限状态机 (Finite State Machine, FSM) 框架 — STM32 优化版
 * @details 本文件提供了一个轻量级、通用的有限状态机工具集，适用于工控设备中
 *          需要按步骤执行的流程控制场景，例如:
 *          - 步进电机的多段运动流程 (回零 → 加速 → 匀速 → 减速 → 停止)
 *          - 通信协议的握手流程 (连接 → 认证 → 数据交换 → 断开)
 *          - 设备自检流程 (上电 → 检查传感器 → 检查通信 → 就绪)
 *
 *          核心设计理念:
 *          移除了 51 单片机时代为节省 RAM 而大量使用的位域和合并标志位技巧，
 *          改用结构体成员明确表示状态值、子步骤、进入标志和定时器，
 *          以牺牲少量 RAM (每个 FSM 约 20 字节) 换取显著的可读性和维护性提升。
 *
 *          此外还提供了 StepTimer_t 分步延时工具，适用于在同一个任务函数的
 *          多步 switch-case 结构中实现非阻塞延时。
 *
 * @note    全部为 static inline 函数，零调用开销，编译器直接内联展开。
 */

#ifndef __FSM_H__
#define __FSM_H__

#include "timer.h"      /* 依赖 SoftTimer_t 及 timer_set() / timer_expired() */
#include "app_config.h" /* 依赖 SCHEDULER_TICK_MS 进行 ms → ticks 换算 */
#include <stdint.h>     /* uint8_t, uint32_t */
#include <stdbool.h>    /* bool */

/* ==========================================================================
 * 分步延时工具 (Step Delay Helper)
 * ==========================================================================
 * 用于在 switch-case 多步骤流程中实现非阻塞延时。
 * 典型用法:
 *   case STEP_WAIT:
 *       if (Step_Delay(&my_timer, 500)) {  // 500ms 后返回 true
 *           goto_next_step();
 *       }
 *       break;
 */

/**
 * @brief 分步延时器结构体
 * @details 与 SoftTimer_t 的区别在于: StepTimer_t 自带"运行中"标志，
 *          首次调用 Step_Delay() 时自动启动，无需显式调用 timer_set()。
 */
typedef struct
{
    uint32_t start_tick; /**< 延时起始时间快照 (应用节拍) */
    uint32_t pause_mark; /**< 暂停时刻快照 (恢复时用于补偿暂停时长) */
    bool running;        /**< true = 正在计时中, false = 空闲 (可重新启动) */
    bool paused;         /**< true = 已暂停(冻结计时), false = 正常计时 */
} StepTimer_t;

/**
 * @brief 状态机控制块 (Fsm_t)
 * @details 每个独立的状态机流程应实例化一个 Fsm_t 对象。
 *          内部集成了:
 *          - state:   当前状态值 (对应 switch-case 的 case 标号)
 *          - step:    子步骤计数器 (用于同一状态内的多步顺序执行)
 *          - entered: "刚进入状态"标志 (用于在进入新状态时执行一次性初始化)
 *          - timer:   状态内置软件定时器 (用于超时检测、防抖等待等)
 */
typedef struct
{
    uint8_t state;     /**< 当前状态值 (状态枚举的数值) */
    uint8_t step;      /**< 子步骤计数器 (用于 switch-case 多步逻辑) */
    bool entered;      /**< "刚进入该状态"标志: true=刚切换到此状态,需执行初始化 */
    SoftTimer_t timer; /**< 状态内置非阻塞定时器 (用于超时判断、延时等待) */
} Fsm_t;

/* ==========================================================================
 * FSM 基础操作函数
 * ========================================================================== */

/**
 * @brief  初始化状态机 — 设定初始状态并重置所有内部变量
 * @param  fsm           状态机对象指针
 * @param  initial_state 初始状态值 (通常为枚举的第一个值, 如 STATE_IDLE)
 */
static inline void fsm_init(Fsm_t *fsm, uint8_t initial_state)
{
    fsm->state = initial_state;
    fsm->entered = true;
    fsm->step = 0;
    timer_set(&fsm->timer, 0);
}

/**
 * @brief  检查是否刚刚进入当前状态 (读后自动清除)
 * @param  fsm 状态机对象指针
 * @return true  — 是，这是进入该状态后的首次调用 (可执行初始化逻辑)
 * @return false — 否，之前已经处理过进入逻辑
 */
static inline bool fsm_entered(Fsm_t *fsm)
{
    if (fsm->entered)
    {
        fsm->entered = false;
        return true;
    }
    return false;
}

/**
 * @brief  直接设置状态值 (不触发 entered 标志)
 * @param  fsm       状态机对象指针
 * @param  new_state 新状态值
 */
static inline void fsm_set_state(Fsm_t *fsm, uint8_t new_state)
{
    fsm->state = new_state;
}

/**
 * @brief  获取当前状态值
 * @param  fsm 状态机对象指针
 * @return 当前状态的数值
 */
static inline uint8_t fsm_get_state(Fsm_t *fsm)
{
    return fsm->state;
}

/**
 * @brief  跳转到新状态 (带入口标志和子步骤重置)
 * @param  fsm        状态机对象指针
 * @param  next_state 目标状态值
 * @note   仅当目标状态与当前状态不同时才执行跳转，避免重复触发入口逻辑。
 */
static inline void fsm_goto(Fsm_t *fsm, uint8_t next_state)
{
    if (fsm->state != next_state)
    {
        fsm->state = next_state;
        fsm->entered = true;
        fsm->step = 0;
    }
}

/**
 * @brief  重启当前状态 — 允许同状态自跳转 (重新触发 entered 标志)
 * @param  fsm 状态机对象指针
 */
static inline void fsm_restart(Fsm_t *fsm)
{
    fsm->entered = true;
    fsm->step = 0;
}

/**
 * @brief  子步骤递增 — 用于同一状态内的多步顺序执行
 * @param  fsm 状态机对象指针
 */
static inline void fsm_next_step(Fsm_t *fsm)
{
    fsm->step++;
}

/**
 * @brief  带扫描条件的等待函数 — 条件满足后开始计时，计时到期后返回 true
 * @param  fsm       状态机对象指针
 * @param  condition 前置条件 (true = 条件已满足)
 * @param  ms        条件满足后需等待的延时 (毫秒)
 * @return true  — 条件已满足且延时已到期
 * @return false — 条件未满足或延时尚未到期
 */
static inline bool fsm_wait_input(Fsm_t *fsm, bool condition, uint32_t ms)
{
    if (fsm->entered || !condition)
    {
        uint32_t ticks = ms / SCHEDULER_TICK_MS;
        if (ticks == 0 && ms > 0)
            ticks = 1;
        timer_set(&fsm->timer, ticks);
        return false;
    }
    return timer_expired(&fsm->timer);
}

/**
 * @brief  设置状态超时定时器 — 用于状态执行最大时限保护
 * @param  fsm 状态机对象指针
 * @param  ms  超时时间 (毫秒)
 */
static inline void fsm_set_timeout(Fsm_t *fsm, uint32_t ms)
{
    uint32_t ticks = ms / SCHEDULER_TICK_MS;
    if (ticks == 0 && ms > 0)
        ticks = 1;
    timer_set(&fsm->timer, ticks);
}

/**
 * @brief  检查状态内置定时器是否已超时
 * @param  fsm 状态机对象指针
 * @return true — 已超时, false — 计时中
 */
static inline bool fsm_timeout(Fsm_t *fsm)
{
    return timer_expired(&fsm->timer);
}

/* ==========================================================================
 * 分步延时工具函数
 * ========================================================================== */

/**
 * @brief  分步非阻塞延时 — 首次调用自动启动，后续调用轮询检查是否到期
 * @param  timer 分步延时器对象指针
 * @param  ms    延时时长 (毫秒)
 * @return true  — 延时已到期 (同时自动复位 running 标志，可再次使用)
 * @return false — 延时尚未到期，继续等待
 */
static inline bool Step_Delay(StepTimer_t *timer, uint32_t ms)
{
    uint32_t ticks_needed;

    if (!timer->running)
    {
        timer->start_tick = scheduler_get_ticks();
        timer->pause_mark = 0;
        timer->running = true;
        timer->paused = false;
        return false;
    }

    if (timer->paused)
    {
        return false;
    }

    ticks_needed = ms / SCHEDULER_TICK_MS;
    if (ticks_needed == 0 && ms > 0)
        ticks_needed = 1;

    if ((scheduler_get_ticks() - timer->start_tick) >= ticks_needed)
    {
        timer->running = false;
        timer->paused = false;
        return true;
    }
    return false;
}

/**
 * @brief  手动复位分步延时器 — 强制取消当前延时
 * @param  timer 分步延时器对象指针
 */
static inline void Step_Reset(StepTimer_t *timer)
{
    timer->running = false;
    timer->paused = false;
    timer->start_tick = 0;
    timer->pause_mark = 0;
}

/**
 * @brief  暂停分步延时器 — 冻结当前计时进度
 * @param  timer 分步延时器对象指针
 */
static inline void Step_Pause(StepTimer_t *timer)
{
    if (timer->running && !timer->paused)
    {
        timer->pause_mark = scheduler_get_ticks();
        timer->paused = true;
    }
}

/**
 * @brief  恢复分步延时器 — 继续剩余等待时间
 * @param  timer 分步延时器对象指针
 */
static inline void Step_Resume(StepTimer_t *timer)
{
    if (timer->running && timer->paused)
    {
        uint32_t paused_ticks = scheduler_get_ticks() - timer->pause_mark;
        timer->start_tick += paused_ticks;
        timer->paused = false;
    }
}

/**
 * @brief  查询分步延时器是否处于暂停状态
 * @param  timer 分步延时器对象指针
 * @return true=暂停中, false=未暂停
 */
static inline bool Step_IsPaused(const StepTimer_t *timer)
{
    return timer->paused;
}

#endif /* __FSM_H__ */
