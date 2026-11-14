/**
 * @file    timer.h
 * @brief   非阻塞式软件定时器 (Non-blocking Software Timer) — STM32 优化版
 * @details 基于调度器的应用节拍 (scheduler_get_ticks()) 实现的纯软件定时器，
 *          不占用任何硬件定时器资源。与 HAL_Delay() 等阻塞式延时不同，
 *          软件定时器在等待期间不阻塞 CPU，任务函数可在每次被调度时检查定时器
 *          是否到期，从而实现"设定-轮询"的非阻塞等待模式。
 *
 *          使用 32 位计数器 (uint32_t)，相比 51 单片机的 16 位定时器:
 *          - 溢出周期大幅延长: 以 1ms 节拍计算, 2^32 × 1ms ≈ 49.7 天
 *          - 无需手动处理多次溢出拼接，减少了出错概率
 *
 *          溢出回绕安全性:
 *          elapsed = current_ticks - start_ticks
 *          由于 unsigned 减法的天然回绕特性，即使 current_ticks 发生 uint32 溢出，
 *          上述差值仍然是正确的 (前提: 定时周期 < 2^31 个节拍)。
 *
 * @note    本文件全部为 static inline 函数，直接在头文件中定义，
 *          编译器会将其内联到调用点，零函数调用开销。
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#include "scheduler.h"  /* 依赖 scheduler_get_ticks() 获取系统节拍 */
#include <stdint.h>     /* uint32_t */
#include <stdbool.h>    /* bool */

/**
 * @brief 软件定时器对象结构体
 * @details 每个需要非阻塞延时的场景 (如 FSM 超时、通信帧间隔) 均可实例化一个。
 *          当前占用约 16 字节 (3 × uint32_t + bool + 对齐填充)，仍为轻量级开销。
 */
typedef struct
{
    uint32_t start;   /**< 定时器启动时的节拍快照 (调用 timer_set 时记录) */
    uint32_t timeout; /**< 设定的超时时长 (单位: 应用节拍数, App Ticks) */
    uint32_t pause_mark; /**< 暂停时刻快照 (用于恢复时补偿暂停时长) */
    bool paused;         /**< true=已暂停(冻结计时), false=正常计时 */
} SoftTimer_t;

/**
 * @brief  启动 / 重置软件定时器
 * @param  t             软件定时器对象指针
 * @param  timeout_ticks 超时时长 (单位: 应用节拍数)
 *                       实际毫秒 = timeout_ticks × SCHEDULER_TICK_MS
 *                       例: timeout_ticks = 500 时, 实际超时 = 500 × 1ms = 500ms
 * @note   多次调用会重新开始计时 (即"重置"效果)。
 */
static inline void timer_set(SoftTimer_t *t, uint32_t timeout_ticks)
{
    t->start = scheduler_get_ticks();  /* 记录当前系统节拍作为起始时间戳 */
    t->timeout = timeout_ticks;        /* 保存超时阈值 */
    t->pause_mark = 0;
    t->paused = false;
}

/**
 * @brief  暂停软件定时器 (冻结计时)
 * @param  t 软件定时器对象指针
 * @note   重复调用安全: 若已暂停则不重复记录 pause_mark。
 */
static inline void timer_pause(SoftTimer_t *t)
{
    if (!t->paused)
    {
        t->pause_mark = scheduler_get_ticks();
        t->paused = true;
    }
}

/**
 * @brief  恢复软件定时器 (继续计时)
 * @param  t 软件定时器对象指针
 * @details 通过将暂停时长补偿到 start，实现"暂停期间不计时"。
 */
static inline void timer_resume(SoftTimer_t *t)
{
    if (t->paused)
    {
        uint32_t pause_duration = scheduler_get_ticks() - t->pause_mark;
        t->start += pause_duration;
        t->paused = false;
    }
}

/**
 * @brief  查询定时器是否处于暂停状态
 * @param  t 软件定时器对象指针
 * @return true=暂停中, false=未暂停
 */
static inline bool timer_is_paused(const SoftTimer_t *t)
{
    return t->paused;
}

/**
 * @brief  检查软件定时器是否已超时
 * @param  t 软件定时器对象指针
 * @return true  — 已超时 (elapsed >= timeout)
 * @return false — 尚在计时中
 * @details 通过无符号减法 (current - start) 自动处理 uint32_t 溢出回绕，
 *          无需额外的溢出判断逻辑。只要超时时长不超过 2^31 个节拍
 *          (约 24.8 天 @1ms 节拍)，判断结果始终正确。
 */
static inline bool timer_expired(SoftTimer_t *t)
{
    uint32_t now = t->paused ? t->pause_mark : scheduler_get_ticks();
    uint32_t elapsed = now - t->start;  /* 计算已经历的节拍数(暂停时冻结到 pause_mark) */
    return (elapsed >= t->timeout);  /* 是否达到或超过设定的超时阈值 */
}

#endif /* __TIMER_H__ */
