/**
 * @file    input.h
 * @brief   积分式输入信号消抖模块 (Input Debounce) 头文件 — 纯软件解耦版
 * @details 本模块实现了一种基于"连续一致采样计数"的数字输入消抖算法 (也称积分式消抖)。
 *          与简单的延时消抖不同，积分式消抖要求输入信号在连续 N 次采样中保持一致，
 *          才确认状态发生了真实变化。若中途出现一次抖动 (电平跳变)，计数器立即清零
 *          重新开始，从而有效过滤机械开关的弹跳噪声。
 *
 *          设计理念:
 *          - 硬件解耦: 通过函数指针 (read_pin) 获取物理电平，不直接依赖 GPIO HAL，
 *            便于在不同平台间移植或进行单元测试。
 *          - 事件驱动: 除了提供实时状态查询 (input_get)，还支持边沿事件检测
 *            (input_check_down / input_check_up)，采用"读后清零"机制避免事件丢失。
 *          - 临界区保护: 事件标志的读取-清除操作使用 ATOMIC 宏保护，
 *            确保 ISR 与主循环之间的数据一致性。
 *
 * @note    极性约定: read_pin() 返回"逻辑态"：
 *          - read_pin() = 1 表示输入逻辑为 True (触发/按下)
 *          - read_pin() = 0 表示输入逻辑为 False (未触发/释放)
 *          具体物理电平极性由上层 I/O 适配层负责转换。
 */

#ifndef __INPUT_H__
#define __INPUT_H__

#include "app_config.h"  /* 依赖 ATOMIC_ENTER / ATOMIC_EXIT 临界区宏 */
#include <stdbool.h>     /* bool / true / false */
#include <stdint.h>      /* uint8_t, uint16_t */

/**
 * @brief 输入消抖对象结构体
 * @details 每个物理输入通道 (如工控板上的 X0 ~ X17) 对应一个 Input_t 实例。
 *          结构体内部封装了完整的消抖状态机，外部仅需通过 API 函数交互。
 */
typedef struct
{
    /**
     * @brief 获取物理引脚电平的回调函数指针
     * @note  返回值: 1 = 逻辑 True(触发), 0 = 逻辑 False(未触发)
     *        该函数由 IODEF 模块提供 (如 X0, X1 等)，在 input_init 时绑定。
     */
    uint8_t (*read_pin)(void);
    
    uint16_t filter_cnt;    /**< 连续一致采样计数器 (每次采样值与上次相同则 +1) */
    uint16_t filter_max;    /**< 消抖阈值 (计数器达到此值时确认状态变化) */

    bool raw_last;          /**< 上一次的原始采样值 (true=高电平, false=低电平) */
    bool stable;            /**< 当前确认的稳定电平状态 (经过消抖后的最终结果) */
    bool is_initialized;    /**< 首次采样同步标志 (用于开机时同步初始状态,避免误触发) */

    bool evt_down;          /**< 按下事件标志 (逻辑上升沿: False→True，待 input_check_down 读取) */
    bool evt_up;            /**< 释放事件标志 (逻辑下降沿: True→False，待 input_check_up 读取) */
} Input_t;

/**
 * @brief  初始化输入消抖对象
 * @param  in              输入对象指针
 * @param  read_pin        获取输入逻辑态的回调函数 (返回 1=True, 0=False)
 * @param  debounce_ticks  消抖时间阈值 (连续一致采样次数)
 *                         实际消抖时间 = debounce_ticks × 扫描周期
 *                         例: debounce_ticks=10, 扫描周期=1ms → 消抖时间=10ms
 * @note   调用后对象进入"未初始化"状态，首次 input_update() 会同步当前物理电平。
 */
void input_init(Input_t *in, uint8_t (*read_pin)(void), uint16_t debounce_ticks);

/**
 * @brief  周期性更新输入状态 (消抖状态机驱动函数)
 * @param  in 输入对象指针
 * @note   必须以固定周期 (如 1ms) 被反复调用。可在定时器 ISR 中直接调用，
 *         也可在调度器的高频钩子 (Tick_Hook) 中通过 IO_Scan() 间接调用。
 *         内部逻辑:
 *         1) 读取物理电平
 *         2) 若与上次不同 → 计数器清零 (抖动)
 *         3) 若与上次相同 → 计数器递增
 *         4) 计数器达到阈值 → 确认新状态，触发边沿事件
 */
void input_update(Input_t *in);

/**
 * @brief  获取当前稳定的逻辑状态 (经过消抖后的结果)
 * @param  in 输入对象指针
 * @return true  — 输入当前处于触发/按下状态
 * @return false — 输入当前处于未触发/释放状态
 */
bool input_get(Input_t *in);

/**
 * @brief  检测"按下"边沿事件 (False->True, 读后自动清除)
 * @param  in 输入对象指针
 * @return true  — 自上次检查以来发生了一次按下事件
 * @return false — 无按下事件
 * @note   使用 ATOMIC 临界区保护，确保标志位的读取-清除操作不会被 ISR 打断。
 *         每次按下只能被检测到一次 (即"消费"该事件)。
 */
bool input_check_down(Input_t *in);

/**
 * @brief  检测"释放"边沿事件 (True->False, 读后自动清除)
 * @param  in 输入对象指针
 * @return true  — 自上次检查以来发生了一次释放事件
 * @return false — 无释放事件
 */
bool input_check_up(Input_t *in);

#endif /* __INPUT_H__ */
