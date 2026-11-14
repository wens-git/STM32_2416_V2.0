#ifndef __BSP_IWDG_H
#define __BSP_IWDG_H

#include "main.h"

/**
 * @file bsp_iwdg.h
 * @brief 独立看门狗 (IWDG) 驱动 — 工业级必备
 * @details IWDG 使用独立的 LSI (典型 32kHz) 时钟源，即使主时钟故障也能复位系统。
 *          必须在主循环或专用任务中周期性调用 BSP_IWDG_Feed()。
 */

/**
 * @brief  初始化独立看门狗
 * @param  timeout_ms 看门狗标称超时时间 (毫秒)
 * @note   驱动按 LSI 47kHz 上界和 256 分频计算，保证实际窗口不短于请求值；
 *         LSI 较慢时实际窗口会相应延长。
 */
void BSP_IWDG_Init(uint32_t timeout_ms);

/**
 * @brief  喂狗 (刷新看门狗计数器)
 * @note   必须在超时之前调用，否则系统将被硬件复位
 */
void BSP_IWDG_Feed(void);

#endif /* __BSP_IWDG_H */
