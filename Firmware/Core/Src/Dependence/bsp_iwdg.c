/**
 * @file    bsp_iwdg.c
 * @brief   独立看门狗 (IWDG) 驱动源文件
 * @details 独立看门狗使用片内 LSI 低速振荡器 (~32kHz) 作为时钟源，
 *          即使主时钟 (HSI/HSE/PLL) 全部故障，IWDG 仍可正常计时并触发硬件复位。
 *          
 *          这是工业控制系统中防止程序跑飞(死锁/死循环)的最后一道防线。
 *
 *          为保证 LSI 快端频差下仍满足请求窗口，重载值按 47kHz 上界计算，
 *          并使用 256 分频。LSI 较慢时实际窗口会更长。
 */

#include "bsp_iwdg.h"
#include <stdbool.h>

/** @brief IWDG HAL 句柄 (模块内部使用，不对外暴露) */
static IWDG_HandleTypeDef hiwdg;
static bool iwdg_started = false;

#define IWDG_LSI_MAX_HZ       47000U
#define IWDG_PRESCALER_VALUE  256U

/* ==========================================================================
 *  BSP_IWDG_Init() — 初始化并启动独立看门狗
 * ========================================================================== */

/**
 * @brief  初始化独立看门狗
 * @param  timeout_ms 期望的看门狗超时时间 (单位: 毫秒)
 * @details 计算过程:
 *          - LSI 实际值可能在 17~47kHz 之间波动
 *          - 分频系数选择 256
 *          - Reload 按 47kHz 上界计算，保证实际超时不短于 timeout_ms
 *          - Reload 范围: 1 ~ 4095 (12位计数器)
 *
 * @note   IWDG 一旦启动就无法停止 (硬件特性)。
 *         调试时需注意: 如果使用 DBGMCU 可以冻结 IWDG 计数器。
 */
void BSP_IWDG_Init(uint32_t timeout_ms)
{
    uint32_t reload = (timeout_ms * IWDG_LSI_MAX_HZ +
                       (IWDG_PRESCALER_VALUE * 1000U - 1U)) /
                      (IWDG_PRESCALER_VALUE * 1000U);

    /* 限幅: IWDG 重载寄存器为 12 位。 */
    if (reload > 4095) reload = 4095;
    /* 最小值保护: 避免设为 0 导致立即复位 */
    if (reload < 1)    reload = 1;

    hiwdg.Instance       = IWDG;               /* 独立看门狗外设实例 */
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Reload    = reload;              /* 重载值: 计数到 0 时触发复位 */

    /* 调用 HAL 初始化 — 内部会依次写入 Key 寄存器、预分频、重载值，并启动看门狗 */
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();  /* 初始化失败：进入错误处理 */
    }
    iwdg_started = true;
}

/* ==========================================================================
 *  BSP_IWDG_Feed() — 喂狗 (刷新计数器)
 * ========================================================================== */

/**
 * @brief  刷新看门狗计数器 (俗称"喂狗")
 * @note   必须在超时时间内周期性调用此函数，否则看门狗将触发硬件复位。
 *         在本系统中由调度器的 Task_WDT_Feed 任务 (500ms 周期) 调用。
 *         当前窗口还需覆盖 NVRAM 扇区擦除期间的主循环停顿。
 */
void BSP_IWDG_Feed(void)
{
    if (iwdg_started) {
        HAL_IWDG_Refresh(&hiwdg);  /* 向 Key 寄存器写入 0xAAAA，重载计数器 */
    }
}
