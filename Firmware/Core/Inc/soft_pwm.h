/**
 * @file    soft_pwm.h
 * @brief   普通数字输出的软件脉冲发生器接口。
 * @details 使用 TIM5 作为 1 MHz 自由运行时基，通过比较中断调度多个普通 Y 输出，
 *          为没有硬件 PWM 复用功能的输出端提供独立频率和占空比。该模块主要用于
 *          扩展步进电机脉冲通道，不适合对抖动极其敏感的高速脉冲场景。
 */
#ifndef __SOFT_PWM_H__
#define __SOFT_PWM_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SOFT_PWM_CH_Y6 = 0, /**< 软件脉冲通道 Y6。 */
    SOFT_PWM_CH_Y10,    /**< 软件脉冲通道 Y10。 */
    SOFT_PWM_CH_Y12,    /**< 软件脉冲通道 Y12。 */
    SOFT_PWM_CH_Y14,    /**< 软件脉冲通道 Y14。 */
    SOFT_PWM_CH_COUNT   /**< 软件脉冲通道总数，不是有效通道。 */
} SoftPWM_Channel_t;

#define SOFT_PWM_FREQ_MIN  100U    /**< 允许设置的最低频率，单位 Hz。 */
#define SOFT_PWM_FREQ_MAX  100000U /**< 允许设置的最高频率，单位 Hz。 */

/** @brief 脉冲周期完成回调；每个下降沿结束后在 TIM5 中断中调用。 */
typedef void (*SoftPWM_PulseCallback_t)(SoftPWM_Channel_t ch);

/** @brief 初始化 TIM5 时基和软件脉冲运行状态。 */
bool SoftPWM_Init(void);
/**
 * @brief 启用指定软件脉冲通道。
 * @param ch       软件脉冲通道。
 * @param freq_hz  目标频率，超出范围时自动限幅。
 * @param duty_pct 占空比百分数，内部限制为 1% 至 99%。
 * @param callback 周期完成回调，可为 NULL；回调在中断上下文执行。
 */
void SoftPWM_EnableIT(SoftPWM_Channel_t ch, uint32_t freq_hz, uint8_t duty_pct,
                      SoftPWM_PulseCallback_t callback);
/** @brief 停止指定通道并立即将对应 Y 输出拉低。 */
void SoftPWM_Disable(SoftPWM_Channel_t ch);
/** @brief 查询指定通道是否正在输出脉冲。 */
bool SoftPWM_IsEnabled(SoftPWM_Channel_t ch);
/** @brief 获取通道当前频率；通道无效或未启用时返回 0。 */
uint32_t SoftPWM_GetFreq(SoftPWM_Channel_t ch);
/** @brief 获取通道当前占空比；通道无效或未启用时返回 0。 */
uint8_t SoftPWM_GetDuty(SoftPWM_Channel_t ch);

/** @brief 返回 TIM5 句柄，供统一中断入口识别软件脉冲定时器。 */
TIM_HandleTypeDef *SoftPWM_GetTimHandle(void);
/** @brief TIM5 比较中断处理入口，由芯片中断服务函数调用。 */
void SoftPWM_TimerElapsedISR(TIM_HandleTypeDef *htim);

#endif /* __SOFT_PWM_H__ */
