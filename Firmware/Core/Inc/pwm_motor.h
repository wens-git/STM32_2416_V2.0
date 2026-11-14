/**
 * @file    pwm_motor.h
 * @brief   硬件 PWM 电机控制模块 — 支持 GPIO/PWM 双模式切换
 * @details 本模块利用 STM32F407 的硬件定时器 (TIM1/3/4/8) 产生 PWM 信号，
 *          为步进电机/直流电机提供精确的频率和占空比控制，取代纯软件脉冲方案。
 * 
 *          那现在只使用4个通道，Y1，Y3，Y4和Y16，让这些控制电机的引脚的定时器独立，让频率可以单独控制
 *          因为步进电机可能使用较高细分，所以配置上限为 100kHz；实际速度仍需按硬件与负载降额。
 *          
 *   
 *          【支持的 PWM 通道】(4 路, 均为具备 TIM AF 功能的 Y 输出口)
 *          ┌─────────┬──────┬────────────┬────────────────────────────┐
 *          │ 通道    │ 引脚 │ 定时器     │ 同组说明                   │
 *          ├─────────┼──────┼────────────┼────────────────────────────┤
 *          │ Y1      │ PC6  │ TIM8_CH1   │ 独立频率 (TIM8)            │
 *          │ Y3      │ PA8  │ TIM1_CH1   │ 独立频率 (TIM1)            │
 *          │ Y4      │ PB7  │ TIM4_CH2   │ 独立频率 (TIM4)            │
 *          │ Y16     │ PB5  │ TIM3_CH2   │ 独立频率 (TIM3)            │
 *          └─────────┴──────┴────────────┴────────────────────────────┘
 *
 *          【频率范围】 100 ~ 100000 Hz (1 MHz 计数器时钟, 100kHz 时 ARR=9)
 *
 *          【双模式切换】
 *          - 默认: GPIO 推挽输出模式 (与 IODEF.c 中 Yn() 函数兼容)
 *          - PWM_Enable():  切换为 AF 推挽模式 → 硬件定时器自动产生 PWM
 *          - PWM_Disable(): 切换回 GPIO 推挽输出 → 恢复为普通数字输出
 *
 *          【独立频率控制】
 *          目前配置的 4 个通道分别使用不同的定时器 (TIM8/TIM1/TIM4/TIM3)，
 *          因此各自的频率可以完全独立设置，互不影响。
 */

#ifndef __PWM_MOTOR_H__
#define __PWM_MOTOR_H__

#include "app_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * PWM 通道标识枚举
 * ========================================================================== */

/**
 * @brief PWM 通道枚举 — 仅列出具备硬件 PWM 功能的 Y 输出口
 * @note  枚举值同时作为内部查找表索引使用
 */
typedef enum
{
    PWM_CH_Y1 = 0,   /**< Y1  → PC6, TIM8_CH1 (TIM8 组, 独立频率) */
    PWM_CH_Y3,       /**< Y3  → PA8, TIM1_CH1 (TIM1 组, 独立频率) */
    PWM_CH_Y4,       /**< Y4  → PB7, TIM4_CH2 (TIM4 组, 独立频率) */
    PWM_CH_Y16,      /**< Y16 → PB5, TIM3_CH2 (TIM3 组, 独立频率) */
    PWM_CH_COUNT     /**< 通道总数 (4 路) */
} PWM_Channel_t;

/* ==========================================================================
 * 频率限制常量
 * ========================================================================== */
#define PWM_FREQ_MIN   100U    /**< 最低频率 100 Hz */
#define PWM_FREQ_MAX   100000U /**< 最高频率 100000 Hz (100kHz) */

/* ==========================================================================
 * 公共 API
 * ========================================================================== */

/**
 * @brief  初始化 PWM 模块 — 使能定时器时钟并配置时基
 * @note   必须在 IO_Init() 之后调用。初始化后所有通道仍为 GPIO 模式，
 *         需调用 PWM_Enable() 才会切换到 PWM 输出。
 */
void PWM_Motor_Init(void);

/**
 * @brief  启用指定通道的 PWM 输出 (GPIO → AF 模式切换)
 * @param  ch       PWM 通道标识 (PWM_CH_Y1 ~ PWM_CH_Y17)
 * @param  freq_hz  PWM 频率 (Hz), 自动钳位到 [PWM_FREQ_MIN, PWM_FREQ_MAX]
 * @param  duty_pct 占空比 (0~100%), 50% 适用于步进电机脉冲
 * @return true=启动成功，false=通道无效、同组频率冲突或 HAL 启动失败
 * @note   同一定时器组已有其他活动通道时，只接受相同频率。
 */
bool PWM_Enable(PWM_Channel_t ch, uint32_t freq_hz, uint8_t duty_pct);

/**
 * @brief  禁用指定通道的 PWM 输出 (AF → GPIO 模式切换)
 * @param  ch PWM 通道标识
 * @note   禁用后引脚恢复为 GPIO 推挽输出 (默认高电平)，
 *         可继续使用 IODEF.h 中对应的 Yn() 函数控制。
 */
void PWM_Disable(PWM_Channel_t ch);

/**
 * @brief  修改指定通道所在定时器组的频率
 * @param  ch      PWM 通道标识 (通过它确定定时器组)
 * @param  freq_hz 新频率 (Hz), 自动钳位到 [PWM_FREQ_MIN, PWM_FREQ_MAX]
 * @return true=频率设置成功，false=通道未启用、无效或同组频率冲突
 * @note   仅在通道已启用 (PWM_Enable 后) 时有效。
 */
bool PWM_SetFreq(PWM_Channel_t ch, uint32_t freq_hz);

/**
 * @brief  修改指定通道的占空比
 * @param  ch       PWM 通道标识
 * @param  duty_pct 新占空比 (0~100%)
 * @note   各通道占空比独立可调，不影响同组其他通道。
 *         仅在通道已启用 (PWM_Enable 后) 时有效。
 */
void PWM_SetDuty(PWM_Channel_t ch, uint8_t duty_pct);

/**
 * @brief  启用指定通道的 PWM 输出并开启脉冲计数中断 (GPIO → AF 模式切换)
 * @param  ch       PWM 通道标识 (PWM_CH_Y1 ~ PWM_CH_Y17)
 * @param  freq_hz  PWM 频率 (Hz), 自动钳位到 [PWM_FREQ_MIN, PWM_FREQ_MAX]
 * @param  duty_pct 占空比 (0~100%), 50% 适用于步进电机脉冲
 * @return true=启动成功，false=通道无效、同组频率冲突或 HAL 启动失败
 * @note   启动后每个 PWM 周期触发一次 CC 中断，用于精确脉冲计数。
 *         中断回调: HAL_TIM_PWM_PulseFinishedCallback()
 */
bool PWM_EnableIT(PWM_Channel_t ch, uint32_t freq_hz, uint8_t duty_pct);

/**
 * @brief  获取指定通道所属定时器的 HAL 句柄指针
 * @param  ch PWM 通道标识
 * @return 定时器句柄指针, 无效通道返回 NULL
 * @note   供 IRQ Handler 中调用 HAL_TIM_IRQHandler() 使用
 */
TIM_HandleTypeDef* PWM_GetTimHandle(PWM_Channel_t ch);

/**
 * @brief  根据中断上下文识别触发的 PWM 通道
 * @param  htim HAL 定时器句柄 (由 HAL 回调提供)
 * @return 对应的 PWM_Channel_t, 未找到返回 PWM_CH_COUNT
 * @note   在 HAL_TIM_PWM_PulseFinishedCallback() 中调用以确定哪个通道触发
 */
PWM_Channel_t PWM_IdentifyIRQ(TIM_HandleTypeDef *htim);

/**
 * @brief  查询指定通道是否处于 PWM 输出模式
 * @param  ch PWM 通道标识
 * @return true — PWM 已启用, false — GPIO 模式 (或无效通道)
 */
bool PWM_IsEnabled(PWM_Channel_t ch);

/**
 * @brief  获取指定通道所在定时器组的当前频率
 * @param  ch PWM 通道标识
 * @return 当前频率 (Hz), 未初始化或无效通道返回 0
 */
uint32_t PWM_GetFreq(PWM_Channel_t ch);

/**
 * @brief  获取指定通道的当前占空比
 * @param  ch PWM 通道标识
 * @return 当前占空比 (0~100), 未启用或无效通道返回 0
 */
uint8_t PWM_GetDuty(PWM_Channel_t ch);

#endif /* __PWM_MOTOR_H__ */
