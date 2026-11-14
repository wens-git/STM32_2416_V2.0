/**
 * @file    soft_pwm.c
 * @brief   基于 TIM5 比较中断的软件脉冲发生器实现。
 * @details TIM5 以 1 MHz 连续计数。每个通道保存下一次翻转时刻，比较寄存器始终
 *          指向所有活动通道中最近的事件。计数和时刻比较均采用无符号回绕算法，
 *          因此 TIM5 溢出时无需重置通道状态。
 */
#include "soft_pwm.h"
#include "IODEF.h"
#include "app_config.h"

#define SOFT_PWM_TIMER_CLK_HZ  1000000UL  /**< TIM5 计数频率：每计数一次为 1 us。 */
#define SOFT_PWM_TIMER_PERIOD  0xFFFFFFFFUL /**< 32 位自由运行计数器最大周期。 */

typedef struct
{
    void (*write)(uint8_t state); /**< 对应 Y 输出的电平写入函数。 */
} SoftPWM_ChannelDesc_t;

typedef struct
{
    bool enabled;                         /**< 通道是否参与事件调度。 */
    bool level_high;                      /**< 当前输出电平是否为高。 */
    uint32_t freq_hz;                     /**< 经限幅后的实际设定频率。 */
    uint8_t duty_pct;                     /**< 经限幅后的实际占空比。 */
    uint32_t high_ticks;                  /**< 一个周期内的高电平计数。 */
    uint32_t low_ticks;                   /**< 一个周期内的低电平计数。 */
    uint32_t next_tick;                   /**< 下一次电平翻转的绝对计数值。 */
    SoftPWM_PulseCallback_t callback;     /**< 周期完成回调。 */
} SoftPWM_ChannelState_t;

static TIM_HandleTypeDef soft_pwm_htim;
static SoftPWM_ChannelState_t soft_pwm_state[SOFT_PWM_CH_COUNT];
static bool soft_pwm_initialized;

static const SoftPWM_ChannelDesc_t soft_pwm_desc[SOFT_PWM_CH_COUNT] = {
    [SOFT_PWM_CH_Y6]  = { Y6  },
    [SOFT_PWM_CH_Y10] = { Y10 },
    [SOFT_PWM_CH_Y12] = { Y12 },
    [SOFT_PWM_CH_Y14] = { Y14 },
};

static uint32_t timer_now(void)
{
    return __HAL_TIM_GET_COUNTER(&soft_pwm_htim);
}

static uint32_t ticks_until(uint32_t now, uint32_t target)
{
    return target - now;
}

static bool time_reached(uint32_t now, uint32_t target)
{
    /* 有符号差值可在时间间隔小于 2^31 个计数时正确处理 32 位回绕。 */
    return (int32_t)(now - target) >= 0;
}

static void schedule_next_compare(void)
{
    /* 只安排最近事件，届时 ISR 会一次处理所有已经到期的通道。 */
    uint32_t now = timer_now();
    uint32_t best_delta = SOFT_PWM_TIMER_PERIOD;
    bool found = false;

    for (uint8_t i = 0; i < SOFT_PWM_CH_COUNT; i++)
    {
        if (!soft_pwm_state[i].enabled) continue;

        uint32_t delta = time_reached(now, soft_pwm_state[i].next_tick)
                       ? 2U
                       : ticks_until(now, soft_pwm_state[i].next_tick);
        if (!found || delta < best_delta)
        {
            best_delta = delta;
            found = true;
        }
    }

    if (!found)
    {
        __HAL_TIM_DISABLE_IT(&soft_pwm_htim, TIM_IT_CC1);
        return;
    }

    if (best_delta < 2U) best_delta = 2U;
    __HAL_TIM_SET_COMPARE(&soft_pwm_htim, TIM_CHANNEL_1,
                          now + best_delta);
    __HAL_TIM_CLEAR_FLAG(&soft_pwm_htim, TIM_FLAG_CC1);
    __HAL_TIM_ENABLE_IT(&soft_pwm_htim, TIM_IT_CC1);
}

static void disable_channel_no_schedule(SoftPWM_Channel_t ch)
{
    if (ch >= SOFT_PWM_CH_COUNT) return;

    soft_pwm_state[ch].enabled = false;
    soft_pwm_state[ch].level_high = false;
    soft_pwm_state[ch].callback = NULL;
    soft_pwm_desc[ch].write(0);
}

bool SoftPWM_Init(void)
{
    soft_pwm_initialized = false;
    __HAL_RCC_TIM5_CLK_ENABLE();

    soft_pwm_htim.Instance = TIM5;
    /* APB1 定时器时钟为 84 MHz，分频后得到 1 MHz 计数时基。 */
    soft_pwm_htim.Init.Prescaler =
        (uint32_t)((SYS_CLK_FREQ / 2U) / SOFT_PWM_TIMER_CLK_HZ) - 1U;
    soft_pwm_htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    soft_pwm_htim.Init.Period = SOFT_PWM_TIMER_PERIOD;
    soft_pwm_htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    soft_pwm_htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&soft_pwm_htim) != HAL_OK) return false;
    if (HAL_TIM_Base_Start(&soft_pwm_htim) != HAL_OK) return false;

    HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    soft_pwm_initialized = true;
    return true;
}

void SoftPWM_EnableIT(SoftPWM_Channel_t ch, uint32_t freq_hz, uint8_t duty_pct,
                      SoftPWM_PulseCallback_t callback)
{
    if (!soft_pwm_initialized || ch >= SOFT_PWM_CH_COUNT) return;

    if (freq_hz < SOFT_PWM_FREQ_MIN) freq_hz = SOFT_PWM_FREQ_MIN;
    if (freq_hz > SOFT_PWM_FREQ_MAX) freq_hz = SOFT_PWM_FREQ_MAX;
    if (duty_pct == 0U) duty_pct = 1U;
    if (duty_pct > 99U) duty_pct = 99U;

    uint32_t period_ticks = SOFT_PWM_TIMER_CLK_HZ / freq_hz;
    if (period_ticks < 4U) period_ticks = 4U;

    uint32_t high_ticks = (period_ticks * duty_pct) / 100U;
    if (high_ticks < 2U) high_ticks = 2U;
    if (high_ticks > (period_ticks - 2U)) high_ticks = period_ticks - 2U;

    ATOMIC_ENTER();
    soft_pwm_state[ch].freq_hz = freq_hz;
    soft_pwm_state[ch].duty_pct = duty_pct;
    soft_pwm_state[ch].high_ticks = high_ticks;
    soft_pwm_state[ch].low_ticks = period_ticks - high_ticks;
    soft_pwm_state[ch].next_tick = timer_now() + 2U;
    soft_pwm_state[ch].callback = callback;
    soft_pwm_state[ch].level_high = false;
    soft_pwm_state[ch].enabled = true;
    soft_pwm_desc[ch].write(0);
    schedule_next_compare();
    ATOMIC_EXIT();
}

void SoftPWM_Disable(SoftPWM_Channel_t ch)
{
    ATOMIC_ENTER();
    disable_channel_no_schedule(ch);
    schedule_next_compare();
    ATOMIC_EXIT();
}

bool SoftPWM_IsEnabled(SoftPWM_Channel_t ch)
{
    if (ch >= SOFT_PWM_CH_COUNT) return false;
    return soft_pwm_state[ch].enabled;
}

uint32_t SoftPWM_GetFreq(SoftPWM_Channel_t ch)
{
    if (ch >= SOFT_PWM_CH_COUNT) return 0;
    return soft_pwm_state[ch].enabled ? soft_pwm_state[ch].freq_hz : 0U;
}

uint8_t SoftPWM_GetDuty(SoftPWM_Channel_t ch)
{
    if (ch >= SOFT_PWM_CH_COUNT) return 0;
    return soft_pwm_state[ch].enabled ? soft_pwm_state[ch].duty_pct : 0U;
}

TIM_HandleTypeDef *SoftPWM_GetTimHandle(void)
{
    return &soft_pwm_htim;
}

void SoftPWM_TimerElapsedISR(TIM_HandleTypeDef *htim)
{
    if (htim == NULL || htim->Instance != TIM5) return;
    if ((__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) == RESET) ||
        (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) == RESET)) {
        return;
    }
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);

    uint32_t now = timer_now();

    /* 同一比较时刻可能有多个通道到期，必须在一次中断内全部处理。 */
    for (uint8_t i = 0; i < SOFT_PWM_CH_COUNT; i++)
    {
        SoftPWM_ChannelState_t *ch = &soft_pwm_state[i];
        if (!ch->enabled || !time_reached(now, ch->next_tick)) continue;

        if (ch->level_high)
        {
            soft_pwm_desc[i].write(0);
            ch->level_high = false;
            ch->next_tick += ch->low_ticks;
            if (ch->callback) ch->callback((SoftPWM_Channel_t)i);
        }
        else
        {
            soft_pwm_desc[i].write(1);
            ch->level_high = true;
            ch->next_tick += ch->high_ticks;
        }
    }

    schedule_next_compare();
}
