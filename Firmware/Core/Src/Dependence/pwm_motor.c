/**
 * @file    pwm_motor.c
 * @brief   硬件 PWM 电机控制模块实现
 * @details 使用 TIM1/3/4/8 四个定时器为 4 路 Y 输出口提供硬件 PWM 功能。
 *          每个通道可在 GPIO 普通输出与 PWM 输出之间动态切换。
 *
 *          【定时器时钟】
 *          - TIM1/TIM8 (APB2): 168 MHz, Prescaler=167 → 1 MHz 计数器时钟
 *          - TIM3/TIM4 (APB1):  84 MHz, Prescaler=83  → 1 MHz 计数器时钟
 *          - ARR = 1_000_000 / freq_hz - 1
 *            100 Hz -> ARR=9999, 100000 Hz -> ARR=9
 */

#include "pwm_motor.h"
#include "stm32f4xx_hal.h"

/* ==========================================================================
 * 内部类型定义
 * ========================================================================== */

/** @brief 定时器组标识 (同组通道共享频率) */
typedef enum
{
    TIM_GRP_TIM1 = 0,  /**< TIM1 — APB2 (168 MHz), 仅 Y3 */
    TIM_GRP_TIM3,       /**< TIM3 — APB1 (84 MHz),  Y16, Y17 */
    TIM_GRP_TIM4,       /**< TIM4 — APB1 (84 MHz),  Y4, Y5, Y15 */
    TIM_GRP_TIM8,       /**< TIM8 — APB2 (168 MHz), Y1, Y2 */
    TIM_GRP_COUNT
} PWM_TimerGroup_t;

/** @brief 通道硬件描述符 (只读, const 存 Flash) */
typedef struct
{
    GPIO_TypeDef     *gpio_port;    /**< GPIO 端口 (GPIOA/B/C) */
    uint16_t          gpio_pin;     /**< GPIO 引脚掩码 (GPIO_PIN_x) */
    uint8_t           gpio_af;      /**< GPIO 复用功能编号 (AF1/AF2/AF3) */
    PWM_TimerGroup_t  timer_group;  /**< 所属定时器组 */
    uint32_t          tim_channel;  /**< 定时器通道 (TIM_CHANNEL_x) */
} ChannelDesc_t;

/** @brief 定时器组运行时状态 */
typedef struct
{
    TIM_HandleTypeDef  htim;            /**< HAL 定时器句柄 */
    TIM_TypeDef       *instance;        /**< 定时器外设基地址 (TIM1/3/4/8) */
    uint32_t           prescaler;       /**< 预分频值 (使计数器时钟 = 1 MHz) */
    uint32_t           current_freq_hz; /**< 当前配置的频率 (Hz), 0 = 未配置 */
} TimerGroupState_t;

/** @brief 通道运行时状态 */
typedef struct
{
    bool    enabled;    /**< PWM 是否已启用 */
    uint8_t duty_pct;   /**< 当前占空比 (0~100) */
} ChannelState_t;

/* ==========================================================================
 * 计数器时钟 — 所有定时器统一为 1 MHz
 * ========================================================================== */
#define PWM_COUNTER_CLK_HZ  1000000UL

/* ==========================================================================
 * 静态数据 — 通道描述符表 (只读)
 * ========================================================================== */

static const ChannelDesc_t ch_desc[PWM_CH_COUNT] = {
    /*              GPIO端口  引脚          AF编号          定时器组        定时器通道        */
    [PWM_CH_Y1]  = { GPIOC, GPIO_PIN_6, GPIO_AF3_TIM8,  TIM_GRP_TIM8, TIM_CHANNEL_1 },
    [PWM_CH_Y3]  = { GPIOA, GPIO_PIN_8, GPIO_AF1_TIM1,  TIM_GRP_TIM1, TIM_CHANNEL_1 },
    [PWM_CH_Y4]  = { GPIOB, GPIO_PIN_7, GPIO_AF2_TIM4,  TIM_GRP_TIM4, TIM_CHANNEL_2 },
    [PWM_CH_Y16] = { GPIOB, GPIO_PIN_5, GPIO_AF2_TIM3,  TIM_GRP_TIM3, TIM_CHANNEL_2 },
};

/* ==========================================================================
 * 静态数据 — 定时器组运行时状态
 * ========================================================================== */

static TimerGroupState_t tim_grp[TIM_GRP_COUNT] = {
    [TIM_GRP_TIM1] = { .htim = {0}, .instance = TIM1, .prescaler = 167, .current_freq_hz = 0 },
    [TIM_GRP_TIM3] = { .htim = {0}, .instance = TIM3, .prescaler =  83, .current_freq_hz = 0 },
    [TIM_GRP_TIM4] = { .htim = {0}, .instance = TIM4, .prescaler =  83, .current_freq_hz = 0 },
    [TIM_GRP_TIM8] = { .htim = {0}, .instance = TIM8, .prescaler = 167, .current_freq_hz = 0 },
};

/* ==========================================================================
 * 静态数据 — 通道运行时状态
 * ========================================================================== */

static ChannelState_t ch_state[PWM_CH_COUNT] = {0};

/* ==========================================================================
 * 内部辅助函数
 * ========================================================================== */

/**
 * @brief  将频率转换为 ARR 值 (自动重载寄存器)
 * @param  freq_hz 频率 (Hz), 已钳位
 * @return ARR = counter_clk / freq - 1
 */
static uint32_t freq_to_arr(uint32_t freq_hz)
{
    if (freq_hz < PWM_FREQ_MIN) freq_hz = PWM_FREQ_MIN;
    if (freq_hz > PWM_FREQ_MAX) freq_hz = PWM_FREQ_MAX;
    return (PWM_COUNTER_CLK_HZ / freq_hz) - 1;
}

/**
 * @brief  将占空比转换为 CCR 值 (比较寄存器)
 * @param  arr      当前 ARR 值
 * @param  duty_pct 占空比 (0~100)
 * @return CCR = (ARR + 1) × duty / 100
 */
static uint32_t duty_to_ccr(uint32_t arr, uint8_t duty_pct)
{
    if (duty_pct > 100) duty_pct = 100;
    return (uint32_t)((uint64_t)(arr + 1) * duty_pct / 100);
}

/**
 * @brief  初始化一个定时器的 PWM 时基
 * @param  grp 定时器组状态指针
 */
static void init_timer_base(TimerGroupState_t *grp)
{
    TIM_HandleTypeDef *htim = &grp->htim;

    htim->Instance               = grp->instance;
    htim->Init.Prescaler         = grp->prescaler;
    htim->Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim->Init.Period            = freq_to_arr(PWM_FREQ_MIN);  /* 默认 ARR, 启用通道时覆盖 */
    htim->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    htim->Init.RepetitionCounter = 0;

    HAL_TIM_PWM_Init(htim);
}

/**
 * @brief  将 GPIO 引脚切换为 AF 推挽模式 (用于 PWM 输出)
 * @param  desc 通道硬件描述符
 */
static void gpio_switch_to_af(const ChannelDesc_t *desc)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = desc->gpio_pin;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = desc->gpio_af;
    HAL_GPIO_Init(desc->gpio_port, &gpio);
}

/**
 * @brief  将 GPIO 引脚切换回推挽输出模式 (恢复为普通数字输出)
 * @param  desc 通道硬件描述符
 * @note   恢复后默认输出高电平 (与 IODEF.c IO_Init 中的初始化一致)
 */
static void gpio_switch_to_output(const ChannelDesc_t *desc)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = desc->gpio_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(desc->gpio_port, &gpio);

    /* 恢复为默认高电平 (安全状态, 与 IODEF.c 初始化一致) */
    HAL_GPIO_WritePin(desc->gpio_port, desc->gpio_pin, GPIO_PIN_SET);
}

/**
 * @brief  更新定时器组内所有已启用通道的 CCR (频率变更后保持各通道占空比)
 * @param  grp_idx 定时器组索引
 * @param  new_arr 新的 ARR 值
 */
static void recalc_group_ccrs(PWM_TimerGroup_t grp_idx, uint32_t new_arr)
{
    TIM_HandleTypeDef *htim = &tim_grp[grp_idx].htim;

    for (uint8_t i = 0; i < PWM_CH_COUNT; i++)
    {
        if (ch_desc[i].timer_group == grp_idx && ch_state[i].enabled)
        {
            uint32_t ccr = duty_to_ccr(new_arr, ch_state[i].duty_pct);
            __HAL_TIM_SET_COMPARE(htim, ch_desc[i].tim_channel, ccr);
        }
    }
}

static bool group_frequency_available(PWM_Channel_t ch, uint32_t freq_hz)
{
    PWM_TimerGroup_t group = ch_desc[ch].timer_group;

    for (uint8_t i = 0; i < PWM_CH_COUNT; i++)
    {
        if (i != (uint8_t)ch && ch_desc[i].timer_group == group &&
            ch_state[i].enabled && tim_grp[group].current_freq_hz != freq_hz)
        {
            return false;
        }
    }
    return true;
}

/* ==========================================================================
 * 公共 API 实现
 * ========================================================================== */

void PWM_Motor_Init(void)
{
    /* ---- 使能所有用到的定时器时钟 ---- */
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    /* ---- 初始化 4 个定时器的 PWM 时基 ---- */
    for (uint8_t i = 0; i < TIM_GRP_COUNT; i++)
    {
        init_timer_base(&tim_grp[i]);
    }

    /* ---- 使能定时器 CC 中断 (供 PWM_EnableIT 脉冲计数使用) ---- */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);

    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);

    /* 所有通道保持 GPIO 输出模式 (由 IO_Init 已配置), 不启动任何 PWM */
}

bool PWM_Enable(PWM_Channel_t ch, uint32_t freq_hz, uint8_t duty_pct)
{
    if (ch >= PWM_CH_COUNT) return false;

    const ChannelDesc_t *desc = &ch_desc[ch];
    TimerGroupState_t   *grp  = &tim_grp[desc->timer_group];

    /* 频率/占空比钳位 */
    if (freq_hz < PWM_FREQ_MIN) freq_hz = PWM_FREQ_MIN;
    if (freq_hz > PWM_FREQ_MAX) freq_hz = PWM_FREQ_MAX;
    if (duty_pct > 100) duty_pct = 100;
    if (!group_frequency_available(ch, freq_hz)) return false;

    /* 更新定时器组频率 (ARR) */
    uint32_t arr = freq_to_arr(freq_hz);
    if (grp->current_freq_hz != freq_hz)
    {
        __HAL_TIM_SET_AUTORELOAD(&grp->htim, arr);
        grp->current_freq_hz = freq_hz;
        /* 同组其他已启用通道重新计算 CCR 以保持占空比 */
        recalc_group_ccrs(desc->timer_group, arr);
    }

    /* 配置本通道的 OC (输出比较) 参数 */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = duty_to_ccr(arr, duty_pct);
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&grp->htim, &oc, desc->tim_channel) != HAL_OK)
        return false;

    /* 切换 GPIO 引脚到 AF 模式 */
    gpio_switch_to_af(desc);

    /* Publish state before start so an immediate interrupt can stop the channel. */
    ch_state[ch].enabled  = true;
    ch_state[ch].duty_pct = duty_pct;

    /* 启动 PWM 输出 (对 TIM1/TIM8 高级定时器自动使能 MOE) */
    if (HAL_TIM_PWM_Start(&grp->htim, desc->tim_channel) != HAL_OK)
    {
        ch_state[ch].enabled  = false;
        ch_state[ch].duty_pct = 0;
        gpio_switch_to_output(desc);
        return false;
    }

    return true;
}

void PWM_Disable(PWM_Channel_t ch)
{
    if (ch >= PWM_CH_COUNT) return;
    if (!ch_state[ch].enabled) return;

    const ChannelDesc_t *desc = &ch_desc[ch];
    TimerGroupState_t   *grp  = &tim_grp[desc->timer_group];

    /* 停止 PWM 输出 (同时禁用 CC 中断, 兼容 _IT 和非 _IT 启动模式) */
    HAL_TIM_PWM_Stop_IT(&grp->htim, desc->tim_channel);

    /* 切换 GPIO 引脚回推挽输出模式 (默认高电平) */
    gpio_switch_to_output(desc);

    /* 清除通道状态 */
    ch_state[ch].enabled  = false;
    ch_state[ch].duty_pct = 0;
}

bool PWM_SetFreq(PWM_Channel_t ch, uint32_t freq_hz)
{
    if (ch >= PWM_CH_COUNT) return false;
    if (!ch_state[ch].enabled) return false;

    if (freq_hz < PWM_FREQ_MIN) freq_hz = PWM_FREQ_MIN;
    if (freq_hz > PWM_FREQ_MAX) freq_hz = PWM_FREQ_MAX;

    const ChannelDesc_t *desc = &ch_desc[ch];
    TimerGroupState_t   *grp  = &tim_grp[desc->timer_group];

    if (grp->current_freq_hz == freq_hz) return true;  /* 频率未变，无需操作 */
    if (!group_frequency_available(ch, freq_hz)) return false;

    uint32_t arr = freq_to_arr(freq_hz);
    __HAL_TIM_SET_AUTORELOAD(&grp->htim, arr);
    grp->current_freq_hz = freq_hz;

    /* 同组所有已启用通道重新计算 CCR，保持各自占空比不变 */
    recalc_group_ccrs(desc->timer_group, arr);
    return true;
}

void PWM_SetDuty(PWM_Channel_t ch, uint8_t duty_pct)
{
    if (ch >= PWM_CH_COUNT) return;
    if (!ch_state[ch].enabled) return;
    if (duty_pct > 100) duty_pct = 100;

    const ChannelDesc_t *desc = &ch_desc[ch];
    TimerGroupState_t   *grp  = &tim_grp[desc->timer_group];

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&grp->htim);
    uint32_t ccr = duty_to_ccr(arr, duty_pct);
    __HAL_TIM_SET_COMPARE(&grp->htim, desc->tim_channel, ccr);

    ch_state[ch].duty_pct = duty_pct;
}

bool PWM_IsEnabled(PWM_Channel_t ch)
{
    if (ch >= PWM_CH_COUNT) return false;
    return ch_state[ch].enabled;
}

uint32_t PWM_GetFreq(PWM_Channel_t ch)
{
    if (ch >= PWM_CH_COUNT) return 0;
    return tim_grp[ch_desc[ch].timer_group].current_freq_hz;
}

uint8_t PWM_GetDuty(PWM_Channel_t ch)
{
    if (ch >= PWM_CH_COUNT) return 0;
    return ch_state[ch].duty_pct;
}

bool PWM_EnableIT(PWM_Channel_t ch, uint32_t freq_hz, uint8_t duty_pct)
{
    if (ch >= PWM_CH_COUNT) return false;

    const ChannelDesc_t *desc = &ch_desc[ch];
    TimerGroupState_t   *grp  = &tim_grp[desc->timer_group];

    if (freq_hz < PWM_FREQ_MIN) freq_hz = PWM_FREQ_MIN;
    if (freq_hz > PWM_FREQ_MAX) freq_hz = PWM_FREQ_MAX;
    if (duty_pct > 100) duty_pct = 100;
    if (!group_frequency_available(ch, freq_hz)) return false;

    uint32_t arr = freq_to_arr(freq_hz);
    if (grp->current_freq_hz != freq_hz)
    {
        __HAL_TIM_SET_AUTORELOAD(&grp->htim, arr);
        grp->current_freq_hz = freq_hz;
        recalc_group_ccrs(desc->timer_group, arr);
    }

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = duty_to_ccr(arr, duty_pct);
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&grp->htim, &oc, desc->tim_channel) != HAL_OK)
        return false;

    gpio_switch_to_af(desc);

    /* Publish state before enabling CC interrupts; a short move may finish immediately. */
    ch_state[ch].enabled  = true;
    ch_state[ch].duty_pct = duty_pct;

    /* 启动 PWM 输出 + 使能 CC 中断 (每个 PWM 周期触发一次) */
    if (HAL_TIM_PWM_Start_IT(&grp->htim, desc->tim_channel) != HAL_OK)
    {
        ch_state[ch].enabled  = false;
        ch_state[ch].duty_pct = 0;
        gpio_switch_to_output(desc);
        return false;
    }

    return true;
}

TIM_HandleTypeDef* PWM_GetTimHandle(PWM_Channel_t ch)
{
    if (ch >= PWM_CH_COUNT) return NULL;
    return &tim_grp[ch_desc[ch].timer_group].htim;
}

PWM_Channel_t PWM_IdentifyIRQ(TIM_HandleTypeDef *htim)
{
    /* 将 HAL 的 active channel 标志转换为 TIM_CHANNEL_x 值 */
    uint32_t tim_ch;
    switch (htim->Channel)
    {
        case HAL_TIM_ACTIVE_CHANNEL_1: tim_ch = TIM_CHANNEL_1; break;
        case HAL_TIM_ACTIVE_CHANNEL_2: tim_ch = TIM_CHANNEL_2; break;
        case HAL_TIM_ACTIVE_CHANNEL_3: tim_ch = TIM_CHANNEL_3; break;
        case HAL_TIM_ACTIVE_CHANNEL_4: tim_ch = TIM_CHANNEL_4; break;
        default: return PWM_CH_COUNT;
    }

    for (uint8_t i = 0; i < PWM_CH_COUNT; i++)
    {
        if (ch_desc[i].gpio_port != NULL &&
            tim_grp[ch_desc[i].timer_group].instance == htim->Instance &&
            ch_desc[i].tim_channel == tim_ch)
        {
            return (PWM_Channel_t)i;
        }
    }
    return PWM_CH_COUNT;
}
