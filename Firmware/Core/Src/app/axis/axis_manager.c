/**
 * @file    axis_manager.c
 * @brief   F407 2416 控制板八轴固定引脚模板实现。
 * @details 前四个模板使用独立硬件 PWM 通道，后四个模板使用 TIM5 调度的普通
 *          Y 输出软件脉冲。当前 AXIS_APP_AUTO_REGISTER=1，启动时注册全部八轴；
 *          可通过该宏改变注册策略。
 */
#include "app/axis/axis_manager.h"
#include "IODEF.h"
#include "nvram.h"
#include <string.h>

typedef struct
{
    Axis_Config_t axis;       /**< 传给运动控制核心的轴配置。 */
    uint16_t pulse_y;         /**< 脉冲输出对应的物理 Y 编号。 */
    uint16_t dir_y;           /**< 方向输出对应的物理 Y 编号。 */
    uint16_t sensor_latch;    /**< 预留的高速锁存输入编号。 */
    bool home_dir_positive;   /**< true=正向搜索原点，false=负向搜索。 */
} AxisTemplate_t;

#define AXIS_NO_PWM   PWM_CH_COUNT
#define AXIS_NO_SOFT  SOFT_PWM_CH_COUNT

/* 软限位参数 (掉电保持): 默认全部关闭。数组长度固定为 8, 与协议寄存器一一对应。
 * 注: NVRAM_PERSISTENT 宏只能接单值初始化; 数组需先声明再注册描述符。 */
static uint8_t soft_limit_enable[AXIS_APP_CAPACITY] = { 0U };
static int32_t soft_limit_min[AXIS_APP_CAPACITY]  = { 0 };
static int32_t soft_limit_max[AXIS_APP_CAPACITY]  = { 0 };
NVRAM_PERSISTENT_REGISTER(soft_limit_enable);
NVRAM_PERSISTENT_REGISTER(soft_limit_min);
NVRAM_PERSISTENT_REGISTER(soft_limit_max);

static const AxisTemplate_t axis_templates[AXIS_APP_CAPACITY] = {
    /* A-D 轴：硬件 PWM，适合较高脉冲频率和更严格的抖动要求。 */
    {
        .axis = { AXIS_PULSE_MODE_PWM, PWM_CH_Y1, AXIS_NO_SOFT, Y0,
                  NULL, NULL, NULL, false },
        .pulse_y = 1U, .dir_y = 0U, .sensor_latch = AXIS_PIN_NONE
    },
    {
        .axis = { AXIS_PULSE_MODE_PWM, PWM_CH_Y3, AXIS_NO_SOFT, Y2,
                  NULL, NULL, NULL, false },
        .pulse_y = 3U, .dir_y = 2U, .sensor_latch = AXIS_PIN_NONE
    },
    {
        .axis = { AXIS_PULSE_MODE_PWM, PWM_CH_Y4, AXIS_NO_SOFT, Y5,
                  NULL, NULL, NULL, false },
        .pulse_y = 4U, .dir_y = 5U, .sensor_latch = AXIS_PIN_NONE
    },
    {
        .axis = { AXIS_PULSE_MODE_PWM, PWM_CH_Y16, AXIS_NO_SOFT, Y17,
                  NULL, NULL, NULL, false },
        .pulse_y = 14U, .dir_y = 15U, .sensor_latch = AXIS_PIN_NONE
    },
    {
        .axis = { AXIS_PULSE_MODE_SOFT_PWM, AXIS_NO_PWM, SOFT_PWM_CH_Y6, Y7,
                  NULL, NULL, NULL, false },
        .pulse_y = 6U, .dir_y = 7U, .sensor_latch = AXIS_PIN_NONE
    },
    /* E-H 轴：TIM5 软件脉冲，用普通 Y 输出扩展轴容量。 */
    {
        .axis = { AXIS_PULSE_MODE_SOFT_PWM, AXIS_NO_PWM, SOFT_PWM_CH_Y10, Y11,
                  NULL, NULL, NULL, false },
        .pulse_y = 8U, .dir_y = 9U, .sensor_latch = AXIS_PIN_NONE
    },
    {
        .axis = { AXIS_PULSE_MODE_SOFT_PWM, AXIS_NO_PWM, SOFT_PWM_CH_Y12, Y13,
                  NULL, NULL, NULL, false },
        .pulse_y = 10U, .dir_y = 11U, .sensor_latch = AXIS_PIN_NONE
    },
    {
        .axis = { AXIS_PULSE_MODE_SOFT_PWM, AXIS_NO_PWM, SOFT_PWM_CH_Y14, Y15,
                  NULL, NULL, NULL, false },
        .pulse_y = 12U, .dir_y = 13U, .sensor_latch = AXIS_PIN_NONE
    }
};

uint8_t Axis_ID[AXIS_APP_CAPACITY];
uint8_t Axis_A = 0xFFU;
uint8_t Axis_B = 0xFFU;
uint8_t Axis_C = 0xFFU;
uint8_t Axis_D = 0xFFU;

static uint16_t input_to_index(const Input_t *input)
{
    static Input_t *const inputs[] = {
        &In_X0, &In_X1, &In_X2, &In_X3, &In_X4, &In_X5, &In_X6, &In_X7,
        &In_X10, &In_X11, &In_X12, &In_X13, &In_X14, &In_X15, &In_X16, &In_X17,
        &In_X20, &In_X21, &In_X22, &In_X23, &In_X24, &In_X25, &In_X26, &In_X27
    };

    if (input == NULL) return AXIS_PIN_NONE;
    for (uint16_t i = 0; i < (uint16_t)(sizeof(inputs) / sizeof(inputs[0])); i++) {
        if (input == inputs[i]) return i;
    }
    return AXIS_PIN_NONE;
}

void AxisApp_Init(void)
{
    memset(Axis_ID, 0xFF, sizeof(Axis_ID));

#if AXIS_APP_AUTO_REGISTER
    for (uint8_t i = 0; i < AXIS_APP_CAPACITY; i++) {
        Axis_ID[i] = Axis_Create(&axis_templates[i].axis);
    }
#endif

    Axis_A = Axis_ID[0];
    Axis_B = Axis_ID[1];
    Axis_C = Axis_ID[2];
    Axis_D = Axis_ID[3];
}

uint8_t AxisApp_GetRegisteredCount(void)
{
    return Axis_GetCount();
}

uint8_t AxisApp_GetAxisId(uint8_t idx)
{
    return (idx < AXIS_APP_CAPACITY) ? Axis_ID[idx] : 0xFFU;
}

bool AxisApp_IsRegistered(uint8_t idx)
{
    return idx < AXIS_APP_CAPACITY && Axis_ID[idx] < Axis_GetCount();
}

bool AxisApp_GetPinConfig(uint8_t idx, Axis_PinConfig_t *out)
{
    if (idx >= AXIS_APP_CAPACITY || out == NULL) return false;

    const AxisTemplate_t *tpl = &axis_templates[idx];
    const Axis_Config_t *cfg = &tpl->axis;
    out->pulse_mode = (uint16_t)cfg->pulse_mode;
    out->pwm_ch = (cfg->pulse_mode == AXIS_PULSE_MODE_PWM)
                ? (uint16_t)cfg->pwm_ch : AXIS_PIN_NONE;
    out->soft_ch = (cfg->pulse_mode == AXIS_PULSE_MODE_SOFT_PWM)
                 ? (uint16_t)cfg->soft_ch : AXIS_PIN_NONE;
    out->pulse_y = tpl->pulse_y;
    out->dir_y = tpl->dir_y;
    out->sensor_home = input_to_index(cfg->sensor_home);
    out->sensor_limit_pos = input_to_index(cfg->sensor_limit_pos);
    out->sensor_limit_neg = input_to_index(cfg->sensor_limit_neg);
    out->sensor_latch = tpl->sensor_latch;
    out->flags = (cfg->dir_invert ? 0x0001U : 0U) |
                 (tpl->home_dir_positive ? 0x0004U : 0U) |
                 (cfg->sensor_home != NULL ? 0x0008U : 0U) |
                 (cfg->sensor_limit_pos != NULL ? 0x0010U : 0U) |
                 (cfg->sensor_limit_neg != NULL ? 0x0020U : 0U);
    return true;
}

bool AxisApp_GetHomeDirPositive(uint8_t idx, bool *out)
{
    if (idx >= AXIS_APP_CAPACITY || out == NULL) return false;
    *out = axis_templates[idx].home_dir_positive;
    return true;
}

bool AxisApp_IsOutputReserved(uint16_t y_index)
{
    for (uint8_t i = 0; i < AXIS_APP_CAPACITY; i++) {
        if (!AxisApp_IsRegistered(i)) continue;
        if (axis_templates[i].pulse_y == y_index ||
            axis_templates[i].dir_y == y_index) {
            return true;
        }
    }
    return false;
}

void AxisApp_ApplySoftLimits(void)
{
    for (uint8_t i = 0; i < AXIS_APP_CAPACITY; i++)
    {
        if (!AxisApp_IsRegistered(i)) continue;

        bool enable = soft_limit_enable[i] != 0U;
        int32_t min_pos = soft_limit_min[i];
        int32_t max_pos = soft_limit_max[i];

        /* 存储数据损坏时按关闭处理, 避免上电后立刻锁死运动。 */
        if (enable && min_pos >= max_pos)
        {
            enable = false;
            soft_limit_enable[i] = 0U;
        }
        (void)Axis_SetSoftLimit(AxisApp_GetAxisId(i), enable, min_pos, max_pos);
    }
}

bool AxisApp_GetSoftLimit(uint8_t idx, bool *enable, int32_t *min_pos, int32_t *max_pos)
{
    if (idx >= AXIS_APP_CAPACITY) return false;
    if (enable)  *enable  = soft_limit_enable[idx] != 0U;
    if (min_pos) *min_pos = soft_limit_min[idx];
    if (max_pos) *max_pos = soft_limit_max[idx];
    return true;
}

bool AxisApp_SetSoftLimit(uint8_t idx, bool enable, int32_t min_pos, int32_t max_pos)
{
    if (idx >= AXIS_APP_CAPACITY) return false;
    if (enable && min_pos >= max_pos) return false;

    uint8_t id = AxisApp_GetAxisId(idx);
    if (!AxisApp_IsRegistered(idx)) return false;
    if (Axis_IsBusy(id)) return false;
    if (!Axis_SetSoftLimit(id, enable, min_pos, max_pos)) return false;

    soft_limit_enable[idx] = enable ? 1U : 0U;
    soft_limit_min[idx] = min_pos;
    soft_limit_max[idx] = max_pos;
    NVRAM_Auto_MarkDirty();
    return true;
}

bool AxisApp_IsOutputBusy(uint16_t y_index)
{
    for (uint8_t i = 0; i < AXIS_APP_CAPACITY; i++) {
        if (!AxisApp_IsRegistered(i)) continue;
        if (axis_templates[i].pulse_y != y_index &&
            axis_templates[i].dir_y != y_index) {
            continue;
        }
        if (Axis_IsBusy(AxisApp_GetAxisId(i))) return true;
    }
    return false;
}
