/**
 * @file    axis_manager.h
 * @brief   F407 2416 控制板的轴容量、固定引脚模板和查询接口。
 * @details 统一维护最多 8 个逻辑轴的脉冲输出、方向输出和传感器映射。
 *          上位机通过本模块读取模板信息，底层运动控制通过 Axis_Create 注册实例。
 */
#ifndef __AXIS_MANAGER_H__
#define __AXIS_MANAGER_H__

#include <stdbool.h>
#include <stdint.h>
#include "motor_axis.h"

#define AXIS_APP_CAPACITY       8U  /**< 应用层可配置的最大轴数。 */
#define AXIS_APP_COUNT          AXIS_APP_CAPACITY /**< 兼容旧代码的轴数量别名。 */
#define AXIS_APP_AUTO_REGISTER  1U  /**< 1=启动时注册全部模板，0=等待上位机配置。 */
#define AXIS_PIN_NONE           0xFFFFU /**< 未绑定物理输入或输出。 */

typedef struct
{
    uint16_t pulse_mode;       /**< 脉冲模式：0=硬件 PWM，1=软件脉冲。 */
    uint16_t pwm_ch;           /**< 硬件 PWM 通道，无绑定时为 AXIS_PIN_NONE。 */
    uint16_t soft_ch;          /**< 软件脉冲通道，无绑定时为 AXIS_PIN_NONE。 */
    uint16_t pulse_y;          /**< 脉冲输出占用的物理 Y 编号。 */
    uint16_t dir_y;            /**< 方向输出占用的物理 Y 编号。 */
    uint16_t sensor_home;      /**< 原点传感器 X 编号。 */
    uint16_t sensor_limit_pos; /**< 正限位传感器 X 编号。 */
    uint16_t sensor_limit_neg; /**< 负限位传感器 X 编号（仅回原使用）。 */
    uint16_t sensor_latch;     /**< 高速锁存输入 X 编号，未配置时为 AXIS_PIN_NONE。 */
    uint16_t flags;            /**< 方向、回原方向和传感器有效位组成的位图。 */
} Axis_PinConfig_t;

extern uint8_t Axis_ID[AXIS_APP_CAPACITY];
extern uint8_t Axis_A;
extern uint8_t Axis_B;
extern uint8_t Axis_C;
extern uint8_t Axis_D;

/** @brief 初始化轴 ID 表，并按编译配置决定是否自动注册模板。 */
void AxisApp_Init(void);
/** @brief 返回底层运动控制器中已经注册的轴数量。 */
uint8_t AxisApp_GetRegisteredCount(void);
/** @brief 将应用轴序号转换为底层轴 ID；无效序号返回 0xFF。 */
uint8_t AxisApp_GetAxisId(uint8_t idx);
/** @brief 判断指定应用轴是否已经成功注册。 */
bool AxisApp_IsRegistered(uint8_t idx);
/** @brief 获取指定应用轴的完整物理引脚映射。 */
bool AxisApp_GetPinConfig(uint8_t idx, Axis_PinConfig_t *out);
/** @brief 获取指定轴的回原搜索方向。 */
bool AxisApp_GetHomeDirPositive(uint8_t idx, bool *out);
/** @brief 判断物理 Y 输出是否被已注册轴占用为脉冲或方向信号。 */
bool AxisApp_IsOutputReserved(uint16_t y_index);
/** @brief 判断占用指定物理 Y 输出的轴当前是否正在运动。 */
bool AxisApp_IsOutputBusy(uint16_t y_index);

/* ---- 软限位 (掉电保持, 存放于通用 NVRAM 区) ---- */

/**
 * @brief  把 NVRAM 中的软限位参数下发到各轴运行时配置。
 * @note   必须在 NVRAM_Auto_Init() 之后调用一次。
 */
void AxisApp_ApplySoftLimits(void);

/** @brief 读取指定应用轴的软限位设置 (RAM 中的持久化副本)。 */
bool AxisApp_GetSoftLimit(uint8_t idx, bool *enable, int32_t *min_pos, int32_t *max_pos);

/**
 * @brief  设置指定应用轴的软限位并下发到运行时。
 * @return true=成功; false=轴索引无效、轴忙或使能时 min_pos >= max_pos
 * @note   仅修改 RAM, 由 NVRAM 自动保存机制在空闲时写入 Flash。
 */
bool AxisApp_SetSoftLimit(uint8_t idx, bool enable, int32_t min_pos, int32_t max_pos);

#endif /* __AXIS_MANAGER_H__ */
