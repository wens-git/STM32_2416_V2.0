/**
 * @file    motor_axis.h
 * @brief   多轴步进/伺服电机运动控制模块 — PLC 风格运动控制器
 * @details 本模块在硬件 PWM 脉冲发生器 (pwm_motor) 之上，提供完整的工业级
 *          多轴运动控制功能，类似三菱/西门子 PLC 的定位模块:
 *
 *          【运动模式】
 *          - JOG (点动):     持续运行，用于手动操作/调试
 *          - MoveRel (相对): 从当前位置移动指定脉冲数
 *          - MoveAbs (绝对): 移动到指定绝对坐标位置
 *          - Homing  (回原):  多段式回原点 (快找→退出→慢找→置零)
 *
 *          【传感器】
 *          每轴可配置 3 个传感器 (均为可选):
 *          - 原点传感器 (Home):    回原点基准
 *          - 正向极限 (Limit+):   仅用于回原寻参考, 运行中不检测
 *          - 负向极限 (Limit-):   仅用于回原寻参考, 运行中不检测
 *
 *          【脉冲计数】
 *          使用定时器 CC 中断精确计数每一个 PWM 脉冲，确保步数准确。
 *          JOG 模式也持续追踪位置。
 *
 *          【保护策略】
 *          - 本模块不产生轴报警, 也不存在 AXIS_ALARM 状态。
 *          - 硬限位只服务回原: 回原朝原点方向碰限位时自动反向寻找原点;
 *            回原两端都找不到原点时停止回原并返回 IDLE (homed=false)。
 *          - 正常运行/JOG/定位碰到限位输入不会停止、不会锁轴、不会报警。
 *          - 行程保护改为可选的软限位 (Axis_SetSoftLimit):
 *            默认关闭; 开启后越界目标被拒绝, 运行中越界立即停止并回到 IDLE。
 *          - 软限位参数由上位机配置, 掉电保持。
 */

#ifndef __MOTOR_AXIS_H__
#define __MOTOR_AXIS_H__

#include "pwm_motor.h"
#include "soft_pwm.h"
#include "input.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * 常量
 * ========================================================================== */
#define MAX_AXES  8   /**< 最大同时管理的轴数 */

/* ==========================================================================
 * 类型定义
 * ========================================================================== */

/** @brief 轴运动状态 */
typedef enum
{
    AXIS_IDLE = 0,   /**< 空闲 — 已停止，可接受新指令 */
    AXIS_JOG,        /**< 点动 — 持续运行 (无目标，直到 Axis_Stop) */
    AXIS_MOVING,     /**< 定位 — 正在移动到目标 (相对/绝对) */
    AXIS_HOMING      /**< 回原 — 正在执行回原点序列 */
} Axis_State_t;

typedef enum
{
    AXIS_HOME_MODE_PRECISION = 0,
    AXIS_HOME_MODE_TRIGGER   = 1
} Axis_HomeMode_t;

typedef enum
{
    AXIS_PULSE_MODE_PWM = 0,
    AXIS_PULSE_MODE_SOFT_PWM = 1
} Axis_PulseMode_t;

/**
 * @brief 轴配置结构体 — 创建轴时传入
 * @note  传感器指针为 NULL 表示未配置该传感器。
 *        极限传感器仅用于回原参考, 不参与运行保护。
 *        未配置原点传感器时 Axis_Home() 将返回失败。
 */
typedef struct
{
    Axis_PulseMode_t pulse_mode;       /**< 脉冲输出模式: 硬件 PWM 或软件 PWM */
    PWM_Channel_t    pwm_ch;           /**< 硬件 PWM 脉冲输出通道 */
    SoftPWM_Channel_t soft_ch;         /**< 软件 PWM 脉冲输出通道 */
    void           (*set_dir)(uint8_t);/**< 方向引脚控制函数 (如 Y1, Y6 等) */
    Input_t         *sensor_home;      /**< 原点传感器 (消抖后) — 可为 NULL */
    Input_t         *sensor_limit_pos; /**< 正向极限传感器 — 仅供回原使用, 可为 NULL */
    Input_t         *sensor_limit_neg; /**< 负向极限传感器 — 仅供回原使用, 可为 NULL */
    bool             dir_invert;       /**< 方向反转: true 时正方向 set_dir(0) */
} Axis_Config_t;

/* ==========================================================================
 * 公共 API
 * ========================================================================== */

/**
 * @brief  创建并注册一个运动轴
 * @param  config 轴配置 (PWM 通道、方向函数、传感器绑定)
 * @return 轴 ID (0 ~ MAX_AXES-1), 失败返回 0xFF
 * @note   创建后轴处于 AXIS_IDLE 状态，位置 = 0，未回原。
 */
uint8_t Axis_Create(const Axis_Config_t *config);

/* ---- 运动指令 ---- */

/**
 * @brief  JOG 点动 — 持续运行直到 Axis_Stop()
 * @param  id      轴 ID
 * @param  freq_hz 脉冲频率 (Hz), 钳位到 [100, 100000]
 * @param  dir     运动方向: true = 正方向, false = 负方向
 * @return true=实际启动，false=轴状态、安全条件或脉冲资源不允许启动
 */
bool Axis_Jog(uint8_t id, uint32_t freq_hz, bool dir);

/**
 * @brief  相对定位 — 从当前位置移动指定脉冲数
 * @param  id      轴 ID
 * @param  freq_hz 脉冲频率 (Hz)
 * @param  steps   移动步数: >0 = 正方向, <0 = 负方向
 * @return true=实际启动，false=参数、轴状态、安全条件或脉冲资源不允许启动
 */
bool Axis_MoveRel(uint8_t id, uint32_t freq_hz, int32_t steps);

/**
 * @brief  绝对定位 — 移动到指定绝对坐标
 * @param  id         轴 ID
 * @param  freq_hz    脉冲频率 (Hz)
 * @param  target_pos 目标绝对位置 (脉冲数)
 * @return true=已在目标位置或实际启动，false=命令未被接受
 */
bool Axis_MoveAbs(uint8_t id, uint32_t freq_hz, int32_t target_pos);

/**
 * @brief  回原点 — 多段式自动寻找原点传感器
 * @param  id             轴 ID
 * @param  home_dir       原点方向: true = 正方向有原点, false = 负方向有原点
 * @param  fast_freq      快速寻找频率 (Hz)
 * @param  slow_freq      慢速精确定位频率 (Hz)
 * @param  backoff_steps  首次碰到传感器后回退的步数 (用于精确二次定位)
 * @return true=回原已完成或实际启动，false=命令未被接受或脉冲源启动失败
 */
bool Axis_Home(uint8_t id, bool home_dir,
               uint32_t fast_freq, uint32_t slow_freq,
               uint32_t backoff_steps);

/**
 * @brief  立即停止轴运动
 * @param  id 轴 ID
 * @return true=轴有效且已停止，false=轴 ID 无效
 * @note   定位运动被停止时保留未输出的剩余脉冲；取消任务请使用 Axis_Abort()。
 */
bool Axis_Stop(uint8_t id);

/* ---- 状态查询 ---- */

Axis_State_t  Axis_GetState(uint8_t id);     /**< 获取轴运动状态 */
int32_t       Axis_GetPosition(uint8_t id);  /**< 获取当前绝对位置 (脉冲数) */
bool          Axis_IsHomed(uint8_t id);      /**< 是否已完成回原 */
bool          Axis_IsBusy(uint8_t id);       /**< 是否正在运动 */
uint8_t       Axis_GetCount(void);           /**< 获取实际已注册轴数 */

/* ---- 参数设置 ---- */

/**
 * @brief  设置当前位置 (手动校准/清零)
 * @param  id  轴 ID
 * @param  pos 新位置值 (脉冲数)
 * @note   仅在 AXIS_IDLE 状态下有效。
 * @return true=设置成功，false=轴无效或不在空闲状态
 */
bool Axis_SetPosition(uint8_t id, int32_t pos);

/* ---- 软限位 (行程保护) ---- */

/**
 * @brief  设置软限位 (运行保护, 不参与回原)
 * @param  id      轴 ID
 * @param  enable  true=启用软限位, false=关闭 (默认关闭)
 * @param  min_pos 负向软限位坐标 (脉冲数)
 * @param  max_pos 正向软限位坐标 (脉冲数)
 * @return true=设置成功, false=轴无效/轴忙/使能时 min_pos >= max_pos
 * @note   轴必须处于 IDLE。参数由调用者负责持久化。
 */
bool Axis_SetSoftLimit(uint8_t id, bool enable, int32_t min_pos, int32_t max_pos);

/**
 * @brief  读取软限位设置
 * @param  id       轴 ID
 * @param  enable   输出: 是否启用 (可为 NULL)
 * @param  min_pos  输出: 负向软限位 (可为 NULL)
 * @param  max_pos  输出: 正向软限位 (可为 NULL)
 * @return true=读取成功, false=轴无效
 */
bool Axis_GetSoftLimit(uint8_t id, bool *enable, int32_t *min_pos, int32_t *max_pos);

bool Axis_HasRemaining(uint8_t id);              /**< 是否保存了原定位的剩余脉冲。 */
int32_t Axis_GetRemainingSteps(uint8_t id);      /**< 获取带方向的剩余脉冲数。 */
void Axis_ClearRemaining(uint8_t id);            /**< 明确放弃已保存的剩余任务。 */
bool Axis_ResumeRemaining(uint8_t id);           /**< 空闲时恢复剩余脉冲，成功启动返回 true。 */
bool Axis_Abort(uint8_t id);                     /**< 停止脉冲并清除计数、剩余任务和回原标志。 */
bool Axis_Pause(uint8_t id);                     /**< 暂停定位并保存剩余脉冲，成功返回 true。 */
bool Axis_ResumePaused(uint8_t id);              /**< 恢复由 Pause 保存的定位，成功返回 true。 */

/* ---- 定位点 / NVRAM 存储 ---- */

int32_t Axis_GetPointPosition(uint8_t id, uint8_t index);
void Axis_SetPointPosition(uint8_t id, uint8_t index, int32_t pos);
uint16_t Axis_GetPointNameUnit(uint8_t id, uint8_t index, uint8_t char_index);
void Axis_SetPointNameUnit(uint8_t id, uint8_t index, uint8_t char_index, uint16_t value);

/**
 * @brief  读取指定轴指定定位点的运行速度
 * @param  id    轴 ID
 * @param  index 定位点索引 (0~7)
 * @return 速度 (Hz)。该值由上位机写入并掉电保持, 越界时钳位到 [100, 100000]。
 */
uint32_t Axis_GetPointSpeed(uint8_t id, uint8_t index);

/**
 * @brief  写入指定轴指定定位点的运行速度 (仅修改 RAM)
 * @param  id    轴 ID
 * @param  index 定位点索引 (0~7)
 * @param  speed 速度 (Hz), 自动钳位到 [100, 100000]
 * @note   修改后需调用 Axis_SavePointPositions() 写入 Flash 才能掉电保持。
 */
void Axis_SetPointSpeed(uint8_t id, uint8_t index, uint32_t speed);

bool Axis_SavePointPositions(void);

/* ---- 多轴批量操作 (便捷封装) — 每轴独立执行，不联动 ---- */

/**
 * @brief  移动到指定定位点 (坐标与速度均取该点位的掉电保持参数)
 * @param  id          轴 ID
 * @param  point_index 定位点索引 (0~7)
 * @note   坐标由上位机写入 Axis_SetPointPosition()/Modbus 400 区, 掉电保持;
 *         运行速度由上位机写入 Axis_SetPointSpeed()/Modbus 1600 区, 掉电保持。
 *         业务代码只给轴号和点号, 不再硬编码速度, 例如:
 *             Axis_MoveToPoint(Axis_ID[6], 0);
 *         点位速度数据异常 (0 或超出驱动能力) 时回退到 1250 Hz。
 */
void Axis_MoveToPoint(uint8_t id, uint8_t point_index);
void Axis_HomeEx(uint8_t id, bool home_dir, uint32_t fast_hz, uint32_t slow_hz, uint32_t backoff);

/**
 * @brief  回原点 — 完整参数版本 (两种模式, 回退速度独立可调)
 *
 * 两种模式共用流程骨架, 区别只在触发后的处理:
 * - PRECISION: 快找触发 → 反向以 backoff_hz 退到传感器释放 → 慢速 slow_hz 再找
 *              → 触发沿置零 (单向慢速逼近, 消除回差, 精度最高)
 * - TRIGGER:   快找触发 → 反向以 backoff_hz 退到传感器释放 → 置零
 *              (释放沿置零, 免去慢找段, 回原最快; 精度受传感器释放沿重复性限制)
 *
 * 两种模式均不再使用固定步数回退: 反向退到"传感器不触发"为止,
 * 自适应任何快找速度的停止过冲, 免标定。
 *
 * @param  id          轴 ID
 * @param  home_dir    原点方向: true = 正方向有原点, false = 负方向有原点
 * @param  fast_hz     快速寻找频率 (Hz)
 * @param  backoff_hz  触发后反向回退频率 (Hz), 建议低于等于 slow_hz, 保证释放沿检测可靠
 * @param  slow_hz     慢速精确定位频率 (Hz), 仅 PRECISION 模式使用
 * @param  mode        回原模式
 * @return true=回原已完成或实际启动，false=命令未被接受或脉冲源启动失败
 */
bool Axis_HomeWithModeEx(uint8_t id, bool home_dir,
                         uint32_t fast_hz, uint32_t backoff_hz, uint32_t slow_hz,
                         Axis_HomeMode_t mode);

bool Axis_HomeWithMode(uint8_t id, bool home_dir, uint32_t fast_hz, uint32_t slow_hz,
                       uint32_t backoff, Axis_HomeMode_t mode);
void Axis_StopAll(void);
void Axis_AbortAll(void);
void Axis_PauseAll(void);
void Axis_ResumePausedAll(void);
void Axis_ZeroAllSoftwarePositions(void);
bool Axis_AllHomed(void);

/* ---- 系统钩子 (由中断/调度器调用) ---- */

/**
 * @brief  1ms 周期任务 — 传感器监控 + 回原状态机
 * @note   必须在 Tick_Hook() 中调用 (1ms SysTick ISR 上下文)。
 */
void Axis_Tick(void);

/**
 * @brief  PWM 脉冲完成中断回调 — 步数计数 + 到位停止
 * @param  htim HAL 定时器句柄 (由 HAL_TIM_PWM_PulseFinishedCallback 传入)
 */
void Axis_PulseISR(TIM_HandleTypeDef *htim);

void Axis_SoftPulseISR(SoftPWM_Channel_t ch);

/**
 * @brief  动态重新配置已创建的轴 (不改变轴 ID)
 * @param  id     轴 ID (Axis_Create 返回值)
 * @param  config 新的轴配置
 * @return true=重配置成功，false=轴、配置或脉冲通道冲突
 * @note   调用后轴被停止、位置清零、回原标志重置。
 *         适用于上位机动态切换电机引脚绑定。
 */
bool Axis_Reconfigure(uint8_t id, const Axis_Config_t *config);

#endif /* __MOTOR_AXIS_H__ */
