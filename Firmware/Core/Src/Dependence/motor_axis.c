/**
 * @file    motor_axis.c
 * @brief   多轴步进/伺服电机运动控制模块实现
 * @details 基于硬件 PWM (pwm_motor) + 定时器 CC 中断精确计数实现:
 *          - JOG (点动): 持续运行, 不限步数, 适合手动操作
 *          - MoveRel (相对定位): 从当前位置移动指定脉冲步数后自动停止
 *          - MoveAbs (绝对定位): 移动到指定绝对坐标后自动停止
 *          - Homing (多段回原): 快找→退出→慢找→置零, 精确定位机械原点
 *          - 硬限位仅服务回原; 运行保护使用可选软限位
 *          - 绝对位置持续追踪: 每个PWM脉冲都通过CC中断更新position
 *
 *          【架构分层】
 *          应用层 (main_task.c) → motor_axis (本模块, 轴状态管理) → pwm_motor (PWM硬件驱动)
 *
 *          【中断安全】
 *          position / step_counter / target_steps / move_complete 被声明为 volatile,
 *          因为它们在 ISR (Axis_PulseISR) 和主循环 (Axis_Tick/应用层) 间共享。
 */

#include "motor_axis.h"       /* 本模块头文件: Axis_Config_t / Axis_State_t / API 声明 */
#include "app_config.h"
#include "stm32f4xx_hal.h"    /* HAL 库: TIM_HandleTypeDef 等类型定义 (Axis_PulseISR 参数需要) */
#include <limits.h>

/* ==========================================================================
 * 回原 (Homing) 状态机阶段定义 — 宏常量
 *
 * 流程 (PRECISION): 快找触发 → 反向退到释放沿(BACKOFF_RELEASE) → 慢找触发置零
 * 流程 (TRIGGER):   快找触发 → 反向退到释放沿(BACKOFF_RELEASE) → 置零
 * 两种模式均以"退到传感器不触发"取代固定步数回退, 免标定。
 * ========================================================================== */
#define HOME_PHASE_BACKOFF_SENSOR  0  /* 阶段 0: 初始时已在传感器上 → 先反向退出, 等传感器释放后进入阶段1 */
#define HOME_PHASE_FAST_APPROACH   1  /* 阶段 1: 快速向 home_dir 方向运动 → 碰到传感器停止, 进入阶段2 */
#define HOME_PHASE_BACKOFF_RELEASE 2  /* 阶段 2: 反向以回退速度运动 → 传感器释放 → PRECISION 转阶段3, TRIGGER 置零 */
#define HOME_PHASE_SLOW_APPROACH   3  /* 阶段 3: 慢速再次接近传感器 → 碰到传感器停止, 位置清零, 进入阶段4 */
#define HOME_PHASE_DONE            4  /* 阶段 4: 回原完成 → homed=true, state=IDLE */
#define HOME_PHASE_REVERSE_SEARCH  5  /* 阶段 5: 朝 home_dir 先碰到限位 → 反向寻找原点, 找到后转阶段0越过原点 */

/* ==========================================================================
 * 内部轴状态结构体 — 每个物理轴对应一个实例
 * ========================================================================== */
typedef struct
{
    /* ------ 配置区 (Axis_Create 时由调用者传入, 运行时只读, 不会被修改) ------ */
    Axis_Config_t config;

    /* ------ 运行状态区 ------ */
    Axis_State_t  state;

    /* ------ 软限位 (可选行程保护, 不参与回原) ------ */
    bool    soft_limit_enable;
    int32_t soft_limit_min;
    int32_t soft_limit_max;

    /* ------ 位置与计数区 — ISR 和主循环共享, 必须 volatile ------ */
    volatile int32_t  position;
    volatile uint32_t step_counter;
    volatile uint32_t target_steps;
    volatile bool     move_complete;

    /* ------ 运动参数区 ------ */
    bool     current_dir;
    uint32_t current_freq;
    bool     homed;
    bool     use_counting;
    bool     has_remaining;
    int32_t  remaining_steps;
    uint32_t remaining_freq;
    bool     running_remaining;

    /* ------ 回原参数区 ------ */
    uint8_t  home_phase;
    bool     home_dir;
    uint32_t home_fast_freq;
    uint32_t home_slow_freq;
    uint32_t home_backoff_freq;    /* 触发后反向回退速度 (Hz), 独立可调参数 */
    uint32_t home_timeout_counter;
    Axis_HomeMode_t home_mode;
} Axis_Internal_t;

/* ==========================================================================
 * 静态数据
 * ========================================================================== */
static Axis_Internal_t axes[MAX_AXES];
static uint8_t axis_count = 0;

static uint8_t pwm_to_axis[PWM_CH_COUNT] = {
    0xFFU, 0xFFU, 0xFFU, 0xFFU
};

static uint8_t soft_to_axis[SOFT_PWM_CH_COUNT] = {
    0xFFU, 0xFFU, 0xFFU, 0xFFU
};

/* ==========================================================================
 * 内部辅助函数
 * ========================================================================== */

static void set_direction(Axis_Internal_t *axis, bool positive)
{
    /*
     * 方向输出约定:
     * - positive=true  表示本模块认为的“正方向”运动。
     * - 未反转时正方向输出 1，负方向输出 0。
     * - dir_invert=true 时取反，用于适配驱动器 DIR 极性或机械安装方向相反的情况。
     *
     * 注意: 方向必须在启动 PWM 脉冲前设置，避免驱动器在方向建立时间不足时丢步。
     */
    axis->current_dir = positive;
    uint8_t pin_level = positive ? 1 : 0;
    if (axis->config.dir_invert) pin_level = !pin_level;
    axis->config.set_dir(pin_level);
}

static bool read_home_sensor(Axis_Internal_t *axis)
{
    if (!axis->config.sensor_home) return false;
    return input_get(axis->config.sensor_home);
}

static bool read_limit_pos(Axis_Internal_t *axis)
{
    if (!axis->config.sensor_limit_pos) return false;
    return input_get(axis->config.sensor_limit_pos);
}

static bool read_limit_neg(Axis_Internal_t *axis)
{
    if (!axis->config.sensor_limit_neg) return false;
    return input_get(axis->config.sensor_limit_neg);
}

static bool read_limit_dir(Axis_Internal_t *axis, bool positive)
{
    return positive ? read_limit_pos(axis) : read_limit_neg(axis);
}

static bool start_pulses(Axis_Internal_t *axis, uint32_t freq_hz, bool counting)
{
    /*
     * 所有轴运动都启用 CC 中断，保证 position 按真实脉冲持续更新。
     * counting 只决定是否累计 step_counter 并按 target_steps 自动停止:
     * - false: JOG 或回原快速/慢速找点，持续输出直到外部逻辑停止。
     * - true : 相对定位或回原回退固定步数，到达 target_steps 后自动停。
     */
    axis->current_freq = freq_hz;
    axis->use_counting = counting;
    axis->step_counter = 0;
    axis->move_complete = false;

    if (axis->config.pulse_mode == AXIS_PULSE_MODE_SOFT_PWM) {
        SoftPWM_EnableIT(axis->config.soft_ch, freq_hz, 50, Axis_SoftPulseISR);
        return SoftPWM_IsEnabled(axis->config.soft_ch);
    }
    return PWM_EnableIT(axis->config.pwm_ch, freq_hz, 50);
}

static void stop_pulses(Axis_Internal_t *axis)
{
    if (axis->config.pulse_mode == AXIS_PULSE_MODE_SOFT_PWM)
        SoftPWM_Disable(axis->config.soft_ch);
    else
        PWM_Disable(axis->config.pwm_ch);
    axis->use_counting = false;
}

static void clear_remaining(Axis_Internal_t *axis)
{
    axis->has_remaining = false;
    axis->remaining_steps = 0;
    axis->remaining_freq = 0;
    axis->running_remaining = false;
}

static void stop_and_preserve_move(Axis_Internal_t *axis)
{
    bool was_counted_move = axis->state == AXIS_MOVING &&
                            axis->use_counting &&
                            axis->target_steps > 0U;
    bool was_running_remaining = axis->running_remaining;

    stop_pulses(axis);
    axis->running_remaining = false;

    if (!was_counted_move)
        return;

    if (axis->step_counter >= axis->target_steps)
    {
        if (was_running_remaining)
            clear_remaining(axis);
        return;
    }

    /* An inserted move must not overwrite the suspended original move. */
    if (axis->has_remaining && !was_running_remaining)
        return;

    uint32_t remaining = axis->target_steps - axis->step_counter;
    axis->remaining_steps = axis->current_dir
                          ? (int32_t)remaining
                          : -(int32_t)remaining;
    axis->remaining_freq = axis->current_freq;
    axis->has_remaining = true;
}

static bool pulse_config_valid(const Axis_Config_t *config)
{
    if (!config || !config->set_dir) return false;

    if (config->pulse_mode == AXIS_PULSE_MODE_SOFT_PWM)
        return (config->soft_ch < SOFT_PWM_CH_COUNT);

    if (config->pulse_mode == AXIS_PULSE_MODE_PWM)
        return (config->pwm_ch < PWM_CH_COUNT);

    return false;
}

static bool pulse_channel_available(const Axis_Config_t *config, uint8_t owner_id)
{
    uint8_t mapped_id = (config->pulse_mode == AXIS_PULSE_MODE_SOFT_PWM)
                      ? soft_to_axis[config->soft_ch]
                      : pwm_to_axis[config->pwm_ch];
    return mapped_id == 0xFFU || mapped_id == owner_id;
}

static void map_pulse_channel(const Axis_Config_t *config, uint8_t id)
{
    if (config->pulse_mode == AXIS_PULSE_MODE_SOFT_PWM)
        soft_to_axis[config->soft_ch] = id;
    else
        pwm_to_axis[config->pwm_ch] = id;
}

static void unmap_pulse_channel(const Axis_Config_t *config)
{
    if (config->pulse_mode == AXIS_PULSE_MODE_SOFT_PWM)
    {
        if (config->soft_ch < SOFT_PWM_CH_COUNT)
            soft_to_axis[config->soft_ch] = 0xFF;
    }
    else
    {
        if (config->pwm_ch < PWM_CH_COUNT)
            pwm_to_axis[config->pwm_ch] = 0xFF;
    }
}

static void handle_axis_pulse(Axis_Internal_t *axis)
{
    if (axis->current_dir)
    {
        if (axis->position == INT32_MAX)
        {
            stop_pulses(axis);
            axis->state = AXIS_IDLE;
            axis->running_remaining = false;
            clear_remaining(axis);
            return;
        }
        axis->position++;
    }
    else
    {
        if (axis->position == INT32_MIN)
        {
            stop_pulses(axis);
            axis->state = AXIS_IDLE;
            axis->running_remaining = false;
            clear_remaining(axis);
            return;
        }
        axis->position--;
    }

    /* 软限位: 仅约束 JOG/定位, 回原阶段不受影响。 */
    if (axis->soft_limit_enable && axis->state != AXIS_HOMING &&
        (axis->position > axis->soft_limit_max ||
         axis->position < axis->soft_limit_min))
    {
        stop_pulses(axis);
        axis->state = AXIS_IDLE;
        axis->running_remaining = false;
        clear_remaining(axis);
        return;
    }

    if (axis->use_counting)
    {
        axis->step_counter++;

        if (axis->target_steps > 0U && axis->step_counter >= axis->target_steps)
        {
            bool completed_remaining = axis->running_remaining;

            stop_pulses(axis);
            axis->move_complete = true;
            axis->running_remaining = false;
            if (completed_remaining)
                clear_remaining(axis);

            if (axis->state == AXIS_MOVING)
                axis->state = AXIS_IDLE;
        }
    }
}

/* 无报警体系: 任何异常都只停止脉冲并回到 IDLE, 由调用者根据返回值处理。 */
static void abort_motion(Axis_Internal_t *axis)
{
    stop_pulses(axis);
    axis->running_remaining = false;
    clear_remaining(axis);
    axis->state = AXIS_IDLE;
}

/* 回原失败清理: 停止脉冲并取消已回原标志, 不产生报警。 */
static void abort_homing(Axis_Internal_t *axis)
{
    abort_motion(axis);
    axis->homed = false;
    axis->home_phase = HOME_PHASE_DONE;
}

static void finish_homing(Axis_Internal_t *axis)
{
    stop_pulses(axis);
    axis->position = 0;
    axis->homed = true;
    axis->state = AXIS_IDLE;
    axis->home_phase = HOME_PHASE_DONE;
}

/* ==========================================================================
 * 回原状态机
 * ========================================================================== */
static void homing_tick(Axis_Internal_t *axis)
{
    /*
     * 回原状态机每 1ms 被 Axis_Tick() 驱动一次。这里不使用阻塞等待，原因是:
     * 1) 主循环和 Modbus 通信不能被长时间占用；
     * 2) 限位检测、急停、通信看门狗仍需持续运行；
     * 3) 机械回原过程耗时不确定，必须拆成多个短小阶段逐步推进。
     */
    axis->home_timeout_counter++;
    if (axis->home_timeout_counter >= 30000)
    {
        abort_homing(axis);
        return;
    }

    switch (axis->home_phase)
    {
    case HOME_PHASE_BACKOFF_SENSOR:
        if (!read_home_sensor(axis))
        {
            stop_pulses(axis);
            set_direction(axis, axis->home_dir);
            axis->home_phase = HOME_PHASE_FAST_APPROACH;
            if (!start_pulses(axis, axis->home_fast_freq, false))
                abort_homing(axis);
        }
        break;

    case HOME_PHASE_FAST_APPROACH:
        if (read_home_sensor(axis))
        {
            /* 两种模式统一: 触发后反向回退 (速度独立参数), 退到释放沿再分叉 */
            stop_pulses(axis);
            set_direction(axis, !axis->home_dir);
            axis->home_phase = HOME_PHASE_BACKOFF_RELEASE;
            if (!start_pulses(axis, axis->home_backoff_freq, false))
                abort_homing(axis);
        }
        else if (read_limit_dir(axis, axis->home_dir))
        {
            /* 原点不在前方: 反向寻找原点 (硬限位只作回原参考, 不报警) */
            stop_pulses(axis);
            set_direction(axis, !axis->home_dir);
            axis->home_phase = HOME_PHASE_REVERSE_SEARCH;
            if (!start_pulses(axis, axis->home_fast_freq, false))
                abort_homing(axis);
        }
        break;

    case HOME_PHASE_REVERSE_SEARCH:
        /* 找到原点后保持同向运动, 由阶段0越过原点再按 home_dir 正常回原 */
        if (read_home_sensor(axis))
            axis->home_phase = HOME_PHASE_BACKOFF_SENSOR;
        else if (read_limit_dir(axis, !axis->home_dir))
        {
            /* 反向搜索也碰到硬限位: 原点与两个限位布局异常, 停止回原。 */
            abort_homing(axis);
        }
        break;

    case HOME_PHASE_BACKOFF_RELEASE:
        /*
         * 反向回退阶段: 退到传感器释放沿为止 (自适应快找停止过冲, 免标定)。
         * 回退用 home_backoff_freq (独立可调参数), 速度低则释放沿检测更可靠。
         * - PRECISION: 释放后转慢找, 由慢速触发沿决定最终零点 (消除回差)。
         * - TRIGGER:   释放即置零, 省去慢找段 (精度受释放沿重复性限制)。
         */
        if (!read_home_sensor(axis))
        {
            if (axis->home_mode == AXIS_HOME_MODE_TRIGGER)
            {
                finish_homing(axis);
            }
            else
            {
                stop_pulses(axis);
                set_direction(axis, axis->home_dir);
                axis->home_phase = HOME_PHASE_SLOW_APPROACH;
                if (!start_pulses(axis, axis->home_slow_freq, false))
                    abort_homing(axis);
            }
        }
        else if (read_limit_dir(axis, !axis->home_dir))
        {
            /* 回退过程中碰到反向限位仍未释放: 传感器故障或触发区异常, 终止回原。 */
            abort_homing(axis);
        }
        break;

    case HOME_PHASE_SLOW_APPROACH:
        if (read_home_sensor(axis))
        {
            finish_homing(axis);
        }
        else if (read_limit_dir(axis, axis->home_dir))
        {
            /* 慢找仍未找到原点即碰到限位: 停止回原, 不报警 */
            abort_homing(axis);
        }
        break;

    default:
        break;
    }
}

/* ==========================================================================
 * 公共 API 实现
 * ========================================================================== */

uint8_t Axis_Create(const Axis_Config_t *config)
{
    /*
     * 创建轴时要求 set_dir 必须有效，因为任何运动前都必须能够设置方向。
     * 传感器指针可以为 NULL，表示对应功能未配置；无原点传感器时 Axis_Home()
     * 直接返回 false，普通 JOG/定位仍可使用。
     */
    if (axis_count >= MAX_AXES) return 0xFF;
    if (!pulse_config_valid(config)) return 0xFF;
    if (!pulse_channel_available(config, 0xFFU)) return 0xFFU;

    uint8_t id = axis_count++;
    Axis_Internal_t *axis = &axes[id];

    axis->config = *config;

    axis->state        = AXIS_IDLE;
    axis->soft_limit_enable = false;
    axis->soft_limit_min = 0;
    axis->soft_limit_max = 0;
    axis->position     = 0;
    axis->step_counter = 0;
    axis->target_steps = 0;
    axis->move_complete = false;
    axis->current_dir  = true;
    axis->current_freq = 0;
    axis->homed        = false;
    axis->use_counting = false;
    axis->has_remaining = false;
    axis->remaining_steps = 0;
    axis->remaining_freq = 0;
    axis->running_remaining = false;
    axis->home_phase   = HOME_PHASE_DONE;
    axis->home_mode    = AXIS_HOME_MODE_PRECISION;
    axis->home_backoff_freq = 0;

    map_pulse_channel(config, id);

    config->set_dir(0);

    return id;
}

static bool soft_limit_blocks(const Axis_Internal_t *axis, int64_t target)
{
    if (!axis->soft_limit_enable) return false;
    return target > (int64_t)axis->soft_limit_max ||
           target < (int64_t)axis->soft_limit_min;
}

bool Axis_Jog(uint8_t id, uint32_t freq_hz, bool dir)
{
    if (id >= axis_count || freq_hz == 0U) return false;
    Axis_Internal_t *axis = &axes[id];

    if (axis->state != AXIS_IDLE) return false;

    /* 软限位: 已在限位处不允许继续朝外点动, 朝内可正常脱离。 */
    if (axis->soft_limit_enable)
    {
        if (dir && axis->position >= axis->soft_limit_max) return false;
        if (!dir && axis->position <= axis->soft_limit_min) return false;
    }

    set_direction(axis, dir);

    axis->target_steps = 0;
    axis->state = AXIS_JOG;
    axis->running_remaining = false;
    if (!start_pulses(axis, freq_hz, false))
    {
        abort_motion(axis);
        return false;
    }
    return true;
}

static bool start_relative_move(uint8_t id, uint32_t freq_hz, int32_t steps,
                                bool running_remaining)
{
    if (id >= axis_count || steps == 0 || freq_hz == 0U) return false;
    Axis_Internal_t *axis = &axes[id];

    if (axis->state != AXIS_IDLE) return false;

    int64_t magnitude = (steps >= 0) ? (int64_t)steps : -(int64_t)steps;
    if (magnitude > INT32_MAX) return false;

    bool dir = (steps > 0);
    int64_t target = (int64_t)axis->position + (int64_t)steps;
    if (target > INT32_MAX || target < INT32_MIN) return false;

    /* 软限位: 目标越界直接拒绝启动, 不产生报警也不打断当前任务。 */
    if (soft_limit_blocks(axis, target)) return false;

    set_direction(axis, dir);
    axis->target_steps = (uint32_t)magnitude;
    axis->running_remaining = running_remaining;
    axis->state = AXIS_MOVING;
    if (!start_pulses(axis, freq_hz, true))
    {
        abort_motion(axis);
        return false;
    }
    return true;
}

bool Axis_MoveRel(uint8_t id, uint32_t freq_hz, int32_t steps)
{
    return start_relative_move(id, freq_hz, steps, false);
}

bool Axis_MoveAbs(uint8_t id, uint32_t freq_hz, int32_t target_pos)
{
    if (id >= axis_count) return false;
    Axis_Internal_t *axis = &axes[id];
    if (axis->state != AXIS_IDLE) return false;

    if (soft_limit_blocks(axis, (int64_t)target_pos)) return false;

    int64_t delta = (int64_t)target_pos - (int64_t)axis->position;
    if (delta == 0) return true;
    if (delta < INT32_MIN || delta > INT32_MAX) return false;

    return Axis_MoveRel(id, freq_hz, (int32_t)delta);
}

bool Axis_HomeWithModeEx(uint8_t id, bool home_dir,
                         uint32_t fast_hz, uint32_t backoff_hz, uint32_t slow_hz,
                         Axis_HomeMode_t mode)
{
    if (id >= axis_count || fast_hz == 0U || backoff_hz == 0U || slow_hz == 0U) return false;
    Axis_Internal_t *axis = &axes[id];

    if (axis->state != AXIS_IDLE) return false;
    if (!axis->config.sensor_home) return false;

    axis->home_dir          = home_dir;
    axis->home_fast_freq    = fast_hz;
    axis->home_slow_freq    = slow_hz;
    axis->home_backoff_freq = backoff_hz;
    axis->home_mode         = (mode == AXIS_HOME_MODE_TRIGGER)
                            ? AXIS_HOME_MODE_TRIGGER
                            : AXIS_HOME_MODE_PRECISION;
    axis->running_remaining = false;
    axis->homed             = false;
    axis->state             = AXIS_HOMING;
    axis->home_timeout_counter = 0;

    bool home_active = read_home_sensor(axis);
    if (home_active)
    {
        /* 初始已在传感器上: 反向退到释放沿, 之后两种模式在此分叉 */
        axis->home_phase = HOME_PHASE_BACKOFF_RELEASE;
        set_direction(axis, !home_dir);
        if (!start_pulses(axis, backoff_hz, false))
        {
            abort_homing(axis);
            return false;
        }
    }
    else if (read_limit_dir(axis, home_dir))
    {
        /* 已停在原点方向的硬限位上: 直接反向寻找原点 */
        axis->home_phase = HOME_PHASE_REVERSE_SEARCH;
        set_direction(axis, !home_dir);
        if (!start_pulses(axis, fast_hz, false))
        {
            abort_homing(axis);
            return false;
        }
    }
    else
    {
        axis->home_phase = HOME_PHASE_FAST_APPROACH;
        set_direction(axis, home_dir);
        if (!start_pulses(axis, fast_hz, false))
        {
            abort_homing(axis);
            return false;
        }
    }
    return true;
}

bool Axis_Home(uint8_t id, bool home_dir,
               uint32_t fast_freq, uint32_t slow_freq,
               uint32_t backoff_steps)
{
    /* backoff_steps 已废弃: 回退速度默认取慢速频率, 保证释放沿检测可靠 */
    (void)backoff_steps;
    return Axis_HomeWithModeEx(id, home_dir, fast_freq, slow_freq, slow_freq,
                               AXIS_HOME_MODE_PRECISION);
}

bool Axis_HomeWithMode(uint8_t id, bool home_dir,
                       uint32_t fast_freq, uint32_t slow_freq,
                       uint32_t backoff_steps, Axis_HomeMode_t mode)
{
    /* 兼容旧 API: backoff_steps 已废弃, 回退速度默认取慢速频率 */
    (void)backoff_steps;
    return Axis_HomeWithModeEx(id, home_dir, fast_freq, slow_freq, slow_freq, mode);
}

bool Axis_Stop(uint8_t id)
{
    if (id >= axis_count) return false;
    Axis_Internal_t *axis = &axes[id];

    if (axis->state == AXIS_IDLE) return true;

    stop_and_preserve_move(axis);
    axis->state = AXIS_IDLE;
    return true;
}

/* ---- 状态查询 ---- */

Axis_State_t Axis_GetState(uint8_t id)
{
    if (id >= axis_count) return AXIS_IDLE;
    return axes[id].state;
}

int32_t Axis_GetPosition(uint8_t id)
{
    if (id >= axis_count) return 0;
    return axes[id].position;
}

bool Axis_IsHomed(uint8_t id)
{
    if (id >= axis_count) return false;
    return axes[id].homed;
}

bool Axis_IsBusy(uint8_t id)
{
    if (id >= axis_count) return false;
    Axis_State_t s = axes[id].state;
    return (s == AXIS_JOG || s == AXIS_MOVING || s == AXIS_HOMING);
}

uint8_t Axis_GetCount(void)
{
    return axis_count;
}

bool Axis_SetPosition(uint8_t id, int32_t pos)
{
    if (id >= axis_count) return false;
    if (axes[id].state != AXIS_IDLE) return false;
    axes[id].position = pos;
    return true;
}

bool Axis_SetSoftLimit(uint8_t id, bool enable, int32_t min_pos, int32_t max_pos)
{
    if (id >= axis_count) return false;
    Axis_Internal_t *axis = &axes[id];
    if (axis->state != AXIS_IDLE) return false;
    if (enable && min_pos >= max_pos) return false;

    axis->soft_limit_enable = enable;
    axis->soft_limit_min = min_pos;
    axis->soft_limit_max = max_pos;
    return true;
}

bool Axis_GetSoftLimit(uint8_t id, bool *enable, int32_t *min_pos, int32_t *max_pos)
{
    if (id >= axis_count) return false;
    if (enable)  *enable  = axes[id].soft_limit_enable;
    if (min_pos) *min_pos = axes[id].soft_limit_min;
    if (max_pos) *max_pos = axes[id].soft_limit_max;
    return true;
}

bool Axis_HasRemaining(uint8_t id)
{
    if (id >= axis_count) return false;
    return axes[id].has_remaining;
}

int32_t Axis_GetRemainingSteps(uint8_t id)
{
    if (id >= axis_count) return 0;
    return axes[id].has_remaining ? axes[id].remaining_steps : 0;
}

void Axis_ClearRemaining(uint8_t id)
{
    if (id >= axis_count) return;
    clear_remaining(&axes[id]);
}

bool Axis_ResumeRemaining(uint8_t id)
{
    if (id >= axis_count) return false;
    Axis_Internal_t *axis = &axes[id];
    if (axis->state != AXIS_IDLE || !axis->has_remaining) return false;

    int32_t steps = axis->remaining_steps;
    uint32_t freq = axis->remaining_freq ? axis->remaining_freq : 1250U;
    return start_relative_move(id, freq, steps, true);
}

bool Axis_Abort(uint8_t id)
{
    if (id >= axis_count) return false;

    Axis_Internal_t *axis = &axes[id];
    if (axis->state != AXIS_IDLE)
        stop_pulses(axis);

    axis->state = AXIS_IDLE;
    axis->target_steps = 0;
    axis->step_counter = 0;
    axis->move_complete = false;
    axis->running_remaining = false;
    clear_remaining(axis);
    return true;
}

bool Axis_Pause(uint8_t id)
{
    if (id >= axis_count) return false;

    Axis_Internal_t *axis = &axes[id];
    if (axis->state != AXIS_MOVING) return false;

    bool had_suspended_move = axis->has_remaining;
    bool was_running_remaining = axis->running_remaining;
    (void)Axis_Stop(id);

    return axis->has_remaining && (!had_suspended_move || was_running_remaining);
}

bool Axis_ResumePaused(uint8_t id)
{
    if (id >= axis_count) return false;

    Axis_Internal_t *axis = &axes[id];
    if (axis->state != AXIS_IDLE || !axis->has_remaining) return false;

    return Axis_ResumeRemaining(id) && axis->running_remaining;
}

void Axis_StopAll(void)
{
    for (uint8_t i = 0; i < axis_count; i++)
        (void)Axis_Stop(i);
}

void Axis_AbortAll(void)
{
    for (uint8_t i = 0; i < axis_count; i++)
        (void)Axis_Abort(i);
}

void Axis_PauseAll(void)
{
    for (uint8_t i = 0; i < axis_count; i++)
        (void)Axis_Pause(i);
}

void Axis_ResumePausedAll(void)
{
    for (uint8_t i = 0; i < axis_count; i++)
        (void)Axis_ResumePaused(i);
}

void Axis_ZeroAllSoftwarePositions(void)
{
    for (uint8_t i = 0; i < axis_count; i++)
    {
        Axis_Internal_t *a = &axes[i];
        if (a->state == AXIS_IDLE)  { a->position = 0; a->homed = false; clear_remaining(a); }
    }
}

/* ---- 定位点 / NVRAM 存储 ---- */

#include "nvram.h"

int32_t Axis_GetPointPosition(uint8_t id, uint8_t index)
{
    return NVRAM_GetPositionPoint(id, index);
}

void Axis_SetPointPosition(uint8_t id, uint8_t index, int32_t pos)
{
    NVRAM_SetPositionPoint(id, index, pos);
}

uint16_t Axis_GetPointNameUnit(uint8_t id, uint8_t index, uint8_t char_index)
{
    return NVRAM_GetPositionPointNameUnit(id, index, char_index);
}

void Axis_SetPointNameUnit(uint8_t id, uint8_t index, uint8_t char_index, uint16_t value)
{
    NVRAM_SetPositionPointNameUnit(id, index, char_index, value);
}

uint32_t Axis_GetPointSpeed(uint8_t id, uint8_t index)
{
    uint32_t speed = (uint32_t)NVRAM_GetPositionPointSpeed(id, index);

    /* 存储数据异常 (0 或超出 PWM 能力) 时回退到默认速度, 避免运动无法启动。 */
    if (speed < PWM_FREQ_MIN) speed = NVRAM_POINT_SPEED_DEFAULT;
    if (speed > PWM_FREQ_MAX) speed = PWM_FREQ_MAX;
    if (speed > SOFT_PWM_FREQ_MAX) speed = SOFT_PWM_FREQ_MAX;
    return speed;
}

void Axis_SetPointSpeed(uint8_t id, uint8_t index, uint32_t speed)
{
    if (speed < PWM_FREQ_MIN) speed = PWM_FREQ_MIN;
    if (speed > PWM_FREQ_MAX) speed = PWM_FREQ_MAX;
    if (speed > SOFT_PWM_FREQ_MAX) speed = SOFT_PWM_FREQ_MAX;
    NVRAM_SetPositionPointSpeed(id, index, speed);
}

bool Axis_SavePointPositions(void)
{
    for (uint8_t i = 0; i < axis_count; i++)
    {
        if (Axis_IsBusy(i))
            return false;
    }

    return NVRAM_Save();
}

void Axis_MoveToPoint(uint8_t id, uint8_t point_index)
{
    if (id >= axis_count) return;

    /* 速度来自该点位由上位机写入并掉电保持的自带速度, 业务代码不再硬编码速度。 */
    uint32_t freq_hz = Axis_GetPointSpeed(id, point_index);
    int32_t target = NVRAM_GetPositionPoint(id, point_index);
    (void)Axis_MoveAbs(id, freq_hz, target);
}

void Axis_HomeEx(uint8_t id, bool home_dir, uint32_t fast_hz, uint32_t slow_hz, uint32_t backoff)
{
    (void)Axis_Home(id, home_dir, fast_hz, slow_hz, backoff);
}

bool Axis_AllHomed(void)
{
    for (uint8_t i = 0; i < axis_count; i++) {
        if (!axes[i].homed) return false;
    }
    return true;
}

/* ==========================================================================
 * 系统钩子
 * ========================================================================== */

void Axis_Tick(void)
{
    for (uint8_t i = 0; i < axis_count; i++)
    {
        Axis_Internal_t *axis = &axes[i];

        if (axis->state == AXIS_HOMING)
        {
            homing_tick(axis);
        }
    }
}

bool Axis_Reconfigure(uint8_t id, const Axis_Config_t *config)
{
    if (id >= axis_count || !pulse_config_valid(config)) return false;
    if (!pulse_channel_available(config, id)) return false;
    Axis_Internal_t *axis = &axes[id];

    if (axis->state != AXIS_IDLE)
        stop_pulses(axis);

    unmap_pulse_channel(&axis->config);

    axis->config        = *config;
    axis->state         = AXIS_IDLE;
    /* 软限位属于上位机配置并掉电保持的安全参数, 重新绑定引脚不应清除。 */
    axis->position      = 0;
    axis->step_counter  = 0;
    axis->target_steps  = 0;
    axis->move_complete = false;
    axis->current_dir   = true;
    axis->current_freq  = 0;
    axis->homed         = false;
    axis->use_counting  = false;
    axis->has_remaining = false;
    axis->remaining_steps = 0;
    axis->remaining_freq = 0;
    axis->running_remaining = false;
    axis->home_phase    = HOME_PHASE_DONE;
    axis->home_mode     = AXIS_HOME_MODE_PRECISION;
    axis->home_backoff_freq = 0;

    map_pulse_channel(config, id);

    config->set_dir(0);
    return true;
}

void Axis_PulseISR(TIM_HandleTypeDef *htim)
{
    /*
     * 该函数在 HAL PWM 脉冲完成中断中调用，是位置计数的唯一来源。
     * PWM_IdentifyIRQ() 先把 TIM 句柄反查为 PWM 通道，再通过 pwm_to_axis[] 找到轴。
     * 这样 motor_axis 不需要关心 TIM1/TIM2 的具体通道细节。
     */
    PWM_Channel_t pwm_ch = PWM_IdentifyIRQ(htim);
    if (pwm_ch >= PWM_CH_COUNT) return;

    uint8_t id = pwm_to_axis[pwm_ch];
    if (id >= axis_count) return;

    handle_axis_pulse(&axes[id]);
}

void Axis_SoftPulseISR(SoftPWM_Channel_t ch)
{
    if (ch >= SOFT_PWM_CH_COUNT) return;

    uint8_t id = soft_to_axis[ch];
    if (id >= axis_count) return;

    handle_axis_pulse(&axes[id]);
}
