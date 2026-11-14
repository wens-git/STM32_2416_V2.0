/**
 * @file    motor_debug_service.c
 * @brief   2416 通用电机调试服务 — Modbus RTU 从站寄存器映射与命令处理
 * @details 本文件保留当前 1612 工程的 app/debug 分层方式，但内部资源切换为
 *          2416 机型:
 *          - 最多 8 轴步进电机: 4 路硬件 PWM + 4 路 TIM5 软件脉冲
 *          - 24 路输入 (X0~X27)
 *          - 16 路输出 (Y0~Y17)
 *          - 轴引脚绑定由 app/axis/axis_manager.c 固定维护，通过调试协议只读导出
 *          - 通信看门狗断线停机
 */

#include "app/debug/motor_debug_service.h"
#include "app/axis/axis_manager.h"
#include "IODEF.h"
#include "main.h"
#include "motor_axis.h"
#include "nvram.h"
#include "modbus_rtu.h"
#include "bsp_uart.h"
#include "app_config.h"
#if RS485_ENABLE
#include "bsp_uart485.h"
#endif
#include <string.h>

/* ====================================================================
 *  Modbus 从站实例
 * ==================================================================== */
static Modbus_Slave_t mb_slave;
static uint8_t pending_slave_addr = 0U;
#if RS485_ENABLE
static Modbus_Slave_t mb_slave485;
static uint8_t pending_slave_addr_485 = 0U;
#endif

/* ====================================================================
 *  每轴命令参数 (由上位机写入, CMD 触发时读取)
 * ==================================================================== */
typedef struct {
    uint32_t speed;
    int32_t  target_pos;
} AxisParams_t;

static AxisParams_t axis_params[AXIS_APP_COUNT] = {
    { 1250U, 0 }, { 1250U, 0 }, { 1250U, 0 }, { 1250U, 0 },
    { 1250U, 0 }, { 1250U, 0 }, { 1250U, 0 }, { 1250U, 0 }
};
NVRAM_PERSISTENT_REGISTER(axis_params);

/* ====================================================================
 *  系统参数
 * ==================================================================== */
NVRAM_PERSISTENT(uint32_t, home_fast_freq, 1250U);
NVRAM_PERSISTENT(uint32_t, home_slow_freq, 300U);
NVRAM_PERSISTENT(uint32_t, home_backoff_freq, 300U);  /* 回原触发后反向回退速度 (Hz) */
NVRAM_PERSISTENT(uint16_t, home_backoff, 500U);       /* 已废弃: 回退改为退到释放沿, 不再使用 */
NVRAM_PERSISTENT(uint16_t, home_mode, AXIS_HOME_MODE_PRECISION);
NVRAM_PERSISTENT(uint16_t, home_direction_positive, 0U); /* 全局调试回原方向: 0=负向, 1=正向 */
NVRAM_PERSISTENT(uint16_t, subdivision, 5000U);
NVRAM_PERSISTENT(uint16_t, jog_step, 100U);

/* ====================================================================
 *  通信看门狗与诊断
 * ==================================================================== */
NVRAM_PERSISTENT(uint16_t, comm_wdt_timeout_ms, 2000U);
NVRAM_PERSISTENT(uint8_t, configured_slave_addr, 1U);
#if RS485_ENABLE
NVRAM_PERSISTENT(uint8_t, configured_slave_addr_485, RS485_SLAVE_ADDR);
#endif
/* The active instance is switched only while a frame is being processed.
 * It lets the shared register/transaction callbacks update the address and
 * diagnostics of the port that actually received the request. */
static Modbus_Slave_t *mb_active = &mb_slave;
static volatile uint32_t comm_wdt_counter = 0;
static volatile uint8_t  comm_wdt_triggered = 0;
static volatile uint16_t comm_frame_count = 0;
static volatile uint16_t comm_error_count = 0;
#if RS485_ENABLE
static volatile uint16_t comm_frame_count_485 = 0;
static volatile uint16_t comm_error_count_485 = 0;
#endif
static volatile bool     comm_ever_connected = false;

/* ====================================================================
 *  Y 输出函数查找表 (索引 0-15 → Y0~Y17)
 * ==================================================================== */
static void (*const y_funcs[16])(uint8_t) = {
    Y0, Y1, Y2, Y3, Y4, Y5, Y6, Y7,
    Y10, Y11, Y12, Y13, Y14, Y15, Y16, Y17
};

/* ====================================================================
 *  命令码
 * ==================================================================== */
#define CMD_NOP        0
#define CMD_STOP       1
#define CMD_JOG_POS    2
#define CMD_JOG_NEG    3
#define CMD_MOVE_ABS   4
#define CMD_MOVE_REL   5
#define CMD_HOME       6
#define CMD_SET_POS    7
/* 8 保留 (原清报警命令已删除, 固件不再有轴报警) */
#define CMD_SAVE_POINTS 9  /**< 将所有轴定位点写入 Flash */
#define CMD_RESUME     10

#define AXIS_REG_BASE    320U
#define AXIS_REG_STRIDE  10U
#define AXIS_REG_COUNT   (AXIS_REG_STRIDE * AXIS_APP_COUNT)
#define NUM_Y_OUTPUTS    16U
#define NUM_X_INPUTS     24U

/* 定位点寄存器: 每轴 8 个点 × 2 regs(lo,hi) = 16 regs/轴, Base=400
 * 点位名称寄存器: 每轴 8 个点 × 12 regs(UTF-16 码元) = 96 regs/轴, Base=600
 * 轴状态占 320~399，坐标占 400~527，名称占 600~1367。 */
#define POINTS_REG_BASE   400U
#define POINTS_REG_STRIDE 16U
#define POINT_NAME_REG_BASE   600U
#define POINT_NAME_LEN        NVRAM_POINT_NAME_LEN
#define POINT_NAME_REG_STRIDE (NVRAM_POINTS_PER_AXIS * POINT_NAME_LEN)
#define AXIS_CFG_EXT_BASE     1400U
#define AXIS_CFG_EXT_STRIDE   9U
/* 软限位: 每轴 8 个寄存器, Base=1500
 * +0 使能(0/1)  +1/+2 负限位 lo/hi  +3/+4 正限位 lo/hi
 * +5 有效标志   +6 保留        +7 写 0xA55A 立即保存到 Flash */
#define SOFT_LIMIT_REG_BASE   1500U
#define SOFT_LIMIT_REG_STRIDE 8U
#define SOFT_LIMIT_CMD_SAVE   0xA55AU
/* 点位速度: 每轴 8 个点 x 2 regs(lo,hi) = 16 regs/轴, Base=1600 (1600~1727)
 * 单位 Hz, 取值范围 100~100000, 由上位机写入并掉电保持。 */
#define POINT_SPEED_REG_BASE   1600U
#define POINT_SPEED_REG_STRIDE 16U
#define REG_AXIS_CAPACITY     216U
#define REG_SAVE_POINTS       217U
#define REG_HOME_FAST_HI      218U
#define REG_HOME_SLOW_HI      219U
/* USART1 protocol selector; see app_config.h RS232_RUNTIME_PROTOCOL_SWITCH.
 * 0 = USART1 Modbus RTU slave, 1 = USART1 free port. */
#define REG_RS232_PROTO       220U
/* 回原触发后反向回退速度 (Hz), 回退阶段改为"退到传感器释放沿"后启用 */
#define REG_HOME_BACKOFF_HZ   221U
#define REG_HOME_DIRECTION    222U

static uint8_t get_axis_id(uint8_t idx)
{
    return AxisApp_GetAxisId(idx);
}

static bool axis_speed_valid(uint8_t idx, uint32_t speed)
{
    Axis_PinConfig_t cfg;

    if (speed < PWM_FREQ_MIN || !AxisApp_GetPinConfig(idx, &cfg)) {
        return false;
    }
    if (cfg.pulse_mode == AXIS_PULSE_MODE_SOFT_PWM) {
        return speed <= SOFT_PWM_FREQ_MAX;
    }
    return speed <= PWM_FREQ_MAX;
}

static bool home_speed_valid(uint32_t speed)
{
    return speed >= PWM_FREQ_MIN &&
           speed <= PWM_FREQ_MAX &&
           speed <= SOFT_PWM_FREQ_MAX;
}

/* ====================================================================
 *  运行模式 (保持寄存器 215 OP_MODE 软件切换)
 * ==================================================================== */
#define MODE_NORMAL    0   /* 正常运行: 调试功能锁定 */
#define MODE_DEBUG     1   /* 调试模式: 全功能开放 */
static volatile uint8_t op_mode = MODE_NORMAL;

/* USART1 protocol selector (runtime). Mirrors RS232_PROTOCOL_MODE at boot when
 * runtime switching is enabled, so a compile-time free-port build still starts
 * on the free port. Persisted through NVRAM. */
NVRAM_PERSISTENT(uint8_t, rs232_proto_sel,
#if RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT
                 RS232_PROTO_FREEPORT
#else
                 RS232_PROTO_MODBUS
#endif
);

bool Debug_Service_Rs232IsFreePort(void)
{
    return rs232_proto_sel == RS232_PROTO_FREEPORT;
}

static void comm_wdt_disarm(void);

/* 切换 USART1 协议时统一入口：
 * - 从 Modbus 切到自由口后，原来喂看门狗的那条链路已经不存在，
 *   必须解除看门狗武装，否则会在超时后误判断线并急停所有轴；
 * - 解除后由仍然在线的那条链路(通常是 485)重新喂狗后自动恢复保护。 */
static void apply_rs232_proto(uint8_t sel)
{
    uint8_t previous = rs232_proto_sel;

    rs232_proto_sel = sel;
    if (sel == RS232_PROTO_FREEPORT && previous != RS232_PROTO_FREEPORT) {
        comm_wdt_disarm();
    }
}

static void set_op_mode(uint8_t mode)
{
    uint8_t next_mode = (mode != 0) ? MODE_DEBUG : MODE_NORMAL;

    if (next_mode == op_mode) return;

    /* 进入调试模式时先阻断 1ms 业务回调，避免停轴后被立即重启。 */
    if (next_mode == MODE_DEBUG) op_mode = MODE_DEBUG;

    /* 模式边界不能继承上一模式启动的运动。 */
    for (uint8_t i = 0; i < AXIS_APP_COUNT; i++) {
        if (AxisApp_IsRegistered(i) && Axis_IsBusy(get_axis_id(i))) {
            (void)Axis_Stop(get_axis_id(i));
        }
    }

    /* 退出调试模式时停轴完成后才恢复正常业务。 */
    if (next_mode == MODE_NORMAL) op_mode = MODE_NORMAL;
}

typedef struct {
    bool active;
    bool axis_params_dirty;
    bool system_dirty;
    bool points_dirty;
    bool point_names_dirty;
    bool feed_wdt_request;
    bool stop_all_request;
    AxisParams_t axis_params_shadow[AXIS_APP_COUNT];
    uint32_t home_fast_freq_shadow;
    uint32_t home_slow_freq_shadow;
    uint32_t home_backoff_freq_shadow;
    uint16_t home_backoff_shadow;
    uint16_t home_mode_shadow;
    uint16_t home_direction_shadow;
    uint16_t subdivision_shadow;
    uint16_t jog_step_shadow;
    uint16_t comm_wdt_timeout_ms_shadow;
    uint8_t mb_slave_addr_shadow;
    uint8_t op_mode_shadow;
    uint8_t rs232_proto_shadow;
    bool soft_limit_dirty;
    bool soft_limit_save_request;
    bool soft_limit_enable_shadow[AXIS_APP_COUNT];
    int32_t soft_limit_min_shadow[AXIS_APP_COUNT];
    int32_t soft_limit_max_shadow[AXIS_APP_COUNT];
    int32_t point_shadow[AXIS_APP_COUNT][NVRAM_POINTS_PER_AXIS];
    uint16_t point_name_shadow[AXIS_APP_COUNT][NVRAM_POINTS_PER_AXIS][POINT_NAME_LEN];
    bool point_speeds_dirty;
    uint32_t point_speed_shadow[AXIS_APP_COUNT][NVRAM_POINTS_PER_AXIS];
} RegWriteTxn_t;

static RegWriteTxn_t g_reg_txn;
static void comm_wdt_feed(void);

static bool is_axis_write_allowed(uint8_t off)
{
    if (op_mode != MODE_NORMAL) return true;
    return (off == 6U || off == 7U || off == 8U);
}

static bool is_system_write_allowed(uint16_t addr)
{
    if (op_mode != MODE_NORMAL) return true;
    return (addr == 206U || addr == 214U || addr == 215U ||
            addr == REG_HOME_BACKOFF_HZ ||
            addr == REG_RS232_PROTO || addr == REG_SAVE_POINTS);
}

static void txn_clear(void)
{
    memset(&g_reg_txn, 0, sizeof(g_reg_txn));
}

static bool txn_begin(uint16_t start_addr, uint16_t quantity)
{
    uint32_t end_addr = (uint32_t)start_addr + quantity - 1U;

    txn_clear();
    g_reg_txn.active = true;
    memcpy(g_reg_txn.axis_params_shadow, axis_params, sizeof(axis_params));
    g_reg_txn.home_fast_freq_shadow = home_fast_freq;
    g_reg_txn.home_slow_freq_shadow = home_slow_freq;
    g_reg_txn.home_backoff_freq_shadow = home_backoff_freq;
    g_reg_txn.home_backoff_shadow = home_backoff;
    g_reg_txn.home_mode_shadow = home_mode;
    g_reg_txn.home_direction_shadow = home_direction_positive;
    g_reg_txn.subdivision_shadow = subdivision;
    g_reg_txn.jog_step_shadow = jog_step;
    g_reg_txn.comm_wdt_timeout_ms_shadow = comm_wdt_timeout_ms;
    g_reg_txn.mb_slave_addr_shadow = mb_active->slave_addr;
    g_reg_txn.op_mode_shadow = op_mode;
    g_reg_txn.rs232_proto_shadow = rs232_proto_sel;
    for (uint8_t axis = 0; axis < AXIS_APP_COUNT; axis++) {
        (void)AxisApp_GetSoftLimit(axis,
                                  &g_reg_txn.soft_limit_enable_shadow[axis],
                                  &g_reg_txn.soft_limit_min_shadow[axis],
                                  &g_reg_txn.soft_limit_max_shadow[axis]);
    }

    if ((start_addr <= (POINTS_REG_BASE + AXIS_APP_COUNT * POINTS_REG_STRIDE - 1U)) &&
        (end_addr >= POINTS_REG_BASE)) {
        for (uint8_t axis = 0; axis < AXIS_APP_COUNT; axis++) {
            for (uint8_t point = 0; point < NVRAM_POINTS_PER_AXIS; point++) {
                g_reg_txn.point_shadow[axis][point] =
                    NVRAM_GetPositionPoint(axis, point);
            }
        }
    }

    if ((start_addr <= (POINT_SPEED_REG_BASE + AXIS_APP_COUNT * POINT_SPEED_REG_STRIDE - 1U)) &&
        (end_addr >= POINT_SPEED_REG_BASE)) {
        for (uint8_t axis = 0; axis < AXIS_APP_COUNT; axis++) {
            for (uint8_t point = 0; point < NVRAM_POINTS_PER_AXIS; point++) {
                g_reg_txn.point_speed_shadow[axis][point] =
                    NVRAM_GetPositionPointSpeed(axis, point);
            }
        }
    }

    if ((start_addr <= (POINT_NAME_REG_BASE + AXIS_APP_COUNT * POINT_NAME_REG_STRIDE - 1U)) &&
        (end_addr >= POINT_NAME_REG_BASE)) {
        for (uint8_t axis = 0; axis < AXIS_APP_COUNT; axis++) {
            for (uint8_t point = 0; point < NVRAM_POINTS_PER_AXIS; point++) {
                for (uint8_t ch = 0; ch < POINT_NAME_LEN; ch++) {
                    g_reg_txn.point_name_shadow[axis][point][ch] =
                        NVRAM_GetPositionPointNameUnit(axis, point, ch);
                }
            }
        }
    }

    return true;
}

static void txn_finish(bool commit)
{
    if (!g_reg_txn.active) return;

    if (commit) {
        if (g_reg_txn.axis_params_dirty) {
            memcpy(axis_params, g_reg_txn.axis_params_shadow, sizeof(axis_params));
        }
        if (g_reg_txn.system_dirty) {
            home_fast_freq = g_reg_txn.home_fast_freq_shadow;
            home_slow_freq = g_reg_txn.home_slow_freq_shadow;
            home_backoff_freq = g_reg_txn.home_backoff_freq_shadow;
            home_backoff = g_reg_txn.home_backoff_shadow;
            home_mode = g_reg_txn.home_mode_shadow;
            home_direction_positive = g_reg_txn.home_direction_shadow;
            subdivision = g_reg_txn.subdivision_shadow;
            jog_step = g_reg_txn.jog_step_shadow;
            comm_wdt_timeout_ms = g_reg_txn.comm_wdt_timeout_ms_shadow;
            comm_wdt_counter = 0;
            comm_wdt_triggered = 0;
#if RS485_ENABLE
            if (mb_active == &mb_slave485) {
                if (g_reg_txn.mb_slave_addr_shadow != mb_slave485.slave_addr) {
                    pending_slave_addr_485 = g_reg_txn.mb_slave_addr_shadow;
                }
                configured_slave_addr_485 = g_reg_txn.mb_slave_addr_shadow;
            } else
#endif
            {
                if (g_reg_txn.mb_slave_addr_shadow != mb_slave.slave_addr) {
                    pending_slave_addr = g_reg_txn.mb_slave_addr_shadow;
                }
                configured_slave_addr = g_reg_txn.mb_slave_addr_shadow;
            }
            set_op_mode(g_reg_txn.op_mode_shadow);
            apply_rs232_proto(g_reg_txn.rs232_proto_shadow);
            if (g_reg_txn.feed_wdt_request) {
                comm_wdt_feed();
            }
            if (g_reg_txn.stop_all_request) {
                for (uint8_t i = 0; i < AXIS_APP_COUNT; i++) {
                    if (AxisApp_IsRegistered(i)) (void)Axis_Abort(get_axis_id(i));
                }
            }
        }
        if (g_reg_txn.soft_limit_dirty) {
            for (uint8_t axis = 0; axis < AXIS_APP_COUNT; axis++) {
                if (!AxisApp_IsRegistered(axis)) continue;
                (void)AxisApp_SetSoftLimit(
                    axis,
                    g_reg_txn.soft_limit_enable_shadow[axis],
                    g_reg_txn.soft_limit_min_shadow[axis],
                    g_reg_txn.soft_limit_max_shadow[axis]);
            }
        }
        if (g_reg_txn.soft_limit_save_request) {
            (void)NVRAM_Auto_Save();
        }
        if (g_reg_txn.points_dirty) {
            for (uint8_t axis = 0; axis < AXIS_APP_COUNT; axis++) {
                for (uint8_t point = 0; point < NVRAM_POINTS_PER_AXIS; point++) {
                    NVRAM_SetPositionPoint(axis, point,
                                           g_reg_txn.point_shadow[axis][point]);
                }
            }
        }
        if (g_reg_txn.point_speeds_dirty) {
            for (uint8_t axis = 0; axis < AXIS_APP_COUNT; axis++) {
                for (uint8_t point = 0; point < NVRAM_POINTS_PER_AXIS; point++) {
                    NVRAM_SetPositionPointSpeed(axis, point,
                                                g_reg_txn.point_speed_shadow[axis][point]);
                }
            }
        }
        if (g_reg_txn.point_names_dirty) {
            for (uint8_t axis = 0; axis < AXIS_APP_COUNT; axis++) {
                for (uint8_t point = 0; point < NVRAM_POINTS_PER_AXIS; point++) {
                    for (uint8_t ch = 0; ch < POINT_NAME_LEN; ch++) {
                        NVRAM_SetPositionPointNameUnit(
                            axis, point, ch,
                            g_reg_txn.point_name_shadow[axis][point][ch]);
                    }
                }
            }
        }
    }

    txn_clear();
}

/* ====================================================================
 *  通信看门狗: 刷新 + 超时检测
 * ==================================================================== */
static void comm_wdt_feed(void)
{
    comm_wdt_counter = 0;
    if (comm_wdt_triggered) {
        comm_wdt_triggered = 0;
    }
    comm_ever_connected = true;
}

static void comm_wdt_disarm(void)
{
    comm_wdt_counter = 0;
    comm_wdt_triggered = 0;
    comm_ever_connected = false;
}

static void comm_wdt_tick(void)
{
    if (comm_wdt_timeout_ms == 0 || !comm_ever_connected)
        return;

    comm_wdt_counter++;
    if (comm_wdt_counter >= comm_wdt_timeout_ms && !comm_wdt_triggered) {
        comm_wdt_triggered = 1;
        for (uint8_t i = 0; i < AXIS_APP_COUNT; i++) {
            if (AxisApp_IsRegistered(i)) (void)Axis_Abort(get_axis_id(i));
        }
    }
}

/* ====================================================================
 *  命令执行
 * ==================================================================== */
static uint8_t exec_axis_cmd(uint8_t idx, uint16_t cmd)
{
    if (idx >= AXIS_APP_COUNT) return MB_EX_ILLEGAL_DATA_ADDRESS;
    if (cmd == CMD_NOP) return MB_EX_ILLEGAL_DATA_VALUE;

    /* 正常运行模式下只允许 STOP / SET_POS */
    if (op_mode == MODE_NORMAL &&
        cmd != CMD_STOP && cmd != CMD_SET_POS) {
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    if (!AxisApp_IsRegistered(idx)) return MB_EX_ILLEGAL_DATA_ADDRESS;
    uint8_t id = get_axis_id(idx);
    AxisParams_t *p = &axis_params[idx];
    uint32_t spd = (p->speed > 0) ? p->speed : 1250;
    bool home_dir_positive = false;
    Axis_PinConfig_t pin_cfg;
    Axis_State_t state = Axis_GetState(id);
    int32_t position = Axis_GetPosition(id);

    if (cmd != CMD_STOP &&
        cmd != CMD_SET_POS && cmd != CMD_SAVE_POINTS &&
        cmd != CMD_RESUME && !axis_speed_valid(idx, spd)) {
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    switch (cmd) {
    case CMD_STOP:
        return Axis_Stop(id) ? MB_WRITE_OK : MB_EX_ILLEGAL_DATA_VALUE;
    case CMD_JOG_POS:
        return Axis_Jog(id, spd, true) ? MB_WRITE_OK : MB_EX_ILLEGAL_DATA_VALUE;
    case CMD_JOG_NEG:
        return Axis_Jog(id, spd, false) ? MB_WRITE_OK : MB_EX_ILLEGAL_DATA_VALUE;
    case CMD_MOVE_ABS:
        if (state != AXIS_IDLE) return MB_EX_ILLEGAL_DATA_VALUE;
        if (position == p->target_pos) return MB_WRITE_OK;
        return Axis_MoveAbs(id, spd, p->target_pos)
             ? MB_WRITE_OK : MB_EX_ILLEGAL_DATA_VALUE;
    case CMD_MOVE_REL:
        if (state != AXIS_IDLE) return MB_EX_ILLEGAL_DATA_VALUE;
        if (p->target_pos == 0) return MB_WRITE_OK;
        return Axis_MoveRel(id, spd, p->target_pos)
             ? MB_WRITE_OK : MB_EX_ILLEGAL_DATA_VALUE;
    case CMD_HOME:
        if (state != AXIS_IDLE) return MB_EX_ILLEGAL_DATA_VALUE;
        if (!home_speed_valid(home_fast_freq) ||
            !home_speed_valid(home_slow_freq)) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        if (!AxisApp_GetPinConfig(idx, &pin_cfg) || pin_cfg.sensor_home == AXIS_PIN_NONE) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        home_dir_positive = (home_direction_positive == 1U);
        return Axis_HomeWithModeEx(id, home_dir_positive, home_fast_freq,
                                   home_backoff_freq, home_slow_freq,
                                   (home_mode == AXIS_HOME_MODE_TRIGGER)
                                   ? AXIS_HOME_MODE_TRIGGER
                                   : AXIS_HOME_MODE_PRECISION)
             ? MB_WRITE_OK : MB_EX_ILLEGAL_DATA_VALUE;
    case CMD_SET_POS:
        if (state != AXIS_IDLE) return MB_EX_ILLEGAL_DATA_VALUE;
        return Axis_SetPosition(id, p->target_pos)
             ? MB_WRITE_OK : MB_EX_ILLEGAL_DATA_VALUE;
    case CMD_SAVE_POINTS:
        for (uint8_t axis = 0; axis < AXIS_APP_COUNT; axis++) {
            if (AxisApp_IsRegistered(axis) && Axis_IsBusy(get_axis_id(axis))) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
        }
        return Axis_SavePointPositions() ? MB_WRITE_OK : MB_EX_SLAVE_DEVICE_FAILURE;
    case CMD_RESUME:
        if (state != AXIS_IDLE || !Axis_HasRemaining(id)) {
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
        return Axis_ResumeRemaining(id)
             ? MB_WRITE_OK : MB_EX_ILLEGAL_DATA_VALUE;
    default:
        return MB_EX_ILLEGAL_DATA_VALUE;
    }
}

/* ====================================================================
 *  Modbus 回调: 读保持寄存器 (FC03)
 * ==================================================================== */
static uint8_t mb_read_hreg(uint16_t addr, uint16_t *value)
{
    if (value == NULL) return MB_EX_SLAVE_DEVICE_FAILURE;

    /* --- 轴状态 320-399 --- */
    if (addr >= AXIS_REG_BASE && addr < (AXIS_REG_BASE + AXIS_REG_COUNT)) {
        uint16_t axis_addr = (uint16_t)(addr - AXIS_REG_BASE);
        uint8_t idx = (uint8_t)(axis_addr / AXIS_REG_STRIDE);
        uint8_t off = (uint8_t)(axis_addr % AXIS_REG_STRIDE);
        if (idx >= AXIS_APP_COUNT) return MB_EX_ILLEGAL_DATA_ADDRESS;
        if (!AxisApp_IsRegistered(idx)) return MB_EX_ILLEGAL_DATA_ADDRESS;
        uint8_t id = get_axis_id(idx);
        int32_t pos = Axis_GetPosition(id);
        uint32_t upos = (uint32_t)pos;

        switch (off) {
        case 0: *value = (uint16_t)(upos & 0xFFFF); break;
        case 1: *value = (uint16_t)((upos >> 16) & 0xFFFF); break;
        case 2: *value = (uint16_t)Axis_GetState(id); break;
        case 3: *value = Axis_IsHomed(id) ? 1U : 0U; break;
        case 4: *value = 0U; break;  /* 保留: 固件已删除轴报警 */
        case 5: *value = (uint16_t)(axis_params[idx].speed & 0xFFFFU); break;
        case 6: { uint32_t u = (uint32_t)axis_params[idx].target_pos;
                  *value = (uint16_t)(u & 0xFFFF); break; }
        case 7: { uint32_t u = (uint32_t)axis_params[idx].target_pos;
                  *value = (uint16_t)((u >> 16) & 0xFFFF); break; }
        case 8: *value = 0U; break;
        case 9: *value = (uint16_t)((axis_params[idx].speed >> 16) & 0xFFFFU); break;
        default: return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        return MB_CB_OK;
    }

    /* --- 轴配置 100-131: 只读导出, 每轴 8 寄存器 --- */
    if (addr >= 100 && addr < (uint16_t)(100U + AXIS_APP_COUNT * 8U)) {
        uint8_t idx = (uint8_t)((addr - 100U) / 8U);
        uint8_t off = (uint8_t)((addr - 100U) % 8U);
        Axis_PinConfig_t cfg;

        if (!AxisApp_GetPinConfig(idx, &cfg)) return MB_EX_ILLEGAL_DATA_ADDRESS;
        switch (off) {
        case 0: *value = cfg.pwm_ch; break;
        case 1: *value = cfg.dir_y; break;
        case 2: *value = cfg.sensor_home; break;
        case 3: *value = cfg.sensor_limit_pos; break;
        case 4: *value = cfg.sensor_limit_neg; break;
        case 5: *value = cfg.flags; break;
        case 6: *value = 0U; break;  /* 保留: 原外部报警输入已删除 */
        case 7: *value = cfg.sensor_latch; break;
        default: return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        return MB_CB_OK;
    }

    if (addr >= AXIS_CFG_EXT_BASE &&
        addr < (uint16_t)(AXIS_CFG_EXT_BASE + AXIS_APP_COUNT * AXIS_CFG_EXT_STRIDE)) {
        uint16_t rel = (uint16_t)(addr - AXIS_CFG_EXT_BASE);
        uint8_t idx = (uint8_t)(rel / AXIS_CFG_EXT_STRIDE);
        uint8_t off = (uint8_t)(rel % AXIS_CFG_EXT_STRIDE);
        Axis_PinConfig_t cfg;

        if (!AxisApp_GetPinConfig(idx, &cfg)) return MB_EX_ILLEGAL_DATA_ADDRESS;
        switch (off) {
        case 0: *value = cfg.pulse_mode; break;
        case 1: *value = cfg.pwm_ch; break;
        case 2: *value = cfg.soft_ch; break;
        case 3: *value = cfg.dir_y; break;
        case 4: *value = cfg.sensor_home; break;
        case 5: *value = cfg.sensor_limit_pos; break;
        case 6: *value = cfg.sensor_limit_neg; break;
        case 7: *value = cfg.flags; break;
        case 8: *value = 0U; break;  /* 保留: 原外部报警输入已删除 */
        default: return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        return MB_CB_OK;
    }

    /* --- 系统 200-220 --- */
    if (addr >= 200 && addr <= REG_HOME_DIRECTION) {
        switch (addr) {
        case 200: *value = 0x2416; break;
        case 201: *value = AxisApp_GetRegisteredCount(); break;
        case 202: *value = home_fast_freq; break;
        case 203: *value = home_slow_freq; break;
        case 204: *value = home_backoff; break;
        case 205: *value = home_mode; break;
        case 206: *value = 0U; break;
        case 207: *value = mb_active->slave_addr; break;
        case 208: *value = subdivision; break;
        case 209: *value = jog_step; break;
        case 210: *value = comm_wdt_timeout_ms; break;
        case 211: *value = comm_wdt_triggered; break;
        case 212:
#if RS485_ENABLE
            *value = (mb_active == &mb_slave485)
                   ? comm_frame_count_485 : comm_frame_count;
#else
            *value = comm_frame_count;
#endif
            break;
        case 213:
#if RS485_ENABLE
            if (mb_active == &mb_slave485) {
                *value = (uint16_t)(comm_error_count_485 +
                                    BSP_UART485_GetDroppedFrames());
                break;
            }
#endif
            *value = (uint16_t)(comm_error_count + BSP_UART_GetDroppedFrames());
            break;
        case 214: *value = 0U; break;
        case 215: *value = op_mode; break;  /* 0=正常运行, 1=调试模式 */
        case REG_AXIS_CAPACITY: *value = AXIS_APP_CAPACITY; break;
        case REG_SAVE_POINTS: *value = 0U; break;
        case REG_HOME_FAST_HI: *value = (uint16_t)((home_fast_freq >> 16) & 0xFFFFU); break;
        case REG_HOME_SLOW_HI: *value = (uint16_t)((home_slow_freq >> 16) & 0xFFFFU); break;
        case REG_HOME_BACKOFF_HZ: *value = (uint16_t)(home_backoff_freq & 0xFFFFU); break;
        case REG_HOME_DIRECTION: *value = (home_direction_positive == 1U) ? 1U : 0U; break;
        case REG_RS232_PROTO: *value = rs232_proto_sel; break;
        default:  return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        return MB_CB_OK;
    }

    /* --- I/O 300-302 --- */
    if (addr >= 300 && (addr <= 302)) {
        uint16_t val = 0;
        switch (addr) {
        case 300:
            for (uint8_t i = 0; i < 16U; i++)
                if (X_State[i]) val |= (1u << i);
            *value = val;
            return MB_CB_OK;
        case 301:
            for (uint8_t i = 0; i < 8U; i++)
                if (X_State[16U + i]) val |= (1u << i);
            *value = val;
            return MB_CB_OK;
        case 302:
            for (uint8_t i = 0; i < NUM_Y_OUTPUTS; i++)
                if (Y_State[i]) val |= (1u << i);
            *value = val;
            return MB_CB_OK;
        default:
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
    }

    /* --- 定位点 400-527: 可读写, 每轴 16 寄存器 (8点 × 2 lo/hi) --- */
    if (addr >= POINTS_REG_BASE && addr < (uint16_t)(POINTS_REG_BASE + AXIS_APP_COUNT * POINTS_REG_STRIDE)) {
        uint16_t rel = (uint16_t)(addr - POINTS_REG_BASE);
        uint8_t axis_idx = (uint8_t)(rel / POINTS_REG_STRIDE);
        uint8_t point_off = (uint8_t)(rel % POINTS_REG_STRIDE);
        uint8_t point_idx = point_off / 2U;  /* 0~7 */
        bool    is_hi    = (point_off % 2U) != 0;

        if (axis_idx >= AXIS_APP_COUNT || point_idx >= 8U) return MB_EX_ILLEGAL_DATA_ADDRESS;

        int32_t pos = NVRAM_GetPositionPoint(axis_idx, point_idx);
        uint32_t u = (uint32_t)pos;
        *value = is_hi ? (uint16_t)((u >> 16) & 0xFFFF) : (uint16_t)(u & 0xFFFF);
        return MB_CB_OK;
    }

    /* --- 点位名称 600-1367: 可读写, 每轴 8点 × 12 UTF-16 码元 --- */
    if (addr >= POINT_NAME_REG_BASE &&
        addr < (uint16_t)(POINT_NAME_REG_BASE + AXIS_APP_COUNT * POINT_NAME_REG_STRIDE)) {
        uint16_t rel = (uint16_t)(addr - POINT_NAME_REG_BASE);
        uint8_t axis_idx = (uint8_t)(rel / POINT_NAME_REG_STRIDE);
        uint8_t axis_off = (uint8_t)(rel % POINT_NAME_REG_STRIDE);
        uint8_t point_idx = axis_off / POINT_NAME_LEN;
        uint8_t char_idx = axis_off % POINT_NAME_LEN;

        if (axis_idx >= AXIS_APP_COUNT || point_idx >= NVRAM_POINTS_PER_AXIS) return MB_EX_ILLEGAL_DATA_ADDRESS;
        *value = NVRAM_GetPositionPointNameUnit(axis_idx, point_idx, char_idx);
        return MB_CB_OK;
    }

    /* --- 点位速度 1600-1727: 可读写, 每轴 16 寄存器 (8点 x 2 lo/hi) --- */
    if (addr >= POINT_SPEED_REG_BASE &&
        addr < (uint16_t)(POINT_SPEED_REG_BASE + AXIS_APP_COUNT * POINT_SPEED_REG_STRIDE)) {
        uint16_t rel = (uint16_t)(addr - POINT_SPEED_REG_BASE);
        uint8_t axis_idx = (uint8_t)(rel / POINT_SPEED_REG_STRIDE);
        uint8_t point_off = (uint8_t)(rel % POINT_SPEED_REG_STRIDE);
        uint8_t point_idx = point_off / 2U;  /* 0~7 */
        bool    is_hi    = (point_off % 2U) != 0;
        uint32_t uspeed;

        if (axis_idx >= AXIS_APP_COUNT || point_idx >= NVRAM_POINTS_PER_AXIS)
            return MB_EX_ILLEGAL_DATA_ADDRESS;

        uspeed = NVRAM_GetPositionPointSpeed(axis_idx, point_idx);
        *value = is_hi ? (uint16_t)((uspeed >> 16) & 0xFFFFU)
                       : (uint16_t)(uspeed & 0xFFFFU);
        return MB_CB_OK;
    }

    /* --- 软限位 1500-1563: 每轴 8 寄存器 --- */
    if (addr >= SOFT_LIMIT_REG_BASE &&
        addr < (uint16_t)(SOFT_LIMIT_REG_BASE + AXIS_APP_COUNT * SOFT_LIMIT_REG_STRIDE)) {
        uint16_t rel = (uint16_t)(addr - SOFT_LIMIT_REG_BASE);
        uint8_t idx = (uint8_t)(rel / SOFT_LIMIT_REG_STRIDE);
        uint8_t off = (uint8_t)(rel % SOFT_LIMIT_REG_STRIDE);
        bool en = false;
        int32_t min_pos = 0, max_pos = 0;

        if (!AxisApp_GetSoftLimit(idx, &en, &min_pos, &max_pos))
            return MB_EX_ILLEGAL_DATA_ADDRESS;

        switch (off) {
        case 0: *value = en ? 1U : 0U; break;
        case 1: *value = (uint16_t)((uint32_t)min_pos & 0xFFFFU); break;
        case 2: *value = (uint16_t)(((uint32_t)min_pos >> 16) & 0xFFFFU); break;
        case 3: *value = (uint16_t)((uint32_t)max_pos & 0xFFFFU); break;
        case 4: *value = (uint16_t)(((uint32_t)max_pos >> 16) & 0xFFFFU); break;
        case 5: *value = (en && min_pos < max_pos) ? 1U : 0U; break;
        case 6: *value = 0U; break;
        case 7: *value = 0U; break;
        default: return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
        return MB_CB_OK;
    }

    return MB_EX_ILLEGAL_DATA_ADDRESS;
}

/* ====================================================================
 *  Modbus 回调: 写保持寄存器 (FC06/FC16)
 * ==================================================================== */
static uint8_t mb_write_hreg(uint16_t addr, uint16_t value)
{
    /* --- 轴参数/命令 320-399 --- */
    if (addr >= AXIS_REG_BASE && addr < (AXIS_REG_BASE + AXIS_REG_COUNT)) {
        uint16_t axis_addr = (uint16_t)(addr - AXIS_REG_BASE);
        uint8_t idx = (uint8_t)(axis_addr / AXIS_REG_STRIDE);
        uint8_t off = (uint8_t)(axis_addr % AXIS_REG_STRIDE);
        if (idx >= AXIS_APP_COUNT) return MB_EX_ILLEGAL_DATA_ADDRESS;
        if (!AxisApp_IsRegistered(idx)) return MB_EX_ILLEGAL_DATA_ADDRESS;
        if (!is_axis_write_allowed(off)) return MB_EX_ILLEGAL_DATA_VALUE;
        switch (off) {
        case 5: {
            uint32_t old_speed = g_reg_txn.active
                               ? g_reg_txn.axis_params_shadow[idx].speed
                               : axis_params[idx].speed;
            uint32_t speed = (old_speed & 0xFFFF0000U) | value;
            if (!axis_speed_valid(idx, speed)) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            if (g_reg_txn.active) {
                g_reg_txn.axis_params_shadow[idx].speed = speed;
                g_reg_txn.axis_params_dirty = true;
            } else {
                axis_params[idx].speed = speed;
            }
            return MB_WRITE_OK;
        }
        case 6: {
            int32_t target = g_reg_txn.active ? g_reg_txn.axis_params_shadow[idx].target_pos
                                              : axis_params[idx].target_pos;
            uint32_t hi = (uint32_t)target & 0xFFFF0000u;
            target = (int32_t)(hi | value);
            if (g_reg_txn.active) {
                g_reg_txn.axis_params_shadow[idx].target_pos = target;
                g_reg_txn.axis_params_dirty = true;
            } else {
                axis_params[idx].target_pos = target;
            }
            return MB_WRITE_OK;
        }
        case 7: {
            int32_t target = g_reg_txn.active ? g_reg_txn.axis_params_shadow[idx].target_pos
                                              : axis_params[idx].target_pos;
            uint32_t lo = (uint32_t)target & 0x0000FFFFu;
            target = (int32_t)(((uint32_t)value << 16) | lo);
            if (g_reg_txn.active) {
                g_reg_txn.axis_params_shadow[idx].target_pos = target;
                g_reg_txn.axis_params_dirty = true;
            } else {
                axis_params[idx].target_pos = target;
            }
            return MB_WRITE_OK;
        }
        case 8:
            if (g_reg_txn.active) return MB_EX_ILLEGAL_DATA_VALUE;
            return exec_axis_cmd(idx, value);
        case 9: {
            uint32_t old_speed = g_reg_txn.active
                               ? g_reg_txn.axis_params_shadow[idx].speed
                               : axis_params[idx].speed;
            uint32_t speed = ((uint32_t)value << 16) | (old_speed & 0xFFFFU);
            if (value > (PWM_FREQ_MAX >> 16)) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            if (g_reg_txn.active) {
                g_reg_txn.axis_params_shadow[idx].speed = speed;
                g_reg_txn.axis_params_dirty = true;
            } else {
                axis_params[idx].speed = speed;
            }
            return MB_WRITE_OK;
        }
        default:
            return MB_EX_ILLEGAL_DATA_VALUE;
        }
    }

    /* --- 轴配置 100-131: 只读, 绑定不对外修改 --- */
    if (addr >= 100 && addr < (uint16_t)(100U + AXIS_APP_COUNT * 8U)) {
        (void)value;
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    if (addr >= AXIS_CFG_EXT_BASE &&
        addr < (uint16_t)(AXIS_CFG_EXT_BASE + AXIS_APP_COUNT * AXIS_CFG_EXT_STRIDE)) {
        (void)value;
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    /* --- 系统 200-220 --- */
    if (addr >= 200 && addr <= REG_HOME_DIRECTION) {
        if (!is_system_write_allowed(addr)) return MB_EX_ILLEGAL_DATA_VALUE;
        if (addr == 215) {
            if (g_reg_txn.active) {
                g_reg_txn.op_mode_shadow = (value != 0U) ? MODE_DEBUG : MODE_NORMAL;
                g_reg_txn.system_dirty = true;
            } else {
                set_op_mode((uint8_t)value);
            }
            return MB_WRITE_OK;
        }
#if RS232_RUNTIME_PROTOCOL_SWITCH
        if (addr == REG_RS232_PROTO) {
            if (value != RS232_PROTO_MODBUS && value != RS232_PROTO_FREEPORT) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            /* Routing only, no motion impact; never feeds the comm watchdog.
               Switching USART1 away from Modbus simply leaves a pending frame
               to be drained by the new owner on the next poll. */
            if (g_reg_txn.active) {
                g_reg_txn.rs232_proto_shadow = (uint8_t)value;
                g_reg_txn.system_dirty = true;
            } else {
                apply_rs232_proto((uint8_t)value);
            }
            return MB_WRITE_OK;
        }
#else
        if (addr == REG_RS232_PROTO) {
            /* Runtime switching compiled out: report the fixed build mode. */
            if (value != rs232_proto_sel) return MB_EX_ILLEGAL_DATA_VALUE;
            return MB_WRITE_OK;
        }
#endif
        switch (addr) {
        case REG_HOME_DIRECTION:
            if (value > 1U) return MB_EX_ILLEGAL_DATA_VALUE;
            for (uint8_t i = 0; i < AXIS_APP_COUNT; i++) {
                if (AxisApp_IsRegistered(i) && Axis_IsBusy(get_axis_id(i)))
                    return MB_EX_ILLEGAL_DATA_VALUE;
            }
            if (g_reg_txn.active) {
                g_reg_txn.home_direction_shadow = value;
                g_reg_txn.system_dirty = true;
            } else {
                home_direction_positive = value;
            }
            return MB_WRITE_OK;
        case 202: {
            uint32_t old_freq = g_reg_txn.active
                              ? g_reg_txn.home_fast_freq_shadow
                              : home_fast_freq;
            uint32_t freq = (old_freq & 0xFFFF0000U) | value;
            if (!home_speed_valid(freq)) return MB_EX_ILLEGAL_DATA_VALUE;
            if (g_reg_txn.active) {
                g_reg_txn.home_fast_freq_shadow = freq;
                g_reg_txn.system_dirty = true;
            } else {
                home_fast_freq = freq;
            }
            return MB_WRITE_OK;
        }
        case 203: {
            uint32_t old_freq = g_reg_txn.active
                              ? g_reg_txn.home_slow_freq_shadow
                              : home_slow_freq;
            uint32_t freq = (old_freq & 0xFFFF0000U) | value;
            if (!home_speed_valid(freq)) return MB_EX_ILLEGAL_DATA_VALUE;
            if (g_reg_txn.active) {
                g_reg_txn.home_slow_freq_shadow = freq;
                g_reg_txn.system_dirty = true;
            } else {
                home_slow_freq = freq;
            }
            return MB_WRITE_OK;
        }
        case 204:
            if (value == 0U) return MB_EX_ILLEGAL_DATA_VALUE;
            if (g_reg_txn.active) {
                g_reg_txn.home_backoff_shadow = value;
                g_reg_txn.system_dirty = true;
            } else {
                home_backoff = value;
            }
            return MB_WRITE_OK;
        case 208:
            if (g_reg_txn.active) {
                g_reg_txn.subdivision_shadow = value;
                g_reg_txn.system_dirty = true;
            } else {
                subdivision = value;
            }
            return MB_WRITE_OK;
        case 209:
            if (g_reg_txn.active) {
                g_reg_txn.jog_step_shadow = value;
                g_reg_txn.system_dirty = true;
            } else {
                jog_step = value;
            }
            return MB_WRITE_OK;
        case 205:
            value = (value == AXIS_HOME_MODE_TRIGGER)
                  ? AXIS_HOME_MODE_TRIGGER
                  : AXIS_HOME_MODE_PRECISION;
            if (g_reg_txn.active) {
                g_reg_txn.home_mode_shadow = value;
                g_reg_txn.system_dirty = true;
            } else {
                home_mode = value;
            }
            return MB_WRITE_OK;
        case 206:
            if (value != 0x00FF) return MB_EX_ILLEGAL_DATA_VALUE;
            if (g_reg_txn.active) {
                g_reg_txn.stop_all_request = true;
                g_reg_txn.system_dirty = true;
            } else {
                for (uint8_t i = 0; i < AXIS_APP_COUNT; i++) {
                    if (AxisApp_IsRegistered(i)) (void)Axis_Abort(get_axis_id(i));
                }
            }
            return MB_WRITE_OK;
        case 207:
            if (value < 1 || value > 247) return MB_EX_ILLEGAL_DATA_VALUE;
            if (g_reg_txn.active) {
                g_reg_txn.mb_slave_addr_shadow = (uint8_t)value;
                g_reg_txn.system_dirty = true;
            }
#if RS485_ENABLE
            else if (mb_active == &mb_slave485) {
                pending_slave_addr_485 = (uint8_t)value;
            }
#endif
            else {
                pending_slave_addr = (uint8_t)value;
            }
            return MB_WRITE_OK;
        case 210:
            if (g_reg_txn.active) {
                g_reg_txn.comm_wdt_timeout_ms_shadow = value;
                g_reg_txn.system_dirty = true;
            } else {
                comm_wdt_timeout_ms = value;
                comm_wdt_counter = 0;
                comm_wdt_triggered = 0;
            }
            return MB_WRITE_OK;
        case 214:
            if (g_reg_txn.active) {
                g_reg_txn.feed_wdt_request = true;
                g_reg_txn.system_dirty = true;
            } else {
                comm_wdt_feed();
            }
            return MB_WRITE_OK;
        case REG_SAVE_POINTS:
            if (value != 0xA55AU || g_reg_txn.active) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            return Axis_SavePointPositions() ? MB_WRITE_OK : MB_EX_SLAVE_DEVICE_FAILURE;
        case REG_HOME_FAST_HI: {
            uint32_t old_freq = g_reg_txn.active
                              ? g_reg_txn.home_fast_freq_shadow
                              : home_fast_freq;
            uint32_t freq = ((uint32_t)value << 16) | (old_freq & 0xFFFFU);
            if (value > (PWM_FREQ_MAX >> 16)) return MB_EX_ILLEGAL_DATA_VALUE;
            if (g_reg_txn.active) {
                g_reg_txn.home_fast_freq_shadow = freq;
                g_reg_txn.system_dirty = true;
            } else {
                home_fast_freq = freq;
            }
            return MB_WRITE_OK;
        }
        case REG_HOME_SLOW_HI: {
            uint32_t old_freq = g_reg_txn.active
                              ? g_reg_txn.home_slow_freq_shadow
                              : home_slow_freq;
            uint32_t freq = ((uint32_t)value << 16) | (old_freq & 0xFFFFU);
            if (value > (PWM_FREQ_MAX >> 16)) return MB_EX_ILLEGAL_DATA_VALUE;
            if (g_reg_txn.active) {
                g_reg_txn.home_slow_freq_shadow = freq;
                g_reg_txn.system_dirty = true;
            } else {
                home_slow_freq = freq;
            }
            return MB_WRITE_OK;
        }
        case REG_HOME_BACKOFF_HZ:
            if (!home_speed_valid(value)) return MB_EX_ILLEGAL_DATA_VALUE;
            if (g_reg_txn.active) {
                g_reg_txn.home_backoff_freq_shadow = value;
                g_reg_txn.system_dirty = true;
            } else {
                home_backoff_freq = value;
            }
            return MB_WRITE_OK;
        default: return MB_EX_ILLEGAL_DATA_VALUE;
        }
    }

    /* --- 定位点 400-527: 可读写 --- */
    if (addr >= POINTS_REG_BASE && addr < (uint16_t)(POINTS_REG_BASE + AXIS_APP_COUNT * POINTS_REG_STRIDE)) {
        uint16_t rel = (uint16_t)(addr - POINTS_REG_BASE);
        uint8_t axis_idx = (uint8_t)(rel / POINTS_REG_STRIDE);
        uint8_t point_off = (uint8_t)(rel % POINTS_REG_STRIDE);
        uint8_t point_idx = point_off / 2U;
        bool    is_hi    = (point_off % 2U) != 0;

        if (op_mode == MODE_NORMAL) return MB_EX_ILLEGAL_DATA_VALUE;
        if (axis_idx >= AXIS_APP_COUNT || point_idx >= 8U) return MB_EX_ILLEGAL_DATA_ADDRESS;

        int32_t old_pos = g_reg_txn.active ? g_reg_txn.point_shadow[axis_idx][point_idx]
                                           : NVRAM_GetPositionPoint(axis_idx, point_idx);
        uint32_t u = (uint32_t)old_pos;

        if (is_hi)
            u = (u & 0x0000FFFFU) | ((uint32_t)value << 16);
        else
            u = (u & 0xFFFF0000U) | (value & 0xFFFFU);

        if (g_reg_txn.active) {
            g_reg_txn.point_shadow[axis_idx][point_idx] = (int32_t)u;
            g_reg_txn.points_dirty = true;
        } else {
            NVRAM_SetPositionPoint(axis_idx, point_idx, (int32_t)u);
        }
        return MB_WRITE_OK;
    }

    /* --- 点位名称 600-1367: 可读写 --- */
    if (addr >= POINT_NAME_REG_BASE &&
        addr < (uint16_t)(POINT_NAME_REG_BASE + AXIS_APP_COUNT * POINT_NAME_REG_STRIDE)) {
        uint16_t rel = (uint16_t)(addr - POINT_NAME_REG_BASE);
        uint8_t axis_idx = (uint8_t)(rel / POINT_NAME_REG_STRIDE);
        uint8_t axis_off = (uint8_t)(rel % POINT_NAME_REG_STRIDE);
        uint8_t point_idx = axis_off / POINT_NAME_LEN;
        uint8_t char_idx = axis_off % POINT_NAME_LEN;

        if (op_mode == MODE_NORMAL) return MB_EX_ILLEGAL_DATA_VALUE;
        if (axis_idx >= AXIS_APP_COUNT || point_idx >= NVRAM_POINTS_PER_AXIS) return MB_EX_ILLEGAL_DATA_ADDRESS;
        if (g_reg_txn.active) {
            g_reg_txn.point_name_shadow[axis_idx][point_idx][char_idx] = value;
            g_reg_txn.point_names_dirty = true;
        } else {
            NVRAM_SetPositionPointNameUnit(axis_idx, point_idx, char_idx, value);
        }
        return MB_WRITE_OK;
    }

    /* --- 点位速度 1600-1727: 可读写, 掉电保持, 仅调试模式 --- */
    if (addr >= POINT_SPEED_REG_BASE &&
        addr < (uint16_t)(POINT_SPEED_REG_BASE + AXIS_APP_COUNT * POINT_SPEED_REG_STRIDE)) {
        uint16_t rel = (uint16_t)(addr - POINT_SPEED_REG_BASE);
        uint8_t axis_idx = (uint8_t)(rel / POINT_SPEED_REG_STRIDE);
        uint8_t point_off = (uint8_t)(rel % POINT_SPEED_REG_STRIDE);
        uint8_t point_idx = point_off / 2U;
        bool    is_hi    = (point_off % 2U) != 0;
        uint32_t uspeed;

        if (op_mode == MODE_NORMAL) return MB_EX_ILLEGAL_DATA_VALUE;
        if (axis_idx >= AXIS_APP_COUNT || point_idx >= NVRAM_POINTS_PER_AXIS)
            return MB_EX_ILLEGAL_DATA_ADDRESS;

        uspeed = g_reg_txn.active ? g_reg_txn.point_speed_shadow[axis_idx][point_idx]
                                  : NVRAM_GetPositionPointSpeed(axis_idx, point_idx);

        if (is_hi)
            uspeed = (uspeed & 0x0000FFFFU) | ((uint32_t)value << 16);
        else
            uspeed = (uspeed & 0xFFFF0000U) | (uint32_t)value;

        /* 速度必须落在驱动能力范围内, 否则拒绝整次写入。 */
        if (uspeed < PWM_FREQ_MIN || uspeed > PWM_FREQ_MAX ||
            uspeed > SOFT_PWM_FREQ_MAX)
            return MB_EX_ILLEGAL_DATA_VALUE;

        if (g_reg_txn.active) {
            g_reg_txn.point_speed_shadow[axis_idx][point_idx] = uspeed;
            g_reg_txn.point_speeds_dirty = true;
        } else {
            NVRAM_SetPositionPointSpeed(axis_idx, point_idx, uspeed);
        }
        return MB_WRITE_OK;
    }

    /* --- 软限位 1500-1563: 可读写, 掉电保持 --- */
    if (addr >= SOFT_LIMIT_REG_BASE &&
        addr < (uint16_t)(SOFT_LIMIT_REG_BASE + AXIS_APP_COUNT * SOFT_LIMIT_REG_STRIDE)) {
        uint16_t rel = (uint16_t)(addr - SOFT_LIMIT_REG_BASE);
        uint8_t idx = (uint8_t)(rel / SOFT_LIMIT_REG_STRIDE);
        uint8_t off = (uint8_t)(rel % SOFT_LIMIT_REG_STRIDE);

        if (idx >= AXIS_APP_COUNT || !AxisApp_IsRegistered(idx))
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        if (op_mode == MODE_NORMAL) return MB_EX_ILLEGAL_DATA_VALUE;
        if (Axis_IsBusy(get_axis_id(idx))) return MB_EX_ILLEGAL_DATA_VALUE;

        bool en;
        int32_t min_pos, max_pos;
        (void)AxisApp_GetSoftLimit(idx, &en, &min_pos, &max_pos);
        if (g_reg_txn.active) {
            en = g_reg_txn.soft_limit_enable_shadow[idx];
            min_pos = g_reg_txn.soft_limit_min_shadow[idx];
            max_pos = g_reg_txn.soft_limit_max_shadow[idx];
        }

        switch (off) {
        case 0:
            en = (value != 0U);
            break;
        case 1:
            min_pos = (int32_t)(((uint32_t)min_pos & 0xFFFF0000U) | (value & 0xFFFFU));
            break;
        case 2:
            min_pos = (int32_t)(((uint32_t)value << 16) | ((uint32_t)min_pos & 0xFFFFU));
            break;
        case 3:
            max_pos = (int32_t)(((uint32_t)max_pos & 0xFFFF0000U) | (value & 0xFFFFU));
            break;
        case 4:
            max_pos = (int32_t)(((uint32_t)value << 16) | ((uint32_t)max_pos & 0xFFFFU));
            break;
        case 5:
            return MB_WRITE_OK;
        case 6:
            return MB_WRITE_OK;
        case 7:
            if (value != SOFT_LIMIT_CMD_SAVE) return MB_EX_ILLEGAL_DATA_VALUE;
            if (g_reg_txn.active) {
                g_reg_txn.soft_limit_save_request = true;
            } else {
                return NVRAM_Auto_Save() ? MB_WRITE_OK : MB_EX_SLAVE_DEVICE_FAILURE;
            }
            return MB_WRITE_OK;
        default:
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }

        if (en && min_pos >= max_pos) return MB_EX_ILLEGAL_DATA_VALUE;

        if (g_reg_txn.active) {
            g_reg_txn.soft_limit_enable_shadow[idx] = en;
            g_reg_txn.soft_limit_min_shadow[idx] = min_pos;
            g_reg_txn.soft_limit_max_shadow[idx] = max_pos;
            g_reg_txn.soft_limit_dirty = true;
        } else {
            if (!AxisApp_SetSoftLimit(idx, en, min_pos, max_pos))
                return MB_EX_ILLEGAL_DATA_VALUE;
        }
        return MB_WRITE_OK;
    }

    return MB_EX_ILLEGAL_DATA_ADDRESS;
}

/* ====================================================================
 *  Modbus 回调: 线圈 (FC01/FC05) → Y 输出
 * ==================================================================== */
static uint8_t mb_read_coil(uint16_t addr, uint8_t *value)
{
    if (value == NULL) return MB_EX_SLAVE_DEVICE_FAILURE;
    if (addr >= NUM_Y_OUTPUTS) return MB_EX_ILLEGAL_DATA_ADDRESS;
    *value = Y_State[addr];
    return MB_CB_OK;
}

static uint8_t mb_write_coil(uint16_t addr, uint8_t state)
{
    if (addr >= NUM_Y_OUTPUTS) return MB_EX_ILLEGAL_DATA_ADDRESS;
    if (op_mode == MODE_NORMAL) return MB_EX_ILLEGAL_DATA_VALUE;
    /* 调试模式可测试轴复用输出，但运动期间仍由轴独占。 */
    if (AxisApp_IsOutputReserved(addr) && AxisApp_IsOutputBusy(addr)) {
        return MB_EX_ILLEGAL_DATA_VALUE;
    }
    y_funcs[addr](state);
    return MB_WRITE_OK;
}

/* ====================================================================
 *  Modbus 回调: 离散输入 (FC02) → X 输入
 * ==================================================================== */
static uint8_t mb_read_di(uint16_t addr, uint8_t *value)
{
    if (value == NULL) return MB_EX_SLAVE_DEVICE_FAILURE;
    if (addr >= NUM_X_INPUTS) return MB_EX_ILLEGAL_DATA_ADDRESS;
    *value = X_State[addr];
    return MB_CB_OK;
}

/* ====================================================================
 *  UART 发送回调
 * ==================================================================== */
static void mb_uart_send(uint8_t *data, uint16_t len)
{
    if (BSP_UART_Send(data, len)) {
        /* 站号写入必须先用旧地址响应，发送完成后再接受新地址。 */
        if (pending_slave_addr != 0U) {
            mb_slave.slave_addr = pending_slave_addr;
        }
    } else {
        comm_error_count++;
    }
    pending_slave_addr = 0U;
}

/* ====================================================================
 *  公共接口: 初始化
 * ==================================================================== */
#if RS485_ENABLE
/* Independent USART3 / RS485 transmit callback. */
static void mb485_uart_send(uint8_t *data, uint16_t len)
{
    if (BSP_UART485_Send(data, len)) {
        if (pending_slave_addr_485 != 0U) {
            mb_slave485.slave_addr = pending_slave_addr_485;
        }
    } else {
        comm_error_count_485++;
    }
    pending_slave_addr_485 = 0U;
}
#endif

void Debug_Service_Init(void)
{
    if (configured_slave_addr < 1U || configured_slave_addr > 247U) {
        configured_slave_addr = 1U;
    }

    /* 寄存器 220 的值来自 NVRAM，上电必须校验，避免掉电半写或旧固件残留值
       把 USART1 路由到不存在的协议分支。 */
    if (rs232_proto_sel != RS232_PROTO_MODBUS &&
        rs232_proto_sel != RS232_PROTO_FREEPORT) {
        rs232_proto_sel = (RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT)
                        ? RS232_PROTO_FREEPORT : RS232_PROTO_MODBUS;
    }

    Modbus_Init(&mb_slave, configured_slave_addr);
    mb_slave.uart_send             = mb_uart_send;
    mb_slave.read_holding_register = mb_read_hreg;
    mb_slave.write_holding_register = mb_write_hreg;
    mb_slave.begin_write_holding_registers = txn_begin;
    mb_slave.end_write_holding_registers = txn_finish;
    mb_slave.read_coil             = mb_read_coil;
    mb_slave.write_coil            = mb_write_coil;
    mb_slave.read_discrete_input   = mb_read_di;
}

#if RS485_ENABLE
void Debug_Service_Init485(void)
{
    if (configured_slave_addr_485 < 1U || configured_slave_addr_485 > 247U) {
        configured_slave_addr_485 = RS485_SLAVE_ADDR;
    }

    Modbus_Init(&mb_slave485, configured_slave_addr_485);
    mb_slave485.uart_send             = mb485_uart_send;
    mb_slave485.read_holding_register = mb_read_hreg;
    mb_slave485.write_holding_register = mb_write_hreg;
    mb_slave485.begin_write_holding_registers = txn_begin;
    mb_slave485.end_write_holding_registers = txn_finish;
    mb_slave485.read_coil             = mb_read_coil;
    mb_slave485.write_coil            = mb_write_coil;
    mb_slave485.read_discrete_input   = mb_read_di;
}
#endif

/* ====================================================================
 *  公共接口: 通信看门狗 1ms 周期 (由 Tick_Hook 调用)
 * ==================================================================== */
void Debug_Service_Tick1ms(void)
{
    comm_wdt_tick();
}

void Debug_Service_Poll(void)
{
    if (!BSP_UART_FrameReady()) return;

    uint8_t *raw;
    uint16_t rlen;
    BSP_UART_GetFrame(&raw, &rlen);

    /* 正式 Modbus 模式:
     * - 仅处理发给本机地址的帧
     * - CRC 正确才计为有效帧并喂通信看门狗
     */
    if (rlen >= 4 && raw[0] == mb_slave.slave_addr) {
        uint16_t calc_crc = Modbus_CRC16(raw, rlen - 2);
        uint16_t recv_crc = ((uint16_t)raw[rlen - 1] << 8) | raw[rlen - 2];

        if (calc_crc == recv_crc) {
            Modbus_Slave_t *saved = mb_active;
            mb_active = &mb_slave;
            Modbus_ProcessFrame(&mb_slave, raw, rlen);
            mb_active = saved;
            comm_frame_count++;
            /* 本帧可能刚把 220 切到自由口: 232 从此不再喂狗, 此时不能再武装
               看门狗, 否则超时会误判断线并急停所有轴。 */
            if (!Debug_Service_Rs232IsFreePort()) {
                comm_wdt_feed();
            }
        } else {
            comm_error_count++;
        }
    }

    BSP_UART_ReleaseFrame();
}

#if RS485_ENABLE
void Debug_Service_Poll485(void)
{
    if (!BSP_UART485_FrameReady()) return;

    uint8_t *raw;
    uint16_t rlen;
    BSP_UART485_GetFrame(&raw, &rlen);

    if (rlen >= 4 && raw[0] == mb_slave485.slave_addr) {
        uint16_t calc_crc = Modbus_CRC16(raw, rlen - 2);
        uint16_t recv_crc = ((uint16_t)raw[rlen - 1] << 8) | raw[rlen - 2];

        if (calc_crc == recv_crc) {
            Modbus_Slave_t *saved = mb_active;
            mb_active = &mb_slave485;
            Modbus_ProcessFrame(&mb_slave485, raw, rlen);
            mb_active = saved;
            comm_frame_count_485++;
            /* 485 保持历史语义: 不自动喂通信看门狗, 需要保护时上位机每帧写 214。
               这样单独用 485 的老工程升级后不会因心跳间隔而误急停。 */
        } else {
            comm_error_count_485++;
        }
    }

    BSP_UART485_ReleaseFrame();
}
#endif

void Debug_Service_ToggleMode(void)
{
    set_op_mode((op_mode == MODE_NORMAL) ? MODE_DEBUG : MODE_NORMAL);
}

bool Debug_Service_IsDebugMode(void)
{
    return op_mode == MODE_DEBUG;
}
