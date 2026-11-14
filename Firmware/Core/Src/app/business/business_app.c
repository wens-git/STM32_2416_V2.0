/**
 * @file    business_app.c
 * @brief   正常业务逻辑骨架。
 * @details 当前为量产业务扩展预留点。
 *          - init 中注册轴硬件绑定
 *          - tick1ms 预留给高频轻量检测
 *          - poll 预留给主业务状态机
 *
 *          轴操作 API 见 motor_axis.h:
 *            单轴: Axis_Jog / MoveRel / MoveAbs / Home / Stop / MoveToPoint
 *            状态: Axis_GetState / IsBusy / IsHomed / GetPosition
 *            全局: Axis_AllHomed / Axis_StopAll / Axis_AbortAll
 *            定位点: Axis_GetPointPosition / Axis_GetPointSpeed / SavePointPositions
 *
 *          本模块不产生轴报警: 限位只服务回原, 行程保护使用可选软限位。
 *          定位点坐标和速度均由上位机写入并掉电保持,
 *          Axis_MoveToPoint(Axis_ID[n], point) 直接使用该点位的速度运行。
 */
#include "app/business/business_app.h"

/* C 标准类型和常量。 */
#include <stdbool.h>
#include <stdint.h>

/* 应用层。 */
#include "app/app.h"
#include "app/axis/axis_manager.h"
#include "app/debug/motor_debug_service.h"
#include "app_config.h"

/* 硬件抽象层和应用服务。 */
#include "IODEF.h"
#include "bsp_iwdg.h"
#include "bsp_uart.h"
#include "fsm.h"
#include "input.h"
#include "modbus_rtu.h"
#include "motor_axis.h"
#include "nvram.h"
#include "plc_timer.h"
#include "pwm_motor.h"
#include "scheduler.h"
#include "soft_pwm.h"
#include "timer.h"

/* STM32 平台头文件。 */
#include "main.h"
#include "stm32f4xx_hal.h"

void axis_test(void) {}

void Business_App_Init(void) { AxisApp_Init(); }

void Business_App_Tick1ms(void) { /* 1ms 高频轻量任务预留 */}

/* ---- 回原参数 (业务层固化, 调试协议可另行覆盖) ---- */
#define HOME_FAST_HZ    5000U /* 快速寻找频率 */
#define HOME_BACKOFF_HZ  300U /* 触发后反向回退频率 */
#define HOME_SLOW_HZ     800U /* 慢速精确定位频率 */

__attribute__((unused))
static bool axis_done(uint8_t id) { return Axis_GetState(id) == AXIS_IDLE; }

/**
 * @brief  启动单轴回原, 完成返回 true。
 *         每次扫描: 未回原且空闲则发一次回原指令, 然后只等 Axis_IsHomed。
 *         方向、限位换向、碰限位反向搜索、回原失败自动回 IDLE 全在底层完成。
 * @note   预留业务函数: 在 Business_App_Poll() 中按轴号调用即可生效,
 *         例如 home_axis(0) 启动 0 号轴回原; 未调用时不影响任何现有行为。
 */
__attribute__((unused))
static bool home_axis(uint8_t axis_index) {
  uint8_t id = Axis_ID[axis_index];
  if (Axis_IsHomed(id)) return true;
  if (Axis_GetState(id) == AXIS_IDLE) {
    bool home_dir = false;
    (void)AxisApp_GetHomeDirPositive(axis_index, &home_dir);
    (void)Axis_HomeWithModeEx(id, home_dir, HOME_FAST_HZ, HOME_BACKOFF_HZ,
                              HOME_SLOW_HZ, AXIS_HOME_MODE_PRECISION);
  }
  return false;
}

void Business_App_Poll(void) { /* 主业务状态机预留: 需要自动回原时在此调用 home_axis(n) */ }
