/**
 * @file    IODEF.h
 * @brief   硬件 I/O 抽象层头文件 — 定义工控板全部数字输入/输出端口映射
 * @details 本文件为整个工程的"引脚映射中心"，将 STM32 的 GPIO 物理引脚抽象为
 *          PLC 风格的编号命名 (X0~X27 输入, Y0~Y17 输出, LED_RUN, LED_ERR)，
 *          使应用层代码无需关心具体的 GPIO 端口和引脚号。
 *
 *          【命名规则】(兼容工控行业惯例，沿用三菱 PLC 八进制分组风格)
 *          - 输入: X0~X7 (第 0 组), X10~X17 (第 1 组), X20~X27 (第 2 组)
 *            共 24 路数字输入，均配有独立的积分式消抖实例 (In_Xn)
 *          - 输出: Y0~Y7 (第 0 组), Y10~Y17 (第 1 组)
 *            共 16 路数字输出，状态同步记录到 Y_State[] 数组
 *          - LED:  LED_RUN (运行指示灯), LED_ERR (故障指示灯)
 *
 *          【使用方式】
 *          - 输出控制:  Y0(1);  // 第 0 号输出置高
 *                       Y0(0);  // 第 0 号输出置低
 *          - 输入读取:  uint8_t val = X0();  // 读取 X0 归一化逻辑 (1=逻辑高=已触发, 0=未触发)
 *          - 消抖读取:  input_get(&In_X0);   // 获取消抖后的稳定状态 (true=按下)
 *          - 全局扫描:  IO_Scan();  // 更新全部 24 路消抖 + X_State[] 数组
 */

#ifndef __IODEF_H
#define __IODEF_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"   /* 包含 HAL GPIO 定义 */
#include "input.h"  /* 包含 Input_t 结构体及消抖 API */

/* ==========================================================================
 * 消抖输入实例声明 (共 24 个，与 IODEF.c 中的实体定义对应)
 * ==========================================================================
 * 每个 Input_t 实例绑定了一个物理输入读取函数 (如 X0)，并持有独立的
 * 消抖状态机数据。应用层可通过 input_get(&In_X0) 获取消抖后的稳定状态，
 * 或通过 input_check_down(&In_X0) 检测按下边沿事件。
 *
 * 注释栏留空，供用户按实际硬件接线填写每路输入的信号名称。
 */

/* ---- 第 0 组输入 (X0 ~ X7) ---- */
extern Input_t In_X0; /* X0  启动开关  — 对应 PA0  */
extern Input_t In_X1; /* X1  暂停开关 — 对应 PA1  */
extern Input_t In_X2; /* X2  急停开关  — 对应 PB15 */
extern Input_t In_X3; /* X3   — 对应 PA15 */
extern Input_t In_X4; /* X4   — 对应 PB3  */
extern Input_t In_X5; /* X5   — 对应 PC4  */
extern Input_t In_X6; /* X6   — 对应 PD6  */
extern Input_t In_X7; /* X7   — 对应 PD5  */

/* ---- 第 1 组输入 (X10 ~ X17) ---- */
extern Input_t In_X10; /* X10  — 对应 PD10 */
extern Input_t In_X11; /* X11  — 对应 PC8  */
extern Input_t In_X12; /* X12  — 对应 PC9  */
extern Input_t In_X13; /* X13  — 对应 PA12 */
extern Input_t In_X14; /* X14  — 对应 PA11 */
extern Input_t In_X15; /* X15   — 对应 PD15 */
extern Input_t In_X16; /* X16  — 对应 PD14 */
extern Input_t In_X17; /* X17  — 对应 PD13 */

/* ---- 第 2 组输入 (X20 ~ X27) ---- */
extern Input_t In_X20; /* X20  — 对应 PD12 */
extern Input_t In_X21; /* X21  — 对应 PD11 */
extern Input_t In_X22; /* X22  — 对应 PE15 */
extern Input_t In_X23; /* X23  — 对应 PE14 */
extern Input_t In_X24; /* X24  — 对应 PE13 */
extern Input_t In_X25; /* X25  — 对应 PE12 */
extern Input_t In_X26; /* X26  — 对应 PE11 */
extern Input_t In_X27; /* X27  — 对应 PE10 */

/* ==========================================================================
 * 初始化函数
 * ========================================================================== */

/**
 * @brief  初始化全部 I/O 端口 (GPIO 时钟使能 + 引脚配置 + 消抖实例绑定)
 * @note   必须在系统启动早期调用 (在调度器和其他模块之前)。
 */
void IO_Init(void);

/* ==========================================================================
 * 输出端口控制函数 (Y0 ~ Y17)
 * ==========================================================================
 * 参数 state: 1 = 输出高电平 (触发), 0 = 输出低电平 (未触发)
 * 每个函数同时更新 Y_State[] 数组中对应索引的值。
 */

/* ---- 第 0 组输出 (Y0 ~ Y7) ---- */
void Y0(uint8_t state); /* Y0  → PA6  */
void Y1(uint8_t state); /* Y1  → PC6  */
void Y2(uint8_t state); /* Y2  → PC7  */
void Y3(uint8_t state); /* Y3  → PA8  */
void Y4(uint8_t state); /* Y4  → PB7  */
void Y5(uint8_t state); /* Y5  → PB8  */
void Y6(uint8_t state); /* Y6  → PD7  */
void Y7(uint8_t state); /* Y7  → PD4  */

/* ---- 第 1 组输出 (Y10 ~ Y17) ---- */
void Y10(uint8_t state); /* Y10  — 对应 PC13 */
void Y11(uint8_t state); /* Y11  — 对应 PE2  */
void Y12(uint8_t state); /* Y12  — 对应 PE1  */
void Y13(uint8_t state); /* Y13  — 对应 PE0  */
void Y14(uint8_t state); /* Y14  — 对应 PB9  */
void Y15(uint8_t state); /* Y15  — 对应 PB6  */
void Y16(uint8_t state); /* Y16  — 对应 PB5  */
void Y17(uint8_t state); /* Y17  — 对应 PB4  */

/* ==========================================================================
 * LED 指示灯控制
 * ========================================================================== */
void LED_RUN(uint8_t state); /* 运行指示灯 → PB10 (1=亮, 0=灭) */
void LED_ERR(uint8_t state); /* 故障指示灯 → PB11 (1=亮, 0=灭) */

/* ==========================================================================
 * 输入端口读取函数 (X0 ~ X27)
 * ==========================================================================
 * 返回值: 1 = 逻辑高 = 已触发(外部端子低有效已动作), 0 = 未触发。
 * 2416 全部 Xn() 经 io_read_di_normalized 按 MCU 低有效取反；瞬时读值未经消抖，主要供 Input_t 回调。
 * 应用层通常不直接调用这些函数，而是使用 input_get(&In_Xn) 获取消抖后的状态。
 */

/* ---- 第 0 组输入 (X0 ~ X7) ---- */
uint8_t X0(void); /* X0  ← PA0  */
uint8_t X1(void); /* X1  ← PA1  */
uint8_t X2(void); /* X2  ← PB15 */
uint8_t X3(void); /* X3  ← PA15 */
uint8_t X4(void); /* X4  ← PB3  */
uint8_t X5(void); /* X5  ← PC4  */
uint8_t X6(void); /* X6  ← PD6  */
uint8_t X7(void); /* X7  ← PD5  */

/* ---- 第 1 组输入 (X10 ~ X17) ---- */
uint8_t X10(void); /* X10 ← PD10 */
uint8_t X11(void); /* X11 ← PC8  */
uint8_t X12(void); /* X12 ← PC9  */
uint8_t X13(void); /* X13 ← PA12 */
uint8_t X14(void); /* X14 ← PA11 */
uint8_t X15(void); /* X15 ← PD15 */
uint8_t X16(void); /* X16 ← PD14 */
uint8_t X17(void); /* X17 ← PD13 */

/* ---- 第 2 组输入 (X20 ~ X27) ---- */
uint8_t X20(void); /* X20 ← PD12 */
uint8_t X21(void); /* X21 ← PD11 */
uint8_t X22(void); /* X22 ← PE15 */
uint8_t X23(void); /* X23 ← PE14 */
uint8_t X24(void); /* X24 ← PE13 */
uint8_t X25(void); /* X25 ← PE12 */
uint8_t X26(void); /* X26 ← PE11 */
uint8_t X27(void); /* X27 ← PE10 */

/* ==========================================================================
 * 全局扫描与状态数组
 * ========================================================================== */

/**
 * @brief  一次性驱动全部 24 路输入的消抖状态机更新 + 刷新 X_State[] 数组
 * @note   由 Tick_Hook() 在每次硬件中断 (1ms) 中调用，确保消抖实时性。
 */
void IO_Scan(void);

/**
 * @brief 输出状态镜像数组 (16 路, Y_State[0]~Y_State[15] 对应 Y0~Y17)
 * @note  每次调用 Yn(state) 时同步更新。可供 Modbus 读回输出状态使用。
 */
extern uint8_t Y_State[16];

/**
 * @brief 输入状态镜像数组 (24 路, X_State[0]~X_State[23] 对应 X0~X27)
 * @note  由 IO_Scan() 在每次消抖更新后刷新。1 = 逻辑高=已触发, 0 = 未触发。
 */
extern uint8_t X_State[24];

#endif /* __IODEF_H */
