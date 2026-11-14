/**
 * @file    motor_debug_service.h
 * @brief   电机调试服务接口。
 * @details 本模块封装所有与上位机调试相关的功能:
 *          - Modbus RTU 寄存器映射
 *          - 轴参数在线修改
 *          - JOG/定位/回原/急停调试命令
 *          - 通信看门狗与调试诊断计数
 *
 *          正常业务层只能通过本头文件调用调试服务入口，不能直接访问
 *          Modbus 回调、寄存器表或轴配置影子变量。
 */
#ifndef __MOTOR_DEBUG_SERVICE_H__
#define __MOTOR_DEBUG_SERVICE_H__

#include <stdbool.h>
#include "app_config.h"

void Debug_Service_Init(void);
void Debug_Service_Tick1ms(void);
void Debug_Service_Poll(void);
void Debug_Service_ToggleMode(void);

#if RS485_ENABLE
void Debug_Service_Init485(void);
void Debug_Service_Poll485(void);
#endif
bool Debug_Service_IsDebugMode(void);

/* USART1 (RS232) runtime protocol selector, driven by holding register 220.
 * true  = USART1 runs the free-port protocol (custom device protocol).
 * false = USART1 runs the built-in Modbus RTU slave (debug host tool).
 * Always available so App_Poll() can route USART1 at run time; only
 * register 220 becomes writable when RS232_RUNTIME_PROTOCOL_SWITCH = 1. */
bool Debug_Service_Rs232IsFreePort(void);

#endif /* __MOTOR_DEBUG_SERVICE_H__ */
