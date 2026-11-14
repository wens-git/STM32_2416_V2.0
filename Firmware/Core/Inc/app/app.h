/**
 * @file    app.h
 * @brief   应用层总入口: 统一管理正常业务逻辑与调试服务。
 * @details 本文件是 Core/Src/main.c 与应用层之间的唯一公开接口。
 *          main.c 只负责芯片启动、外设初始化和调度器运行；
 *          业务运行、调试协议、通信看门狗等应用功能全部收敛到本层，
 *          避免启动代码与具体业务互相耦合。
 */
#ifndef __APP_H__
#define __APP_H__

#include <stdint.h>

/**
 * @brief 初始化应用层全部服务。
 * @note  必须在 BSP_UART_Init(), PWM_Motor_Init(), IO_Init() 之后调用。
 */
void App_Init(void);

/**
 * @brief 1ms 实时节拍入口。
 * @note  由 SysTick 调度钩子调用，只放入必须按 1ms 刷新的轻量逻辑。
 */
void App_Tick1ms(void);

/**
 * @brief 应用层轮询任务。
 * @note  由协作式调度器周期调用，用于处理串口帧、业务状态机等非中断逻辑。
 */
void App_Poll(void);

#endif /* __APP_H__ */
