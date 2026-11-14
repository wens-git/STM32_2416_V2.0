#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "main.h"
#include <stdio.h>
#include <stdbool.h>

/**
 * @file bsp_uart.h
 * @brief 工业级串口驱动 (DMA + IDLE 空闲中断帧检测)
 * @details 利用 STM32 IDLE 线路空闲中断自动检测帧结束，
 *          配合 DMA 接收实现零 CPU 数据搬运，适用于 Modbus RTU 等协议。
 */

#define UART_RX_BUF_SIZE  256
#define UART_TX_BUF_SIZE  256

/* 全局串口1句柄，外部需要调用时可引用 */
extern UART_HandleTypeDef huart1;

/**
 * @brief 板级串口初始化 (含 DMA + IDLE 中断配置)
 */
void BSP_UART_Init(void);

/**
 * @brief 串口发送数据 (DMA 非阻塞, 带忙等待保护)
 * @param data 数据指针
 * @param len  数据长度
 */
bool BSP_UART_Send(const uint8_t *data, uint16_t len);

/**
 * @brief 检查是否收到完整帧 (由 IDLE 中断置位)
 * @return true=有新帧待处理
 */
bool BSP_UART_FrameReady(void);

/**
 * @brief 获取接收帧数据
 * @param[out] buf  接收缓冲区指针
 * @param[out] len  帧长度
 */
void BSP_UART_GetFrame(uint8_t **buf, uint16_t *len);

/**
 * @brief 帧处理完成后调用, 重新启动 DMA 接收
 */
void BSP_UART_ReleaseFrame(void);

/**
 * @brief 获取 UART 接收层累计丢弃的完整帧数
 * @return 丢帧计数
 */
uint16_t BSP_UART_GetDroppedFrames(void);

/**
 * @brief USART1 IDLE 中断处理函数 (在 USART1_IRQHandler 中调用)
 */
void BSP_UART_IDLE_Handler(void);

/**
 * @brief 轮询方式发送链路测试串 (不依赖 HAL gState, 与 RX DMA 可并存)
 * @note  当 USART1 正在 DMA 发送 (DMAT=1) 时本调用直接返回; 由 UART_LINK_TEST_ENABLE 控制是否真正发数据
 */
void BSP_UART_LinkTest_Ping(void);

#endif /* __BSP_UART_H__ */
