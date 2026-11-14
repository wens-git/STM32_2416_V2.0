#ifndef __BSP_UART485_H__
#define __BSP_UART485_H__

#include "main.h"
#include <stdbool.h>

/*
 * RS485 driver for the C2416MR board.
 *
 * USART3: PC10(TX) / PC11(RX), AF7
 * Direction: PB14 (DE and /RE tied together)
 *   PB14 = 1 -> transmit enabled
 *   PB14 = 0 -> receive enabled
 *
 * This driver is deliberately independent from bsp_uart.c (USART1/RS232):
 * separate UART, DMA streams, IRQ handlers, buffers and frame queue.
 */

#define UART485_RX_BUF_SIZE  256U
#define UART485_TX_BUF_SIZE  256U

extern UART_HandleTypeDef huart3;

/* Called by HAL_UART_MspInit() in bsp_uart.c for USART3 only. */
void BSP_UART485_MspInit(UART_HandleTypeDef *huart);

/* Initialise USART3, PB14 and DMA1. 115200-8N1 is the default. */
void BSP_UART485_Init(uint32_t baudrate);

/* Non-blocking DMA transmit. Returns false when busy or invalid. */
bool BSP_UART485_Send(const uint8_t *data, uint16_t len);
bool BSP_UART485_TxBusy(void);

/* Frame queue produced by the USART3 IDLE interrupt. */
bool BSP_UART485_FrameReady(void);
void BSP_UART485_GetFrame(uint8_t **buf, uint16_t *len);
void BSP_UART485_ReleaseFrame(void);
uint16_t BSP_UART485_GetDroppedFrames(void);

/* USART3 IDLE handling, called from USART3_IRQHandler(). */
void BSP_UART485_IDLE_Handler(void);

#endif /* __BSP_UART485_H__ */