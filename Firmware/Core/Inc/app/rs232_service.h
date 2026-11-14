/**
 * @file    rs232_service.h
 * @brief   RS232 free-port service (USART1), independent from RS485/USART3.
 * @details Protocol selection:
 *          - RS232_PROTO_MODBUS   -> USART1 stays a Modbus RTU slave; this
 *                                    module compiles to nothing (unless runtime
 *                                    switching is enabled).
 *          - RS232_PROTO_FREEPORT -> raw IDLE-delimited frames are delivered
 *                                    to RS232_FreePort_OnFrame().
 *          - RS232_RUNTIME_PROTOCOL_SWITCH = 1 keeps BOTH paths resident and
 *                                    selects at run time through holding
 *                                    register 220 (0 = Modbus, 1 = free port).
 *                                    RS232_PROTOCOL_MODE then only sets the
 *                                    power-up default for a virgin NVRAM.
 *
 *          USART1, DMA2 Stream5/7, PA9/PA10 and the USART1/DMA2 IRQs stay
 *          owned by bsp_uart.c and are never shared with the USART3 RS485 path.
 */

#ifndef __RS232_SERVICE_H__
#define __RS232_SERVICE_H__

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"

/* Compiled in either when USART1 is statically free-port, or when runtime
 * switching is enabled (both protocol paths stay resident). */
#if (RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT) || RS232_RUNTIME_PROTOCOL_SWITCH

/* Drain at most one received frame and dispatch it to the user callback. */
void RS232_Service_Poll(void);

/* Send one raw frame on USART1. Blocking, bounded (see .c notes). */
bool RS232_Service_Send(const uint8_t *data, uint16_t len);

/* True while a USART1 transmit is still in progress. */
bool RS232_Service_TxBusy(void);

/*
 * Weak receive callback. Define it in application code to consume a frame.
 * The buffer is valid only during the callback; copy it to keep the data.
 */
void RS232_FreePort_OnFrame(const uint8_t *data, uint16_t len);

#endif /* (RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT) || RS232_RUNTIME_PROTOCOL_SWITCH */

#endif /* __RS232_SERVICE_H__ */