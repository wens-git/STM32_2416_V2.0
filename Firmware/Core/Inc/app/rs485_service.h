/**
 * @file    rs485_service.h
 * @brief   RS485 application service (independent from the USART1 RS232 path).
 * @details Protocol selection is compile-time:
 *          - RS485_PROTO_MODBUS  -> Modbus RTU slave on USART3, same register map
 *          - RS485_PROTO_FREEPORT -> raw frames delivered to RS485_FreePort_OnFrame()
 */

#ifndef __RS485_SERVICE_H__
#define __RS485_SERVICE_H__

#include <stdbool.h>
#include <stdint.h>
#include "app_config.h"

#if RS485_ENABLE

/* Bind the selected protocol to an already initialised USART3 driver. */
void RS485_Service_Init(void);

/* Poll received frames. Call once per application poll cycle. */
void RS485_Service_Poll(void);

/* Send a raw frame. Non-blocking; false when the driver is busy or invalid. */
bool RS485_Service_Send(const uint8_t *data, uint16_t len);

#if RS485_PROTOCOL_MODE == RS485_PROTO_FREEPORT
/*
 * Weak free-port receive callback. Define this function in application code
 * to consume a frame. The buffer is valid only during the callback.
 */
void RS485_FreePort_OnFrame(const uint8_t *data, uint16_t len);
#endif

#endif /* RS485_ENABLE */

#endif /* __RS485_SERVICE_H__ */
