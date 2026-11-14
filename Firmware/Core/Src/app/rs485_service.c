/**
 * @file    rs485_service.c
 * @brief   RS485 application service.
 * @details USART3/DMA1/PB14 are owned exclusively by bsp_uart485.c. This file
 *          only selects Modbus RTU or free-port framing and exposes a send API.
 *          No USART1/DMA2 state is referenced here.
 */

#include "app/rs485_service.h"

#if RS485_ENABLE

#include "bsp_uart485.h"
#include "app/debug/motor_debug_service.h"
#include <stddef.h>

#if RS485_PROTOCOL_MODE != RS485_PROTO_MODBUS &&     RS485_PROTOCOL_MODE != RS485_PROTO_FREEPORT
#error "RS485_PROTOCOL_MODE must be RS485_PROTO_MODBUS or RS485_PROTO_FREEPORT"
#endif

#if RS485_PROTOCOL_MODE == RS485_PROTO_FREEPORT
__attribute__((weak)) void RS485_FreePort_OnFrame(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
}
#endif

void RS485_Service_Init(void)
{
#if RS485_PROTOCOL_MODE == RS485_PROTO_MODBUS
    Debug_Service_Init485();
#endif
}

void RS485_Service_Poll(void)
{
#if RS485_PROTOCOL_MODE == RS485_PROTO_MODBUS
    Debug_Service_Poll485();
#else
    if (!BSP_UART485_FrameReady()) return;

    uint8_t *frame;
    uint16_t len;
    BSP_UART485_GetFrame(&frame, &len);
    if (len > 0U) {
        RS485_FreePort_OnFrame(frame, len);
    }
    BSP_UART485_ReleaseFrame();
#endif
}

bool RS485_Service_Send(const uint8_t *data, uint16_t len)
{
    return BSP_UART485_Send(data, len);
}

#endif /* RS485_ENABLE */
