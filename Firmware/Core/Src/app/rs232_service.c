/**
 * @file    rs232_service.c
 * @brief   RS232 free-port service on USART1.
 * @details Compiled when RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT, or when
 *          RS232_RUNTIME_PROTOCOL_SWITCH == 1 (both protocol paths resident).
 *          USART1 / DMA2 / PA9 / PA10 are configured by bsp_uart.c; this file
 *          adds no hardware setup and no ISR. It only drains the existing
 *          IDLE frame queue and forwards each frame to the user callback.
 *
 *          BSP_UART_Send() is a bounded blocking call (worst case roughly
 *          120 ms: 100 ms waiting for gState + 20 ms waiting for TC). Do not
 *          call RS232_Service_Send() from a 1 ms context or a tight loop.
 */

#include "app/rs232_service.h"

#if (RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT) || RS232_RUNTIME_PROTOCOL_SWITCH

#include "bsp_uart.h"
#include <stddef.h>
#include <string.h>

/* Dedicated staging buffer: USART1 TX DMA reads from here, so the caller's
 * buffer may be released as soon as RS232_Service_Send() returns. */
static uint8_t rs232_tx_buf[UART_TX_BUF_SIZE];
static volatile bool rs232_tx_busy = false;

__attribute__((weak)) void RS232_FreePort_OnFrame(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
}

void RS232_Service_Poll(void)
{
    if (!BSP_UART_FrameReady()) return;

    uint8_t *frame;
    uint16_t len;
    BSP_UART_GetFrame(&frame, &len);
    if (len > 0U) {
        RS232_FreePort_OnFrame(frame, len);
    }
    BSP_UART_ReleaseFrame();
}

bool RS232_Service_Send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > UART_TX_BUF_SIZE) {
        return false;
    }

    memcpy(rs232_tx_buf, data, len);
    rs232_tx_busy = true;
    bool ok = BSP_UART_Send(rs232_tx_buf, len);
    rs232_tx_busy = false;
    return ok;
}

bool RS232_Service_TxBusy(void)
{
    return rs232_tx_busy;
}

#endif /* (RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT) || RS232_RUNTIME_PROTOCOL_SWITCH */