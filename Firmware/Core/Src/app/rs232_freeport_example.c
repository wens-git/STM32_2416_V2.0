/**
 * @file    rs232_freeport_example.c
 * @brief   Reference RS232 free-port callback (USART1).
 * @details Active only when RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT and
 *          RS232_FREEPORT_EXAMPLE == 1.
 *
 *          This file exists so the free-port mode has a working, testable
 *          behaviour out of the box: every received frame is echoed back
 *          byte-for-byte. Open a serial terminal on the RS232 port at
 *          115200-8N1, send any text, and it comes back unchanged.
 *
 *          Replace the body with your own protocol. Set
 *          RS232_FREEPORT_EXAMPLE to 0 (or delete this file) once your own
 *          RS232_FreePort_OnFrame() is in place.
 *
 *          Framing: USART1 uses DMA + IDLE detection, so a frame ends when
 *          the line stays silent for roughly 1 character time. Modbus RTU
 *          style 3.5-character gaps are naturally handled.
 *
 *          Threading: this callback runs in the main poll loop, never in an
 *          ISR. RS232_Service_Send() is blocking (up to about 120 ms worst
 *          case), so avoid replying to very high-rate requests here.
 */

#include "app/rs232_service.h"

#if ((RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT) || RS232_RUNTIME_PROTOCOL_SWITCH) && RS232_FREEPORT_EXAMPLE

void RS232_FreePort_OnFrame(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }

    /* Echo example. Replace with your own request/response handling. */
    (void)RS232_Service_Send(data, len);
}

#endif /* free-port example guard */