/**
 * @file plc_timer.c
 * @brief IEC 61131-3 style non-retentive on-delay timer (TON).
 */

#include "plc_timer.h"

#include "stm32f4xx_hal.h"

#include <limits.h>
#include <stddef.h>

void TON_Init(TON_Timer_t *timer)
{
    TON_Reset(timer);
}

void TON_Reset(TON_Timer_t *timer)
{
    if (timer == NULL)
    {
        return;
    }

    timer->IN = false;
    timer->PT = 0U;
    timer->Q = false;
    timer->ET = 0U;
    timer->_last_ms = 0U;
    timer->_elapsed_ms = 0U;
    timer->_running = false;
}

bool TON(TON_Timer_t *timer, bool input, uint32_t preset_ms)
{
    return TON_UpdateAt(timer, input, preset_ms, HAL_GetTick());
}

bool TON_UpdateAt(TON_Timer_t *timer,
                  bool input,
                  uint32_t preset_ms,
                  uint32_t now_ms)
{
    uint32_t delta_ms;

    if (timer == NULL)
    {
        return false;
    }

    timer->IN = input;
    timer->PT = preset_ms;

    if (!input)
    {
        timer->Q = false;
        timer->ET = 0U;
        timer->_elapsed_ms = 0U;
        timer->_last_ms = now_ms;
        timer->_running = false;
        return false;
    }

    if (!timer->_running)
    {
        timer->_last_ms = now_ms;
        timer->_elapsed_ms = 0U;
        timer->_running = true;
    }
    else
    {
        delta_ms = now_ms - timer->_last_ms;
        timer->_last_ms = now_ms;

        if (delta_ms > (UINT32_MAX - timer->_elapsed_ms))
        {
            timer->_elapsed_ms = UINT32_MAX;
        }
        else
        {
            timer->_elapsed_ms += delta_ms;
        }
    }

    if (timer->_elapsed_ms >= preset_ms)
    {
        timer->ET = preset_ms;
        timer->Q = true;
    }
    else
    {
        timer->ET = timer->_elapsed_ms;
        timer->Q = false;
    }

    return timer->Q;
}
