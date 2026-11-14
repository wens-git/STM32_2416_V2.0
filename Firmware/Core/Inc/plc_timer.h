/**
 * @file plc_timer.h
 * @brief IEC 61131-3 style non-retentive on-delay timer (TON).
 *
 * Call TON() once in every application scan. The public fields use the same
 * meaning as a PLC TON function block:
 *   IN - enable input
 *   PT - preset time in milliseconds
 *   Q  - done output
 *   ET - elapsed time in milliseconds, saturated at PT
 */

#ifndef PLC_TIMER_H
#define PLC_TIMER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool IN;
    uint32_t PT;
    bool Q;
    uint32_t ET;

    /* Internal state. Application code must not modify these members. */
    uint32_t _last_ms;
    uint32_t _elapsed_ms;
    bool _running;
} TON_Timer_t;

/** Static initializer for global, static, or local TON instances. */
#define TON_TIMER_INIT { false, 0U, false, 0U, 0U, 0U, false }

/** Reset all inputs, outputs, and internal state. */
void TON_Init(TON_Timer_t *timer);

/**
 * Force the timer back to its initial state.
 *
 * IN, PT, Q, ET and all internal timing state are cleared. The next TON()
 * call with input == true starts a new timing cycle from zero.
 */
void TON_Reset(TON_Timer_t *timer);

/**
 * Execute one TON scan using the STM32 HAL 1 ms time base.
 *
 * When IN is false, Q and ET reset immediately. When IN remains true, ET
 * increases until PT and Q then becomes true. PT may change while running.
 * PT == 0 makes Q true immediately whenever IN is true.
 *
 * @return Current Q value. A null timer pointer returns false.
 */
bool TON(TON_Timer_t *timer, bool input, uint32_t preset_ms);

/** Descriptive alias for TON(). */
static inline bool TON_Update(TON_Timer_t *timer,
                              bool input,
                              uint32_t preset_ms)
{
    return TON(timer, input, preset_ms);
}

/**
 * Execute one TON scan with an explicit timestamp.
 *
 * This variant is useful for deterministic tests and for projects that use a
 * different 1 ms clock. Unsigned subtraction makes a uint32_t clock rollover
 * safe, provided this function is scanned at least once per clock wrap.
 */
bool TON_UpdateAt(TON_Timer_t *timer,
                  bool input,
                  uint32_t preset_ms,
                  uint32_t now_ms);

#endif /* PLC_TIMER_H */
