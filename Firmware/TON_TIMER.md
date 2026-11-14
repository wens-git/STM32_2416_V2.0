# PLC TON Timer

`plc_timer` is a non-blocking, non-retentive IEC 61131-3 style on-delay timer.
It uses the STM32F407 HAL 1 ms tick and consumes no hardware timer.

## Typical use

Use one persistent instance for each independent timing condition:

```c
#include "plc_timer.h"
#include "IODEF.h"

static TON_Timer_t sensor_filter = TON_TIMER_INIT;

void Business_App_Poll(void)
{
    bool sensor_ok = TON(&sensor_filter, X1() != 0U, 500U);

    if (sensor_ok)
    {
        /* X1 has stayed true continuously for at least 500 ms. */
    }
}
```

Call `TON()` every application scan, including when the input is false. A false
input immediately resets `Q` and `ET`. The instance must be `static`, global,
or owned by another persistent object.

## Behavior

| IN | PT | Q | ET |
|---|---:|---|---:|
| false | any | false | 0 |
| true, elapsed < PT | > 0 | false | elapsed |
| true, elapsed >= PT | > 0 | true | PT |
| true | 0 | true | 0 |

`PT` is expressed in milliseconds and may change while timing. Unsigned delta
arithmetic handles the 32-bit `HAL_GetTick()` rollover. The output is observed
on the first scan at or after `PT`, so the application scan period is the
maximum additional output latency.

For normal PLC reset behavior, scan `TON(&timer, false, preset_ms)`. Use
`TON_Reset(&timer)` to forcibly clear the complete timer instance, including
`IN` and `PT`. Do not update one instance concurrently from an interrupt and
the main loop.
