#include "plc_timer.h"

#include <limits.h>
#include <stdio.h>

static uint32_t fake_tick;
static unsigned int failures;

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

#define CHECK(expression)                                                       \
    do                                                                          \
    {                                                                           \
        if (!(expression))                                                      \
        {                                                                       \
            printf("FAIL line %d: %s\n", __LINE__, #expression);               \
            failures++;                                                         \
        }                                                                       \
    } while (0)

static void test_normal_delay_and_reset(void)
{
    TON_Timer_t timer = TON_TIMER_INIT;

    CHECK(!TON_UpdateAt(&timer, true, 1000U, 100U));
    CHECK(timer.IN && timer.PT == 1000U && !timer.Q && timer.ET == 0U);
    CHECK(!TON_UpdateAt(&timer, true, 1000U, 1099U));
    CHECK(!timer.Q && timer.ET == 999U);
    CHECK(TON_UpdateAt(&timer, true, 1000U, 1100U));
    CHECK(timer.Q && timer.ET == 1000U);

    CHECK(!TON_UpdateAt(&timer, false, 1000U, 1101U));
    CHECK(!timer.IN && !timer.Q && timer.ET == 0U);
    CHECK(!TON_UpdateAt(&timer, true, 1000U, 1200U));
    CHECK(!timer.Q && timer.ET == 0U);

    TON_Reset(&timer);
    CHECK(!timer.IN && timer.PT == 0U && !timer.Q && timer.ET == 0U);
}

static void test_zero_and_dynamic_preset(void)
{
    TON_Timer_t timer;

    TON_Init(&timer);
    CHECK(TON_UpdateAt(&timer, true, 0U, 10U));
    CHECK(timer.Q && timer.ET == 0U);

    CHECK(!TON_UpdateAt(&timer, false, 0U, 11U));
    CHECK(!TON_UpdateAt(&timer, true, 1000U, 100U));
    CHECK(!TON_UpdateAt(&timer, true, 1000U, 700U));
    CHECK(timer.ET == 600U);
    CHECK(TON_UpdateAt(&timer, true, 500U, 700U));
    CHECK(timer.Q && timer.ET == 500U);
    CHECK(!TON_UpdateAt(&timer, true, 1000U, 700U));
    CHECK(!timer.Q && timer.ET == 600U);
}

static void test_tick_rollover_and_saturation(void)
{
    TON_Timer_t timer = TON_TIMER_INIT;

    CHECK(!TON_UpdateAt(&timer, true, 50U, UINT32_MAX - 15U));
    CHECK(!TON_UpdateAt(&timer, true, 50U, 32U));
    CHECK(timer.ET == 48U);
    CHECK(TON_UpdateAt(&timer, true, 50U, 34U));

    TON_Init(&timer);
    CHECK(!TON_UpdateAt(&timer, true, UINT32_MAX, 0U));
    CHECK(!TON_UpdateAt(&timer, true, UINT32_MAX, UINT32_MAX - 5U));
    CHECK(TON_UpdateAt(&timer, true, UINT32_MAX, 10U));
    CHECK(timer.Q && timer.ET == UINT32_MAX);
}

static void test_hal_wrapper_and_null_pointer(void)
{
    TON_Timer_t timer = TON_TIMER_INIT;

    fake_tick = 1000U;
    CHECK(!TON(&timer, true, 25U));
    fake_tick = 1025U;
    CHECK(TON_Update(&timer, true, 25U));

    TON_Init(NULL);
    TON_Reset(NULL);
    CHECK(!TON(NULL, true, 1U));
    CHECK(!TON_UpdateAt(NULL, true, 1U, 1U));
}

int main(void)
{
    test_normal_delay_and_reset();
    test_zero_and_dynamic_preset();
    test_tick_rollover_and_saturation();
    test_hal_wrapper_and_null_pointer();

    if (failures != 0U)
    {
        printf("TON tests failed: %u\n", failures);
        return 1;
    }

    printf("TON tests passed\n");
    return 0;
}
