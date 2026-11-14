/**
 * @file    error_code.c
 * @brief   统一错误/故障码体系 — 环形缓冲区实现
 * @details 使用静态分配的环形缓冲区存储故障记录，支持 ISR 中安全调用。
 *          通过 ATOMIC 临界区保护共享数据，不依赖动态内存分配。
 */

#include "error_code.h"
#include "app_config.h"     /* ATOMIC_ENTER / ATOMIC_EXIT */

/* ==========================================================================
 * 私有静态变量
 * ========================================================================== */

/** @brief 故障记录环形缓冲区 */
static ErrorEntry_t g_error_log[ERROR_LOG_SIZE];

/** @brief 环形缓冲区写入索引 (指向下一个可写位置) */
static volatile uint16_t g_write_idx = 0;

/** @brief 环形缓冲区当前有效条目数 (最大 = ERROR_LOG_SIZE) */
static volatile uint16_t g_entry_count = 0;

/** @brief 自启动/清除以来的故障总次数 (含已被覆盖的) */
static volatile uint16_t g_total_count = 0;

/* ==========================================================================
 * 公共接口实现
 * ========================================================================== */

void Error_Init(void)
{
    uint16_t i;
    for (i = 0; i < ERROR_LOG_SIZE; i++)
    {
        g_error_log[i].code = ERR_NONE;
        g_error_log[i].timestamp = 0;
    }
    g_write_idx = 0;
    g_entry_count = 0;
    g_total_count = 0;
}

void Error_Log(ErrorCode_t code)
{
    ATOMIC_ENTER();

    g_error_log[g_write_idx].code = code;
    g_error_log[g_write_idx].timestamp = HAL_GetTick();

    g_write_idx++;
    if (g_write_idx >= ERROR_LOG_SIZE)
    {
        g_write_idx = 0;  /* 环形回绕 */
    }

    if (g_entry_count < ERROR_LOG_SIZE)
    {
        g_entry_count++;
    }

    if (g_total_count < 0xFFFF)
    {
        g_total_count++;
    }

    ATOMIC_EXIT();
}

ErrorCode_t Error_GetLast(void)
{
    ErrorCode_t code;

    ATOMIC_ENTER();

    if (g_entry_count == 0)
    {
        code = ERR_NONE;
    }
    else
    {
        /* 最近一条在 write_idx - 1 位置 */
        uint16_t last = (g_write_idx == 0) ? (ERROR_LOG_SIZE - 1) : (g_write_idx - 1);
        code = g_error_log[last].code;
    }

    ATOMIC_EXIT();

    return code;
}

uint16_t Error_GetCount(void)
{
    uint16_t cnt;

    ATOMIC_ENTER();
    cnt = g_total_count;
    ATOMIC_EXIT();

    return cnt;
}

bool Error_GetEntry(uint16_t index, ErrorEntry_t *entry)
{
    if (entry == NULL)
        return false;

    ATOMIC_ENTER();

    if (index >= g_entry_count)
    {
        ATOMIC_EXIT();
        return false;
    }

    uint16_t start;
    if (g_entry_count < ERROR_LOG_SIZE)
    {
        start = 0;
    }
    else
    {
        start = g_write_idx;
    }

    uint16_t actual = (start + index) % ERROR_LOG_SIZE;
    *entry = g_error_log[actual];

    ATOMIC_EXIT();

    return true;
}

void Error_Clear(void)
{
    ATOMIC_ENTER();

    g_write_idx = 0;
    g_entry_count = 0;
    g_total_count = 0;

    ATOMIC_EXIT();
}

bool Error_HasFault(void)
{
    return (g_entry_count > 0);
}
