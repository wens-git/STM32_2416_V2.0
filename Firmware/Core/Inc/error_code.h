/**
 * @file    error_code.h
 * @brief   统一错误/故障码体系 — 工业级故障分类、记录与查询
 * @details 本模块提供:
 *          1) 标准化的故障码枚举 (ErrorCode_t)，覆盖时钟、外设、电机、通信等域
 *          2) 故障日志环形缓冲区，支持掉电前保存最近 N 条故障记录
 *          3) 查询接口，便于 Modbus 上位机远程读取故障历史
 *
 *          【使用方式】
 *          - 发生故障时:   Error_Log(ERR_MOTOR_LIMIT);
 *          - 查询最近故障: ErrorCode_t e = Error_GetLast();
 *          - 读取故障计数: uint16_t n = Error_GetCount();
 *          - 清除故障记录: Error_Clear();
 */

#ifndef __ERROR_CODE_H
#define __ERROR_CODE_H

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
 * 错误码枚举定义
 * ========================================================================== */

/**
 * @brief 系统错误码枚举 — 按域分组
 * @note  值 0 保留为"无错误"，各域使用不同的起始值便于快速识别故障类型
 */
typedef enum {
    ERR_NONE = 0,               /**< 无错误 */

    /* ---- 系统域 (0x01 ~ 0x0F) ---- */
    ERR_CLOCK_FAIL     = 0x01,  /**< 系统时钟配置失败 (PLL 锁定超时等) */
    ERR_HAL_INIT       = 0x02,  /**< HAL 库初始化失败 */
    ERR_STACK_OVERFLOW = 0x03,  /**< 栈溢出 (HardFault 中检测) */

    /* ---- 外设域 (0x10 ~ 0x1F) ---- */
    ERR_UART_INIT      = 0x10,  /**< 串口初始化失败 */
    ERR_UART_TX_TIMEOUT= 0x11,  /**< 串口发送超时 */
    ERR_UART_FRAME_LOST= 0x12,  /**< 串口帧丢失 (上层处理过慢) */
    ERR_SPI_INIT       = 0x13,  /**< SPI 初始化失败 */
    ERR_ADC_INIT       = 0x14,  /**< ADC 初始化失败 */

    /* ---- Flash 域 (0x20 ~ 0x2F) ---- */
    ERR_FLASH_ID       = 0x20,  /**< Flash 芯片 ID 读取异常 */
    ERR_FLASH_CRC      = 0x21,  /**< Flash 参数 CRC 校验失败 */
    ERR_FLASH_WRITE    = 0x22,  /**< Flash 写入失败 */
    ERR_FLASH_ERASE    = 0x23,  /**< Flash 擦除失败 */

    /* ---- 电机域 (0x30 ~ 0x3F) ---- */
    ERR_MOTOR_LIMIT    = 0x30,  /**< 电机撞限位开关 */
    ERR_MOTOR_HOME_FAIL= 0x31,  /**< 电机回原超时 */
    ERR_MOTOR_STALL    = 0x32,  /**< 电机堵转 (若有编码器反馈) */
    ERR_MOTOR_OVERRUN  = 0x33,  /**< 电机超行程 */

    /* ---- 通信域 (0x40 ~ 0x4F) ---- */
    ERR_MODBUS_CRC     = 0x40,  /**< Modbus CRC 校验失败 */
    ERR_MODBUS_TIMEOUT = 0x41,  /**< Modbus 通信超时 (心跳丢失) */
    ERR_MODBUS_EXCEPTION= 0x42, /**< Modbus 异常响应 */

    /* ---- 应用域 (0x50 ~ 0x5F) ---- */
    ERR_TASK_OVERRUN   = 0x50,  /**< 任务执行超时 (阻塞其他任务) */
    ERR_SENSOR_FAIL    = 0x51,  /**< 传感器异常 */
    ERR_LOGIC_CONFLICT = 0x52,  /**< 逻辑冲突 (不该出现的状态组合) */

} ErrorCode_t;

/* ==========================================================================
 * 故障日志条目
 * ========================================================================== */

/** @brief 故障日志环形缓冲区容量 (条目数) */
#define ERROR_LOG_SIZE  32

/**
 * @brief 单条故障记录
 */
typedef struct {
    ErrorCode_t code;       /**< 故障码 */
    uint32_t    timestamp;  /**< 发生时刻 (系统节拍数或 HAL_GetTick 毫秒数) */
} ErrorEntry_t;

/* ==========================================================================
 * 公共接口
 * ========================================================================== */

/**
 * @brief  初始化故障记录模块 (清空环形缓冲区)
 */
void Error_Init(void);

/**
 * @brief  记录一条故障码到环形缓冲区
 * @param  code 故障码 (参见 ErrorCode_t 枚举)
 * @note   自动附加时间戳 (HAL_GetTick)。ISR 安全。
 */
void Error_Log(ErrorCode_t code);

/**
 * @brief  获取最近一次故障码
 * @return 最近一条故障码；若无故障记录则返回 ERR_NONE
 */
ErrorCode_t Error_GetLast(void);

/**
 * @brief  获取累计故障记录数 (含已被环形覆盖的)
 * @return 自系统启动/清除以来的故障总次数
 */
uint16_t Error_GetCount(void);

/**
 * @brief  获取指定索引处的故障记录 (用于 Modbus 上位机批量读取)
 * @param  index 索引 (0 = 最旧的可用记录)
 * @param  entry 输出参数: 故障记录指针
 * @return true = 读取成功, false = 索引越界
 */
bool Error_GetEntry(uint16_t index, ErrorEntry_t *entry);

/**
 * @brief  清除所有故障记录
 */
void Error_Clear(void);

/**
 * @brief  检查当前是否存在未清除的故障
 * @return true = 有故障记录
 */
bool Error_HasFault(void);

#endif /* __ERROR_CODE_H */
